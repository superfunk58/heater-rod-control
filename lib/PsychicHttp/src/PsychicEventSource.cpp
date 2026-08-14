/*
  Asynchronous WebServer library for Espressif MCUs

  Copyright (c) 2016 Hristo Gochkov. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "PsychicEventSource.h"
#include "freertos/semphr.h"

// Patched: protect _clients list from concurrent access between the SSE send
// task (iterating) and the httpd task (addClient/removeClient). Without this,
// a client disconnect during events.send() invalidates the std::list iterator
// and crashes the ESP32.
static SemaphoreHandle_t s_sseListMutex = nullptr;
static void sseListLock() {
  if (!s_sseListMutex) s_sseListMutex = xSemaphoreCreateMutex();
  if (s_sseListMutex) xSemaphoreTake(s_sseListMutex, portMAX_DELAY);
}
static void sseListUnlock() {
  if (s_sseListMutex) xSemaphoreGive(s_sseListMutex);
}

/*****************************************/
// PsychicEventSource - Handler
/*****************************************/

PsychicEventSource::PsychicEventSource() :
  PsychicHandler(),
  _onOpen(NULL),
  _onClose(NULL)
{}

PsychicEventSource::~PsychicEventSource() {
}

PsychicEventSourceClient * PsychicEventSource::getClient(int socket)
{
  PsychicClient *client = PsychicHandler::getClient(socket);

  if (client == NULL)
    return NULL;

  return (PsychicEventSourceClient *)client->_friend;
}

PsychicEventSourceClient * PsychicEventSource::getClient(PsychicClient *client) {
  return getClient(client->socket());
}

esp_err_t PsychicEventSource::handleRequest(PsychicRequest *request)
{
  //start our open ended HTTP response
  PsychicEventSourceResponse response(request);
  esp_err_t err = response.send();

  //lookup our client
  PsychicClient *client = checkForNewClient(request->client());
  if (client->isNew)
  {
    //did we get our last id?
    if(request->hasHeader("Last-Event-ID"))
    {
      PsychicEventSourceClient *buddy = getClient(client);
      buddy->_lastId = atoi(request->header("Last-Event-ID").c_str());
    }

    //let our handler know.
    openCallback(client);
  }

  return err;
}

PsychicEventSource * PsychicEventSource::onOpen(PsychicEventSourceClientCallback fn) {
  _onOpen = fn;
  return this;
}

PsychicEventSource * PsychicEventSource::onClose(PsychicEventSourceClientCallback fn) {
  _onClose = fn;
  return this;
}

void PsychicEventSource::addClient(PsychicClient *client) {
  sseListLock();
  client->_friend = new PsychicEventSourceClient(client);
  PsychicHandler::addClient(client);
  sseListUnlock();
}

void PsychicEventSource::removeClient(PsychicClient *client) {
  sseListLock();
  PsychicHandler::removeClient(client);
  delete (PsychicEventSourceClient*)client->_friend;
  client->_friend = NULL;
  sseListUnlock();
}

void PsychicEventSource::openCallback(PsychicClient *client) {
  PsychicEventSourceClient *buddy = getClient(client);
  if (buddy == NULL)
  {
    return;
  }

  if (_onOpen != NULL)
    _onOpen(buddy);
}

void PsychicEventSource::closeCallback(PsychicClient *client) {
  PsychicEventSourceClient *buddy = getClient(client);
  if (buddy == NULL)
  {
    return;
  }

  if (_onClose != NULL)
    _onClose(getClient(buddy));
}

void PsychicEventSource::send(const char *message, const char *event, uint32_t id, uint32_t reconnect)
{
  String ev = generateEventMessage(message, event, id, reconnect);
  // Copy client list under mutex to avoid iterator invalidation if
  // addClient/removeClient runs on the httpd task during iteration.
  sseListLock();
  std::list<PsychicClient*> clientsCopy = _clients;
  sseListUnlock();
  for(PsychicClient *c : clientsCopy) {
    if (c->_friend != NULL)
      ((PsychicEventSourceClient*)c->_friend)->sendEvent(ev.c_str());
  }
}

// Thread-safe count for cross-task access (loop task / SSE task).
int PsychicEventSource::safeCount() {
  sseListLock();
  int n = _clients.size();
  sseListUnlock();
  return n;
}

// Thread-safe close-all: copy the client list under the mutex, then close
// each socket. httpd_sess_trigger_close fires the actual closeCallback
// asynchronously on the httpd task, which is safe.
void PsychicEventSource::closeAll() {
  sseListLock();
  std::list<PsychicClient*> clientsCopy = _clients;
  sseListUnlock();
  for (PsychicClient *c : clientsCopy) {
    if (c) c->close();
  }
}

/*****************************************/
// PsychicEventSourceClient
/*****************************************/

PsychicEventSourceClient::PsychicEventSourceClient(PsychicClient *client) :
  PsychicClient(client->server(), client->socket()),
  _lastId(0)
{
}

PsychicEventSourceClient::~PsychicEventSourceClient(){
}

void PsychicEventSourceClient::send(const char *message, const char *event, uint32_t id, uint32_t reconnect){
  String ev = generateEventMessage(message, event, id, reconnect);
  sendEvent(ev.c_str());
}

void PsychicEventSourceClient::sendEvent(const char *event) {
  // Patched: no retry on timeout — drop event instead of blocking the SSE task
  // for up to 5s per slow client (send_wait_timeout). Original had an infinite
  // do-while retry loop that could block events.send() for 20+ seconds.
  httpd_socket_send(this->server(), this->socket(), event, strlen(event), 0);

  //if (result < 0)
  //error log here
}

/*****************************************/
// PsychicEventSourceResponse
/*****************************************/

PsychicEventSourceResponse::PsychicEventSourceResponse(PsychicRequest *request) 
  : PsychicResponse(request)
{
}

esp_err_t PsychicEventSourceResponse::send() {

  //build our main header
  String out = String();
  out.concat("HTTP/1.1 200 OK\r\n");
  out.concat("Content-Type: text/event-stream\r\n");
  out.concat("Cache-Control: no-cache\r\n");
  out.concat("Connection: keep-alive\r\n");

  //get our global headers out of the way first
  for (HTTPHeader header : DefaultHeaders::Instance().getHeaders())
    out.concat(String(header.field) + ": " + String(header.value) + "\r\n");

  //separator
  out.concat("\r\n");

  // Patched: no retry on timeout — drop instead of blocking httpd task.
  int result = httpd_send(_request->request(), out.c_str(), out.length());

  if (result < 0)
    ESP_LOGE(PH_TAG, "EventSource send failed with %s", esp_err_to_name(result));

  if (result > 0)
    return ESP_OK;
  else
    return ESP_ERR_HTTPD_RESP_SEND;
}

/*****************************************/
// Event Message Generator
/*****************************************/

String generateEventMessage(const char *message, const char *event, uint32_t id, uint32_t reconnect) {
  String ev = "";

  if(reconnect){
    ev += "retry: ";
    ev += String(reconnect);
    ev += "\r\n";
  }

  if(id){
    ev += "id: ";
    ev += String(id);
    ev += "\r\n";
  }

  if(event != NULL){
    ev += "event: ";
    ev += String(event);
    ev += "\r\n";
  }

  if(message != NULL){
    ev += "data: ";
    ev += String(message);
    ev += "\r\n";
  }
  ev += "\r\n";

  return ev;
}