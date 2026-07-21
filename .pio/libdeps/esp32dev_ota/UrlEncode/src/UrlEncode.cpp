#include "UrlEncode.h"
static inline bool isUnreserved(char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
         (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~';
}
String urlEncode(const char *input) {
  if (!input) return String();
  String out;
  size_t len = strlen(input);
  out.reserve(len * 3 + 1);
  static const char hex[] = "0123456789ABCDEF";
  for (size_t i = 0; i < len; ++i) {
    unsigned char c = static_cast<unsigned char>(input[i]);
    if (isUnreserved(static_cast<char>(c))) out += static_cast<char>(c);
    else {
      out += '%';
      out += hex[(c >> 4) & 0x0F];
      out += hex[c & 0x0F];
    }
  }
  return out;
}
String urlEncode(const String &input) {
  return urlEncode(input.c_str());
}
