#!/usr/bin/env python3
"""Pre-build patch for PsychicHttp's PsychicEventSource.cpp.

The upstream library has an infinite retry loop in
PsychicEventSourceClient::sendEvent():

    do {
        result = httpd_socket_send(..., 0);
    } while (result == HTTPD_SOCK_ERR_TIMEOUT);

A half-dead SSE client (laptop sleep, browser tab in background, etc.)
makes that loop spin forever in the caller's task (loopTask in our
case), which starves the watchdog and freezes the ESP32. We limit the
retries to 2 attempts so a dead client merely drops one event instead
of taking down the whole device.

This runs as a PlatformIO pre-build script so the patch survives
`pio lib update` / clean builds.
"""
from __future__ import annotations

import os
import sys

Import("env")  # noqa: F821 — provided by PlatformIO SCons env

OLD = (
    "void PsychicEventSourceClient::sendEvent(const char *event) {\n"
    "  int result;\n"
    "  do {\n"
    "    result = httpd_socket_send(this->server(), this->socket(), event, strlen(event), 0);\n"
    "  } while (result == HTTPD_SOCK_ERR_TIMEOUT);"
)

NEW = (
    "void PsychicEventSourceClient::sendEvent(const char *event) {\n"
    "  int result;\n"
    "  int retries = 0;\n"
    "  do {\n"
    "    result = httpd_socket_send(this->server(), this->socket(), event, strlen(event), 0);\n"
    "    if (result == HTTPD_SOCK_ERR_TIMEOUT && ++retries >= 2) break; /* PATCHED: avoid infinite spin on dead SSE client */\n"
    "  } while (result == HTTPD_SOCK_ERR_TIMEOUT);"
)

SENTINEL = "/* PATCHED: avoid infinite spin on dead SSE client */"


def patch_file(path: str) -> bool:
    try:
        with open(path, "r", encoding="utf-8") as f:
            src = f.read()
    except OSError:
        return False
    if SENTINEL in src:
        return False  # already patched
    if OLD not in src:
        print(f"[patch_psychic_sse] WARNING: expected pattern not found in {path}", file=sys.stderr)
        return False
    src = src.replace(OLD, NEW, 1)
    with open(path, "w", encoding="utf-8") as f:
        f.write(src)
    print(f"[patch_psychic_sse] patched {path}")
    return True


def find_targets() -> list[str]:
    project_dir = env["PROJECT_DIR"]  # noqa: F821
    libdeps = os.path.join(project_dir, ".pio", "libdeps")
    hits: list[str] = []
    if not os.path.isdir(libdeps):
        return hits
    for root, _dirs, files in os.walk(libdeps):
        if "PsychicEventSource.cpp" in files:
            hits.append(os.path.join(root, "PsychicEventSource.cpp"))
    return hits


for tgt in find_targets():
    patch_file(tgt)
