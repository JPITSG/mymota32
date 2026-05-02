/*
  WebServer.cpp - Dead simple web-server.
  Supports only one simultaneous client, knows how to handle GET and POST.

  Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.

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
  Modified 8 May 2015 by Hristo Gochkov (proper post and file upload handling)
*/

#include <Arduino.h>
#include <esp32-hal-log.h>
#include "NetworkServer.h"
#include "NetworkClient.h"
#include "WebServer.h"
#include "FS.h"
#include "detail/RequestHandlersImpl.h"

static const char Content_Length[] = "Content-Length";

WebServer::WebServer(IPAddress addr, int port) : _server(addr, port) {
  log_v("WebServer::Webserver(addr=%s, port=%d)", addr.toString().c_str(), port);
}

WebServer::WebServer(int port) : _server(port) {
  log_v("WebServer::Webserver(port=%d)", port);
}

WebServer::~WebServer() {
  _server.close();

  _clearRequestHeaders();
  _clearResponseHeaders();

  RequestHandler *handler = _firstHandler;
  while (handler) {
    RequestHandler *next = handler->next();
    delete handler;
    handler = next;
  }
  _firstHandler = nullptr;
}

void WebServer::begin() {
  close();
  _server.begin();
  _server.setNoDelay(true);
}

void WebServer::begin(uint16_t port) {
  close();
  _server.begin(port);
  _server.setNoDelay(true);
}

bool WebServer::authenticateBasicSHA1(const char *_username, const char *_sha1Base64orHex) {
  (void)_username;
  (void)_sha1Base64orHex;
  return false;
}

bool WebServer::authenticate(const char *_username, const char *_password) {
  (void)_username;
  (void)_password;
  return false;
}

bool WebServer::authenticate(THandlerFunctionAuthCheck fn) {
  (void)fn;
  return false;
}

void WebServer::requestAuthentication(HTTPAuthMethod mode, const char *realm, const String &authFailMsg) {
  (void)mode;
  (void)realm;
  send(401, F("text/html"), authFailMsg);
}

RequestHandler &WebServer::on(const Uri &uri, WebServer::THandlerFunction handler) {
  return on(uri, HTTP_ANY, handler);
}

RequestHandler &WebServer::on(const Uri &uri, HTTPMethod method, WebServer::THandlerFunction fn) {
  return on(uri, method, fn, _fileUploadHandler);
}

RequestHandler &WebServer::on(const Uri &uri, HTTPMethod method, WebServer::THandlerFunction fn, WebServer::THandlerFunction ufn) {
  FunctionRequestHandler *handler = new FunctionRequestHandler(fn, ufn, uri, method);
  _addRequestHandler(handler);
  return *handler;
}

bool WebServer::removeRoute(const char *uri) {
  return removeRoute(String(uri), HTTP_ANY);
}

bool WebServer::removeRoute(const char *uri, HTTPMethod method) {
  return removeRoute(String(uri), method);
}

bool WebServer::removeRoute(const String &uri) {
  return removeRoute(uri, HTTP_ANY);
}

bool WebServer::removeRoute(const String &uri, HTTPMethod method) {
  bool anyHandlerRemoved = false;
  RequestHandler *handler = _firstHandler;
  RequestHandler *previousHandler = nullptr;

  while (handler) {
    if (handler->canHandle(method, uri)) {
      if (_removeRequestHandler(handler)) {
        anyHandlerRemoved = true;
        // Move to the next handler
        if (previousHandler) {
          handler = previousHandler->next();
        } else {
          handler = _firstHandler;
        }
        continue;
      }
    }
    previousHandler = handler;
    handler = handler->next();
  }

  return anyHandlerRemoved;
}

void WebServer::addHandler(RequestHandler *handler) {
  _addRequestHandler(handler);
}

bool WebServer::removeHandler(RequestHandler *handler) {
  return _removeRequestHandler(handler);
}

void WebServer::_addRequestHandler(RequestHandler *handler) {
  if (!_lastHandler) {
    _firstHandler = handler;
    _lastHandler = handler;
  } else {
    _lastHandler->next(handler);
    _lastHandler = handler;
  }
}

bool WebServer::_removeRequestHandler(RequestHandler *handler) {
  RequestHandler *current = _firstHandler;
  RequestHandler *previous = nullptr;

  while (current != nullptr) {
    if (current == handler) {
      if (previous == nullptr) {
        _firstHandler = current->next();
      } else {
        previous->next(current->next());
      }

      if (current == _lastHandler) {
        _lastHandler = previous;
      }

      // Delete 'matching' handler
      delete current;
      return true;
    }
    previous = current;
    current = current->next();
  }
  return false;
}

void WebServer::serveStatic(const char *uri, FS &fs, const char *path, const char *cache_header) {
  (void)uri;
  (void)fs;
  (void)path;
  (void)cache_header;
}

void WebServer::handleClient() {
  if (_currentStatus == HC_NONE) {
    _currentClient = _server.accept();
    if (!_currentClient) {
      if (_nullDelay) {
        delay(1);
      }
      return;
    }

    log_v("New client: client.localIP()=%s", _currentClient.localIP().toString().c_str());

    _currentStatus = HC_WAIT_READ;
    _statusChange = millis();
  }

  bool keepCurrentClient = false;
  bool callYield = false;

  if (_currentClient.connected()) {
    switch (_currentStatus) {
      case HC_NONE:
        // No-op to avoid C++ compiler warning
        break;
      case HC_WAIT_READ:
        // Wait for data from client to become available
        if (_currentClient.available()) {
          _currentClient.setTimeout(HTTP_MAX_SEND_WAIT); /* / 1000 removed, WifiClient setTimeout changed to ms */
          if (_parseRequest(_currentClient)) {
            _contentLength = CONTENT_LENGTH_NOT_SET;
            _responseCode = 0;
            _clearResponseHeaders();

            _handleRequest();

            // Fix for issue with Chrome based browsers: https://github.com/espressif/arduino-esp32/issues/3652
            //           if (_currentClient.connected()) {
            //             _currentStatus = HC_WAIT_CLOSE;
            //             _statusChange = millis();
            //             keepCurrentClient = true;
            //           }
          }
        } else {  // !_currentClient.available()
          if (millis() - _statusChange <= HTTP_MAX_DATA_WAIT) {
            keepCurrentClient = true;
          }
          callYield = true;
        }
        break;
      case HC_WAIT_CLOSE:
        // Wait for client to close the connection
        if (millis() - _statusChange <= HTTP_MAX_CLOSE_WAIT) {
          keepCurrentClient = true;
          callYield = true;
        }
    }
  }

  if (!keepCurrentClient) {
    _currentClient = NetworkClient();
    _currentStatus = HC_NONE;
    _currentUpload.reset();
    _currentRaw.reset();
  }

  if (callYield) {
    yield();
  }
}

void WebServer::close() {
  _server.close();
  _currentStatus = HC_NONE;
}

void WebServer::stop() {
  close();
}

void WebServer::sendHeader(const String &name, const String &value, bool first) {
  RequestArgument *header = new RequestArgument();
  header->key = name;
  header->value = value;

  if (!_responseHeaders || first) {
    header->next = _responseHeaders;
    _responseHeaders = header;
  } else {
    RequestArgument *last = _responseHeaders;
    while (last->next) {
      last = last->next;
    }
    last->next = header;
  }

  _responseHeaderCount++;
}

void WebServer::setContentLength(const size_t contentLength) {
  _contentLength = contentLength;
}

void WebServer::enableDelay(boolean value) {
  _nullDelay = value;
}

void WebServer::enableCORS(boolean value) {
  (void)value;
}

void WebServer::enableCrossOrigin(boolean value) {
  enableCORS(value);
}

void WebServer::enableETag(bool enable, ETagFunction fn) {
  (void)enable;
  (void)fn;
}

void WebServer::_prepareHeader(String &response, int code, const char *content_type, size_t contentLength) {
  _responseCode = code;

  response.concat(version());
  response.concat(' ');
  response.concat(String(code));
  response.concat(' ');
  response.concat(responseCodeToString(code));
  response.concat(F("\r\n"));

  if (!content_type) {
    content_type = "text/html";
  }

  sendHeader(String(F("Content-Type")), String(FPSTR(content_type)), true);
  if (_contentLength == CONTENT_LENGTH_NOT_SET) {
    sendHeader(String(FPSTR(Content_Length)), String(contentLength));
  } else if (_contentLength != CONTENT_LENGTH_UNKNOWN) {
    sendHeader(String(FPSTR(Content_Length)), String(_contentLength));
  } else if (_contentLength == CONTENT_LENGTH_UNKNOWN && _currentVersion) {  //HTTP/1.1 or above client
    //let's do chunked
    _chunked = true;
    sendHeader(String(F("Accept-Ranges")), String(F("none")));
    sendHeader(String(F("Transfer-Encoding")), String(F("chunked")));
  }
  sendHeader(String(F("Connection")), String(F("close")));

  for (RequestArgument *header = _responseHeaders; header; header = header->next) {
    response.concat(header->key);
    response.concat(F(": "));
    response.concat(header->value);
    response.concat(F("\r\n"));
  }

  response.concat(F("\r\n"));
}

void WebServer::send(int code, const char *content_type, const String &content) {
  String header;
  // Can we assume the following?
  //if(code == 200 && content.length() == 0 && _contentLength == CONTENT_LENGTH_NOT_SET)
  //  _contentLength = CONTENT_LENGTH_UNKNOWN;
  _prepareHeader(header, code, content_type, content.length());
  _currentClientWrite(header.c_str(), header.length());
  if (content.length()) {
    sendContent(content);
  }
}

void WebServer::send(int code, char *content_type, const String &content) {
  send(code, (const char *)content_type, content);
}

void WebServer::send(int code, const String &content_type, const String &content) {
  send(code, (const char *)content_type.c_str(), content);
}

void WebServer::send(int code, const char *content_type, const char *content) {
  const String passStr = (String)content;
  if (strlen(content) != passStr.length()) {
    log_e("String cast failed.  Use send_P for long arrays");
  }
  send(code, content_type, passStr);
}

void WebServer::send_P(int code, PGM_P content_type, PGM_P content) {
  size_t contentLength = 0;

  if (content != NULL) {
    contentLength = strlen_P(content);
  }

  String header;
  char type[64];
  memccpy_P((void *)type, (PGM_VOID_P)content_type, 0, sizeof(type));
  _prepareHeader(header, code, (const char *)type, contentLength);
  _currentClientWrite(header.c_str(), header.length());
  sendContent_P(content);
}

void WebServer::send_P(int code, PGM_P content_type, PGM_P content, size_t contentLength) {
  String header;
  char type[64];
  memccpy_P((void *)type, (PGM_VOID_P)content_type, 0, sizeof(type));
  _prepareHeader(header, code, (const char *)type, contentLength);
  sendContent(header);
  sendContent_P(content, contentLength);
}

void WebServer::sendContent(const String &content) {
  sendContent(content.c_str(), content.length());
}

void WebServer::sendContent(const char *content, size_t contentLength) {
  const char *footer = "\r\n";
  if (_chunked) {
    char *chunkSize = (char *)malloc(11);
    if (chunkSize) {
      sprintf(chunkSize, "%x%s", contentLength, footer);
      _currentClientWrite(chunkSize, strlen(chunkSize));
      free(chunkSize);
    }
  }
  _currentClientWrite(content, contentLength);
  if (_chunked) {
    _currentClient.write(footer, 2);
    if (contentLength == 0) {
      _chunked = false;
    }
  }
}

void WebServer::sendContent_P(PGM_P content) {
  sendContent_P(content, strlen_P(content));
}

void WebServer::sendContent_P(PGM_P content, size_t size) {
  const char *footer = "\r\n";
  if (_chunked) {
    char *chunkSize = (char *)malloc(11);
    if (chunkSize) {
      sprintf(chunkSize, "%x%s", size, footer);
      _currentClientWrite(chunkSize, strlen(chunkSize));
      free(chunkSize);
    }
  }
  _currentClientWrite_P(content, size);
  if (_chunked) {
    _currentClient.write(footer, 2);
    if (size == 0) {
      _chunked = false;
    }
  }
}

void WebServer::_streamFileCore(const size_t fileSize, const String &fileName, const String &contentType, const int code) {
  (void)fileName;
  setContentLength(fileSize);
  send(code, contentType, "");
}

String WebServer::pathArg(unsigned int i) const {
  if (_currentHandler != nullptr) {
    return _currentHandler->pathArg(i);
  }
  return "";
}

String WebServer::arg(const String &name) const {
  for (int j = 0; j < _postArgsLen; ++j) {
    if (_postArgs[j].key == name) {
      return _postArgs[j].value;
    }
  }
  for (int i = 0; i < _currentArgCount; ++i) {
    if (_currentArgs[i].key == name) {
      return _currentArgs[i].value;
    }
  }
  return "";
}

String WebServer::arg(int i) const {
  if (i < _currentArgCount) {
    return _currentArgs[i].value;
  }
  return "";
}

String WebServer::argName(int i) const {
  if (i < _currentArgCount) {
    return _currentArgs[i].key;
  }
  return "";
}

int WebServer::args() const {
  return _currentArgCount;
}

bool WebServer::hasArg(const String &name) const {
  for (int j = 0; j < _postArgsLen; ++j) {
    if (_postArgs[j].key == name) {
      return true;
    }
  }
  for (int i = 0; i < _currentArgCount; ++i) {
    if (_currentArgs[i].key == name) {
      return true;
    }
  }
  return false;
}

String WebServer::header(const String &name) const {
  for (RequestArgument *current = _currentHeaders; current; current = current->next) {
    if (current->key.equalsIgnoreCase(name)) {
      return current->value;
    }
  }
  return "";
}

void WebServer::collectHeaders(const char *headerKeys[], const size_t headerKeysCount) {
  _clearRequestHeaders();
  _collectAllHeaders = false;
  _headerKeysCount = headerKeysCount;

  RequestArgument *last = nullptr;
  for (size_t i = 0; i < headerKeysCount; i++) {
    RequestArgument *entry = new RequestArgument();
    entry->key = headerKeys[i];
    if (last) {
      last->next = entry;
    } else {
      _currentHeaders = entry;
    }
    last = entry;
  }
}

String WebServer::header(int i) const {
  RequestArgument *current = _currentHeaders;
  while (current && i--) {
    current = current->next;
  }
  return current ? current->value : emptyString;
}

String WebServer::headerName(int i) const {
  RequestArgument *current = _currentHeaders;
  while (current && i--) {
    current = current->next;
  }
  return current ? current->key : emptyString;
}

int WebServer::headers() const {
  return _headerKeysCount;
}

bool WebServer::hasHeader(const String &name) const {
  return header(name).length() > 0;
}

String WebServer::hostHeader() const {
  return _hostHeader;
}

void WebServer::onFileUpload(THandlerFunction fn) {
  _fileUploadHandler = fn;
}

void WebServer::onNotFound(THandlerFunction fn) {
  _notFoundHandler = fn;
}

bool WebServer::_handleRequest() {
  bool handled = false;
  if (_currentHandler) {
    handled = _currentHandler->process(*this, _currentMethod, _currentUri);
    if (!handled) {
      log_e("request handler failed to handle request");
    }
  }
  // DO NOT LOG if _currentHandler == null !!
  // This is is valid use case to handle any other requests
  // Also, this is just causing log flooding
  if (!handled && _notFoundHandler) {
    _notFoundHandler();
    handled = true;
  }
  if (!handled) {
    send(404, F("text/html"), String(F("Not found: ")) + _currentUri);
    handled = true;
  }
  if (handled) {
    _finalizeResponse();
  }
  _currentUri = "";
  return handled;
}

void WebServer::_finalizeResponse() {
  if (_chunked) {
    sendContent("");
  }
}

String WebServer::responseCodeToString(int code) {
  switch (code) {
    case 200: return F("OK");
    case 204: return F("No Content");
    case 302: return F("Found");
    case 303: return F("See Other");
    case 400: return F("Bad Request");
    case 401: return F("Unauthorized");
    case 404: return F("Not Found");
    case 500: return F("Internal Server Error");
    default:  return F("");
  }
}

void WebServer::_clearResponseHeaders() {
  _responseHeaderCount = 0;
  RequestArgument *current = _responseHeaders;
  while (current) {
    RequestArgument *next = current->next;
    delete current;
    current = next;
  }
  _responseHeaders = nullptr;
}

void WebServer::_clearRequestHeaders() {
  _headerKeysCount = 0;
  RequestArgument *current = _currentHeaders;
  while (current) {
    RequestArgument *next = current->next;
    delete current;
    current = next;
  }
  _currentHeaders = nullptr;
}

void WebServer::collectAllHeaders() {
  _clearRequestHeaders();
  _headerKeysCount = 0;
  _collectAllHeaders = true;
}

const String &WebServer::responseHeader(String name) const {
  for (RequestArgument *current = _responseHeaders; current; current = current->next) {
    if (current->key.equalsIgnoreCase(name)) {
      return current->value;
    }
  }
  return emptyString;
}

const String &WebServer::responseHeader(int i) const {
  RequestArgument *current = _responseHeaders;
  while (current && i--) {
    current = current->next;
  }
  return current ? current->value : emptyString;
}

const String &WebServer::responseHeaderName(int i) const {
  RequestArgument *current = _responseHeaders;
  while (current && i--) {
    current = current->next;
  }
  return current ? current->key : emptyString;
}

bool WebServer::hasResponseHeader(const String &name) const {
  return header(name).length() > 0;
}

int WebServer::clientContentLength() const {
  return _clientContentLength;
}

const String WebServer::version() const {
  String v;
  v.reserve(8);
  v.concat(F("HTTP/1."));
  v.concat(_currentVersion);
  return v;
}
int WebServer::responseCode() const {
  return _responseCode;
}
int WebServer::responseHeaders() const {
  return _responseHeaderCount;
}

WebServer &WebServer::addMiddleware(Middleware *middleware) {
  (void)middleware;
  return *this;
}

WebServer &WebServer::addMiddleware(Middleware::Function fn) {
  (void)fn;
  return *this;
}

WebServer &WebServer::removeMiddleware(Middleware *middleware) {
  (void)middleware;
  return *this;
}
