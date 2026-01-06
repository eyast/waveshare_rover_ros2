#ifndef WEBSOCKETS_H
#define WEBSOCKETS_H

#include <WebSocketsClient.h>
#include "config.h"

// Variable declarations (extern = "exists elsewhere")
extern WebSocketsClient webSocket;
extern String wifiSSID;
extern String wifiPassword;
extern String serverIP;
extern uint16_t serverPort;
extern bool wssendingEnabled;
extern bool wsConnected;

// Function declarations (no body)
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
bool connectWiFi();
void connectWebSocket();
void wsStatus();

// Class definition (this IS allowed in headers - it's a type, not a variable)
class TextBridge {
private:
    String buffer;
public:
    void print(const char* s)    { buffer += s; }
    void print(int v)            { buffer += String(v); }
    void print(float v, int d=2) { buffer += String(v, d); }
    
    void println(int v)            { print(v); flush(); }
    void println(float v, int d=2) { print(v, d); flush(); }
    void println(const char* s="") { print(s); flush(); }
    void flush();  // Declare only - definition needs webSocket
};

extern TextBridge Out;  // Declaration

#endif