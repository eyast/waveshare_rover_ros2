/*
 * WebSocket Module Implementation
 */

#include "websocket.h"
#include "protocol.h"
#include <WiFi.h>
#include <WebSocketsClient.h>

// Global WebSocket client
WebSocketsClient webSocket;

// Connection state
static bool ws_connected = false;
static bool ws_enabled = false;

// =============================================================================
// WebSocket Event Handler
// =============================================================================

static void websocket_event(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            ws_connected = false;
            protocol_notify_ws_state(false);
            out_system("WS_EVENT", "disconnected");
            break;
            
        case WStype_CONNECTED:
            ws_connected = true;
            protocol_notify_ws_state(true);
            out_system("WS_EVENT", "connected");
            break;
            
        case WStype_TEXT:
            // Forward incoming WebSocket commands to command parser
            // (payload is null-terminated by WebSocketsClient)
            if (length > 0) {
                // We need to include commands.h for this, but that creates
                // a circular dependency. Instead, we'll just print to Serial
                // and let the user handle it.
                Serial.print("WS_RX:");
                Serial.println((char*)payload);
            }
            break;
            
        case WStype_ERROR:
            out_error("WS_EVENT", "error");
            break;
            
        default:
            break;
    }
}

// =============================================================================
// WebSocket Functions
// =============================================================================

void websocket_init() {
    ws_connected = false;
    ws_enabled = false;
}

void websocket_connect(const char* ssid, const char* password,
                       const char* server, uint16_t port) {
    // Disconnect if already connected
    if (ws_enabled) {
        websocket_disconnect();
    }
    
    // Connect to WiFi
    out_system("WIFI", "connecting");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > WIFI_TIMEOUT_MS) {
            out_error("WIFI", "timeout");
            return;
        }
        delay(500);
    }
    
    // Report IP
    char ip_str[16];
    IPAddress ip = WiFi.localIP();
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    out_system("WIFI", ip_str);
    
    // Connect to WebSocket
    out_system("WS", "connecting");
    webSocket.begin(server, port, "/ws");
    webSocket.onEvent(websocket_event);
    webSocket.setReconnectInterval(WS_RECONNECT_MS);
    
    ws_enabled = true;
}

void websocket_disconnect() {
    if (ws_enabled) {
        webSocket.disconnect();
        WiFi.disconnect();
        ws_connected = false;
        ws_enabled = false;
        protocol_notify_ws_state(false);
        out_system("WS", "disconnected");
    }
}

void websocket_loop() {
    if (ws_enabled) {
        webSocket.loop();
    }
}

bool websocket_is_connected() {
    return ws_connected;
}

void websocket_send(const char* data) {
    if (ws_connected) {
        webSocket.sendTXT(data);
    }
}
