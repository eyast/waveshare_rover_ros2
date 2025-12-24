#include "websockets.h"

// Variable definitions (actual storage allocated HERE, once)
WebSocketsClient webSocket;
String wifiSSID = "";
String wifiPassword = "";
String serverIP = "";
uint16_t serverPort = 8080;
bool wssendingEnabled = false;
bool wsConnected = false;
TextBridge Out;

// Method definition
void TextBridge::flush() {
    if (wssendingEnabled) {
        webSocket.sendTXT(buffer);
    } else {
        Serial.println(buffer);
    }
    buffer = "";
}

// Function definitions
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            wsConnected = false;
            Serial.println("[WS] Disconnected");
            break;
        case WStype_CONNECTED:
            wsConnected = true;
            Serial.print("[WS] Connected to: ");
            Serial.println((char*)payload);
            break;
        case WStype_TEXT:
            Serial.print("[WS] Received: ");
            Serial.println((char*)payload);
            break;
        case WStype_ERROR:
            Serial.println("[WS] Error occurred");
            break;
        default:
            break;
    }
}

bool connectWiFi() {
    if (wifiSSID.length() == 0) {
        Serial.println("[WiFi] No SSID configured.");
        return false;
    }
    Serial.print("[WiFi] Connecting to: ");
    Serial.println(wifiSSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    
    uint32_t startTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - startTime > WIFI_CONNECT_TIMEOUT_MS) {
            Serial.println("[WiFi] Connection timeout");
            return false;
        }
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("[WiFi] Connected! IP: ");
    Serial.println(WiFi.localIP());
    return true;
}

void connectWebSocket() {
    if (serverIP.length() == 0) {
        Serial.println("[WS] No server configured.");
        return;
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WS] WiFi not connected");
        return;
    }
    Serial.print("[WS] Connecting to ws://");
    Serial.print(serverIP);
    Serial.print(":");
    Serial.println(serverPort);
    
    webSocket.begin(serverIP.c_str(), serverPort, "/ws");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(WS_RECONNECT_INTERVAL_MS);
}

void wsStatus() {
    Serial.println("======== WS STATUS ========");
    Serial.print("WiFi: ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.print("WebSocket: ");
    Serial.println(wsConnected ? "Connected" : "Disconnected");
    Serial.print("Streaming: ");
    Serial.println(wssendingEnabled ? "Enabled" : "Disabled");
    Serial.println("===========================");
}