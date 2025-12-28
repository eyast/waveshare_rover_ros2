/*
 * WebSocket Module
 * 
 * Provides optional WebSocket connectivity for remote telemetry.
 * When connected, all protocol output is routed to WebSocket instead of Serial.
 */

#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include <Arduino.h>
#include "config.h"

// =============================================================================
// WebSocket Functions
// =============================================================================

// Initialize WebSocket module (call in setup)
void websocket_init();

// Connect to WiFi and WebSocket server
void websocket_connect(const char* ssid, const char* password,
                       const char* server, uint16_t port = WS_PORT_DEFAULT);

// Disconnect from WebSocket and WiFi
void websocket_disconnect();

// Process WebSocket events (call in loop)
void websocket_loop();

// Check connection status
bool websocket_is_connected();

// Send data via WebSocket (used by protocol module)
void websocket_send(const char* data);

#endif // WEBSOCKET_H
