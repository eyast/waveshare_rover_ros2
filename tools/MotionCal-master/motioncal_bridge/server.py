#!/usr/bin/env python3
"""
Distributed Counter - FastAPI WebSocket Server

Receives counter messages via WebSocket at 50Hz and forwards them
to a virtual serial port that other applications can read from.

Usage:
    python server.py [--host HOST] [--port PORT] [--baud BAUD]

Example:
    python server.py --host 0.0.0.0 --port 8080 --baud 115200
"""

import os
import pty
import tty
import termios
import asyncio
import argparse
import signal
import sys
from datetime import datetime
from typing import Optional
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect


# ============== Virtual Serial Port ==============

class VirtualSerial:
    """
    Creates a pseudo-terminal pair that acts as a virtual serial port.
    
    - We write to the master file descriptor
    - Other applications open the slave device (e.g., /dev/ttys001) to read
    - Baud rate is configured via termios (mostly cosmetic for PTY, but 
      applications may check it)
    """
    
    def __init__(self, baud: int = 115200):
        self.baud = baud
        self.master_fd: Optional[int] = None
        self.slave_fd: Optional[int] = None
        self.slave_path: Optional[str] = None
        self._lock = asyncio.Lock()
        
        # Baud rate constants (termios)
        self._baud_map = {
            9600: termios.B9600,
            19200: termios.B19200,
            38400: termios.B38400,
            57600: termios.B57600,
            115200: termios.B115200,
            230400: termios.B230400,
        }
    
    def open(self) -> str:
        """
        Create the PTY pair and configure it.
        Returns the slave device path that other applications should open.
        """
        # Create pseudo-terminal pair
        self.master_fd, self.slave_fd = pty.openpty()
        self.slave_path = os.ttyname(self.slave_fd)
        
        # Configure the master for raw mode (no echo, no line buffering)
        tty.setraw(self.master_fd)
        
        # Set baud rate on slave (for applications that check it)
        if self.baud in self._baud_map:
            try:
                attrs = termios.tcgetattr(self.slave_fd)
                baud_const = self._baud_map[self.baud]
                attrs[4] = baud_const  # ispeed
                attrs[5] = baud_const  # ospeed
                termios.tcsetattr(self.slave_fd, termios.TCSANOW, attrs)
            except termios.error as e:
                print(f"[VSerial] Warning: Could not set baud rate: {e}")
        
        print(f"[VSerial] Created virtual serial port: {self.slave_path}")
        print(f"[VSerial] Configured baud rate: {self.baud}")
        return self.slave_path
    
    async def write(self, data: bytes) -> int:
        """
        Write data to the virtual serial port (thread-safe).
        Returns number of bytes written.
        """
        if self.master_fd is None:
            raise RuntimeError("Virtual serial port not open")
        
        async with self._lock:
            # Use run_in_executor for non-blocking write
            loop = asyncio.get_event_loop()
            try:
                written = await loop.run_in_executor(
                    None, os.write, self.master_fd, data
                )
                return written
            except OSError as e:
                print(f"[VSerial] Write error: {e}")
                return 0
    
    async def write_line(self, text: str) -> int:
        """Write a line of text (adds newline, encodes as UTF-8)."""
        return await self.write((text + "\n").encode('utf-8'))
    
    def close(self):
        """Close the PTY pair."""
        if self.master_fd is not None:
            try:
                os.close(self.master_fd)
            except OSError:
                pass
            self.master_fd = None
        
        if self.slave_fd is not None:
            try:
                os.close(self.slave_fd)
            except OSError:
                pass
            self.slave_fd = None
        
        print("[VSerial] Closed virtual serial port")


# ============== Statistics Tracker ==============

class Stats:
    """Track message statistics for monitoring."""
    
    def __init__(self):
        self.messages_received = 0
        self.bytes_written = 0
        self.start_time: Optional[datetime] = None
        self.last_message_time: Optional[datetime] = None
        self.connected_clients = 0
    
    def reset(self):
        self.messages_received = 0
        self.bytes_written = 0
        self.start_time = datetime.now()
    
    def record_message(self, bytes_written: int):
        self.messages_received += 1
        self.bytes_written += bytes_written
        self.last_message_time = datetime.now()
    
    def get_rate(self) -> float:
        """Calculate messages per second."""
        if self.start_time is None or self.messages_received == 0:
            return 0.0
        elapsed = (datetime.now() - self.start_time).total_seconds()
        if elapsed <= 0:
            return 0.0
        return self.messages_received / elapsed
    
    def summary(self) -> str:
        rate = self.get_rate()
        return (
            f"Messages: {self.messages_received}, "
            f"Bytes: {self.bytes_written}, "
            f"Rate: {rate:.1f} msg/s, "
            f"Clients: {self.connected_clients}"
        )


# ============== Global State ==============

vserial = VirtualSerial()
stats = Stats()


# ============== FastAPI Application ==============

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup and shutdown handlers."""
    # Startup
    print("\n" + "=" * 50)
    print("  Distributed Counter - WebSocket Server")
    print("=" * 50)
    
    # Get baud from app state (set in main)
    baud = getattr(app.state, 'baud', 115200)
    vserial.baud = baud
    slave_path = vserial.open()
    
    print(f"\n  ► Virtual Serial Port: {slave_path}")
    print(f"  ► Baud Rate: {baud}")
    print(f"\n  To read from the virtual serial port:")
    print(f"    screen {slave_path} {baud}")
    print(f"  Or in Python:")
    print(f"    serial.Serial('{slave_path}', {baud})")
    print("\n" + "=" * 50 + "\n")
    
    stats.reset()
    
    yield
    
    # Shutdown
    vserial.close()
    print(f"\n[Server] Final stats: {stats.summary()}")


app = FastAPI(
    title="Distributed Counter Server",
    lifespan=lifespan
)


@app.get("/")
async def root():
    """Health check and status endpoint."""
    return {
        "status": "running",
        "virtual_serial": vserial.slave_path,
        "stats": {
            "messages": stats.messages_received,
            "rate": round(stats.get_rate(), 1),
            "clients": stats.connected_clients
        }
    }


@app.get("/stats")
async def get_stats():
    """Detailed statistics endpoint."""
    return {
        "messages_received": stats.messages_received,
        "bytes_written": stats.bytes_written,
        "rate_per_second": round(stats.get_rate(), 2),
        "connected_clients": stats.connected_clients,
        "virtual_serial_path": vserial.slave_path,
        "last_message": stats.last_message_time.isoformat() if stats.last_message_time else None
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for receiving counter data.
    
    Each message received is forwarded to the virtual serial port.
    Expected format: plain text counter value (e.g., "12345")
    """
    await websocket.accept()
    stats.connected_clients += 1
    
    client_addr = f"{websocket.client.host}:{websocket.client.port}"
    print(f"[WS] Client connected: {client_addr}")
    
    try:
        while True:
            # Receive message (text)
            data = await websocket.receive_text()
            
            # Forward to virtual serial port
            written = await vserial.write_line(data)
            stats.record_message(written)
            
            # Log every 50 messages (1 per second at 50 Hz)
            if stats.messages_received % 50 == 0:
                print(f"[Stats] {stats.summary()}")
    
    except WebSocketDisconnect:
        print(f"[WS] Client disconnected: {client_addr}")
    except Exception as e:
        print(f"[WS] Error with {client_addr}: {e}")
    finally:
        stats.connected_clients -= 1


# ============== Main Entry Point ==============

def main():
    parser = argparse.ArgumentParser(
        description="WebSocket server that forwards messages to a virtual serial port"
    )
    parser.add_argument(
        "--host", 
        default="0.0.0.0",
        help="Host to bind to (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--port", 
        type=int, 
        default=8080,
        help="Port to listen on (default: 8080)"
    )
    parser.add_argument(
        "--baud", 
        type=int, 
        default=115200,
        help="Virtual serial baud rate (default: 115200)"
    )
    
    args = parser.parse_args()
    
    # Store baud in app state for lifespan handler
    app.state.baud = args.baud
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\n[Server] Shutting down...")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print(f"[Server] Starting on {args.host}:{args.port}")
    
    uvicorn.run(
        app,
        host=args.host,
        port=args.port,
        log_level="warning",  # Reduce uvicorn noise
        access_log=False
    )


if __name__ == "__main__":
    main()
