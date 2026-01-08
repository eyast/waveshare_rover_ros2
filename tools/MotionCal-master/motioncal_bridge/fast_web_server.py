#!/usr/bin/env python3
"""
Optimized HTTP → Virtual Serial Bridge
"""

import os
import pty
import tty
import fcntl
import asyncio
import argparse
import signal
import sys
from typing import Optional
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import PlainTextResponse


class FastVirtualSerial:
    """Non-blocking PTY with direct writes (no executor overhead)."""
    
    def __init__(self):
        self.master_fd: Optional[int] = None
        self.slave_path: Optional[str] = None
    
    def open(self) -> str:
        self.master_fd, slave_fd = pty.openpty()
        self.slave_path = os.ttyname(slave_fd)
        
        # Set master to non-blocking
        flags = fcntl.fcntl(self.master_fd, fcntl.F_GETFL)
        fcntl.fcntl(self.master_fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        
        # Raw mode
        tty.setraw(self.master_fd)
        
        print(f"[VSerial] Port: {self.slave_path}")
        return self.slave_path
    
    def write_line(self, text: str) -> int:
        """Direct write - no async overhead."""
        if self.master_fd is None:
            return 0
        try:
            data = (text + "\n").encode('utf-8')
            return os.write(self.master_fd, data)
        except BlockingIOError:
            return 0  # Buffer full, drop frame (acceptable at 50Hz)
        except OSError:
            return 0
    
    def close(self):
        if self.master_fd:
            os.close(self.master_fd)
            self.master_fd = None


vserial = FastVirtualSerial()
msg_count = 0

@asynccontextmanager
async def lifespan(app: FastAPI):
    vserial.open()
    print(f"  Connect MotionCal to: {vserial.slave_path}")
    yield
    vserial.close()

app = FastAPI(lifespan=lifespan)

@app.post("/data")
async def receive_data(request: Request):
    """Receive IMU data via HTTP POST and write to virtual serial."""
    global msg_count
    
    # Read raw body as text
    data = await request.body()
    text = data.decode('utf-8').strip()
    
    # Write to virtual serial
    vserial.write_line(text)
    
    msg_count += 1
    if msg_count % 500 == 0:  # Log every 500 messages
        print(f"[Stats] {msg_count} messages")
    
    return PlainTextResponse("OK", status_code=200)


@app.get("/")
async def root():
    """Health check endpoint."""
    return {"status": "running", "messages_received": msg_count}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="warning")