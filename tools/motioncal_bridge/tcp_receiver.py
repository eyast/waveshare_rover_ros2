#!/usr/bin/env python3
"""
Simple Serial Forwarder - Server
Receives data via TCP and writes to virtual serial port
"""
import socket
import os
import pty
import tty

def create_virtual_serial():
    """Create a virtual serial port."""
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    tty.setraw(master)
    print(f"Virtual serial port: {slave_name}")
    print(f"Connect MotionCal to: {slave_name}")
    return master

def main(port: int):
    # Create virtual serial
    vserial = create_virtual_serial()
    
    # Start TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(1)
    print(f"Listening on port {port}...")
    
    try:
        while True:
            # Wait for client
            print("Waiting for connection...")
            client, addr = server.accept()
            print(f"Client connected from {addr}")
            
            counter = 0
            try:
                while True:
                    # Receive data
                    data = client.recv(4096)
                    if not data:
                        break
                    
                    # Write to virtual serial
                    os.write(vserial, data)
                    counter += 1
                    
                    if counter % 500 == 0:
                        print(f"Received {counter} messages")
                        
            except Exception as e:
                print(f"Error: {e}")
            finally:
                client.close()
                print("Client disconnected")
                
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        server.close()
        os.close(vserial)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    
    main(args.port)