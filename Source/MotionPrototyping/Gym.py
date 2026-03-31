import socket
import sys
import time
import signal
import traceback
import os

import struct
from dataclasses import dataclass

@dataclass
class VelocityData:
    speed: float
    direction_x: float
    direction_y: float
    direction_z: float
    
    @classmethod
    def from_bytes(cls, data: bytes):
        # Format: 'f' for float, total 4 floats (speed + direction xyz)
        speed, dx, dy, dz = struct.unpack('ffff', data)
        return cls(speed, dx, dy, dz)
    
    def to_bytes(self):
        return struct.pack('ffff', 
                         self.speed,
                         self.direction_x,
                         self.direction_y,
                         self.direction_z)


def log_to_file(message):
    """Log messages to a file for debugging"""
    log_path = 'gym_server.log'
    with open(log_path, 'a') as f:
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        f.write(f'[{timestamp}] {message}\n')

def signal_handler(sig, frame):
    log_to_file("Received shutdown signal. Shutting down server gracefully...")
    if 'server_socket' in globals():
        server_socket.close()
    sys.exit(0)

def start_server():
    try:
        log_to_file("=====================================")
        log_to_file(f"Starting server. PID: {os.getpid()}")
        log_to_file(f"Working directory: {os.getcwd()}")
        log_to_file(f"Python version: {sys.version}")
        
        global server_socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            server_socket.bind(('127.0.0.1', 7777))
        except socket.error as e:
            log_to_file(f"Bind failed: {e}")
            sys.exit(1)
            
        server_socket.listen(1)
        log_to_file("Server started successfully on 127.0.0.1:7777")
        
        # Set up signal handler for graceful shutdown
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        while True:
            try:
                log_to_file("Waiting for connection...")
                client_socket, address = server_socket.accept()
                log_to_file(f"Connection from {address}")
                
                while True:
                    try:
                        # Receive exactly the size of our struct
                        data = client_socket.recv(16)  # 4 floats * 4 bytes each
                        if not data:
                            break
                            
                        velocity_data = VelocityData.from_bytes(data)
                        process_velocity(velocity_data)
                        client_socket.send(b'')  # Send empty acknowledgment
                            
                    except socket.error as e:
                        log_to_file(f"Socket error: {e}")
                        break
                    except KeyboardInterrupt:
                        raise
                    except Exception as e:
                        log_to_file(f"Error processing data: {e}")
                        log_to_file(traceback.format_exc())
                        continue
                        
                log_to_file("Client disconnected")
                client_socket.close()
                
            except socket.error as e:
                log_to_file(f"Accept failed: {e}")
                continue
    except KeyboardInterrupt:
        pass

    except Exception as e:
        log_to_file(f"Server error: {e}")
        log_to_file(traceback.format_exc())
    finally:
        server_socket.close()

def process_velocity(velocity_data: VelocityData):
    log_to_file(f"Speed: {velocity_data.speed:.2f} units/s")
    log_to_file(f"Direction: X={velocity_data.direction_x:.2f}, "
                f"Y={velocity_data.direction_y:.2f}, "
                f"Z={velocity_data.direction_z:.2f}")

if __name__ == "__main__":
    try:
        start_server()
    except Exception as e:
        log_to_file(f"Fatal error: {e}")
        log_to_file(traceback.format_exc())
        sys.exit(1)