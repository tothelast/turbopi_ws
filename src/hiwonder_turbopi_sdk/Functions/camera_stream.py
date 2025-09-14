#!/usr/bin/env python3
# coding=utf8
"""
Camera Web Streaming Script for TurboPi
Provides live camera feed accessible via web browser at http://192.168.0.11:8080
"""

import sys
import os
import threading
import time

# Add the parent directory to the path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import Camera
    import MjpgServer
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Make sure you're running this from the correct directory")
    sys.exit(1)

class CameraStreamer:
    def __init__(self, port=8080, fps=30):
        """
        Initialize camera streamer
        :param port: Port for web server (default: 8080)
        :param fps: Target frames per second (default: 30)
        """
        self.port = port
        self.fps = fps
        self.frame_delay = 1.0 / fps
        self.camera = None
        self.streaming = False
        
    def start_camera(self):
        """Initialize and start the camera"""
        try:
            self.camera = Camera.Camera()
            self.camera.camera_open()
            print(f"Camera initialized successfully")

            # Wait for camera to warm up and get first frame
            print("Waiting for camera to warm up...")
            for i in range(50):  # Wait up to 5 seconds
                if self.camera.frame is not None:
                    print(f"Camera ready! Got first frame after {i/10:.1f} seconds")
                    return True
                time.sleep(0.1)

            print("Warning: Camera initialized but no frames received")
            return True  # Continue anyway, might work later

        except Exception as e:
            print(f"Failed to initialize camera: {e}")
            return False
    
    def stop_camera(self):
        """Stop and cleanup camera"""
        if self.camera:
            try:
                self.camera.camera_close()
                print("Camera closed")
            except Exception as e:
                print(f"Error closing camera: {e}")
    
    def update_stream(self):
        """Update the MJPEG stream with camera frames"""
        print(f"Starting camera stream update thread (target: {self.fps} FPS)")

        frame_count = 0
        warning_count = 0

        while self.streaming:
            try:
                if self.camera and self.camera.frame is not None:
                    # Update the MJPEG server with the latest frame
                    MjpgServer.img_show = self.camera.frame
                    frame_count += 1

                    # Print status every 5 seconds
                    if frame_count % (self.fps * 5) == 0:
                        print(f"Streaming: {frame_count} frames processed")

                else:
                    warning_count += 1
                    # Only print warnings for first 10 times, then every 100 times
                    if warning_count <= 10 or warning_count % 100 == 0:
                        print(f"Warning: No camera frame available (count: {warning_count})")

                time.sleep(self.frame_delay)

            except Exception as e:
                print(f"Error in stream update: {e}")
                time.sleep(0.1)  # Brief pause on error
    
    def start_streaming(self):
        """Start the camera streaming service"""
        print("Starting TurboPi Camera Web Streaming...")
        
        # Initialize camera
        if not self.start_camera():
            return False
        
        # Start streaming flag
        self.streaming = True
        
        # Start the frame update thread
        update_thread = threading.Thread(target=self.update_stream, daemon=True)
        update_thread.start()
        
        # Start the MJPEG web server
        try:
            print(f"Camera stream will be available at:")
            print(f"  http://localhost:{self.port}")
            print(f"  http://192.168.0.11:{self.port}")
            print(f"  http://<your-pi-ip>:{self.port}")
            print("\nPress Ctrl+C to stop streaming")
            
            # This will block and serve the web interface
            MjpgServer.startMjpgServer()
            
        except KeyboardInterrupt:
            print("\nShutting down camera stream...")
        except Exception as e:
            print(f"Error starting MJPEG server: {e}")
        finally:
            self.stop_streaming()
    
    def stop_streaming(self):
        """Stop the streaming service"""
        self.streaming = False
        self.stop_camera()
        print("Camera streaming stopped")

def main():
    """Main function to run the camera streamer"""
    print("TurboPi Camera Web Streaming")
    print("=" * 40)
    
    # Create and start the camera streamer
    streamer = CameraStreamer(port=8080, fps=30)
    
    try:
        streamer.start_streaming()
    except Exception as e:
        print(f"Fatal error: {e}")
        return 1
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
