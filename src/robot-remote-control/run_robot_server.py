#!/usr/bin/env python3

"""
Robot Remote Control Server Launcher with Auto-Ngrok
"""

import os
import sys
import subprocess
import threading
import time
import signal
import argparse

# Custom ngrok URL
NGROK_URL = "adapting-star-porpoise.ngrok-free.app"
NGROK_BINARY = "/usr/local/bin/ngrok"

def start_ngrok():
    """Start ngrok tunnel with custom URL"""
    print("🌐 Starting ngrok tunnel...")
    print(f"🎯 Public URL: https://{NGROK_URL}")
    
    try:
        # Check if ngrok is configured
        config_file = "/home/admin/.config/ngrok/ngrok.yml"
        if not os.path.exists(config_file):
            print("⚠️  Ngrok not configured. Run 'ngrok config add-authtoken YOUR_TOKEN' to enable remote access.")
            print("🌐 Server will be available locally only at http://localhost:8000")
            return None
        
        # Start ngrok as subprocess
        ngrok_cmd = [NGROK_BINARY, "http", f"--domain={NGROK_URL}", "8000"]
        process = subprocess.Popen(
            ngrok_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        # Give ngrok a moment to start
        time.sleep(3)
        
        if process.poll() is None:
            print("✅ Ngrok tunnel started successfully!")
            return process
        else:
            stdout, stderr = process.communicate()
            print(f"❌ Ngrok failed to start: {stderr.decode()}")
            print("🌐 Server will be available locally only at http://localhost:8000")
            return None
            
    except Exception as e:
        print(f"❌ Error starting ngrok: {e}")
        print("🌐 Server will be available locally only at http://localhost:8000")
        return None

def start_server(port=8000):
    """Start the FastAPI server"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    print("🚀 Starting Robot Remote Control Server...")
    print(f"🌐 Local URL: http://localhost:{port}")
    print(f"🌍 Public URL: https://{NGROK_URL}")
    print("🛑 Stop with Ctrl+C")
    print("=" * 50)
    
    try:
        import uvicorn
        uvicorn.run("main:app", host="0.0.0.0", port=port, reload=False)
    except ImportError:
        print("Installing uvicorn...")
        subprocess.run([sys.executable, '-m', 'pip', 'install', 'uvicorn[standard]'], check=True)
        import uvicorn
        uvicorn.run("main:app", host="0.0.0.0", port=port, reload=False)

def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print("\n🛑 Shutting down server and ngrok...")
    sys.exit(0)

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Robot Remote Control Server")
    parser.add_argument("--port", type=int, default=8000, help="Port to run the server on (default: 8000)")
    args = parser.parse_args()
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    print("🤖 Robot Remote Control Server")
    print("=" * 50)
    
    # Start ngrok in background thread
    ngrok_process = None
    if os.path.exists(NGROK_BINARY):
        ngrok_thread = threading.Thread(target=lambda: start_ngrok())
        ngrok_thread.daemon = True
        ngrok_thread.start()
        
        # Give ngrok time to start
        time.sleep(2)
    else:
        print("⚠️  Ngrok not found. Server will only be available locally.")
    
    # Start the web server (this blocks)
    try:
        start_server(args.port)
    except KeyboardInterrupt:
        print("\n🛑 Server stopped")

if __name__ == "__main__":
    main()
