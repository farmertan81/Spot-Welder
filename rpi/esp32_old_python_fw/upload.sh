#!/bin/bash
# Stop server if running
echo "🛑 Stopping server..."
pkill -f "python3 app.py" 2>/dev/null
sleep 1

# Kill anything using serial port
sudo fuser -k /dev/serial0 2>/dev/null
sleep 1

# Upload
echo "📤 Uploading to ESP32..."
ampy --port /dev/serial0 put main.py

# Wait for upload to complete
sleep 2

# Start server
echo "🚀 Starting server..."
cd ~/weldctl/server
python3 app.py &

echo "✅ Done!"
