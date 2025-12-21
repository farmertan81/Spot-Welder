#!/bin/bash
# Upload main.py via FTP and trigger soft reset

echo "📤 Uploading main.py via FTP..."

curl -T main.py ftp://192.168.68.65/ --user esp32:welder123

if [ $? -eq 0 ]; then
    echo "✅ Upload complete!"
    echo "🔄 Triggering ESP32 soft reset..."
    
    # Send RESET command via serial (server must be stopped)
    echo "RESET" > /dev/serial0 2>/dev/null || echo "⚠️ Could not send reset (server running?)"
    
    echo "✅ Done! ESP32 will reload on next connection."
else
    echo "❌ Upload failed!"
fi
