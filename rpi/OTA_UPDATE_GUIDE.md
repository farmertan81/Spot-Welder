# ESP32 OTA Update Guide

## Quick Upload
cd ~/weldctl/esp32
./ftp_upload.sh

## ESP32 FTP Details
- IP: 192.168.68.65:21
- User: esp32 / Pass: welder123

## Manual Upload
curl -T main.py ftp://192.168.68.65/ --user esp32:welder123
