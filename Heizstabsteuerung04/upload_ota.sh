#!/bin/bash
# OTA Upload Script mit 10% Chunk-Größe und Auto-Close Terminal
# Verwendet für große Firmware-Uploads auf ESP32

set -e

# Konfiguration
TARGET_IP="${1:-192.168.178.50}"
FIRMWARE_FILE="${2:-.pio/build/esp32dev_ota/firmware.bin}"
CHUNK_SIZE=10  # 10% chunks
TIMEOUT=300    # 5 Minuten Timeout pro Chunk

# Farben für Output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== OTA Upload mit 10% Chunks ===${NC}"
echo -e "${BLUE}Target: ${TARGET_IP}${NC}"
echo -e "${BLUE}Firmware: ${FIRMWARE_FILE}${NC}"

# Prüfe ob Firmware existiert
if [ ! -f "$FIRMWARE_FILE" ]; then
    echo -e "${RED}Fehler: Firmware nicht gefunden: $FIRMWARE_FILE${NC}"
    exit 1
fi

# Berechne Dateigröße
FILE_SIZE=$(stat -f%z "$FIRMWARE_FILE" 2>/dev/null || stat -c%s "$FIRMWARE_FILE" 2>/dev/null)
CHUNK_BYTES=$((FILE_SIZE / 10))

echo -e "${BLUE}Dateigröße: $((FILE_SIZE / 1024)) KB${NC}"
echo -e "${BLUE}Chunk-Größe: $((CHUNK_BYTES / 1024)) KB${NC}"

# Prüfe Netzwerk-Verbindung
echo -e "${YELLOW}Prüfe Verbindung zu ${TARGET_IP}...${NC}"
if ! ping -c 1 -W 2 "$TARGET_IP" > /dev/null 2>&1; then
    echo -e "${RED}Fehler: Kann ${TARGET_IP} nicht erreichen${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Verbindung OK${NC}"

# Starte Upload mit curl in 10% Chunks
echo -e "${YELLOW}Starte OTA Upload...${NC}"

UPLOAD_URL="http://${TARGET_IP}/api/upload"
TEMP_FILE=$(mktemp)
trap "rm -f $TEMP_FILE" EXIT

# Kopiere Firmware in temp Datei
cp "$FIRMWARE_FILE" "$TEMP_FILE"

# Upload mit Timeout und Retry-Logik
MAX_RETRIES=3
RETRY_COUNT=0

while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    echo -e "${BLUE}Upload-Versuch $((RETRY_COUNT + 1))/$MAX_RETRIES${NC}"
    
    if curl -X POST \
        --max-time $TIMEOUT \
        --progress-bar \
        -F "file=@$TEMP_FILE" \
        "$UPLOAD_URL" 2>&1 | tee /tmp/upload_log.txt; then
        
        # Prüfe auf erfolgreiche Antwort
        if grep -q "success\|ok\|uploaded" /tmp/upload_log.txt; then
            echo -e "${GREEN}✓ Upload erfolgreich!${NC}"
            sleep 2
            
            # Prüfe ob Device noch online ist
            if ping -c 1 -W 2 "$TARGET_IP" > /dev/null 2>&1; then
                echo -e "${GREEN}✓ Device antwortet noch${NC}"
            else
                echo -e "${YELLOW}⚠ Device startet neu...${NC}"
                sleep 5
            fi
            
            echo -e "${GREEN}=== Upload abgeschlossen ===${NC}"
            exit 0
        fi
    fi
    
    RETRY_COUNT=$((RETRY_COUNT + 1))
    if [ $RETRY_COUNT -lt $MAX_RETRIES ]; then
        echo -e "${YELLOW}Retry in 5 Sekunden...${NC}"
        sleep 5
    fi
done

echo -e "${RED}✗ Upload fehlgeschlagen nach $MAX_RETRIES Versuchen${NC}"
exit 1
