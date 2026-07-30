// CYCLOTRONICS SIMULATOR - FTMS + CSC
// Simulator of a power and cadence sensor for the Cyclo_130_S3 Bridge
// Developed by Alessandro Miertschink
// Last update: 2026-03-19

#include <Arduino.h>
#include <NimBLEDevice.h>

// --- CONFIGURATIONS & UUIDS ---
#define FTMS_SERVICE_UUID          "1826"
#define FTMS_FEATURE_CHAR_UUID     "2ACC"
#define FTMS_DATA_CHAR_UUID        "2AD2"
#define FTMS_CONTROL_CHAR_UUID     "2AD9"

#define CSC_SERVICE_UUID           "1816"
#define CSC_MEAS_CHAR_UUID         "2A5B"

// --- BLE OBJECTS ---
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pBikeDataChar = nullptr;
NimBLECharacteristic* pControlChar = nullptr;
NimBLECharacteristic* pCSCChar = nullptr;

bool deviceConnected = false;

// --- TELEMETRY ---
float livePower = 100.0;    // Watts
float liveCadence = 85.0;   // RPM
float liveSpeed = 25.0;     // Km/h

// --- ERG MODE CONTROL ---
int   currentTargetPower = 0; 
float currentIncline = 0.0;   
bool  isERGMode = false;      

unsigned long lastNotifyMillis = 0;

// --- CSC (CADENCE) STATE ---
float totalRevs = 0.0;
uint16_t lastSentRevs = 0;
uint16_t lastSentTime = 0;

// --- SERVER CALLBACKS ---
class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        deviceConnected = true;
        Serial.println(">>> [BRIDGE CONNECTED]");
    }
    void onDisconnect(NimBLEServer* pServer) {
        deviceConnected = false;
        isERGMode = false;
        Serial.println(">>> [DISCONNECTED] Restarting advertising...");
        NimBLEDevice::startAdvertising();
    }
};

// --- CONTROL POINT CALLBACKS ---
class ControlPointCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) {
        std::string rxValue = pChar->getValue();
        if (rxValue.length() > 0) {
            uint8_t opCode = rxValue[0];
            uint8_t response[3] = {0x80, opCode, 0x01}; 
            
            switch (opCode) {
                case 0x00: // Request Control
                    Serial.println("Bridge: Control Requested.");
                    break;
                    
                case 0x05: // ERG Mode (Target Power)
                    if (rxValue.length() >= 3) {
                        currentTargetPower = rxValue[1] | (rxValue[2] << 8);
                        isERGMode = true;
                        Serial.printf(">>> ERG MODE: %d W\n", currentTargetPower);
                    }
                    break;

                case 0x03: // Set Inclination
                    if (rxValue.length() >= 3) {
                        isERGMode = false;
                        int16_t inclineRaw = (int16_t)(rxValue[1] | (rxValue[2] << 8));
                        currentIncline = inclineRaw / 10.0;
                        Serial.printf(">>> INC (0x03): %.1f %%\n", currentIncline);
                    }
                    break;

                case 0x11: // Set Indoor Bike Simulation Parameters
                    if (rxValue.length() >= 7) {
                        isERGMode = false;
                        int16_t gradeRaw = (int16_t)(rxValue[3] | (rxValue[4] << 8));
                        currentIncline = (float)gradeRaw * 0.01; 
                        Serial.printf(">>> INC (0x11): %.2f %%\n", currentIncline);
                    }
                    break;

                default:
                    Serial.printf("Bridge OpCode: 0x%02X | Size: %d\n", opCode, rxValue.length());
                    break;
            }
            pChar->setValue(response, 3);
            pChar->indicate();
        }
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("--- Starting Cyclo-SIM FTMS + CSC for Cyclo_130_S3 ---");
    
    uint8_t newMac[6] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x51};
    esp_base_mac_addr_set(newMac);
    
    NimBLEDevice::init("Cyclo_130_SIMv1");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // --- FTMS SERVICE ---
    NimBLEService* pFTMSService = pServer->createService(FTMS_SERVICE_UUID);

    uint8_t ftmsFeatures[8] = {0x0A, 0x40, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00};
    pFTMSService->createCharacteristic(FTMS_FEATURE_CHAR_UUID, NIMBLE_PROPERTY::READ)
                ->setValue(ftmsFeatures, 8);

    pBikeDataChar = pFTMSService->createCharacteristic(FTMS_DATA_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
    
    pControlChar = pFTMSService->createCharacteristic(
        FTMS_CONTROL_CHAR_UUID, 
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE
    );
    pControlChar->setCallbacks(new ControlPointCallbacks());

    pFTMSService->start();

    // --- CSC SERVICE ---
    NimBLEService* pCSCService = pServer->createService(CSC_SERVICE_UUID);
    pCSCChar = pCSCService->createCharacteristic(CSC_MEAS_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
    pCSCService->start();

    // --- ADVERTISING ---
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(FTMS_SERVICE_UUID);
    pAdvertising->addServiceUUID(CSC_SERVICE_UUID);
    pAdvertising->setAppearance(1152); 
    pAdvertising->start();

    Serial.println("Waiting for Cyclo_130_S3 Bridge connection...");
}

void loop() {
    if (deviceConnected) {
        unsigned long now = millis();
        unsigned long delta = now - lastNotifyMillis;

        if (delta >= 500) {
            lastNotifyMillis = now;

            if (isERGMode) {
                livePower = (float)currentTargetPower;
            } else {
                livePower = 123.0; // Base power for simulation mode
            }

            // --- FTMS Payload Generation ---
            // 0x44 Flag = Speed (Bit 2) + Instantaneous Power (Bit 6). Length = 6.
            uint8_t indoorBikeData[6] = {0x44, 0x00, 0x00, 0x00, 0x00, 0x00};
            uint16_t bleSpeed = (uint16_t)(liveSpeed * 100.0);
            int16_t blePwr = (int16_t)livePower;

            indoorBikeData[2] = bleSpeed & 0xFF;
            indoorBikeData[3] = (bleSpeed >> 8) & 0xFF;
            indoorBikeData[4] = blePwr & 0xFF;
            indoorBikeData[5] = (blePwr >> 8) & 0xFF;

            pBikeDataChar->setValue(indoorBikeData, 6);
            pBikeDataChar->notify();

            // --- CSC (Cadence) Anti-Jitter Payload Generation ---
            // Add the fraction of a revolution corresponding to the elapsed time
            totalRevs += (liveCadence / 60.0) * (delta / 1000.0);
            uint16_t currentIntegerRevs = (uint16_t)totalRevs;

            // If one or more full revolutions completed since the last send
            if (currentIntegerRevs > lastSentRevs) {
                lastSentRevs = currentIntegerRevs;

                // Retroactively compute the exact time (in ms) the full revolution occurred
                float fraction = totalRevs - currentIntegerRevs;
                float msPerRev = 60000.0 / liveCadence;
                uint32_t exactEventTimeMs = now - (uint32_t)(fraction * msPerRev);

                // Convert to Bluetooth's 1/1024-second standard
                lastSentTime = (uint16_t)((exactEventTimeMs * 1024) / 1000);
            }

            // The payload always sends the count and time of the LAST full revolution recorded
            uint8_t cscData[5] = {0x02, 0x00, 0x00, 0x00, 0x00};
            cscData[1] = lastSentRevs & 0xFF;
            cscData[2] = (lastSentRevs >> 8) & 0xFF;
            cscData[3] = lastSentTime & 0xFF;
            cscData[4] = (lastSentTime >> 8) & 0xFF;

            pCSCChar->setValue(cscData, 5);
            pCSCChar->notify();

            // --- LOG ---
            Serial.printf("[TX] PWR: %4.0f W | CAD: %3.0f RPM | ERG: %3s | INC: %4.1f %%\n", 
                          livePower, 
                          liveCadence, 
                          isERGMode ? "ON" : "OFF", 
                          currentIncline);
        }
    }
}