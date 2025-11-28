#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <esp_now.h>

// FreeRTOS task parameters macros
#define STACK_SMALL 2048
#define PRIORITY_HIGH 1
#define CORE_1 1

// Max Incoming Message Length
#define CONTROL_MESSAGE_LENGTH 14
#define INTERVAL_MESSAGE_LENGTH 4

// MAC address of the Sender
//uint8_t senderAddress[] = { 0x38, 0x18, 0x2B, 0x8A, 0x2C, 0xC4 }; //Dante ESP32 USB C
//uint8_t senderAddress[] = {0x78, 0x42, 0x1C, 0x6A, 0x90, 0x5C}; //Mireles USB C
uint8_t senderAddressRightLeg[] = {0x7C, 0x9E, 0xBD, 0x66, 0xD0, 0x48}; //PCB micro-USB NO-IMU

uint8_t senderAddressLeftLeg[] = {0xB0, 0xA7, 0x32, 0xDB, 0x3B, 0x98}; //RAFA ESP32 


struct __attribute__((packed)) ControlPacket {
  //bool ledState;
  int motorID;
  char mode;
  int intensity;
  int minVal;
  int maxVal;
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t start;
  int16_t qw,qx,qy,qz;
  int16_t pos[3];            // posiciones motores
  int16_t vel[3];            // velocidades motores
  int16_t cur[3];            // corrientes motores
};

// Queues for inter-task communication
QueueHandle_t RightLegTelemetryQueue = NULL;
QueueHandle_t LeftLegTelemetryQueue = NULL;
QueueHandle_t IntervalQueue = NULL;

// Function prototypes
void espnow_tx_task(void *pvParameters);
void espnow_rx_task(void *pvParameters);
void onReceive(const uint8_t *mac_addr, const uint8_t *incoming_data, int len);
void onSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void parseMessage(String msg, ControlPacket* Control);

/**
 * @brief Initializes the ESP32 dongle as an ESP-NOW gateway.
 * 
 * Sets up serial communication at 500000 baud, initializes WiFi in station mode,
 * configures ESP-NOW protocol, registers send/receive callbacks, adds two peer
 * devices (right and left leg), creates FreeRTOS queues for telemetry and interval
 * data, and launches two tasks on Core 1 for TX/RX operations.
 * 
 * @return void
 * @note Blocks indefinitely if ESP-NOW initialization fails.
 */
void setup() {
    Serial.begin(500000);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        while (1);
    }

    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSent);

    // Serial.println("Receiver ready!");
    // Serial.print("Receiver MAC Address: ");
    // Serial.println(WiFi.macAddress());

    // Add right leg as peer 
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, senderAddressRightLeg, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer RIGHT ESP32");
        return;
    }
    // Add left leg as peer
    memset(&peerInfo, 0, sizeof(peerInfo));  // clear structure before reusing
    memcpy(peerInfo.peer_addr, senderAddressLeftLeg, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer LEFT ESP32");
        return;
    }

    RightLegTelemetryQueue = xQueueCreate(1, sizeof(TelemetryPacket));
    LeftLegTelemetryQueue  = xQueueCreate(1, sizeof(TelemetryPacket));
    IntervalQueue          = xQueueCreate(1, sizeof(uint16_t));
    
    xTaskCreatePinnedToCore(
        espnow_tx_task,
        "ESP-NOW_Tx_Task",
        STACK_SMALL,
        NULL,
        PRIORITY_HIGH,
        NULL,
        CORE_1
    );
    
    xTaskCreatePinnedToCore(
        espnow_rx_task,
        "ESP-NOW_Rx_Task",
        STACK_SMALL,
        NULL,
        PRIORITY_HIGH,
        NULL,
        CORE_1
    );

}

/**
 * @brief Main loop (unused in FreeRTOS-based design).
 * 
 * Immediately deletes the Arduino loop task since all functionality is handled
 * by dedicated FreeRTOS tasks created in setup(). This prevents the default
 * Arduino loop from consuming CPU cycles.
 * 
 * @return void
 */
void loop() {
    vTaskDelete(NULL);
}

/**
 * @brief FreeRTOS task for processing incoming serial commands and transmitting to peers.
 * 
 * Continuously monitors the Serial port for incoming messages. Supports two message types:
 * - Motor control commands (14 chars, format: [R|L]<motorID><mode><intensity><minVal><maxVal>)
 *   Example: "R1B050-050+250" - Right leg, motor 1, mode B, intensity 50, range -50 to +250
 * - Interval update commands (4 chars, format: F<interval>)
 *   Example: "F020" - Set telemetry output interval to 20ms
 * 
 * Parses valid commands, sends control packets via ESP-NOW to the appropriate peer
 * (right or left leg), and updates the interval queue for the RX task.
 * 
 * @param pvParameters Unused FreeRTOS task parameter (NULL)
 * @return void (infinite loop, never returns)
 * @note Runs every 10ms. Validates message format before processing.
 */
void espnow_tx_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    String msg = "";

    ControlPacket Control;
    uint16_t interval;

    

    for (;;) {
        if (Serial.available()) {
            msg = Serial.readStringUntil('\n');
            msg.trim();

            if (msg.length() == CONTROL_MESSAGE_LENGTH && (msg.startsWith("R") || msg.startsWith("L"))) {
                parseMessage(msg, &Control);
                esp_err_t result = (msg.startsWith("R")) ? 
                            esp_now_send(senderAddressRightLeg, (uint8_t *) &Control, sizeof(ControlPacket)) :
                            esp_now_send(senderAddressLeftLeg, (uint8_t *) &Control, sizeof(ControlPacket));
                if (result != ESP_OK) {
                    Serial.println("ESP-NOW send failed");
                }

            } else if (msg.length() == INTERVAL_MESSAGE_LENGTH && msg.startsWith("F")){
                interval = msg.substring(1,4).toInt();
                xQueueOverwrite(IntervalQueue, &interval);
                Serial.printf("Period set to %d ms\n",interval);

            } else if (msg.length() > 0) {
                Serial.println("Mensaje invalido.");

            }       
        }

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); 
    }
}

/**
 * @brief FreeRTOS task for receiving telemetry and outputting aggregated data via Serial.
 * 
 * Periodically checks both telemetry queues (right and left leg), merges the data into
 * unified arrays (6 motors total: indices 0-2 for right leg, 3-5 for left leg), and
 * outputs a CSV-formatted line containing:
 * - Quaternion orientation (qw, qx, qy, qz) from right leg IMU
 * - Position, velocity, and current for all 6 motors
 * 
 * Output format: qw,qx,qy,qz,pos0,vel0,cur0,pos1,vel1,cur1,...,pos5,vel5,cur5
 * 
 * The output interval is dynamically adjustable via the IntervalQueue (default 20ms).
 * 
 * @param pvParameters Unused FreeRTOS task parameter (NULL)
 * @return void (infinite loop, never returns)
 * @note Uses non-blocking queue reads to avoid delays. Initializes quaternion to (0,0,0,0).
 */
void espnow_rx_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TelemetryPacket telemetryData;
    
    int16_t pos[6] = {0,0,0,0,0,0};
    int16_t vel[6] = {0,0,0,0,0,0};
    int16_t cur[6] = {0,0,0,0,0,0};
    int16_t qw = 0, qx = 0, qy = 0, qz = 0;

    uint16_t interval = 100; // Interval sets task periodicity to 20 ms by default.

    for (;;) {
        if (xQueueReceive(RightLegTelemetryQueue, &telemetryData, 0) == pdTRUE) {
            qw = telemetryData.qw;
            qx = telemetryData.qx;
            qy = telemetryData.qy;
            qz = telemetryData.qz;
            for (int i = 0; i < 3; i++) {
                pos[i] = telemetryData.pos[i];
                vel[i] = telemetryData.vel[i];
                cur[i] = telemetryData.cur[i];
            }
        }

        if (xQueueReceive(LeftLegTelemetryQueue, &telemetryData, 0) == pdTRUE) {
            for (int i = 3; i < 6; i++) {
                pos[i] = telemetryData.pos[i-3];
                vel[i] = telemetryData.vel[i-3];
                cur[i] = telemetryData.cur[i-3];
            }
        }

        // Update interval if a new value was provided.
        xQueueReceive(IntervalQueue, &interval, 0);

        Serial.printf("%d,%d,%d,%d,", qw, qx, qy, qz);
        for (int i = 0; i < 6; i++) {
            Serial.printf("%d,%d,%d", pos[i], vel[i], cur[i]);
            if (i < 5) Serial.print(",");
        }
        Serial.println();

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(interval)); 
    }
}

/**
 * @brief ESP-NOW receive callback invoked when telemetry packets arrive from peers.
 * 
 * Validates incoming packet size, deserializes the TelemetryPacket structure, and
 * routes it to the appropriate queue based on the start marker:
 * - 0xAA: Right leg telemetry → RightLegTelemetryQueue
 * - 0xBB: Left leg telemetry → LeftLegTelemetryQueue
 * 
 * Uses xQueueOverwrite to ensure the most recent telemetry is always available,
 * discarding older unread packets if necessary.
 * 
 * @param mac_addr Pointer to the 6-byte MAC address of the sender
 * @param incoming_data Pointer to the received data buffer
 * @param len Length of the received data in bytes
 * @return void
 * @note Called from ESP-NOW ISR context; keep processing minimal and fast.
 */
void onReceive(const uint8_t *mac_addr, const uint8_t *incoming_data, int len) {
    if (len != sizeof(TelemetryPacket)) { return; } // Invalid packet size
    TelemetryPacket telemetryData;
    memcpy(&telemetryData, incoming_data, sizeof(telemetryData));

    if (telemetryData.start == 0xAA) {
        xQueueOverwrite(RightLegTelemetryQueue, &telemetryData);
    } else if (telemetryData.start == 0xBB) {
        xQueueOverwrite(LeftLegTelemetryQueue, &telemetryData);
    }
}

/**
 * @brief Parses a 14-character motor control command string into a ControlPacket structure.
 * 
 * Extracts fixed-position fields from the message format:
 * - Position 0: Leg identifier ('R' or 'L')
 * - Position 1: Motor ID (0-9)
 * - Position 2: Control mode (e.g., 'B' for specific control type)
 * - Positions 3-5: Intensity (000-999)
 * - Positions 6-9: Minimum value (-999 to +999)
 * - Positions 10-13: Maximum value (-999 to +999)
 * 
 * Example: "R1B050-050+250" → Right leg, motor 1, mode 'B', intensity 50, min -50, max +250
 * 
 * @param msg The 14-character command string to parse
 * @param Control Pointer to ControlPacket structure to populate with parsed values
 * @return void
 * @note Does not validate the leg identifier; caller must ensure proper format.
 */
void parseMessage(String msg, ControlPacket* Control) {

  // Ejemplo mensaje: L2D500-050+250
  Control->motorID   = msg.substring(1, 2).toInt();     // Char 1
  Control->mode      = msg.charAt(2);                   // Char 2
  Control->intensity = msg.substring(3, 6).toInt();     // Char 3–5
  Control->minVal    = msg.substring(6, 10).toInt();    // Char 6–9
  Control->maxVal    = msg.substring(10, 14).toInt();   // Char 10–13
}

/**
 * @brief ESP-NOW send callback invoked after each transmission attempt.
 * 
 * Called automatically by the ESP-NOW stack after esp_now_send() completes.
 * Currently a stub implementation with optional debug logging (commented out).
 * Can be used for tracking transmission success/failure rates or implementing
 * retry logic for critical commands.
 * 
 * @param mac_addr Pointer to the 6-byte MAC address of the intended recipient
 * @param status Transmission status (ESP_NOW_SEND_SUCCESS or ESP_NOW_SEND_FAIL)
 * @return void
 * @note Called from ESP-NOW ISR context; avoid blocking operations.
 */
void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
        // Optional: uncomment for verbose send status logging.
        // Serial.printf("Send status to %02X:%02X:%02X:%02X:%02X:%02X: %s\n",
        //               mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
        //               status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

// F250
// R1B050-050+250