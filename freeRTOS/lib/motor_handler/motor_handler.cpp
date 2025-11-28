#include <Arduino.h>
#include "motor_handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "common.h"
#include "driver/twai.h"
#include "esp_log.h"

extern HardwareSerial Serial;
extern QueueHandle_t can_tx_queue;
extern QueueHandle_t hip_telemetry_queue;
extern QueueHandle_t knee_telemetry_queue;
extern QueueHandle_t ankle_telemetry_queue;

TaskHandle_t motor_task_hip_handle = NULL;
TaskHandle_t motor_task_knee_handle = NULL;
TaskHandle_t motor_task_ankle_handle = NULL;

QueueHandle_t motor_queue_hip = NULL;
QueueHandle_t motor_queue_knee = NULL;
QueueHandle_t motor_queue_ankle = NULL;

QueueHandle_t motor_control_queue_hip = NULL;
QueueHandle_t motor_control_queue_knee = NULL;
QueueHandle_t motor_control_queue_ankle = NULL;

// Exponential gain profile for smooth torque ramp-up (11 steps from 0% to 100%)
const float torque_gain_profile[10] = {
    0.15f, 0.19f, 0.24f, 0.30f, 0.38f, 0.48f, 0.60f, 0.76f, 0.95f, 1.00f
};

// Torque ramping states for smooth acceleration/deceleration
typedef enum {
    RAMP_IDLE = 0,          // No ramping active, direct torque passthrough
    RAMP_ACCELERATING = 1,  // Ramping torque up from 15% to 100%
    RAMP_DECELERATING = 2   // Ramping torque down from 100% to 15%
} ramp_state_t;

typedef enum {
    NO_MOTION = 0,          // Motor speed lays between deadband thresholds
    FORWARD_MOTION = 1,     // Motor speed is above the positive deadband threshold
    REVERSE_MOTION = -1     // Motor speed is below the negative deadband threshold
} motion_direction_t;

// ---------- Forward Declarations ----------
static void motor_task_hip(void * parameter);
static void motor_task_knee(void * parameter);
static void motor_task_ankle(void * parameter);
static void apply_torque_ramping(int16_t* torque_output, motion_direction_t motor_moving, 
                                ramp_state_t* ramp_state, uint8_t* gain_index, 
                                motion_direction_t* prev_motor_moving, int8_t* prev_torque_dir, 
                                int16_t intensity, char* mode);
static void read_motor_status(motor_status_t* motor);
static void parse_motor_status(twai_message_t* rx_msg, motor_status_t* motor);
static void release_motor_brake(motor_status_t* motor);
static void shutdown_motor(motor_status_t* motor);
static void stop_motor(motor_status_t* motor);
static void torque_control(motor_status_t* motor, int16_t torque_value);
static void parse_motor_control_command(control_packet* packet, motor_control_t* motor_cmd);
static void parse_espnow_status(espnow_motor_status_t* espnow_status, motor_status_t* motor);

// ---------- Initialization ----------
uint8_t init_motor() {

    // Create motor CAN queues - single element, always get latest data
    motor_queue_ankle = xQueueCreate(1, sizeof(twai_message_t));
    motor_queue_knee = xQueueCreate(1, sizeof(twai_message_t));
    motor_queue_hip = xQueueCreate(1, sizeof(twai_message_t));

    // Create motor control queues
    motor_control_queue_ankle = xQueueCreate(1, sizeof(control_packet));
    motor_control_queue_knee = xQueueCreate(1, sizeof(control_packet));
    motor_control_queue_hip = xQueueCreate(1, sizeof(control_packet));

    BaseType_t result_1 = xTaskCreatePinnedToCore(
        motor_task_hip, 
        "MotorHipTask", 
        STACK_MEDIUM,   
        NULL, 
        PRIORITY_HIGH, 
        &motor_task_hip_handle, 
        CORE_1
    );

    BaseType_t result_2 = xTaskCreatePinnedToCore(
        motor_task_knee, 
        "MotorKneeTask", 
        STACK_MEDIUM, 
        NULL, 
        PRIORITY_HIGH, 
        &motor_task_knee_handle, 
        CORE_1
    );

    BaseType_t result_3 = xTaskCreatePinnedToCore(
        motor_task_ankle, 
        "MotorAnkleTask", 
        STACK_MEDIUM, 
        NULL, 
        PRIORITY_HIGH, 
        &motor_task_ankle_handle, 
        CORE_1
    );

    // Check if all tasks were created successfully
    if (result_1 == pdPASS && result_2 == pdPASS && result_3 == pdPASS &&
        motor_queue_ankle != NULL && motor_queue_knee != NULL && motor_queue_hip != NULL) {
        return 1; // Initialization successful
    } else {
        return 0; // Initialization failed
    }
}


// ---------- Motor Tasks ----------

/**
 * @brief Task function to handle hip motor operations.
 * @param parameter Pointer to task parameters (not used).
 * This task runs periodically at a fixed interval of 5ms. It checks for a new CAN
 * message in the motor queue and processes it to update the motor status. This task
 * executes the control logic and sends the appropriate commands to the hip motor through
 * the can_tx_queue.
 */
static void motor_task_hip(void * parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    twai_message_t can_msg;

    motor_status_t motor = {HIP_MOTOR_ID, 0, 0, 0};
    espnow_motor_status_t hip_status = {0, 0, 0};
    
    control_packet control;
    motor_control_t motor_cmd = {'A', 0, 0, 0};
    int16_t torque_output = 0;
    
    // Torque ramping state variables
    ramp_state_t ramp_state = RAMP_IDLE;
    uint8_t gain_index = 9;
    motion_direction_t motor_direction = NO_MOTION;
    motion_direction_t prev_motor_direction = NO_MOTION;
    int8_t prev_torque_dir = 0;

    // Verify motor connection
    read_motor_status(&motor);
    if (xQueueReceive(motor_queue_hip, &can_msg, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Serial.printf("Hip Motor ID %d not responding! Deleting task.\n", motor.id);
        vTaskDelete(NULL);
    } else {
        Serial.printf("Hip Motor ID %d connected successfully.\n", motor.id);
        parse_motor_status(&can_msg, &motor);
    }

    // ----- Control Loop -----
    for(;;) {
        // Check for control command
        if (xQueueReceive(motor_control_queue_hip, &control, pdMS_TO_TICKS(0)) == pdTRUE) {
            parse_motor_control_command(&control, &motor_cmd);
        }
        
        // Calculate torque output based on mode
        switch (motor_cmd.mode) {
            case 'A':
                motor_direction = NO_MOTION;
                torque_output = 0;
                break;

            case 'B':
                torque_output = (motor.speed > 20) ? 10 : (motor.speed < -20) ? -10 : 0;
                break;

            case 'C':  // Assist motion in both directions with hysteresis
                if (motor.speed < 30 && motor.speed > -30) {
                    motor_direction = NO_MOTION;
                } 
                else if (motor.speed > 80 || (motor_direction == FORWARD_MOTION && motor.speed > 30)) {
                    motor_direction = FORWARD_MOTION;
                } 
                else if (motor.speed < -80 || (motor_direction == REVERSE_MOTION && motor.speed < -30)) {
                    motor_direction = REVERSE_MOTION;
                }
                break;

            case 'D':  // Resist motion in both directions with hysteresis, motor directions are swapped compared to mode 'C'
                if (motor.speed < 10 && motor.speed > -10) {
                    motor_direction = NO_MOTION;
                } else if (motor.speed > 80 ||( motor_direction == REVERSE_MOTION && motor.speed > 10)) {
                    motor_direction = REVERSE_MOTION;
                } else if (motor.speed < -80 || (motor_direction == FORWARD_MOTION && motor.speed < -10)) {
                    motor_direction = FORWARD_MOTION;
                }
                break;

            case 'E':  // Assist forward motion only (speed > 0)
                if (motor.speed < 30 && motor_direction == FORWARD_MOTION) {
                    motor_direction = NO_MOTION;
                } else if (motor.speed > 80 || motor_direction == FORWARD_MOTION) {
                    motor_direction = FORWARD_MOTION;  
                } else {
                    motor_direction = NO_MOTION;
                }
                break;

            case 'F':  // Assist backward motion only (speed < 0)
                if (motor.speed > -30 && motor_direction == REVERSE_MOTION) {
                    motor_direction = NO_MOTION;
                } else if (motor.speed < -80 || motor_direction == REVERSE_MOTION) {
                    motor_direction = REVERSE_MOTION;
                } else {
                    motor_direction = NO_MOTION;
                }
                break;

            default:
                motor_direction = NO_MOTION;
                break;
        }

        // Apply smooth torque ramping for acceleration/deceleration
        apply_torque_ramping(&torque_output, motor_direction, &ramp_state, &gain_index, 
                           &prev_motor_direction, &prev_torque_dir, motor_cmd.intensity,
                           &motor_cmd.mode);

        // Send control command (response includes motor status)
        torque_control(&motor, torque_output);

        // Wait for CAN response (blocks up to 3ms for fresh data)
        if (xQueueReceive(motor_queue_hip, &can_msg, pdMS_TO_TICKS(3)) == pdTRUE) {
            parse_motor_status(&can_msg, &motor);
            parse_espnow_status(&hip_status, &motor);
            xQueueOverwrite(hip_telemetry_queue, &hip_status);
        }

        // Fixed 5ms cycle
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

/**
 * @brief Task function to handle knee motor operations.
 * @param parameter Pointer to task parameters (not used).
 * This task runs periodically at a fixed interval of 5ms. It checks for a new CAN
 * message in the motor queue and processes it to update the motor status. This task
 * executes the control logic and sends the appropriate commands to the knee motor through
 * the can_tx_queue.
 */
static void motor_task_knee( void * parameter ) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    twai_message_t can_msg;

    motor_status_t motor = {KNEE_MOTOR_ID, 0, 0, 0};
    espnow_motor_status_t knee_status = {0, 0, 0};
    
    control_packet control;
    motor_control_t motor_cmd = {'A', 0, 0, 0};
    int16_t torque_output = 0;
    
    // Torque ramping state variables
    ramp_state_t ramp_state = RAMP_IDLE;
    uint8_t gain_index = 9;
    motion_direction_t motor_direction = NO_MOTION;
    motion_direction_t prev_motor_direction = NO_MOTION;
    int8_t prev_torque_dir = 0;

    //Proportional controller for resistive mode 'D'
    int16_t kp = 1;
    int16_t max_speed_value = 250;
    int16_t past_values[3] = {0, 0, 0}; // Buffer for derivative calculation

    // Verify motor connection
    read_motor_status(&motor);
    if (xQueueReceive(motor_queue_knee, &can_msg, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Serial.printf("Knee Motor ID %d not responding! Deleting task.\n", motor.id);
        vTaskDelete(NULL);
    } else {
        Serial.printf("Knee Motor ID %d connected successfully.\n", motor.id);
        parse_motor_status(&can_msg, &motor);
    }

    release_motor_brake(&motor);
    vTaskDelay(pdMS_TO_TICKS(50)); // Allow brake release to complete

    // ----- Control Loop -----
    for(;;) {
        // Check for control command
        if (xQueueReceive(motor_control_queue_knee, &control, pdMS_TO_TICKS(0)) == pdTRUE) {
            parse_motor_control_command(&control, &motor_cmd);
        }
        
        // Calculate torque output based on mode
        switch (motor_cmd.mode) {
            case 'A':
                motor_direction = NO_MOTION;
                torque_output = 0;
                break;

            case 'B':
                torque_output = (motor.speed > 20) ? 10 : (motor.speed < -20) ? -10 : 0;
                break;

            case 'C':  // Assist motion in both directions with hysteresis
                if (motor.speed < 30 && motor.speed > -30) {
                    motor_direction = NO_MOTION;
                } 
                else if (motor.speed > 80 || (motor_direction == FORWARD_MOTION && motor.speed > 30)) {
                    motor_direction = FORWARD_MOTION;
                } 
                else if (motor.speed < -80 || (motor_direction == REVERSE_MOTION && motor.speed < -30)) {
                    motor_direction = REVERSE_MOTION;
                }
                break;

            case 'D': // Resist forward motion with proportional gain based on speed
                if (motor.speed <= 20) {
                    motor_direction = NO_MOTION;
                    torque_output = 0;
                }
                else if (motor.speed > 50 || (motor_direction == REVERSE_MOTION && motor.speed > 20)) {
                    motor_direction = REVERSE_MOTION;
                    kp = map(abs(motor.speed), 0, MAX_SPEED, 0, 100);
                    torque_output = (int8_t(motor_direction) * motor_cmd.intensity * kp) / 100;
                }
                else {
                    motor_direction = NO_MOTION;
                    torque_output = 0;
                }
                break;


            case 'E':  // Assist forward motion only (speed > 0)
                if (motor.speed < 30 && motor_direction == FORWARD_MOTION) {
                    motor_direction = NO_MOTION;
                } else if (motor.speed > 80 || motor_direction == FORWARD_MOTION) {
                    motor_direction = FORWARD_MOTION;  
                } else {
                    motor_direction = NO_MOTION;
                }
                break;

            case 'F':  // Assist backward motion only (speed < 0)
                if (motor.speed > -30 && motor_direction == REVERSE_MOTION) {
                    motor_direction = NO_MOTION;
                } else if (motor.speed < -80 || motor_direction == REVERSE_MOTION) {
                    motor_direction = REVERSE_MOTION;
                } else {
                    motor_direction = NO_MOTION;
                }
                break;

            case 'G': // Resist forward motion with proportional gain based on speed
                if (motor.speed >= -20) {
                    motor_direction = NO_MOTION;
                    torque_output = 0;
                }
                else if (motor.speed < -80 || (motor_direction == FORWARD_MOTION && motor.speed < -20)) {
                    motor_direction = FORWARD_MOTION;
                    kp = constrain(map(abs(motor.speed), 80, 300, 0, 100), 0, 100);
                    torque_output = (int8_t(motor_direction) * motor_cmd.intensity * kp) / 100;
                }
                else {
                    motor_direction = NO_MOTION;
                    torque_output = 0;
                }
                break;  

            default:
                motor_direction = NO_MOTION;
                break;
        }

        // Apply smooth torque ramping for acceleration/deceleration only for modes other than 'B', 'D' and 'A'
        if (motor_cmd.mode != 'B' && motor_cmd.mode != 'A' && motor_cmd.mode != 'D' && motor_cmd.mode != 'G') {
            apply_torque_ramping(&torque_output, motor_direction, &ramp_state, &gain_index, 
                               &prev_motor_direction, &prev_torque_dir, motor_cmd.intensity, 
                               &motor_cmd.mode);
        }

        // Send control command (response includes motor status)
        torque_control(&motor, torque_output);

        Serial.printf("%d,%d,%d,%d,%d\n", 
            motor.speed,           // Motor velocity
            motor_direction,
            ramp_state,
            gain_index,
            torque_output          // Final torque after ramping
        );


        // Wait for CAN response (blocks up to 3ms for fresh data)
        if (xQueueReceive(motor_queue_knee, &can_msg, pdMS_TO_TICKS(3)) == pdTRUE) {
            parse_motor_status(&can_msg, &motor);
            parse_espnow_status(&knee_status, &motor);
            xQueueOverwrite(knee_telemetry_queue, &knee_status);
        }

        // Fixed 5ms cycle
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
} 

/**
 * @brief Task function to handle ankle motor operations.
 * @param parameter Pointer to task parameters (not used).
 * This task runs periodically at a fixed interval of 5ms. It checks for a new CAN
 * message in the motor queue and processes it to update the motor status. This task
 * executes the control logic and sends the appropriate commands to the ankle motor through
 * the can_tx_queue.
 */
static void motor_task_ankle( void * parameter ) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    twai_message_t can_msg;

    motor_status_t motor = {ANKLE_MOTOR_ID, 0, 0, 0};
    espnow_motor_status_t ankle_status = {0, 0, 0};
    
    control_packet control;
    motor_control_t motor_cmd = {'A', 0, 0, 0};
    int16_t torque_output = 0;
    
    // Torque ramping state variables
    ramp_state_t ramp_state = RAMP_IDLE;
    uint8_t gain_index = 9;
    motion_direction_t motor_direction = NO_MOTION;
    motion_direction_t prev_motor_direction = NO_MOTION;
    int8_t prev_torque_dir = 0;

    // Verify motor connection
    read_motor_status(&motor);
    if (xQueueReceive(motor_queue_ankle, &can_msg, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Serial.printf("Ankle Motor ID %d not responding! Deleting task.\n", motor.id);
        vTaskDelete(NULL);
    } else {
        Serial.printf("Ankle Motor ID %d connected successfully.\n", motor.id);
        parse_motor_status(&can_msg, &motor);
    }

    // ----- Control Loop -----
    for(;;) {
        // Check for control command
        if (xQueueReceive(motor_control_queue_ankle, &control, pdMS_TO_TICKS(0)) == pdTRUE) {
            parse_motor_control_command(&control, &motor_cmd);
        }
        
        // Calculate torque output based on mode
        switch (motor_cmd.mode) {
            case 'A':
                motor_direction = NO_MOTION;
                torque_output = 0;
                break;

            case 'B':
                torque_output = (motor.speed > 20) ? 10 : (motor.speed < -20) ? -10 : 0;
                break;

            case 'C':  // Assist motion in both directions with hysteresis
                if (motor.speed < 30 && motor.speed > -30) {
                    motor_direction = NO_MOTION;
                } 
                else if (motor.speed > 80 || (motor_direction == FORWARD_MOTION && motor.speed > 30)) {
                    motor_direction = FORWARD_MOTION;
                } 
                else if (motor.speed < -80 || (motor_direction == REVERSE_MOTION && motor.speed < -30)) {
                    motor_direction = REVERSE_MOTION;
                }
                break;

            case 'D':  // Resist motion in both directions with hysteresis, motor directions are swapped compared to mode 'C'
                if (motor.speed < 10 && motor.speed > -10) {
                    motor_direction = NO_MOTION;
                } else if (motor.speed > 80 ||( motor_direction == REVERSE_MOTION && motor.speed > 10)) {
                    motor_direction = REVERSE_MOTION;
                } else if (motor.speed < -80 || (motor_direction == FORWARD_MOTION && motor.speed < -10)) {
                    motor_direction = FORWARD_MOTION;
                }
                break;

            case 'E':  // Assist forward motion only (speed > 0)
                if (motor.speed < 30 && motor_direction == FORWARD_MOTION) {
                    motor_direction = NO_MOTION;
                } else if (motor.speed > 80 || motor_direction == FORWARD_MOTION) {
                    motor_direction = FORWARD_MOTION;  
                } else {
                    motor_direction = NO_MOTION;
                }
                break;

            case 'F':  // Assist backward motion only (speed < 0)
                if (motor.speed > -30 && motor_direction == REVERSE_MOTION) {
                    motor_direction = NO_MOTION;
                } else if (motor.speed < -80 || motor_direction == REVERSE_MOTION) {
                    motor_direction = REVERSE_MOTION;
                } else {
                    motor_direction = NO_MOTION;
                }
                break;

            default:
                motor_direction = NO_MOTION;
                break;
        }

        // Apply smooth torque ramping for acceleration/deceleration
        apply_torque_ramping(&torque_output, motor_direction, &ramp_state, &gain_index, 
                           &prev_motor_direction, &prev_torque_dir, motor_cmd.intensity,
                           &motor_cmd.mode);

        // Send control command (response includes motor status)
        torque_control(&motor, torque_output);

        // Wait for CAN response (blocks up to 3ms for fresh data)
        if (xQueueReceive(motor_queue_ankle, &can_msg, pdMS_TO_TICKS(3)) == pdTRUE) {
            parse_motor_status(&can_msg, &motor);
            parse_espnow_status(&ankle_status, &motor);
            xQueueOverwrite(ankle_telemetry_queue, &ankle_status);
        }

        // Fixed 5ms cycle
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}


// ---------- Helper functions ----------

/**
 * @brief Applies smooth torque ramping for acceleration and deceleration.
 * 
 * Implements a state machine that gradually ramps torque up when starting motion
 * and ramps down when stopping, using an exponential gain profile. This prevents
 * abrupt torque changes that can cause jerky motion or mechanical stress.
 * 
 * @param torque_output Pointer to torque value to be modified with ramping
 * @param motor_moving Current motor direction: -1 (reverse), 0 (stopped), 1 (forward)
 * @param ramp_state Pointer to current ramping state (modified by function)
 * @param gain_index Pointer to current gain profile index (0-9, modified by function)
 * @param prev_motor_moving Pointer to previous motor direction (modified by function)
 * @param prev_torque_dir Pointer to previous torque direction for deceleration (modified by function)
 * @param intensity Base torque intensity to apply ramping to
 * 
 * @note Should be called every control cycle after calculating base torque_output
 * @note Modifies torque_output in-place based on current ramping state
 */
static void apply_torque_ramping(
    int16_t* torque_output,
    motion_direction_t motor_direction,
    ramp_state_t* ramp_state,
    uint8_t* gain_index,
    motion_direction_t* prev_motor_direction,
    int8_t* prev_torque_dir,
    int16_t intensity,
    char* mode
) {
    if (!torque_output || !ramp_state || !gain_index || !prev_motor_direction || !prev_torque_dir) {
        return;
    }

    // Detect state transitions
    bool stopped = (*prev_motor_direction != NO_MOTION && motor_direction == NO_MOTION);
    bool started = (*prev_motor_direction == NO_MOTION && motor_direction != NO_MOTION);

    // Update state machine
    if (stopped) {
        *ramp_state = RAMP_DECELERATING;
        *prev_torque_dir = (int8_t)*prev_motor_direction;
    } 
    else if (started) {
        *ramp_state = RAMP_ACCELERATING;
    }

    // Apply ramping based on current state
    switch (*ramp_state) {
        case RAMP_ACCELERATING:
            if (motor_direction != NO_MOTION) {
                *torque_output = motor_direction * intensity * torque_gain_profile[*gain_index];
                if (*gain_index < 9) {
                    (*gain_index)++;
                } else {
                    *ramp_state = RAMP_IDLE;  // Acceleration complete
                }
            }
            break;

        case RAMP_DECELERATING:
            if (motor_direction == NO_MOTION) {
                *torque_output = (*prev_torque_dir) * intensity * torque_gain_profile[*gain_index];
                    if (*gain_index > 0) {
                    (*gain_index)--;
                } else {
                    *torque_output = 0;
                    *ramp_state = RAMP_IDLE;  // Deceleration complete
                }
            }
            break;

        case RAMP_IDLE:
            // No ramping, direct passthrough of torque_output.
            break;
    }

    *prev_motor_direction = motor_direction;
}

/**
 * @brief Sends a command to read the status of the specified motor.
 * @param motor Pointer to the motor_status_t structure representing the motor.
 * @note This function sends a CAN message request to read the motor's status. This function doesn't
 * wait for a response; it only sends the request.
 * Command description from RMD manual:
 * This command reads the temperature,speed and encoder position of the current motor.
 */
static void read_motor_status(motor_status_t* motor) {
    if (motor == NULL) { return; }

    twai_message_t out_msg;
    out_msg.identifier = 0x140 + motor->id;
    out_msg.extd = 0;
    out_msg.rtr = 0;
    out_msg.data_length_code = 8;
    out_msg.data[0] = 0x9C; // Command to read motor status
    out_msg.data[1] = 0x00;
    out_msg.data[2] = 0x00;
    out_msg.data[3] = 0x00;
    out_msg.data[4] = 0x00;
    out_msg.data[5] = 0x00;
    out_msg.data[6] = 0x00;
    out_msg.data[7] = 0x00;

    // Non-blocking send with short timeout
    xQueueSend(can_tx_queue, &out_msg, pdMS_TO_TICKS(2));
}

/**
 * @brief Parses a received CAN message to extract motor status information.
 * @param rx_msg Pointer to the received CAN message.
 * @param motor Pointer to the motor_status_t structure where the parsed data will be stored.
 * @note The function extracts current, speed, and position from the CAN message data bytes.
*/
static void parse_motor_status(twai_message_t* rx_msg, motor_status_t* motor) {
    if (rx_msg == NULL || motor == NULL) {
        return;
    }

    motor->current = (int16_t)((rx_msg->data[3] << 8) | rx_msg->data[2]);
    motor->speed = (int16_t)((rx_msg->data[5] << 8) | rx_msg->data[4]);
    motor->position = (int16_t)((rx_msg->data[7] << 8) | rx_msg->data[6]);
}

/**
 * @brief Releases the brake of the specified motor.
 * @param motor Pointer to the motor_status_t structure representing the motor.
 */
static void release_motor_brake(motor_status_t* motor) {
    if (motor == NULL) { return; }

    twai_message_t out_msg;
    out_msg.identifier = 0x140 + motor->id;
    out_msg.extd = 0;
    out_msg.rtr = 0;
    out_msg.data_length_code = 8;
    out_msg.data[0] = 0x77;
    out_msg.data[1] = 0x00;
    out_msg.data[2] = 0x00;
    out_msg.data[3] = 0x00;
    out_msg.data[4] = 0x00;
    out_msg.data[5] = 0x00;
    out_msg.data[6] = 0x00;
    out_msg.data[7] = 0x00;

    xQueueSend(can_tx_queue, &out_msg, pdMS_TO_TICKS(10));
}

/**
 * @brief Send a command to shut down the specified motor.
 * @param motor Pointer to the motor_status_t structure representing the motor.
 * 
 * Command description from RMD manual:
 * Turns off the motor output and also clears the motor running state, not in any closed loop mode.
 */
static void shutdown_motor(motor_status_t* motor){
    if (motor == NULL) { return; }

    twai_message_t out_msg;
    out_msg.identifier = 0x140 + motor->id;
    out_msg.extd = 0;
    out_msg.rtr = 0;
    out_msg.data_length_code = 8;

    out_msg.data[0] = 0x80;
    out_msg.data[1] = 0x00;
    out_msg.data[2] = 0x00;
    out_msg.data[3] = 0x00;
    out_msg.data[4] = 0x00;
    out_msg.data[5] = 0x00;
    out_msg.data[6] = 0x00;
    out_msg.data[7] = 0x00;
   
    xQueueSend(can_tx_queue, &out_msg, pdMS_TO_TICKS(10));
}

/**
 * @brief Send a command to stop the specified motor.
 * @param motor Pointer to the motor_status_t structure representing the motor.
 * 
 * Command description from RMD manual:
 * Stop the motor, the closed-loop mode where the motor is still running, just stop the motor speed. 
 */
static void stop_motor(motor_status_t* motor){
    if (motor == NULL) { return; }

    twai_message_t out_msg;
    out_msg.identifier = 0x140 + motor->id;
    out_msg.extd = 0;
    out_msg.rtr = 0;
    out_msg.data_length_code = 8;

    out_msg.data[0] = 0x81;
    out_msg.data[1] = 0x00;
    out_msg.data[2] = 0x00;
    out_msg.data[3] = 0x00;
    out_msg.data[4] = 0x00;
    out_msg.data[5] = 0x00;
    out_msg.data[6] = 0x00;
    out_msg.data[7] = 0x00;

    xQueueSend(can_tx_queue, &out_msg, pdMS_TO_TICKS(10));
}

/**
 * @brief Sends a torque control command to the specified motor.
 * @param motor Pointer to the motor_status_t structure representing the motor.
 * @param torque_value The desired torque value to be set for the motor.
 * 
 * Command description from RMD manual:
 * This command is a control command,which can be run when the motor is not faulty.
 * The host sends this command to control the torque and current output of the motor. 
 * The control value iqControl is of type int16_t and the unit is 0.01A/LSB.
 * For safety reasons,This command cannot open the brake directly. But, you can use the
 * 0x77 command to open the brake first, then you can use A1 command .
 */
static void torque_control(motor_status_t* motor, int16_t torque_value) {
    if (motor == NULL) { return; }
    
    twai_message_t msg;
    msg.identifier = 0x140 + motor->id;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;
    msg.data[0] = 0xA1;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    msg.data[4] = (uint8_t)(torque_value & 0xFF);
    msg.data[5] = (uint8_t)((torque_value >> 8) & 0xFF);
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    // Non-blocking send with short timeout
    xQueueSend(can_tx_queue, &msg, pdMS_TO_TICKS(2));
}

/**
 * @brief Parses a control packet received via ESP-NOW to extract motor control commands.
 * @param packet Pointer to the control_packet structure received via ESP-NOW.
 * @param motor_cmd Pointer to the motor_control_t structure where the parsed commands will be stored.
 * @note This function extracts mode, intensity, minVal, and maxVal from the control packet.
 */
static void parse_motor_control_command(control_packet* packet, motor_control_t* motor_cmd) {
    if (packet == NULL || motor_cmd == NULL) {
        return;
    }
    motor_cmd->mode = packet->mode;
    motor_cmd->intensity = packet->intensity;
    motor_cmd->minVal = packet->minVal;
    motor_cmd->maxVal = packet->maxVal;
}

/**
 * @brief Parses motor status to ESP-NOW telemetry format.
 * @param espnow_status Pointer to the espnow_motor_status_t structure where the data will be stored.
 * @param motor Pointer to the motor_status_t structure containing the motor status.
 */
static void parse_espnow_status(espnow_motor_status_t* espnow_status, motor_status_t* motor) {
    if (espnow_status == NULL || motor == NULL) {
        return;
    }

    espnow_status->position = motor->position;
    espnow_status->speed = motor->speed;
    espnow_status->current = motor->current;
}