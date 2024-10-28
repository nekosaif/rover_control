/**
 * @file rover_control_arduino_mega.ino
 * @brief Advanced motor control system for a robotic arm with wheel base
 * 
 * This system controls a robotic arm with 6 motors and a wheeled base with 2 motors.
 * It uses Monster motor drivers for the arm and BTS drivers for the wheels.
 * The system can be controlled via serial communication.
 * 
 * Hardware Configuration:
 * - 3x Monster motor drivers (for arm motors)
 * - 2x BTS motor drivers (for wheel motors)
 * - Arduino board (with sufficient digital and PWM pins)
 * 
 * Pin Configuration:
 * Monster 1: IN_PIN(22,23,24,25), PWM_PIN(2,3), EN_PIN(34,35)
 * Monster 2: IN_PIN(26,27,28,29), PWM_PIN(4,5), EN_PIN(36,37)
 * Monster 3: IN_PIN(30,31,32,33), PWM_PIN(6,7), EN_PIN(38,39)
 * BTS 1: EN_PIN(40,41), PWM_PIN(8,9)
 * BTS 2: EN_PIN(42,43), PWM_PIN(10,11)
 */

// Motor control modes
enum MotorMode {
    BRAKE_VCC = 0,  ///< Brake to VCC
    CW = 1,         ///< Clockwise rotation
    CCW = 2,        ///< Counter-clockwise rotation
    BRAKE_GND = 3   ///< Brake to GND
};

// Motor identifiers for the robotic arm
enum ArmMotor {
    BASE = 0,       ///< Base rotation motor
    ACTUATOR_1 = 1, ///< First arm segment actuator
    ACTUATOR_2 = 2, ///< Second arm segment actuator
    ACTUATOR_3 = 3, ///< Third arm segment actuator
    WRIST = 4,      ///< Wrist rotation motor
    CLAW = 5        ///< Claw gripper motor
};

// Wheel identifiers
enum Wheel {
    LEFT = 0,       ///< Left wheel
    RIGHT = 1       ///< Right wheel
};

// Speed presets
enum SpeedPreset {
    SPEED_SLOW = 50,    ///< Slow speed (50/255)
    SPEED_NORMAL = 150, ///< Normal speed (150/255)
    SPEED_FAST = 200,   ///< Fast speed (200/255)
    SPEED_MAX = 250     ///< Maximum speed (250/255)
};

// Motion timing constants
const int START_DELAY = 300 / 255; ///< Delay for smooth acceleration
const int STOP_DELAY = 300 / 255;  ///< Delay for smooth deceleration

// Pin configurations for arm motors
const int inAPin[6] = {24, 25, 28, 29, 32, 33};
const int inBPin[6] = {23, 22, 27, 26, 31, 30};
const int pwmPin[6] = {2, 3, 4, 5, 6, 7};
const int enPin[6] = {34, 35, 36, 37, 38, 39};

// Pin configurations for wheel motors
const int enAPinWheel[2] = {40, 42};
const int enBPinWheel[2] = {41, 43};
const int pwmAPinWheel[2] = {8, 10};
const int pwmBPinWheel[2] = {9, 11};

// Global state variables
int currentMaxSpeed = SPEED_NORMAL;
struct MovementState {
    bool forward = false;
    bool reverse = false;
    bool right = false;
    bool left = false;
} movementState;

/**
 * @brief Initialize the system
 * Sets up serial communication and configures all motor pins
 */
void setup() {
    Serial.begin(9600);
    initializeArmMotors();
    initializeWheelMotors();
}

/**
 * @brief Main program loop
 * Handles serial communication and executes commands
 */
void loop() {
    if (Serial.available() > 0) {
        processSerialCommand(Serial.read());
    }
    Serial.flush();
}

/**
 * @brief Initialize all arm motor pins
 */
void initializeArmMotors() {
    for (int i = 0; i < 6; i++) {
        pinMode(inAPin[i], OUTPUT);
        pinMode(inBPin[i], OUTPUT);
        pinMode(pwmPin[i], OUTPUT);
        pinMode(enPin[i], OUTPUT);
        
        // Set initial state
        digitalWrite(inAPin[i], LOW);
        digitalWrite(inBPin[i], LOW);
        digitalWrite(enPin[i], LOW);
    }
}

/**
 * @brief Initialize all wheel motor pins
 */
void initializeWheelMotors() {
    for (int i = 0; i < 2; i++) {
        pinMode(enAPinWheel[i], OUTPUT);
        pinMode(enBPinWheel[i], OUTPUT);
        pinMode(pwmAPinWheel[i], OUTPUT);
        pinMode(pwmBPinWheel[i], OUTPUT);
        
        // Set initial state
        analogWrite(pwmAPinWheel[i], 0);
        analogWrite(pwmBPinWheel[i], 0);
        digitalWrite(enAPinWheel[i], LOW);
        digitalWrite(enBPinWheel[i], LOW);
    }
}

/**
 * @brief Process incoming serial commands
 * @param command The command byte received from serial
 */
void processSerialCommand(byte command) {
    switch (command) {
        // Arm base controls
        case 'n': controlArmMotor(BASE, true, currentMaxSpeed); break;
        case 'm': controlArmMotor(BASE, false, currentMaxSpeed); break;
        
        // First actuator controls
        case 'r': controlArmMotor(ACTUATOR_1, true, currentMaxSpeed); break;
        case 'f': controlArmMotor(ACTUATOR_1, false, currentMaxSpeed); break;
        
        // Second actuator controls
        case 't': controlArmMotor(ACTUATOR_2, true, currentMaxSpeed); break;
        case 'g': controlArmMotor(ACTUATOR_2, false, currentMaxSpeed); break;
        
        // Third actuator controls
        case 'y': controlArmMotor(ACTUATOR_3, true, currentMaxSpeed); break;
        case 'h': controlArmMotor(ACTUATOR_3, false, currentMaxSpeed); break;
        
        // Wrist controls
        case 'v': controlArmMotor(WRIST, true, currentMaxSpeed); break;
        case 'b': controlArmMotor(WRIST, false, currentMaxSpeed); break;
        
        // Claw controls
        case 'o': controlArmMotor(CLAW, true, currentMaxSpeed); break;
        case 'p': controlArmMotor(CLAW, false, currentMaxSpeed); break;
        
        // Wheel movement controls
        case 'w': controlWheelMovement(true, true); break;  // Forward
        case 's': controlWheelMovement(true, false); break; // Reverse
        case 'a': controlWheelMovement(false, true); break; // Left
        case 'd': controlWheelMovement(false, false); break; // Right
        
        // Speed controls
        case '1': currentMaxSpeed = SPEED_SLOW; break;
        case '2': currentMaxSpeed = SPEED_NORMAL; break;
        case '3': currentMaxSpeed = SPEED_FAST; break;
        case '4': currentMaxSpeed = SPEED_MAX; break;
        
        // Emergency stop
        case '-': emergencyStop(); break;
    }
}

/**
 * @brief Control an arm motor
 * @param motor Motor identifier
 * @param mode Direction (true for CW, false for CCW)
 * @param speed Motor speed (0-255)
 */
void controlArmMotor(uint8_t motor, bool direction, uint8_t speed) {
    digitalWrite(enPin[motor], HIGH);
    digitalWrite(inAPin[motor], direction);
    digitalWrite(inBPin[motor], !direction);
    analogWrite(pwmPin[motor], speed);
}

/**
 * @brief Control wheel movement
 * @param isForwardReverse true for forward/reverse, false for left/right
 * @param direction true for forward/right, false for reverse/left
 */
void controlWheelMovement(bool isForwardReverse, bool direction) {
    if (canStartMovement(isForwardReverse, direction)) {
        for (int i = 1; i < currentMaxSpeed; ++i) {
            if (isForwardReverse) {
                wheelGo(LEFT, direction ? CW : CCW, i);
                wheelGo(RIGHT, direction ? CW : CCW, i);
            } else {
                wheelGo(LEFT, direction ? CW : CCW, i);
                wheelGo(RIGHT, direction ? CCW : CW, i);
            }
            delay(START_DELAY);
        }
        updateMovementState(isForwardReverse, direction);
    }
}

/**
 * @brief Check if new movement can be started
 * @param isForwardReverse Movement type
 * @param direction Movement direction
 * @return true if movement can start
 */
bool canStartMovement(bool isForwardReverse, bool direction) {
    if (isForwardReverse) {
        return !movementState.forward && !movementState.reverse;
    } else {
        return !movementState.right && !movementState.left;
    }
}

/**
 * @brief Update movement state flags
 * @param isForwardReverse Movement type
 * @param direction Movement direction
 */
void updateMovementState(bool isForwardReverse, bool direction) {
    if (isForwardReverse) {
        movementState.forward = direction;
        movementState.reverse = !direction;
    } else {
        movementState.right = direction;
        movementState.left = !direction;
    }
}

/**
 * @brief Control a wheel motor
 * @param wheel Wheel identifier
 * @param mode Rotation mode
 * @param speed Motor speed (0-255)
 */
void wheelGo(uint8_t wheel, uint8_t mode, uint8_t speed) {
    digitalWrite(enAPinWheel[wheel], HIGH);
    digitalWrite(enBPinWheel[wheel], HIGH);
    
    if (mode == CW) {
        analogWrite(pwmAPinWheel[wheel], speed);
        analogWrite(pwmBPinWheel[wheel], 0);
    } else if (mode == CCW) {
        analogWrite(pwmAPinWheel[wheel], 0);
        analogWrite(pwmBPinWheel[wheel], speed);
    }
}

/**
 * @brief Stop all motors with smooth deceleration
 */
void emergencyStop() {
    // Smooth deceleration for wheels if they're moving
    decelerateWheels();
    
    // Stop all arm motors
    for (int i = 0; i < 6; i++) {
        motorOff(i);
    }
    
    // Stop all wheel motors
    for (int i = 0; i < 2; i++) {
        wheelOff(i);
    }
    
    // Reset movement state
    movementState = {false, false, false, false};
}

/**
 * @brief Decelerate wheels smoothly
 */
void decelerateWheels() {
    if (movementState.forward || movementState.reverse) {
        for (int i = currentMaxSpeed; i > 1; --i) {
            wheelGo(LEFT, movementState.forward ? CW : CCW, i);
            wheelGo(RIGHT, movementState.forward ? CW : CCW, i);
            delay(STOP_DELAY);
        }
    }
    if (movementState.right || movementState.left) {
        for (int i = currentMaxSpeed; i > 1; --i) {
            wheelGo(LEFT, movementState.right ? CW : CCW, i);
            wheelGo(RIGHT, movementState.right ? CCW : CW, i);
            delay(STOP_DELAY);
        }
    }
}

/**
 * @brief Turn off an arm motor
 * @param motor Motor identifier
 */
void motorOff(uint8_t motor) {
    digitalWrite(inAPin[motor], LOW);
    digitalWrite(inBPin[motor], LOW);
    analogWrite(pwmPin[motor], 0);
    digitalWrite(enPin[motor], LOW);
}

/**
 * @brief Turn off a wheel motor
 * @param wheel Wheel identifier
 */
void wheelOff(uint8_t wheel) {
    digitalWrite(enAPinWheel[wheel], LOW);
    digitalWrite(enBPinWheel[wheel], LOW);
    analogWrite(pwmAPinWheel[wheel], 0);
    analogWrite(pwmBPinWheel[wheel], 0);
}
