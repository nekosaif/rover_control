/*
Arduino : Monster 1
IN_PIN 22, 23, 24, 25 : 4, 7, 8, 9
PWM_PIN 2, 3 : 5, 6
EN_PIN 34, 35 : 0, 1

Arduino : Monster 2
IN_PIN 26, 27, 28, 29 : 4, 7, 8, 9
PWM_PIN 4, 5 : 5, 6
EN_PIN 36, 37 : 0, 1

Arduino : Monster 3
IN_PIN 30, 31, 32, 33 : 4, 7, 8, 9
PWM_PIN 6, 7 : 5, 6
EN_PIN 38, 39 : 0, 1

Arduino : BTS 1
EN_PIN 40, 41 : R_EN, L_EN
PWM_PIN 8, 9 : R_PWM, L_PWM

Arduino : BTS 2
EN_PIN 42, 43 : R_EN, L_EN
PWM_PIN 10, 11 : R_PWM, L_PWM
*/

#define BRAKEVCC 0
#define CW 1
#define CCW 2
#define BRAKEGND 3

#define MOTOR_A 0
#define MOTOR_B 1
#define MOTOR_C 2
#define MOTOR_D 3
#define MOTOR_E 4
#define MOTOR_F 5

#define WHEEL_A 0
#define WHEEL_B 1


byte readByte;


const int inAPin[6] = {24, 25, 28, 29, 32, 33};
const int inBPin[6] = {23, 22, 27, 26, 31, 30};

const int pwmPin[6] = {2, 3, 4, 5, 6, 7};

const int enPin[6] = {34, 35, 36, 37, 38, 39};


const int enAPinWheel[2] = {40, 42};
const int enBPinWheel[2] = {41, 43};

const int pwmAPinWheel[2] = {8, 10};
const int pwmBPinWheel[2] = {9, 11};


const int START_DELAY = 300 / 255;
const int STOP_DELAY = 300 / 255;


int MAX_SPEED;


bool isMovingForward = false;
bool isMovingReverse = false;
bool isMovingRight = false;
bool isMovingLeft = false;


void setup()
{
    Serial.begin(9600);

    motorSetup();
    wheelSetup();

    MAX_SPEED = 150;
}

void loop()
{
    if (Serial.available() > 0)
    {
        readByte = Serial.read();
        if (readByte == 'n')
        {
            arm_base(255, true);
        }
        else if (readByte == 'm')
        {
            arm_base(255, false);
        }
        else if (readByte == 'r')
        {
            arm_act1(255, true);
        }
        else if (readByte == 'f')
        {
            arm_act1(255, false);
        }
        else if (readByte == 't')
        {
            arm_act2(255, true);
        }
        else if (readByte == 'g')
        {
            arm_act2(255, false);
        }
        else if (readByte == 'y')
        {
            arm_act3(255, true);
        }
        else if (readByte == 'h')
        {
            arm_act3(255, false);
        }
        else if (readByte == 'v')
        {
            arm_wrist(255, true);
        }
        else if (readByte == 'b')
        {
            arm_wrist(255, false);
        }
        else if (readByte == 'o')
        {
            arm_claw(255, true);
        }
        else if (readByte == 'p')
        {
            arm_claw(255, false);
        }
        else if (readByte == 'w')
        {
            wheel_onward(255, true);
        }
        else if (readByte == 's')
        {
            wheel_onward(255, false);
        }
        else if (readByte == 'a')
        {
            wheel_steering(255, true);
        }
        else if (readByte == 'd')
        {
            wheel_steering(255, false);
        }
        else if (readByte == '1')
        {
            MAX_SPEED = 50;
        }
        else if (readByte == '2')
        {
            MAX_SPEED = 150;
        }
        else if (readByte == '3')
        {
            MAX_SPEED = 200;
        }
        else if (readByte == '4')
        {
            MAX_SPEED = 250;
        }
        else if (readByte == '-')
        {
            stop_all();
        }
    }
    Serial.flush();
}

void motorSetup()
{
    for (int i = 0; i < 6; i++)
    {
        pinMode(inAPin[i], OUTPUT);
        pinMode(inBPin[i], OUTPUT);
        pinMode(pwmPin[i], OUTPUT);
        pinMode(enPin[i], OUTPUT);

        digitalWrite(inAPin[i], LOW);
        digitalWrite(inBPin[i], LOW);
        digitalWrite(enPin[i], LOW);
    }
}

void wheelSetup()
{
    for (int i = 0; i < 2; i++)
    {
        pinMode(enAPinWheel[i], OUTPUT);
        pinMode(enBPinWheel[i], OUTPUT);
        pinMode(pwmAPinWheel[i], OUTPUT);
        pinMode(pwmBPinWheel[i], OUTPUT);

        analogWrite(pwmAPinWheel[i], 0);
        analogWrite(pwmBPinWheel[i], 0);

        digitalWrite(enAPinWheel[i], LOW);
        digitalWrite(enBPinWheel[i], LOW);
    }
}

void motorGo(uint8_t motor, uint8_t mode, uint8_t speed)
{

    digitalWrite(enPin[motor], HIGH);
    switch (mode)
    {
    case BRAKEVCC: // Brake to VCC
        digitalWrite(inAPin[motor], HIGH);
        digitalWrite(inBPin[motor], HIGH);
        break;
    case CW: // Turn Clockwise
        digitalWrite(inAPin[motor], HIGH);
        digitalWrite(inBPin[motor], LOW);
        break;
    case CCW: // Turn Counter-Clockwise
        digitalWrite(inAPin[motor], LOW);
        digitalWrite(inBPin[motor], HIGH);
        break;
    case BRAKEGND: // Brake to GND
        digitalWrite(inAPin[motor], LOW);
        digitalWrite(inBPin[motor], LOW);
        break;

    default:
        return;
    }
    analogWrite(pwmPin[motor], speed);
    return;
}

void wheelGo(uint8_t wheel, uint8_t mode, uint8_t speed)
{
    digitalWrite(enAPinWheel[wheel], HIGH);
    digitalWrite(enBPinWheel[wheel], HIGH);
    switch (mode)
    {
    case CW:
        analogWrite(pwmAPinWheel[wheel], speed);
        analogWrite(pwmBPinWheel[wheel], 0);
        break;
    case CCW:
        analogWrite(pwmAPinWheel[wheel], 0);
        analogWrite(pwmBPinWheel[wheel], speed);
        break;

    default:
        return;
    }
    return;
}

void motorOff(uint8_t motor)
{
    digitalWrite(inAPin[motor], LOW);
    digitalWrite(inBPin[motor], LOW);
    analogWrite(pwmPin[motor], 0);
    digitalWrite(enPin[motor], LOW);
}

void wheelOff(uint8_t wheel)
{
    digitalWrite(enAPinWheel[wheel], LOW);
    digitalWrite(enBPinWheel[wheel], LOW);
    analogWrite(pwmAPinWheel[wheel], 0);
    analogWrite(pwmBPinWheel[wheel], 0);
}

void arm_base(int pwmSpeed, bool isForward)
{
    motorGo(MOTOR_A, isForward ? CW : CCW, pwmSpeed);
}

void arm_act1(int pwmSpeed, bool isForward)
{
    motorGo(MOTOR_B, isForward ? CW : CCW, pwmSpeed);
}

void arm_act2(int pwmSpeed, bool isForward)
{
    motorGo(MOTOR_C, isForward ? CW : CCW, pwmSpeed);
}

void arm_act3(int pwmSpeed, bool isForward)
{
    motorGo(MOTOR_D, isForward ? CW : CCW, pwmSpeed);
}

void arm_wrist(int pwmSpeed, bool isForward)
{
    motorGo(MOTOR_E, isForward ? CW : CCW, pwmSpeed);
}

void arm_claw(int pwmSpeed, bool isForward)
{
    motorGo(MOTOR_F, isForward ? CW : CCW, pwmSpeed);
}

void wheel_onward(int pwmSpeed, bool isForward)
{
    if (!isMovingForward && !isMovingReverse)
    {
        for (int i = 1; i < MAX_SPEED; ++i) 
        {
            wheelGo(WHEEL_A, isForward ? CW : CCW, i);
            wheelGo(WHEEL_B, isForward ? CW : CCW, i);
            delay(START_DELAY);
        }
        if (isForward)
        {
            isMovingForward = true;
        }
        else
        {
          isMovingReverse = true;
        }
    }
}

void wheel_steering(int pwmSpeed, bool isRight)
{
    if (!isMovingRight && !isMovingLeft)
    {
        for (int i = 1; i < MAX_SPEED; ++i) 
        {
            wheelGo(WHEEL_A, isRight ? CW : CCW, i);
            wheelGo(WHEEL_B, isRight ? CCW : CW, i);
            delay(START_DELAY);
        }
        if (isRight)
        {
            isMovingRight = true;
        }
        else
        {
          isMovingLeft = true;
        }
    }
}

void stop_all()
{
    if (isMovingForward)
    {
        for (int i = MAX_SPEED; i > 1; --i) 
        {
            wheelGo(WHEEL_A, isMovingForward ? CW : CCW, i);
            wheelGo(WHEEL_B, isMovingForward ? CW : CCW, i);
            delay(STOP_DELAY);
        }
        isMovingForward = false;
    }
    if (isMovingReverse)
    {
        for (int i = MAX_SPEED; i > 1; --i) 
        {
            wheelGo(WHEEL_A, isMovingForward ? CW : CCW, i);
            wheelGo(WHEEL_B, isMovingForward ? CW : CCW, i);
            delay(STOP_DELAY);
        }
        isMovingReverse = false;
    }
    if (isMovingRight)
    {
        for (int i = MAX_SPEED; i > 1; --i) 
        {
            wheelGo(WHEEL_A, isMovingRight ? CW : CCW, i);
            wheelGo(WHEEL_B, isMovingRight ? CCW : CW, i);
            delay(STOP_DELAY);
        }
        isMovingRight = false;
    }
    if (isMovingLeft)
    {
        for (int i = MAX_SPEED; i > 1; --i) 
        {
            wheelGo(WHEEL_A, isMovingRight ? CW : CCW, i);
            wheelGo(WHEEL_B, isMovingRight ? CCW : CW, i);
            delay(STOP_DELAY);
        }
        isMovingLeft = false;
    }
    motorOff(MOTOR_A);
    motorOff(MOTOR_B);
    motorOff(MOTOR_C);
    motorOff(MOTOR_D);
    motorOff(MOTOR_E);
    motorOff(MOTOR_F);
    wheelOff(WHEEL_A);
    wheelOff(WHEEL_B);
}
