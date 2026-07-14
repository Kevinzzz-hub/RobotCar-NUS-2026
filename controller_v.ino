#include <Wire.h>
#include <AFMotor.h>

#define MPU_ADDR 0x68

//==================================================
// Function Declarations
//==================================================

void stopCar();
void calibrateIMU();
void moveForward();
void moveBackward();
void steerLeftLarge();
void steerRightLarge();
void rotateRight();
void startTurnAround();

//==================================================
// Ultrasonic Sensor
//==================================================

#define TRIG_PIN 30
#define ECHO_PIN 31

const float STOP_DISTANCE_CM = 20.0;
const float MIN_VALID_DISTANCE_CM = 4.0;
const float MAX_VALID_DISTANCE_CM = 300.0;


//==================================================
// Autonomous Avoidance
//==================================================

bool autonomousMode = false;

const int AUTO_MOVE_SPEED = 130;

const float AUTO_STOP_DISTANCE_CM = 40.0;
const float AUTO_CLEAR_DISTANCE_CM = 55.0;

const unsigned long AUTO_TURN_STEP_MS = 450;
const unsigned long AUTO_SETTLE_MS = 250;

enum AutoState
{
    AUTO_IDLE,
    AUTO_FORWARD,
    AUTO_STOP,
    AUTO_TURN_LEFT,
    AUTO_CHECK_PATH
};

AutoState autoState = AUTO_IDLE;
unsigned long autoStateStartTime = 0;

const unsigned long ULTRASONIC_INTERVAL_MS = 100;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000;

float ultrasonicDistanceCm = 0.0;
unsigned long lastUltrasonicRead = 0;

bool obstacleDetected = false;

//==================================================
// Serial Handshake
//==================================================
bool handshakeComplete = false;
const char HANDSHAKE_HELLO = 'H';
const char HANDSHAKE_ACK = 'A';
const char HANDSHAKE_READY = 'R';
//==================================================
// Wheel Encoders
//==================================================

#define LEFT_ENCODER_PIN  19
#define RIGHT_ENCODER_PIN 18

const float WHEEL_DIAMETER_CM = 6.5;
const float PULSES_PER_REVOLUTION = 4.0;

const unsigned long ENCODER_THRESHOLD_MS = 10;

volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;

volatile unsigned long leftEncoderPreviousTime = 0;
volatile unsigned long rightEncoderPreviousTime = 0;

//==================================================
// Motor Definition
//==================================================

// M1 = Left Rear
// M2 = Right Rear
// M3 = Right Front
// M4 = Left Front

AF_DCMotor leftRear(1);
AF_DCMotor rightRear(2);
AF_DCMotor rightFront(3);
AF_DCMotor leftFront(4);

//==================================================
// Speed Settings
//==================================================

const int MOVE_SPEED = 180;

// A/Dè½¬åæ¶ï¼åä¾§è½®é€åº¦æ´ä½ï¼è½¬åå¹åº¦æ´å¤§
const int STEER_INNER_SPEED = 30;
const int STEER_OUTER_SPEED = 180;

const int ROTATE_SPEED = 160;

//==================================================
// Motor Direction
//==================================================

#define LEFT_FORWARD  BACKWARD
#define LEFT_BACKWARD FORWARD

#define RIGHT_FORWARD  FORWARD
#define RIGHT_BACKWARD BACKWARD

//==================================================
// Keyboard Control
//==================================================

// çµèç«¯åºæ¯éçº¦80mséå¤åé€å½åæé®
const unsigned long COMMAND_TIMEOUT_MS = 250;

// åå§ä¼°è®¡å€¼ï¼å¿é¡»æ ¹æ®å®è½¦è°æ´
const unsigned long TURN_180_TIME_MS = 1300;

unsigned long lastControlCommandTime = 0;
unsigned long turnAroundStartTime = 0;

bool keyboardControlActive = false;
bool turnAroundActive = false;
bool carMoving = false;

char currentMotionCommand = 'X';

//==================================================
// IMU Timing
//==================================================

const unsigned long IMU_PRINT_INTERVAL_MS = 500;
unsigned long lastIMUPrint = 0;

//==================================================
// Raw IMU Data
//==================================================

int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;

int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;

int16_t tempRaw = 0;

//==================================================
// Converted IMU Values
//==================================================

float ax_g = 0.0;
float ay_g = 0.0;
float az_g = 0.0;

float gx_dps = 0.0;
float gy_dps = 0.0;
float gz_dps = 0.0;

float temperature = 0.0;

float pitch = 0.0;
float roll = 0.0;

//==================================================
// IMU Calibration
//==================================================

float gxBias = 0.0;
float gyBias = 0.0;
float gzBias = 0.0;

bool imuCalibrated = false;

const float MAX_PITCH_ANGLE = 35.0;
const float MAX_ROLL_ANGLE = 35.0;

bool tiltWarningActive = false;

const int TILT_CONFIRM_COUNT = 8;
int tiltUnsafeCount = 0;

//==================================================
// Encoder Interrupt Functions
//==================================================

void leftEncoderTick()
{
    unsigned long now = millis();

    if(now - leftEncoderPreviousTime > ENCODER_THRESHOLD_MS)
    {
        leftEncoderCount++;
        leftEncoderPreviousTime = now;
    }
}

void rightEncoderTick()
{
    unsigned long now = millis();

    if(now - rightEncoderPreviousTime > ENCODER_THRESHOLD_MS)
    {
        rightEncoderCount++;
        rightEncoderPreviousTime = now;
    }
}

//==================================================
// Encoder Functions
//==================================================

void getEncoderCounts(
    unsigned long &leftCount,
    unsigned long &rightCount
)
{
    noInterrupts();

    leftCount = leftEncoderCount;
    rightCount = rightEncoderCount;

    interrupts();
}

void resetEncoderCounts()
{
    noInterrupts();

    leftEncoderCount = 0;
    rightEncoderCount = 0;

    leftEncoderPreviousTime = 0;
    rightEncoderPreviousTime = 0;

    interrupts();
}

float pulsesToDistanceCm(unsigned long pulses)
{
    float wheelCircumference =
        PI * WHEEL_DIAMETER_CM;

    return
        pulses
        / PULSES_PER_REVOLUTION
        * wheelCircumference;
}

//==================================================
// MPU Register Functions
//==================================================

void writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU_ADDR);

    Wire.write(reg);
    Wire.write(value);

    Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg)
{
    Wire.beginTransmission(MPU_ADDR);

    Wire.write(reg);

    Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDR, 1);

    if(Wire.available())
    {
        return Wire.read();
    }

    return 0;
}

//==================================================
// Read MPU6500
//==================================================

bool readMPU()
{
    Wire.beginTransmission(MPU_ADDR);

    Wire.write(0x3B);

    if(Wire.endTransmission(false) != 0)
    {
        return false;
    }

    Wire.requestFrom(MPU_ADDR, 14);

    if(Wire.available() < 14)
    {
        return false;
    }

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();

    tempRaw = (Wire.read() << 8) | Wire.read();

    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();

    return true;
}

void convertUnits()
{
    ax_g = ax / 16384.0;
    ay_g = ay / 16384.0;
    az_g = az / 16384.0;

    gx_dps = gx / 131.0;
    gy_dps = gy / 131.0;
    gz_dps = gz / 131.0;

    temperature =
        tempRaw / 333.87 + 21.0;
}

void calculateAttitude()
{
    roll =
        atan2(ay_g, az_g)
        * 180.0
        / PI;

    pitch =
        atan2(
            -ax_g,
            sqrt(
                ay_g * ay_g
                + az_g * az_g
            )
        )
        * 180.0
        / PI;
}

void updateIMU()
{
    if(!readMPU())
    {
        return;
    }

    convertUnits();

    if(imuCalibrated)
    {
        gx_dps -= gxBias;
        gy_dps -= gyBias;
        gz_dps -= gzBias;
    }

    calculateAttitude();
}

//==================================================
// Motor Speed Functions
//==================================================

void setLeftSpeed(int speedValue)
{
    speedValue = constrain(speedValue, 0, 255);

    leftFront.setSpeed(speedValue);
    leftRear.setSpeed(speedValue);
}

void setRightSpeed(int speedValue)
{
    speedValue = constrain(speedValue, 0, 255);

    rightFront.setSpeed(speedValue);
    rightRear.setSpeed(speedValue);
}

//==================================================
// Autonomous Motor Functions
//==================================================

void moveForwardAuto()
{
    setLeftSpeed(AUTO_MOVE_SPEED);
    setRightSpeed(AUTO_MOVE_SPEED);

    leftFront.run(LEFT_FORWARD);
    leftRear.run(LEFT_FORWARD);

    rightFront.run(RIGHT_FORWARD);
    rightRear.run(RIGHT_FORWARD);

    carMoving = true;
    currentMotionCommand = 'W';
}

void rotateLeftAuto()
{
    setLeftSpeed(ROTATE_SPEED);
    setRightSpeed(ROTATE_SPEED);

    leftFront.run(LEFT_BACKWARD);
    leftRear.run(LEFT_BACKWARD);

    rightFront.run(RIGHT_FORWARD);
    rightRear.run(RIGHT_FORWARD);

    carMoving = true;
    currentMotionCommand = 'A';
}


//==================================================
// Motor Movement Functions
//==================================================

void moveForward()
{
    setLeftSpeed(MOVE_SPEED);
    setRightSpeed(MOVE_SPEED);

    leftFront.run(LEFT_FORWARD);
    leftRear.run(LEFT_FORWARD);

    rightFront.run(RIGHT_FORWARD);
    rightRear.run(RIGHT_FORWARD);

    carMoving = true;
    currentMotionCommand = 'W';
}

void moveBackward()
{
    setLeftSpeed(MOVE_SPEED);
    setRightSpeed(MOVE_SPEED);

    leftFront.run(LEFT_BACKWARD);
    leftRear.run(LEFT_BACKWARD);

    rightFront.run(RIGHT_BACKWARD);
    rightRear.run(RIGHT_BACKWARD);

    carMoving = true;
    currentMotionCommand = 'S';
}

void steerLeftLarge()
{
    setLeftSpeed(STEER_INNER_SPEED);
    setRightSpeed(STEER_OUTER_SPEED);

    leftFront.run(LEFT_FORWARD);
    leftRear.run(LEFT_FORWARD);

    rightFront.run(RIGHT_FORWARD);
    rightRear.run(RIGHT_FORWARD);

    carMoving = true;
    currentMotionCommand = 'A';
}

void steerRightLarge()
{
    setLeftSpeed(STEER_OUTER_SPEED);
    setRightSpeed(STEER_INNER_SPEED);

    leftFront.run(LEFT_FORWARD);
    leftRear.run(LEFT_FORWARD);

    rightFront.run(RIGHT_FORWARD);
    rightRear.run(RIGHT_FORWARD);

    carMoving = true;
    currentMotionCommand = 'D';
}

void rotateRight()
{
    setLeftSpeed(ROTATE_SPEED);
    setRightSpeed(ROTATE_SPEED);

    leftFront.run(LEFT_FORWARD);
    leftRear.run(LEFT_FORWARD);

    rightFront.run(RIGHT_BACKWARD);
    rightRear.run(RIGHT_BACKWARD);

    carMoving = true;
    currentMotionCommand = 'U';
}

void stopCar()
{
    leftFront.run(RELEASE);
    leftRear.run(RELEASE);

    rightFront.run(RELEASE);
    rightRear.run(RELEASE);

    carMoving = false;
    currentMotionCommand = 'X';
}

//==================================================
// Ultrasonic Functions
//==================================================

float readUltrasonicDistanceCm()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration =
        pulseIn(
            ECHO_PIN,
            HIGH,
            ULTRASONIC_TIMEOUT_US
        );

    if(duration == 0)
    {
        return 0.0;
    }

    float distance =
        duration * 0.0343 / 2.0;

    if(distance < MIN_VALID_DISTANCE_CM)
    {
        return 0.0;
    }

    if(distance > MAX_VALID_DISTANCE_CM)
    {
        return MAX_VALID_DISTANCE_CM;
    }

    return distance;
}

void updateUltrasonic()
{
    if(
        millis() - lastUltrasonicRead
        < ULTRASONIC_INTERVAL_MS
    )
    {
        return;
    }

    lastUltrasonicRead = millis();

    float newDistance =
        readUltrasonicDistanceCm();
    if(newDistance > 0.0)
    {
        ultrasonicDistanceCm = newDistance;
    }

    bool movingTowardObstacle =
        currentMotionCommand == 'W'
        || currentMotionCommand == 'A'
        || currentMotionCommand == 'D';

    bool obstacleTooClose =
        ultrasonicDistanceCm > 0
        && ultrasonicDistanceCm < STOP_DISTANCE_CM;

    if(!autonomousMode && movingTowardObstacle && obstacleTooClose)
    {
        stopCar();

        keyboardControlActive = false;
        turnAroundActive = false;

        if(!obstacleDetected)
        {
            obstacleDetected = true;

            Serial.println();
            Serial.println("******** OBSTACLE ********");

            Serial.print("Distance: ");
            Serial.print(ultrasonicDistanceCm, 1);
            Serial.println(" cm");

            Serial.println("Movement stopped.");
            Serial.println();
        }
    }
    else if(!obstacleTooClose)
    {
        obstacleDetected = false;
    }
}

//==================================================
// 180-Degree Turn
//==================================================

void startTurnAround()
{
    stopCar();

    keyboardControlActive = false;

    resetEncoderCounts();

    rotateRight();

    turnAroundStartTime = millis();
    turnAroundActive = true;

    Serial.println("180-degree turn started.");
}

void updateTurnAround()
{
    if(!turnAroundActive)
    {
        return;
    }

    if(
        millis() - turnAroundStartTime
        >= TURN_180_TIME_MS
    )
    {
        turnAroundActive = false;

        stopCar();

        Serial.println("180-degree turn finished.");
    }
}

//==================================================
// Autonomous Mode Control
//==================================================

void setAutoState(AutoState newState)
{
    autoState = newState;
    autoStateStartTime = millis();
}

void startAutonomousMode()
{
    keyboardControlActive = false;
    turnAroundActive = false;

    autonomousMode = true;

    stopCar();
    resetEncoderCounts();

    moveForwardAuto();
    setAutoState(AUTO_FORWARD);

    Serial.println();
    Serial.println("Autonomous mode started.");
}

void stopAutonomousMode()
{
    autonomousMode = false;
    keyboardControlActive = false;

    stopCar();
    setAutoState(AUTO_IDLE);

    Serial.println("Autonomous mode stopped.");
}

void updateAutonomousMode()
{
    if (!autonomousMode)
    {
        return;
    }

    unsigned long now = millis();

    switch (autoState)
    {
        case AUTO_FORWARD:
        {
            bool validDistance =
                ultrasonicDistanceCm >= MIN_VALID_DISTANCE_CM
                && ultrasonicDistanceCm <= MAX_VALID_DISTANCE_CM;

            bool obstacleTooClose =
                validDistance
                && ultrasonicDistanceCm < AUTO_STOP_DISTANCE_CM;

            if (obstacleTooClose)
            {
                stopCar();
                setAutoState(AUTO_STOP);
            }
            break;
        }

        case AUTO_STOP:
        {
            if (now - autoStateStartTime >= AUTO_SETTLE_MS)
            {
                rotateLeftAuto();
                setAutoState(AUTO_TURN_LEFT);
            }
            break;
        }

        case AUTO_TURN_LEFT:
        {
            if (now - autoStateStartTime >= AUTO_TURN_STEP_MS)
            {
                stopCar();
                setAutoState(AUTO_CHECK_PATH);
            }
            break;
        }

        case AUTO_CHECK_PATH:
        {
            if (now - autoStateStartTime >= AUTO_SETTLE_MS)
            {
                bool pathIsClear = 
                    ultrasonicDistanceCm >= AUTO_CLEAR_DISTANCE_CM
                    && ultrasonicDistanceCm <= MAX_VALID_DISTANCE_CM;
                if (pathIsClear)
                {
                    moveForwardAuto();
                    setAutoState(AUTO_FORWARD);
                }
                else
                {
                    rotateLeftAuto();
                    setAutoState(AUTO_TURN_LEFT);
                }
            }
            break;
        }

        case AUTO_IDLE:
        default:
        {
            stopCar();
            break;
        }
    }
}

//==================================================
// Serial Handshake
//==================================================

void processHandshake()
{
    while(!handshakeComplete)
    {
        if(!Serial.available())
        {
            stopCar();
            return;
        }

        char command = Serial.read();

        if(command == '\n' || command == '\r')
        {
            continue;
        }

        if(command == HANDSHAKE_HELLO)
        {
            Serial.write(HANDSHAKE_ACK);
        }
        else if(command == HANDSHAKE_READY)
        {
            handshakeComplete = true;

            Serial.println();
            Serial.println("Arduino handshake complete.");
        }
    }
}

//==================================================
// Keyboard Command Processing
//==================================================

void processKeyboardCommand(char command)
{
    command = toupper(command);

    lastControlCommandTime = millis();

    switch(command)
    {
        case 'W':

            if(turnAroundActive)
            {
                turnAroundActive = false;
            }

            keyboardControlActive = true;

            if(
                ultrasonicDistanceCm > 0
                && ultrasonicDistanceCm < STOP_DISTANCE_CM
            )
            {
                stopCar();
                return;
            }

            moveForward();
            break;

        case 'S':

            if(turnAroundActive)
            {
                turnAroundActive = false;
            }

            keyboardControlActive = true;
            moveBackward();
            break;

        case 'A':

            if(turnAroundActive)
            {
                turnAroundActive = false;
            }

            keyboardControlActive = true;

            if(
                ultrasonicDistanceCm > 0
                && ultrasonicDistanceCm < STOP_DISTANCE_CM
            )
            {
                stopCar();
                return;
            }

            steerLeftLarge();
            break;

        case 'D':

            if(turnAroundActive)
            {
                turnAroundActive = false;
            }

            keyboardControlActive = true;

            if(
                ultrasonicDistanceCm > 0
                && ultrasonicDistanceCm < STOP_DISTANCE_CM
            )
            {
                stopCar();
                return;
            }

            steerRightLarge();
            break;

        // çµèPythonç¨åºæç©ºæ ¼è½¬æ¢ä¸ºU
        case 'U':

            startTurnAround();
            break;

        case 'X':
	    if(autonomousMode)
	    {
		stopAutonomousMode();
	    }
	    else
	    {
		stopCar();
	    }

	    keyboardControlActive = false;
	    break;

	case 'O':
	    startAutonomousMode();
	    break;
	case 'M':
	    stopAutonomousMode();
            keyboardControlActive = true;
	    Serial.println("Manual mode started.");
	    break;
    }
}

void readSerialCommand()
{
    while(Serial.available())
    {
        char command = Serial.read();

        if(
            command == '\n'
            || command == '\r'
        )
        
        {
            continue;
        }

        processKeyboardCommand(command);
    }
}

//==================================================
// Keyboard Safety Timeout
//==================================================

void updateKeyboardTimeout()
{
    if(!keyboardControlActive)
    {
        return;
    }

    if(!autonomousMode && 
        millis() - lastControlCommandTime
        > COMMAND_TIMEOUT_MS
    )
    {
        keyboardControlActive = false;

        stopCar();
    }
}

//==================================================
// Tilt Safety
//==================================================
void safetyCheck()
{
    bool unsafeTilt =
        fabs(pitch) > MAX_PITCH_ANGLE
        || fabs(roll) > MAX_ROLL_ANGLE;

    if(unsafeTilt)
    {
        tiltUnsafeCount++;
    }
    else
    {
        tiltUnsafeCount = 0;
        tiltWarningActive = false;
        return;
    }

    // ?????????????
    if(tiltUnsafeCount < TILT_CONFIRM_COUNT)
    {
        return;
    }

    keyboardControlActive = false;
    turnAroundActive = false;
    autonomousMode = false;

    stopCar();
    setAutoState(AUTO_IDLE);

    if(!tiltWarningActive)
    {
        tiltWarningActive = true;

        Serial.println();
        Serial.println("******** TILT WARNING ********");
        Serial.print("Pitch: ");
        Serial.println(pitch, 2);
        Serial.print("Roll: ");
        Serial.println(roll, 2);
        Serial.println("Motors stopped.");
        Serial.println();
    }
}

//==================================================
// Debug Output
//==================================================

void printDebug()
{
    unsigned long leftCount;
    unsigned long rightCount;

    getEncoderCounts(
        leftCount,
        rightCount
    );

    Serial.println("--------------------------------");

    Serial.print("Command: ");
    Serial.println(currentMotionCommand);

    Serial.print("Ultrasonic: ");

    if(ultrasonicDistanceCm < 0)
    {
        Serial.println("No echo");
    }
    else
    {
        Serial.print(ultrasonicDistanceCm, 1);
        Serial.println(" cm");
    }

    Serial.print("Encoder Left: ");
    Serial.print(leftCount);

    Serial.print("    Right: ");
    Serial.println(rightCount);

    Serial.print("Distance Left: ");
    Serial.print(
        pulsesToDistanceCm(leftCount),
        2
    );

    Serial.print(" cm    Right: ");
    Serial.print(
        pulsesToDistanceCm(rightCount),
        2
    );

    Serial.println(" cm");

    Serial.print("Pitch: ");
    Serial.print(pitch, 2);

    Serial.print(" deg    Roll: ");
    Serial.print(roll, 2);

    Serial.println(" deg");

    Serial.print("Temperature: ");
    Serial.print(temperature, 2);
    Serial.println(" C");

    Serial.println();
}

void updateDebug()
{
    if(
        millis() - lastIMUPrint
        < IMU_PRINT_INTERVAL_MS
    )
    {
        return;
    }

    lastIMUPrint = millis();

    printDebug();
}

//==================================================
// IMU Calibration
//==================================================

void calibrateIMU()
{
    Serial.println();
    Serial.println("==================================");
    Serial.println("Calibrating MPU6500...");
    Serial.println("Keep the robot completely still.");
    Serial.println("==================================");

    const int samples = 500;

    long gxSum = 0;
    long gySum = 0;
    long gzSum = 0;

    int validSamples = 0;

    for(int i = 0; i < samples; i++)
    {
        if(readMPU())
        {
            gxSum += gx;
            gySum += gy;
            gzSum += gz;

            validSamples++;
        }

        delay(5);
    }

    if(validSamples == 0)
    {
        Serial.println("IMU calibration failed.");
        return;
    }

    gxBias =
        (gxSum / (float)validSamples)
        / 131.0;

    gyBias =
        (gySum / (float)validSamples)
        / 131.0;

    gzBias =
        (gzSum / (float)validSamples)
        / 131.0;

    imuCalibrated = true;

    Serial.println("Calibration finished.");

    Serial.print("GX Bias = ");
    Serial.println(gxBias, 3);

    Serial.print("GY Bias = ");
    Serial.println(gyBias, 3);

    Serial.print("GZ Bias = ");
    Serial.println(gzBias, 3);

    Serial.println();
}

//==================================================
// Setup
//==================================================

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(10);

    Wire.begin();

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    digitalWrite(TRIG_PIN, LOW);

    pinMode(
        LEFT_ENCODER_PIN,
        INPUT_PULLUP
    );

    pinMode(
        RIGHT_ENCODER_PIN,
        INPUT_PULLUP
    );

    attachInterrupt(
        digitalPinToInterrupt(
            LEFT_ENCODER_PIN
        ),
        leftEncoderTick,
        FALLING
    );

    attachInterrupt(
        digitalPinToInterrupt(
            RIGHT_ENCODER_PIN
        ),
        rightEncoderTick,
        FALLING
    );

    stopCar();

    delay(500);

    writeRegister(0x6B, 0x00);

    delay(100);

    Serial.println();
    Serial.println("====================================");
    Serial.println("RobotCar V4 Keyboard Control");
    Serial.println("MPU6500 + Encoders + HC-SR04");
    Serial.println("====================================");

    Serial.print("WHO_AM_I = 0x");
    Serial.println(
        readRegister(0x75),
        HEX
    );

    calibrateIMU();

    ultrasonicDistanceCm =
        readUltrasonicDistanceCm();

    Serial.println("System ready.");
    Serial.println();
    Serial.println("Controls:");
    Serial.println("Hold W = Forward");
    Serial.println("Hold S = Backward");
    Serial.println("Hold A = Large left turn");
    Serial.println("Hold D = Large right turn");
    Serial.println("Space  = 180-degree turn");
    Serial.println("Esc    = Stop and exit");
    Serial.println();
}

//==================================================
// Main Loop
//==================================================

void loop()
{
    if(!handshakeComplete)
    {
        processHandshake();
        return;
    }
    updateIMU();

    updateUltrasonic();

    readSerialCommand();

    updateTurnAround();
    updateAutonomousMode();
    updateKeyboardTimeout();

    safetyCheck();

    updateDebug();
}