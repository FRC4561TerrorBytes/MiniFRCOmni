// Summary - Field oriented/PID heading works pretty well

#include "Wire.h"
#include <AlfredoConnect.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>
#include <Alfredo_NoU2.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

BluetoothSerial bluetooth;

#define MPU_ADDR 0x68  // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
#define I2C_SDA 4
#define I2C_SCL 5
//static const int LED_BUILTIN = 2;

static const int MOTORAPLUS  = 2;
static const int MOTORAMINUS = 25;
static const int MOTORBPLUS  = 4;
static const int MOTORBMINUS = 5;


MPU6050 mpu(MPU_ADDR);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container

volatile double gyro_x = 0;
volatile double gyro_y = 0;
volatile double gyro_z = 0;

double last_gyro_x = 0;
double last_gyro_y = 0;
double last_gyro_z = 0;

NoU_Motor frontLeftMotor(1);
NoU_Motor frontRightMotor(2);
NoU_Motor rearLeftMotor(4);
NoU_Motor rearRightMotor(3);
NoU_Motor trackMotor(5);
//NoU_Motor intakeMotor(6);

NoU_Servo servoR(1);
NoU_Servo servoL(2);

NoU_Servo flywheel(4);

NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

float xVelocity, yVelocity, rotation, intakeAngle;
long lastTimePacketReceived = 0;

double TurnSetpoint;
double TurnInput;
double TurnOutput;
PID turn_pid(&TurnInput, &TurnOutput, &TurnSetpoint, 0.1, 0.01, 0.01, DIRECT);

void mpu_setup()
{
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-14.5);  //good enough... = 70 = -0.66, 68 = 0.38
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("LowTide Staring...");
  bluetooth.begin("LowTide");
  AlfredoConnect.begin(bluetooth);
  bluetooth.println("LowTide started");

  Wire.begin();
  Wire.setClock(400000);

  frontLeftMotor.setInverted(false);
  frontRightMotor.setInverted(true);
  rearLeftMotor.setInverted(false);
  rearRightMotor.setInverted(true);

  frontLeftMotor.set(0);
  frontRightMotor.set(0);
  rearLeftMotor.set(0);
  rearRightMotor.set(0);

  trackMotor.set(0);
  //intakeMotor.set(0);

  servoR.write(0);
  servoL.write(0);
  flywheel.write(90);

  //pinMode(LED_BUILTIN, OUTPUT);

  pinMode(MOTORAPLUS, OUTPUT);
  pinMode(MOTORAMINUS, OUTPUT);
  pinMode(MOTORBPLUS, OUTPUT);
  pinMode(MOTORBMINUS, OUTPUT);

  digitalWrite(MOTORAPLUS, LOW);
  digitalWrite(MOTORAMINUS, LOW);
  digitalWrite(MOTORBPLUS, LOW);
  digitalWrite(MOTORBMINUS, LOW);

  mpu_setup();

  turn_pid.SetMode(AUTOMATIC);
  turn_pid.SetOutputLimits(-1.0, 1.0);
  turn_pid.SetSampleTime(1);

  // No need to mess with this code
  RSL::initialize();
  RSL::setState(RSL_ENABLED);

  Serial.println("LowTide init complete");
}

float getHeading() {
  // reset interrupt flag and get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);

    double temp_gyro_x = euler[1] * 180 / M_PI;;
    double temp_gyro_y = euler[2] * 180 / M_PI;;
    double temp_gyro_z = euler[0] * 180 / M_PI;;

    double diff;

    // Handle wrap for gyro x, y, z
    if (abs(last_gyro_x - temp_gyro_x) > 30) {
      if (temp_gyro_x > last_gyro_x) {
        diff = (long)(((360 - temp_gyro_x) + last_gyro_x) * -1);
      } else {
        diff = (long)((360 - last_gyro_x) + temp_gyro_x);
      }

      gyro_x = gyro_x + diff;
    } else {
      gyro_x +=  temp_gyro_x - last_gyro_x;
    }

    if (abs(last_gyro_y - temp_gyro_y) > 30) {
      if (temp_gyro_y > last_gyro_y) {
        diff = (long)(((360 - temp_gyro_y) + last_gyro_y) * -1);
      } else {
        diff = (long)((360 - last_gyro_y) + temp_gyro_y);
      }

      gyro_y = gyro_y + diff;
    } else {
      gyro_y +=  temp_gyro_y - last_gyro_y;
    }

    if (abs(last_gyro_z - temp_gyro_z) > 30) {
      if (temp_gyro_z > last_gyro_z) {
        diff = (long)(((360 - temp_gyro_z) + last_gyro_z) * -1);
      } else {
        diff = (long)((360 - last_gyro_z) + temp_gyro_z);
      }

      gyro_z = gyro_z + diff;
    } else {
      gyro_z +=  temp_gyro_z - last_gyro_z;
    }

    last_gyro_x = temp_gyro_x;
    last_gyro_y = temp_gyro_y;
    last_gyro_z = temp_gyro_z;
  }

  float angle = (float) gyro_z;
  return angle;
}

unsigned long  currentPrintInterval = 0;

const float maxDrive = 1.7;
const float maxTurn = 0.75;
float headingOffset = 0;

float headingSetpoint = 0;
boolean resetSetpoint = false;

boolean autoHasStarted = false;
boolean autoIsDone = false;
unsigned long autoBeganAt = 0;

boolean doPidOk = false;

void loop() {
  //float heading = 0;
  float heading = getHeading();

  double errorSignal = 0.0;

  if (AlfredoConnect.getGamepadCount() >= 1) {

    xVelocity   = AlfredoConnect.getAxis(0, 0);
    yVelocity   = AlfredoConnect.getAxis(0, 1) * -1; //invert Y because airplaine joystick maps differently (forward is nagative)
    rotation    =  AlfredoConnect.getAxis(0, 2);
    intakeAngle =  AlfredoConnect.getAxis(0, 3);


    // Drive the DRV8833 first port (One speed only)
    if (AlfredoConnect.buttonHeld(0, 2)) {
      //intakeMotor.set(1.0);
      digitalWrite(MOTORBPLUS, HIGH);
      digitalWrite(MOTORBMINUS, LOW);
    } else if (AlfredoConnect.buttonHeld(0, 1)) {
      //intakeMotor.set(-1.0)g;
      digitalWrite(MOTORBPLUS, LOW);
      digitalWrite(MOTORBMINUS, HIGH);
    } else {
      //intakeMotor.set(0);
      digitalWrite(MOTORBPLUS, LOW);
      digitalWrite(MOTORBMINUS, LOW);
    }

    if (AlfredoConnect.buttonHeld(0, 3)) {
      trackMotor.set(1.0);
    } else if (AlfredoConnect.buttonHeld(0, 0)) {
      trackMotor.set(-1.0);
    } else {
      trackMotor.set(0);
    }

    if (AlfredoConnect.buttonHeld(0, 7)) {
      flywheel.write(179);
    } else {
      flywheel.write(90);
    }

    // Drive the DRV8833 first port (One speed, one direction only)
    if (AlfredoConnect.buttonHeld(0, 8)) {
      //intakeMotor.set(1.0);
      digitalWrite(MOTORAPLUS, HIGH);
      digitalWrite(MOTORAMINUS, HIGH);
    } else {
      //intakeMotor.set(0);
      digitalWrite(MOTORAPLUS, LOW);
      digitalWrite(MOTORAMINUS, LOW);
    }

    // Zero out the gyro
    if (AlfredoConnect.buttonHeld(0, 9)) {
      headingOffset = heading * -1;
      TurnSetpoint = 0;
    }
    heading = heading + headingOffset;
  }

  if (abs(rotation) < 0.5) {

    if (resetSetpoint == true) {
      bluetooth.print("P");
      TurnSetpoint = heading;
      resetSetpoint = false;
    }

    TurnInput = heading;
    turn_pid.Compute();

    //fiddle with error signal (could/should be done in PID algorithm)
    if (TurnOutput > 0.02) {
      errorSignal = TurnOutput;// *-1.0;
    } else if (TurnOutput < -0.02) {
      errorSignal = TurnOutput;// *-1.0;
    }

    if (doPidOk) {
      rotation = errorSignal;
    }
  } else {
    resetSetpoint = true;
  }


  float vector = sqrt(xVelocity * xVelocity + yVelocity * yVelocity);
  float angle = round( atan2 (xVelocity, yVelocity) * 180 / 3.14159265 );

  float adjustedAngle = angle - heading;

  float speedFL = cos(((adjustedAngle - 45) * 71) / 4068.0) * vector * 1.4 + (rotation * maxTurn);
  float speedFR = cos(((adjustedAngle + 45) * 71) / 4068.0) * vector * 1.4 - (rotation * maxTurn);
  float speedRL = cos(((adjustedAngle + 45) * 71) / 4068.0) * vector * 1.4 + (rotation * maxTurn);
  float speedRR = cos(((adjustedAngle - 45) * 71) / 4068.0) * vector * 1.4 - (rotation * maxTurn);

  // DEBUG Stuff
  // Occasionally print sensor values
  unsigned long timeref = millis();
  if (timeref / 333 > currentPrintInterval) {
    //    bluetooth.print(" XV:");
    //    bluetooth.print(xVelocity);
    //    bluetooth.print(" YV:");
    //    bluetooth.print(yVelocity);
    bluetooth.print("V:");
    bluetooth.print(vector);
    bluetooth.print(" A:");
    bluetooth.print(angle);
    //    bluetooth.print(" FL:");
    //    bluetooth.print(speedFL);
    //    bluetooth.print(" FR:");
    //    bluetooth.print(speedFR);
    //    bluetooth.print(" RL:");
    //    bluetooth.print(speedRL);
    //    bluetooth.print(" RR:");
    //    bluetooth.print(speedRR);
    //    bluetooth.print(" RO:");
    //    bluetooth.print(rotation);
    bluetooth.print(" ERR:");
    bluetooth.print(errorSignal);
    bluetooth.print(" Heading:");
    bluetooth.println(heading);
    currentPrintInterval++;
  }

  if (AlfredoConnect.keyHeld(Key::A) && autoHasStarted == false) {
    autoHasStarted = true;
    autoBeganAt = millis();
  } else if (autoHasStarted && (millis() - autoBeganAt > 15000)) {
    doPidOk = true;
  } else if (autoHasStarted && (millis() - autoBeganAt < 5000)) {
    speedFL = 1.0;
    speedFR = 1.0;
    speedRL = 1.0;
    speedRR = 1.0;
  }

  // Before gyro --> drivetrain.holonomicDrive(xVelocity, yVelocity, rotation);
  frontLeftMotor.set(speedFL);
  frontRightMotor.set(speedFR);
  rearLeftMotor.set(speedRL);
  rearRightMotor.set(speedRR);

  //************************************
  //Non-omni wheel stuff below this line
  //servo.write((servoAxis + 1) * 90);
  float finalIntakeAngle = (intakeAngle + 1.0) * 90;
  servoR.write(finalIntakeAngle);
  servoL.write(180.0 - finalIntakeAngle);

  // No need to mess with this code
  AlfredoConnect.update();
  RSL::update();
}
