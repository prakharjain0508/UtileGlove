const int right_flex = A0;
const int right_force  = A1;
int rfi;
int rfr;
int rgy;
int rfi_straight = 775;
int rfi_curled = 1023;
const int rfr_low      = 1023; 
const int rfr_high     = 0;
int rgy_mid;
      int rgy_low;
      int rgy_high;

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  
#define LED_PIN 13 
bool blinkState = false;

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
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
    mpuInterrupt = true;
}

void calibrate_gyro(int Gyro_avg)
{
 int sumy = 0;
 for (int i = 0; i<20; i++)
  {
    sumy = sumy + (Gyro_avg);
  }
  delay(100);
  double avgy = round(sumy/20);
  rgy_mid = avgy;
  rgy_low = 0;
  rgy_high = avgy*2;
}
void print_pretty(int val)
{
  int new_val;
  //All Numbers must range from zero to 255.
  if      (val<0)   new_val = 0;
  else if (val>255) new_val = 255;
  else    new_val = val;
  //Add Leading zeros
  if (new_val < 10)
  {
    Serial.print(0);
    Serial.print(0);
    Serial.print(new_val);
  }
  else if (new_val >= 10 && new_val <100)
  {
    Serial.print(0);
    Serial.print(new_val);
  }
  else Serial.print(new_val);
}

void setup() 
{
   pinMode(right_flex, INPUT);
   pinMode (right_force,  INPUT);
   Serial.begin(9600);
   
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif
    Serial1.begin(9600);
    while (!Serial1); // wait for Leonardo enumeration, others continue immediately
    // initialize device
    Serial1.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    
    // verify connection
    Serial1.println(F("Testing device connections..."));
    Serial1.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial1.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {
        // turn on the DMP, now that it's ready
        Serial1.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial1.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial1.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else 
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial1.print(F("DMP Initialization failed (code "));
        Serial1.print(devStatus);
        Serial1.println(F(")"));
    }
    calibrate_gyro (((ypr[2] * 180/M_PI) + 90));
}


void loop() 
{
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        //wait here
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial1.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
         //   Serial.print("roll\t");
           // Serial.println((ypr[2] * 180/M_PI)+90);
           // delay(10);
        #endif
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

  
  rfi = analogRead(right_flex);
  rgy = ((ypr[2] * 180/M_PI)+90);
  int sumr = 0;
  for (int i=0; i<10; i++)
  {
    sumr = sumr + analogRead(right_force);   //Right Force Ring
  }
  rfr = round(sumr/10);  //Right Force Ring
  
  int RFI = map(rfi, rfi_curled, rfi_straight, 255, 0);
  int RFR = map(rfr, rfr_low,      rfr_high,   0, 255);
  int RGY = map(rgy, 0,  180,   0, 255);
  Serial.print('.');
  print_pretty(RFI);
  print_pretty(RFR);
  print_pretty(RGY);
  delay(10);
  while (Serial.read() != 'a');
  
}
