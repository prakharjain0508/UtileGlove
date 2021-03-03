char buffer[9];
const int buffer_len = 9;
boolean characterSent = false;
boolean leftRight = false;
int accel = 0;
int turn = 0;
int reverse = 0;
const int flex = A0;
const int force = A1;
const byte D1 = 5;
const byte D2 = 3;
const byte D3 = 10;
const byte D4 = 9;
int FlexValue=0;
int ForceValue = 255;
int FlexValueMax = 100;
int FlexValueMin = 0;
int FlexValueMid = 50;
int FlexValueMidUpper = FlexValueMid - 10; 

//DC motor variables
byte motorSpeed = 0;
byte motorSpeedMax = 255;
byte motorSpeedMin = 0; //set to smallest value that make motor move (default 0)
                         // DC motor that I use start to move at 90 pwm value


void clearBuffer()
{
  for(int i = 0; i < buffer_len; i++)
  {
    buffer[i] = 0;
  }
}


void setup() 
{
 
 pinMode(flex, INPUT);
 pinMode(force, INPUT);
 pinMode(D1, OUTPUT);
 pinMode(D2, OUTPUT);
 pinMode(D3, OUTPUT);
 pinMode(D4, OUTPUT);
 Serial.begin(9600);
 clearBuffer();
 Serial.flush();
}

void loop() 
{
  if(!characterSent)
  {
    Serial.print('a'); 
    characterSent = true;
  }
  
  //Get commands
  if(Serial.available())
  {
    characterSent = false;
    char check = Serial.read();
  
    if(check == '.')
    { 
      for (int i=0; i <buffer_len; i++)
      {
        buffer[i] = Serial.read();
        delay(10);
      }
      
      char acceleration[3] =  { buffer[0], buffer[1], buffer[2] };
      char reverseAccel[3] =  { buffer[3], buffer[4], buffer[5] };
      char steering[3] = { buffer[6], buffer[7], buffer[8] };
      accel        = 100*(acceleration[0] - '0') + 10*(acceleration[1]-'0') + (acceleration[2]-'0');
      reverse      = 100*(reverseAccel[0] - '0') + 10*(reverseAccel[1]-'0') + (reverseAccel[2]-'0');
      turn         = 100*(steering[0] - '0') + 10*(steering[1]-'0') + (steering[2]-'0');
      if(accel > FlexValueMidUpper && reverse < 50) //Forward
    {
        motorSpeed = map(accel, FlexValueMax, FlexValueMidUpper, motorSpeedMin, motorSpeedMax);
        MotorForward(motorSpeed);
    }
    
    else if(accel > FlexValueMidUpper && reverse > 50) //Backward
    {
       motorSpeed = map(accel, FlexValueMidUpper, FlexValueMax, motorSpeedMin, motorSpeedMax);
       MotorBackward(motorSpeed);
    }
    else if(turn > 178) //Right
    {
        motorSpeed = map(turn, 255, 178, motorSpeedMin, motorSpeedMax);
        MotorRight(motorSpeed);
    }
    else if(turn < 100)  //Left
    {
        motorSpeed = map(turn, 78, 0, motorSpeedMin, motorSpeedMax);
        MotorLeft(motorSpeed);
    }
    else 
    {
       MotorStop(); 
    }
  }
 }
   
}
 void MotorForward( byte spd)
{
    digitalWrite(D2, HIGH);
    analogWrite(D1, spd);
    digitalWrite(D3, HIGH);
    analogWrite(D4, spd);
   
}
void MotorBackward( byte SpdY)
{
    digitalWrite(D2, LOW);
    analogWrite(D1, SpdY); 
    digitalWrite(D3, LOW);
    analogWrite(D4, SpdY);
}
void MotorLeft( byte spdL)
{
    digitalWrite(D2, HIGH);
    analogWrite(D1, spdL);
    digitalWrite(D4, HIGH);
    analogWrite(D3, spdL);
}

void MotorRight( byte SpdR)
{
    digitalWrite(D2, LOW);
    analogWrite(D1, SpdR); 
    digitalWrite(D4, LOW);
    analogWrite(D3, SpdR);
}
void MotorStop()
{
    analogWrite(D1, LOW);   
    digitalWrite(D2, LOW);
    analogWrite(D3, LOW);   
    digitalWrite(D4, LOW);
}

    
