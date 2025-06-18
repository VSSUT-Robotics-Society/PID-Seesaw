/* PID Structure - To be used with multiple I/O Sensors
   TODO : <4> Debugging Options   */

// Debugging Options;
#define DEBUG_TUNING 01
#define DEBUG_PID 01

// Libraries Defined Here;
#include <NoDelay.h>
#include <Servo.h>
#include <SharpIR.h>

// Pins Defined Here;
#define servoPin 11
#define sensorPin A5
// #define echoPin1 4
// #define trigPin1 3
// #define echoPin2 6
// #define trigPin2 5

// Constants Defined Here;
#define Kp 0.1        // Proportional Gain
#define Ki 0.0        // Integral Gain
#define Kd 0.0        // Derivative Gain
#define timeStep 50  // Time to wait before executing PID
#define setPoint 30.0 // Value in terms of Input to Achieve through PID
#define minOut -30.0  // Maximum value achievable by P in Negative Direction wrt SetPoint
#define maxOut 30.0   // Maximum value achievable by P in Positive Direction wrt SetPoint

// Variables Defined Here;
double P, I, D, lastP; // To be accessed by calcPID() only
double Val, lastIn;

// Functions Defined Here;
double getInput(double = 0.0), setOutput(double), calcPID(double);

// Obj Constructors Here;
noDelay delayPID(timeStep); // To be accessed by calcPID() only
noDelay timeOut(3000, false);
Servo servo;

void setup()
{
   servo.attach(servoPin);
  //  pinMode(trigPin1, OUTPUT);
  //  pinMode(echoPin1, INPUT);
   // pinMode(trigPin2, OUTPUT);
   // pinMode(echoPin2, INPUT);
   // pinMode(7, OUTPUT);
   // digitalWrite(7, HIGH);
#if DEBUG_TUNING || DEBUG_PID
   Serial.begin(115200);
   while (!Serial)
      ;
#endif
}

void loop()
{
   if (delayPID.update())
   {
      delayPID.start();
      byte in = getInput();
      Val = setOutput(calcPID(in));

#if DEBUG_TUNING
      Serial.print(F(">SetPoint:"));
      Serial.println(setPoint);
      Serial.print(F(">Input:"));
      Serial.println(in);
#endif
#if DEBUG_PID
      Serial.print(F(">P:"));
      Serial.println(P);
      Serial.print(F(">I:"));
      Serial.println(I);
      Serial.print(F(">D:"));
      Serial.println(D);
      Serial.print(F(">Output:"));
      Serial.println(Val);
#endif
   }
}

double getInput(double Input = 0.0)
{
   double In, LIn, RIn;
   if (Input == 0.0)
   {
      // TODO: Code for processing Input from Sensors;
      // digitalWrite(trigPin1, LOW);
      // delayMicroseconds(2);
      // digitalWrite(trigPin1, HIGH);
      // delayMicroseconds(10);
      // digitalWrite(trigPin1, LOW);
      // unsigned long duration = pulseIn(echoPin1, HIGH);
      // RIn = (duration * 0.017);
      // if (RIn <= 0 || RIn > 60)
      // {
      //    digitalWrite(trigPin2, LOW);
      //    delayMicroseconds(2);
      //    digitalWrite(trigPin2, HIGH);
      //    delayMicroseconds(10);
      //    digitalWrite(trigPin2, LOW);
      //    duration = pulseIn(echoPin2, HIGH);
      //    LIn = 60 - (duration * 0.017);
      // if ((LIn <= 0 || LIn > 68) && (RIn <= 0 || RIn > 68))
      // {
      //    while (!timeOut.isEnabled())
      //       timeOut.start();
      //    if (timeOut.update())
      //    {
      //       Serial.println(F("No Input!"));
      //       return setPoint;
      //    }
      //    else
      //    {
      //       In = lastIn;
      //    }
      // }
      // }
      // else
      // {
      //    timeOut.stop();
         // In = (LIn + RIn) / 2;
         In = max(LIn, RIn);
         abs(In - lastIn) > 15 ? In = lastIn : In;
         lastIn = In;
      }
      // delay(50);
   }
   else
   {
      // TODO: Code for conversion of Output to Input Params;
      Serial.println(F("Remove Params!"));
   }
   return In;
}

double calcPID(double in)
{
   // if (delayPID.update())
   // {
   // delayPID.start();
   P = setPoint - in;
   P == 0 ? I = 0 : I += P;
   I = constrain(I, -1000, 1000);
   D = P - lastP;
   lastP = P;
   double PID = P * Kp + I * Ki + D * Kd;
   PID = constrain(PID, minOut, maxOut);
   return -PID;
   // }
}

double setOutput(double Output)
{
   // TODO: Code for Processing Param and mapping it to the Output;
   double Out = map(Output, minOut, maxOut, 90 - 10, 90 + 10);
   servo.write(Out);
   return Out;
}
