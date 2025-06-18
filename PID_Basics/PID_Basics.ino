String readString;

const int Kp, Ki, Kd, setPoint = 512, minOut = 0, maxOut = 180;
int P, I, D, lastP, Input, Output, PIDValue;

void setup()
{
   Serial.begin(115200);
   readString.reserve(1023);
   while (!Serial)
      ;
   Serial.println(F("Input Range: 0 - 1023"));
}

void loop()
{
   while (!Serial.available())
      ;
   readString = Serial.readString();
   readString.trim();
   if (readString == "0" || readString.toFloat() != 0)
   {
      Input = readString.toFloat();
   }
   P = setPoint - Input;
   I += P;
   D = P - lastP;
   lastP = P;
   PIDValue = Kp * P + Ki * I + Kd * D;
   Output = constrain(PIDValue, minOut, maxOut);
   Serial.println((String)"Input: " + Input);
   Serial.println((String)"P: " + P + " I: " + I + " D: " + D + " PID: " + PIDValue);
   Serial.println((String)"Output: " + Output);
}