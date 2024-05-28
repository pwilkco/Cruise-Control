//version.25.05.24


#include <Arduino.h>
#include <Arduino_BuiltIn.h>
#include <Servo.h>
#include <arduino-timer.h>
#include <PID_v1.h>

#define BUTTON_PINon 4
#define BUTTON_PINdeccel 7
#define BUTTON_PINresume 8

 
double Setpoint, Input, Output;
double Kp = 0.3, Ki = 0.2, Kd = 0.35;
double aggKp = 0.303, aggKi = 0.22, aggKd = 0.44;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Timer<2, millis> MyTimer;
Servo servo1;
int servoPin = 11;
 int buttonONon = 0;
 int buttonONoff = 0;
 int Start = 0;
 int on = 1;
 int OnOff;
 int resume1 = 0;
 int FirstOn = 0;
 int isON;
 int Count = 0;
 int StartCount;//where we move the servo to
 int Hz, Test,ServoDeg,ServoDeg1;
 int UpBuffer, DnBuffer;
 int LowestSpeed, HighestSpeed;//cruise on  speed limits
 int SpeedLoop = 0;
 int RoadSpeed = 0;
 int SetSpeed = 0;
 int ButtonBounce = 0;//on/off button debounce
 double NewCount, Counter;
 

 const byte interruptPin = 2; //ABSsensor in
 const byte interruptPin1 = 3;//brake in
 volatile byte state = LOW;
 unsigned long NewMillis = 0;

 
void setup() {
 
 pinMode(BUTTON_PINon, INPUT_PULLUP);
 pinMode(BUTTON_PINresume, INPUT_PULLUP);
 pinMode(BUTTON_PINdeccel, INPUT_PULLUP);

 
 pinMode(interruptPin, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(interruptPin), ABSsensor, FALLING);
 pinMode(interruptPin1, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(interruptPin1), BRAKE, FALLING);
 Serial.begin(9600);
 servo1.attach(11);
 servo1.write(0);//move servo to 0

 Counter = 10;
 LowestSpeed = 60;
 HighestSpeed = 140;
 MyTimer.every(1000, AdjustSpeed);
 MyTimer.every(333, CruiseStatus);
 myPID.SetOutputLimits(-9, 9);
   
 }

bool AdjustSpeed(void*)
{
    if (isON == 1)
    {
            Setpoint = SetSpeed;//SetSpeed is desired road speed
            Input = RoadSpeed;//from ABS rotor Hz
            Output = 0;
            SpeedLoop++;

            double gap = abs(Setpoint - Input); //distance away from setpoint
            if ((gap <= 9) && (SpeedLoop > 1)) //at 114kph is about 3kph error 
            { 
                //we're close to setpoint, adjust speed every 2000ms
                myPID.SetTunings(Kp, Ki, Kd);
                SpeedLoop = 0;
            }
            else
            {
               
                if ((gap > 9) && (SpeedLoop == 1))
                {
                   //we're far from setpoint, adjust speed every 1000ms
                    // & use more aggressive tunings
                    myPID.SetTunings(aggKp, aggKi, aggKd);
                    SpeedLoop = 0;
                }
            }

            myPID.Compute();
            ServoDeg = Output;
                
                                                       
            if (RoadSpeed < SetSpeed)
            {
                if (UpBuffer == 0)
                {
                    DnBuffer = 0;
                }
                                       
                UpBuffer++;
               
                switch (ServoDeg)
                {

                case 0:

                    break;

                case 1:
                                                          
                   
                    Serial.print(" +25 ");
                    Count++;
                    servo1.write(Count);
                    delay(250);
                    Count--;
                    servo1.write(Count);
                    break;

                case 2:
                    
                    Serial.print(" +50 ");
                    Count++;
                    servo1.write(Count);
                    delay(500);
                    Count--;
                    servo1.write(Count);
                    break;

                case 3:
                   
                    Serial.print(" +75 ");
                    Count++;
                    servo1.write(Count);
                    delay(750);
                    Count--;
                    servo1.write(Count);
                    break;


                case 4:

                    Serial.print(" +1000 ");
                    Count++;
                    servo1.write(Count);
                    delay(1000);
                    Count--;
                    servo1.write(Count);
                    break;

                default:

                    break;


                }
               
                if (ServoDeg > 4)//if PID output is => 5 move servo 1 degree 
                {
                   
                    ServoDeg = ServoDeg - 4;
                    Count = Count + ServoDeg;
                    servo1.write(Count);
                }


                Serial.print("Up");
                Serial.print(" ");
                Serial.print(UpBuffer);
                Serial.print(" ");
            }

            if (RoadSpeed > SetSpeed)
            {
                
            
                if (DnBuffer == 0)
                {

                    UpBuffer = 0;
                   

                }

                        
                DnBuffer++;

                

                switch (ServoDeg)
                {

                case 0:

                    break;

                case -1:
                   
                    Serial.print(" -25 ");
                    Count--;
                    if (Count < 0)
                    {
                        Count = 0;
                    }
                    servo1.write(Count);
                    delay(250);
                    Count++;
                    servo1.write(Count);
                    break;

                case -2:
                  
                    Serial.print(" -50 ");
                    Count--;
                    if (Count < 0)
                    {
                        Count = 0;
                    }
                    servo1.write(Count);
                    delay(500);
                    Count++;
                    servo1.write(Count);
                    break;

                case -3:
                   
                    Serial.print(" -75 ");
                    Count--;
                    if (Count < 0)
                    {
                        Count = 0;
                    }
                    servo1.write(Count);
                    delay(750);
                    Count++;
                    servo1.write(Count);
                    break;


                case -4:

                    Serial.print(" -1000 ");
                    Count--;
                    if (Count < 0)
                    {
                        Count = 0;
                    }
                    servo1.write(Count);
                    delay(1000);
                    Count++;
                    servo1.write(Count);
                    break;

                default:


                    break;


                }

               

                if (ServoDeg < -4)
                {
                   
                    ServoDeg = ServoDeg + 4;
                    ServoDeg = abs(ServoDeg);
                    Count = Count - ServoDeg;

                    if (Count < 0)
                    {
                        Count = 0;
                    }

                    servo1.write(Count);
                }


                Serial.print("Dn");
                Serial.print(" ");
                Serial.print(DnBuffer);
                Serial.print(" ");

            }
                                 

            Serial.print(ServoDeg);
            Serial.print(" ");
            Serial.print(Count);
            Serial.print(" ");
            Serial.print(SetSpeed);
            Serial.print(" ");
            Serial.print(RoadSpeed);
            Serial.println();
             
        


    }

        return true; // repeat? true
    }

bool CruiseStatus(void*)
{
   
   
    OnOff = digitalRead(BUTTON_PINon);
    int resume = digitalRead(BUTTON_PINresume);
    int deccel = digitalRead(BUTTON_PINdeccel);

    

    if (OnOff == 0 and buttonONoff == 1)
    {
        if (on == 1)
        {

            Start = 1;
            on = 0;
           

            if (resume1 == 1)
            {
                Start = 0;
            }

            if (Start == 1)
            {
                Serial.println("On");
            }

        }
        else
        {
            on = 1;
            Start = 0;
            isON = 0;
            Serial.println("OFF 1");
           
        }
    }



    if (Start == 1)
    {



        if (isON == 0)
        {
            //initiate cruise - 

            
            isON = 1;
            NewCount = (RoadSpeed * .072);//we calculate where the servo should be for the given roadspeed
            Count = int(NewCount);
            servo1.write(Count);
            UpBuffer = 0;
            DnBuffer = 0;
            Serial.println("  firstON ");
            SetSpeed = RoadSpeed;
            myPID.SetMode(AUTOMATIC);
           

        }



    }

    if (isON == 1)
    {

       

    }

    if (deccel == 0 and isON == 1)
    {

        Serial.print("  deccel ");

        SetSpeed = SetSpeed - 4;
        Serial.print(SetSpeed);


    }
    if (resume == 0)
    {
        if (resume1 == 0 and Start == 0)
        {
            isON = 1;
            resume1 = 1;

            servo1.write(Count);

            Serial.println("  resume ");
            Serial.println(Count);

        }
        else
        {
            SetSpeed = SetSpeed + 4;
            Serial.println("  accel ");

        }

        Start = 1;
    }
    if (Start == 0)
    {

        Serial.print("  OFF ");
        resume1 = 0;
        isON = 0;
        servo1.write(0);//move servo to 0
        myPID.SetMode(MANUAL);
    }

    if (OnOff == 0 and resume == 0)
    {

        Test = 1;
        Serial.println("Test");

    }

    
    buttonONoff = OnOff;


    return true; // repeat? true
}


void ABSsensor()
{
        
    Hz++;
    if (millis() - NewMillis >= 500)
    {
        NewMillis = millis();
        RoadSpeed = Hz;
        Hz = 0;

    }
    
   
         
}
void BRAKE()
{
    Serial.println(" BRAKE ");
    buttonONoff = 1;
    on = 1;
    Start = 0;
    isON = 0;
    servo1.write(0);//move servo to 0
}

void loop() 
{
       
    MyTimer.tick();
    
}
