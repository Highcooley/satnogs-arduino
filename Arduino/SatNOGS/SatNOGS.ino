#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <AccelStepper.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <LSM303.h>
#include <SoftwareSerial.h>
/* The compass declination lookup table is not yet working properly*/
//#include <AP_Declination.h> 

#define DIR_AZ 18 //PIN for Azimuth Direction
#define STEP_AZ 10 //PIN for Azimuth Steps
#define DIR_EL 6 //PIN for Elevation Direction
#define STEP_EL 7 //PIN for Elevation Steps

#define MS1 9 //PIN for step size
#define EN 8 //PIN for Enable or Disable Stepper Motors

#define SPR 200 //Step Per Revolution
#define RATIO 60 //Gear ratio
#define T_DELAY 60000 //Time to disable the motors in millisecond

#define HOME_AZ 4 //Homing switch for Azimuth
#define HOME_EL 5 //Homing switch for Elevation
/*The MAX_ANGLE depends of ANGLE_SCANNING_MULT and maybe misbehave for large values*/
#define ANGLE_SCANNING_MULT 180 //Angle scanning multiplier
#define MAX_AZ_ANGLE 360 //Maximum Angle of Azimuth for homing scanning
#define MAX_EL_ANGLE 360 //Maximum Angle of Elevation for homing scanning

#define HOME_DELAY 6000 //Time for homing Decceleration in millisecond

/*Global Variables*/
unsigned long t_DIS = 0; //time to disable the Motors
unsigned long fix_age, time, date;
//float flat, flon, falt; //TingyGPS
float Heading, Pitch, Roll;
double dlat, dlon, dalt;

/*Set Angular deviation of your board to actual heading and pitch here*/
float devHeading = 0, devPitch = 0;

/*Enter the magnetical declination of your position here,
 *as long as declination lookup Table is not working */
float declinationAngle = 1+53/60;  

/*Define a stepper and the pins it will use*/
AccelStepper AZstepper(1, STEP_AZ, DIR_AZ);
AccelStepper ELstepper(1, STEP_EL, DIR_EL);

/*Create GPS instance*/
static const int RXPin = 5, TXPin = 4;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);
TinyGPSCustom pdop(gps, "GPGSA", 15); // $GPGSA sentence, 15th element

/*Create Accelerometer & Compass instance*/
LSM303 compass;

void setup()
{  
  /*Change these to suit your stepper if you want*/
  AZstepper.setMaxSpeed(200);
  AZstepper.setAcceleration(200);
  
  /*Change these to suit your stepper if you want*/
  ELstepper.setMaxSpeed(200);
  ELstepper.setAcceleration(200);
  
  /*Enable Motors*/
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);
  /*Step size*/
  pinMode(MS1, OUTPUT);
  digitalWrite(MS1, LOW); //Full step
  /*Homing switch*/
  pinMode(HOME_AZ, INPUT);
  pinMode(HOME_EL, INPUT);
  /*Serial Communication*/
  Serial.begin(19200);
  Wire.begin();
  gpsSerial.begin(GPSBaud);
  compass.init();
  compass.enableDefault();
  
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
  
  /*Initial GPS Position*/
  StatGpsPos(15);
  
  /*Initial Homing*/
  Homing(deg2step(-ANGLE_SCANNING_MULT), deg2step(-ANGLE_SCANNING_MULT));
}

void loop()
{ 
  /*Define the steps*/
  static int AZstep = 0;
  static int ELstep = 0;
  /*Time Check*/
  if (t_DIS == 0)
    t_DIS = millis();

  /*Disable Motors*/
  if (AZstep == AZstepper.currentPosition() && ELstep == ELstepper.currentPosition() && millis()-t_DIS > T_DELAY)
  {
    digitalWrite(EN, HIGH);
  }
  /*Enable Motors*/
  else
    digitalWrite(EN, LOW);
    
  /*Read the steps from serial*/
  cmd_proc(AZstep, ELstep);
  /*Move the Azimuth & Elevation Motor*/
  stepper_move(AZstep, ELstep);
}

/*GPS static Position retrieval*/
void StatGpsPos(int NumReads)
{
  unsigned long start = millis();
  int ReadCount = 0;
  //float tempflat = 0, tempflon = 0, tempfalt = 0;  //TinyGPS
  double tempdlat = 0, tempdlon = 0, tempdalt = 0;
  do
  {
    //while (gpsSerial.available()) //TinyGPS
    while (gpsSerial.available() > 0) //TinyGPS++
    {
      //int c = gpsSerial.read(); //TinyGPS
      //if (gps.encode(c)) //TinyGPS
      if (gps.encode(gpsSerial.read()) && gps.satellites.value() >= 8 && *pdop.value() < 4)
      {
        //gps.f_get_position(&flat, &flon, &fix_age);  //TinypGPS
        //falt = gps.f_altitude();
        dlat = gps.location.lat();
        dlon = gps.location.lng();
        dalt = gps.altitude.meters();
        //tempflat = tempflat+flat;  //TinyGPS
        //tempflon = tempflon + flon;
        //tempfalt = tempfalt + falt;
        tempdlat += dlat;
        tempdlon += dlon;
        tempdalt += dalt;
        ReadCount += 1;
        
        /*Average position when desired number of reads is accomplished*/
        if (ReadCount >= NumReads)  
        {
          //flat = tempflat/ReadCount;  //TinyGPS
          //flon = tempflon/ReadCount;
          //falt = tempfalt/ReadCount;
          //gps.get_datetime(&date, &time, &fix_age);
          dlat = tempdlat/ReadCount;
          dlon = tempdlon/ReadCount;
          dalt = tempdalt/ReadCount;
          
          /* Compass Declination lookup table not working yet
          //declinationAngle = AP_Declination::get_declination(flat, flon); //TinyGPS
          //declinationAngle = AP_Declination::get_declination(dlat, dlon);
          */
          break;
        }
      }
    }
    delay(10);
  } while (millis() - start < 60000); 
  if (ReadCount != NumReads)
  {
    error(2);
  }
}

/*Get tilt compensated Compass heading*/
void get_TiltHeading(void)
{
  compass.read();
   // Normalize acceleration measurements so they range from 0 to 1
  float accxnorm = compass.a.x/sqrt(compass.a.x*compass.a.x+compass.a.x*compass.a.x+compass.a.x*compass.a.x);
  float accynorm = compass.a.y/sqrt(compass.a.y*compass.a.y+compass.a.y*compass.a.y+compass.a.y*compass.a.y);

  // calculate pitch and roll
  Pitch = asin(-accxnorm);
  Roll = asin(accynorm/cos(Pitch));

  // tilt compensated magnetic sensor measurements
  float magxcomp = compass.m.x*cos(Pitch)+compass.m.z*sin(Pitch);
  float magycomp = compass.m.x*sin(Roll)*sin(Pitch)+compass.m.y*cos(Roll)-compass.m.z*sin(Roll)*cos(Pitch);

  // arctangent of y/x converted to degrees & add declination and deviation
  Heading = 180*atan2(magycomp,magxcomp)/PI+declinationAngle+devHeading;

  if (Heading < 0)
      Heading +=360; 
  Pitch += devPitch;  //add Pitch deviation
  }  


/*Homing Function*/
void Homing(int AZsteps, int ELsteps)
{
  int value_Home_AZ = HIGH;
  int value_Home_EL = HIGH;
  int n_AZ = 1; //Times that AZ angle has changed
  int n_EL = 1; //Times that EL angle has changed
  boolean isHome_AZ = false;
  boolean isHome_EL = false;
  
  AZstepper.moveTo(AZsteps);
  ELstepper.moveTo(ELsteps);
  
  while(isHome_AZ == false || isHome_EL == false)
  {
    value_Home_AZ = digitalRead(HOME_AZ);
    value_Home_EL = digitalRead(HOME_EL);
    /* Change to LOW according to Home sensor */
    if (value_Home_AZ == HIGH)
    {
      AZstepper.moveTo(AZstepper.currentPosition());
      isHome_AZ = true;
    }   
    /* Change to LOW according to Home sensor */
    if (value_Home_EL == HIGH)
    {
      ELstepper.moveTo(ELstepper.currentPosition());
      isHome_EL = true;
    }
    if (AZstepper.distanceToGo() == 0 && !isHome_AZ)
    {
      n_AZ++;
      AZsteps = deg2step(pow(-1,n_AZ)*n_AZ*ANGLE_SCANNING_MULT);
      if (abs(n_AZ*ANGLE_SCANNING_MULT) > MAX_AZ_ANGLE)
      {
        error(0);
        break;
      }
      AZstepper.moveTo(AZsteps);
    } 
    if (ELstepper.distanceToGo() == 0 && !isHome_EL)
    { 
      n_EL++;
      ELsteps = deg2step(pow(-1,n_EL)*n_EL*ANGLE_SCANNING_MULT);
      if (abs(n_EL*ANGLE_SCANNING_MULT) > MAX_EL_ANGLE)
      {
        error(1);
        break;
      }
      ELstepper.moveTo(ELsteps);
    }
    
    AZstepper.run();
    ELstepper.run();
  }
  /*Delay to Deccelerate*/
  long time = millis();  
  while(millis() - time < HOME_DELAY)
  {  
    AZstepper.run();
    ELstepper.run();
  }
  
  /*Read Compass Heading and Compensate Tilt & Declination*/
  get_TiltHeading();
    
  /*Reset the steps*/
  AZstepper.setCurrentPosition(deg2step(Heading));
  ELstepper.setCurrentPosition(deg2step(Pitch)); 
}
 
/*EasyComm 2 Protocol & Calculate the steps*/
void cmd_proc(int &stepAz, int &stepEl)
{
  /*Serial*/
  char buffer[256];
  char incomingByte;
  char *p=buffer;
  char *str;
  int counter=0;
  char data[100];
  
  double angleAz,angleEl;
  
  /*Read from serial*/
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    /* XXX: Get position using custom and test code */
    if (incomingByte == '!')
    {
      /* Get position */
      Serial.print("TM");
      Serial.print(1);
      Serial.print(" ");
      Serial.print("AZ");
      Serial.print(10*step2deg(AZstepper.currentPosition()), 1);
      Serial.print(" ");
      Serial.print("EL");
      Serial.println(10*step2deg(ELstepper.currentPosition()), 1);
    }
    /*new data*/
    else if (incomingByte == '\n')
    {
      p = buffer;
      buffer[counter] = 0;
      if (buffer[0] == 'A' && buffer[1] == 'Z')
      {
        if (buffer[2] == ' ' && buffer[3] == 'E' && buffer[4] == 'L')
        {
          /* Get position */
          Serial.print("AZ");
          Serial.print(step2deg(AZstepper.currentPosition()), 1);
          Serial.print(" ");
          Serial.print("EL");
          Serial.print(step2deg(ELstepper.currentPosition()), 1);
          Serial.println(" ");
        }
        else
        {
          /*Get the absolute value of angle*/
          str = strtok_r(p, " " , &p);
          strncpy(data, str+2, 10);
          angleAz = atof(data);
          /*Calculate the steps*/
          stepAz = deg2step(angleAz);

          /*Get the absolute value of angle*/
          str = strtok_r(p, " " , &p);
          if (str[0] == 'E' && str[1] == 'L')
          {
            strncpy(data, str+2, 10);
            angleEl = atof(data);
            /*Calculate the steps*/
            stepEl = deg2step(angleEl);
          }
        }
      }
      /* Stop Moving */
      else if (buffer[0] == 'S' && buffer[1] == 'A' && buffer[2] == ' ' && buffer[3] == 'S' && buffer[4] == 'E')
      {
        /* Get position */
        Serial.print("AZ");
        Serial.print(step2deg(AZstepper.currentPosition()), 1);
        Serial.print(" ");
        Serial.print("EL");
        Serial.println(step2deg(ELstepper.currentPosition()), 1);
        stepAz = AZstepper.currentPosition();
        stepEl = ELstepper.currentPosition();
      }
      /* Reset the rotator */
      else if (buffer[0] == 'R' && buffer[1] == 'E' && buffer[2] == 'S' && buffer[3] == 'E' && buffer[4] == 'T')
      {
        /* Get position */
        Serial.print("AZ");
        Serial.print(step2deg(AZstepper.currentPosition()), 1);
        Serial.print(" ");
        Serial.print("EL");
        Serial.println(step2deg(ELstepper.currentPosition()), 1);
        /*Move the steppers to initial position*/
        Homing(0,0);
        /*Zero the steps*/
        stepAz = 0;
        stepEl = 0;
      }      
      counter = 0;
      /*Reset the disable motor time*/
      t_DIS = 0;
    }
    /*Fill the buffer with incoming data*/
    else {
      buffer[counter] = incomingByte;
      counter++;
    }
  }
}

/*Error Handling*/
void error(int num_error)
{
  switch (num_error)
  {
    /*Azimuth error*/
    case (0):
      while(1)
      {
        Serial.println("AL001");
        delay(100);
      }
    /*Elevation error*/
    case (1):
      while(1)
      {
        Serial.println("AL002");
        delay(100);
      }
    /*GPS error*/
    case (2):
      {
        Serial.println("AL003");
        delay(100);
      }
    default:
      while(1)
      {
        Serial.println("AL000");
        delay(100);
      }
  }
}

/*Send pulses to stepper motor drivers*/
void stepper_move(int stepAz, int stepEl)
{
  AZstepper.moveTo(stepAz);
  ELstepper.moveTo(stepEl);
    
  AZstepper.run();
  ELstepper.run();
}

/*Convert degrees to steps*/
int deg2step(double deg)
{
  return(RATIO*SPR*deg/360);
}

/*Convert steps to degrees*/
double step2deg(int Step)
{
  return(360.00*Step/(SPR*RATIO));
}

/*Convert Radians to Degrees*/
float RadiansToDegrees(float rads)
{
  // Correct for when signs are reversed.
  if(rads < 0)
    rads += 2*PI;
  // Check for wrap due to addition of declination.
  if(rads > 2*PI)
    rads -= 2*PI;
  // Convert radians to degrees for readability.
  float heading = rads * 180/PI;
  return heading;
}
