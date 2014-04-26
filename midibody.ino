/* ================================================================================
This code is placed under the MIT license
Copyright (c) 2014 Fabien Felix

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
=================================================================================*/

#include <math.h>                              
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <avr/eeprom.h>

#include "utilities.h"

#include "I2Cdev.h"
#include "MPU6050.h"
#include <Stdio.h>

//Arduino Leonardo: connect SDA to digital pin 2 and SCL to digital pin 3 on your Arduino.
#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

//LiquidCrystal_I2C lcd(0x20,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display


//*** MODES *****
int fModeIR =1;
int fModeMotion = 1;
int fModeDirection = 0;

// for MPU 9150, class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter below when declaring the MPU6050 object
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro1(0x68); //SParkfun MPU9150 & 6050

MPU6050 accelgyro2(0x69); // drotek MPU6050

int16_t axDelta1,ayDelta1,azDelta1;
int16_t axMax1,axMin1,ayMax1,ayMin1,azMax1,azMin1,gzMax1,gzMin1;
int16_t gxDelta1,gyDelta1,gzDelta1;

int16_t axDelta2,ayDelta2,azDelta2;
int16_t axMax2,axMin2,ayMax2,ayMin2,azMax2,azMin2,gzMax2,gzMin2;
int16_t gxDelta2,gyDelta2,gzDelta2;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
int16_t var_compass;

//char previousAx=0;
//char previousAy=0;
char previousFB=0;

int cNotes=0;
int c=0,cMD=0,cResetMD=0;


//#define DO   30

#define DO   36 // also C1 in US notation
#define DOd  37
#define RE   38
#define REd  39
#define MIb  39
#define MI   40
#define FA   41
#define FAd  42
#define SOL  43
#define SOLd 44
#define LA   45 
#define LAd  46
#define SIb  46
#define SI   47
#define DO_HIGH 48 // C2

#define countChords 5 //=> change if you add chords

int vNote= RE; // default note for snare
enum {NoteRH_,NoteRV_,NoteLH_,NoteLV_};
unsigned char tabNote1[]={36,37,38,45};
unsigned char tabNote2[]={42,46,49,51};

//int notesNames[]={"C","C#",  "D","D#",  "E","F","F#",   "G","G#",    "A","A#",  "B"};
char * aNotesNames[]={"DO","DO#","RE","RE#","MI","FA","FA#","SOL","SOL#","LA","LA#","SI"};

int chordsMajor[][5]={
{DO,MI,SOL,DO-24,DO-12},
{DO,FA,LA,FA-24,FA-12},
{SOL,SI,RE,SOL-24,SOL-12},
{RE,FAd,LA,RE-24,RE-12},
{RE,SOL,LAd,LAd-24,LAd-12},
};

#define SensorDirection_pin1  10
#define SensorDirection_pin2  11

#define cMaxSensors 2

//******************************
// Program variables

int led = 13;
int i=0;
char a[17]; // temp var to store text to display on Lc => dont use more than 16 chars!!
//int note=0;

struct config_t
{
char signature; // we store value 0xAA to check if we read a valid config...
char midiMode[cMaxSensors];
int midiControler[cMaxSensors]; // max 127
char motionSensor1RH;
char motionSensor1RV;
char motionSensor1LH;
char motionSensor1LV;
char motionSensor2RH;
char motionSensor2RV;
char motionSensor2LH;
char motionSensor2LV;
} conf;

unsigned int currentDistance[cMaxSensors] = {-1,-1};
unsigned int currentNote[cMaxSensors]={-1,-1};

int midiChannelDrum=10; // drums

int iControler = 0;
int controler;
int iMotionMode=0;
int motionMode;
unsigned long timerButtons;                               // create a time variable
unsigned long timerMotion;
unsigned long timerLastDrum=0;
unsigned long timerLastNoteStart=0;
//unsigned long lastAzNegTime=0xFFFFFFFF;
//unsigned long lastAzPlusTime=0xFFFFFFFF;

int16_t lastAzMin1=0;
unsigned long tAzP1=0;
unsigned long tAzM1=0;
unsigned long tStartNote1=0;
char lastNote1;

int16_t lastAzMin2=0;
unsigned long tAzP2=0;
unsigned long tAzM2=0;
unsigned long tStartNote2=0;
char lastNote2;

int16_t accelSensitivity=2000;
unsigned int azimuth=0;
unsigned int velocity;

boolean up= true;                                 // create a boolean variable 
unsigned long irTimer ;                           // timer for managing the sensor reading flash rate
boolean fRight1=true;
boolean fRight2=true;
boolean fVertical=false;

#define MOTION_MODE_0  0
#define MOTION_MODE_1  1

#define cMotionMode 2
char* TabMotionModeLabel[]= {"Motion Mode 0", "Motion Mode 1"};
int TabMotionMode[]={MOTION_MODE_0, MOTION_MODE_1};
// for string conversion, if you can't just do e.g. dir.toString():
char * azimuthHeadings[] = { "E", "NE", "N", "NW", "W", "SW", "S", "SE" };


//*********************
void calibrate()
{
accelgyro1.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
axDelta1=-ax; ayDelta1=-ay; azDelta1=-az;
//gxDelta=-gx; gyDelta=-gy; gzDelta=-gz; //no need

accelgyro2.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
axDelta2=-ax; ayDelta2=-ay; azDelta2=-az;
}

//***********************
void computeAzimuth()
{
float angle;
/*
  var_compass=atan2((double)my,(double)mx) * (180 / PI) -90; // angle in degrees
if (var_compass>0){var_compass=var_compass-360;}
var_compass=(360+var_compass);
Serial.print("AAzimuth meth1: ");Serial.print(var_compass);Serial.print("\t");
*/
// actual conversion code:
angle = atan2( my, mx );
azimuth = round( 8 * angle / (2*PI) + 8 ) % 8;

/*
Serial.print("AAzimuth meth2: ");Serial.print(angle); Serial.print(" "); Serial.print(headings[octant]);
Serial.print(" ");

// from diy.powet.eu
angle = (180*(atan2(-my, mx )/PI))+180;
Serial.print("AAzimuth meth3: ");Serial.print(angle); Serial.print(" \t");
*/

}

//***********************
void initializeMotionSensor()
{
      // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    accelgyro1.initialize();
    accelgyro1.setMotionDetectionThreshold(1);
    accelgyro1.setMotionDetectionDuration(1);
    accelgyro1.setZeroMotionDetectionThreshold(10);
    accelgyro1.setZeroMotionDetectionDuration(1);
    accelgyro1.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    accelgyro1.setDLPFMode(0x06);// low pass filter
    if (accelgyro1.testConnection()) displayL1("MPU 1 Cnx OK"); else displayL1("MPU 1 Cnx FAILED!");
    //sprintf(a,"DeviceID=%d",accelgyro1.getDeviceID()); displayL2(a);
    delay(1000);
    accelgyro1.setIntEnabled(0x40); //useless
    
    accelgyro2.initialize();
    accelgyro2.setMotionDetectionThreshold(1);
    accelgyro2.setMotionDetectionDuration(1);
    accelgyro2.setZeroMotionDetectionThreshold(10);
    accelgyro2.setZeroMotionDetectionDuration(1);
    accelgyro2.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    accelgyro2.setDLPFMode(0x06);// low pass filter

    if (accelgyro2.testConnection()) displayL1("MPU 2 Cnx OK"); else displayL1("MPU 2 Cnx FAILED!");
    //sprintf(a,"DeviceID=%d",accelgyro1.getDeviceID()); displayL2(a);
    delay(1000);
    accelgyro2.setIntEnabled(0x40); //useless

    calibrate();

    //attachInterrupt(0, acceleroInterrupt, RISING);
    //accelgyro.setIntEnabled(MPU6050_INTERRUPT_FF_BIT|MPU6050_INTERRUPT_MOT_BIT|MPU6050_INTERRUPT_ZMOT_BIT);
    //m=accelgyro.getInterruptMode();
    //sprintf(a,"InterruptMode = %d\t",m); Serial.print(a);
    //accelgyro.setIntEnabled(MPU6050_INTERRUPT_FF_BIT);
    //accelgyro.setIntEnabled(MPU6050_INTERRUPT_ZMOT_BIT);
 }


//**************************************
void setup()
//**************************************
{
initLcd();
displayL1("Initializing..."); delay(1000);

pinMode(led, OUTPUT);
pinMode(SensorDirection_pin1, INPUT);
pinMode(SensorDirection_pin2, INPUT);

Serial.begin(31250); // midi port speed is 31250
loadConfig();
initializeMotionSensor();
displayL1("MIDI Body Ready"); displayL2("Move your body!");
delay(1000); // just to let time to show the welco,e text before it is overwritten
}


//********************************
// si on veut que les boutons commandent la pince
void actionBoutonMotionMode (int bouton)
{
 if ( bouton >0 ) 
 {
  if (bouton ==1) 
   {
     //midiMode= MIDI_MODE_NOTE;
     //displayL1("Play Notes");
   }
  else if (bouton == 2) 
   {
    if (accelSensitivity<30000) accelSensitivity+=1000;
    else accelSensitivity=0;
    sprintf(a,"Sensit=%d",accelSensitivity); displayL2(a);
   }
  else if (bouton == 3) 
   {
    if (accelSensitivity>1000) accelSensitivity-=1000;
    else accelSensitivity=30000;
        sprintf(a,"Sensit=%d",accelSensitivity); displayL2(a);
   }
  else if (bouton == 4)
   {
    sprintf(a,"Note=%d",++vNote); displayL2(a);
   }
  else if (bouton == 5)
   {
    sprintf(a,"Note=%d",--vNote); displayL2(a);
   }
  } 
}


int iSetupMode = -1;
int iParam=-1;
// list of menu level 1
enum { setupMode_Sensor1Mode, setupMode_Sensor2Mode, setupMode_MotionSensor1RH, setupMode_MotionSensor1RV, setupMode_MotionSensor1LH, setupMode_MotionSensor1LV, setupMode_QuitSetup}; // setupMode_QuitSetup should always be the last index

// labels for menu level 1
char *label_SetupMode[]={"Sensor 1 mode","Sensor 2 mode","Drum1 RH","Drum1 RV","Drum1 LH","Drum1 LV","Exit Config Mode"};

#define cParam_DistSensorMode 5 //=> put alays the same amount as values in the array below
char *label_DistSensorModeParams[]={"Single Note","Chords","Ctrl Filter Freq", "Ctrl Filter Reso", "Ctrl Volume"};
int TabControler[]={0,0,74,42,7}; //=> CAREFUL: the list must be synched with the label_DistSensorModeParams. We put 2 zeros first to simplify the management of indexes

#define MIDI_MODE_NOTE 0
#define MIDI_MODE_CHORDS 1
#define MIDI_MODE_EX_CTRL_FILTER 2
#define MIDI_MODE_EX_CTRL_RES 3
#define MIDI_MODE_EX_CTRL_VOL 4

//#define cMidiControler 3
//char* label_MidiControler[]= {"Ctrl Filter Freq", "Ctrl Filter Reso", "Ctrl Volume"};


//********************************
// To handle the config menu
void actionBoutonSetup (int bouton)
{
 if ( bouton >0 ) 
 {
  if (bouton == 1) // Setup Level 1 Menu selection
   {
    if (iSetupMode!=-1) // we are already in the setup mode
     {       iSetupMode++; if (iSetupMode>setupMode_QuitSetup) iSetupMode = 0;     }
    else iSetupMode = 0;
    displayL1(label_SetupMode[iSetupMode]);
    displayL2("To update: S2");
    iParam=-1;
   }

  else if (bouton == 2) // Setup Level 2 Menu selection
   {
     if (iSetupMode == setupMode_QuitSetup)
      {
       iSetupMode = -1;
       displayL1("Saving Config...");displayL2("");
       saveConfig();
       delay(1000);displayL1("Ready");
      }
      
     else if (iSetupMode == setupMode_Sensor1Mode) // select 
      {
       if (iParam!=-1) {iParam++ ; if (iParam>=cParam_DistSensorMode) iParam = 0; } else iParam = 0;
       sprintf(a,"%s",label_DistSensorModeParams[iParam]); displayL2(a);
       conf.midiMode[0]=iParam;
      }

     else if (iSetupMode == setupMode_Sensor2Mode) // select 
      {
       if (iParam!=-1) {iParam++ ; if (iParam>=cParam_DistSensorMode) iParam = 0; } else iParam = 0;
       sprintf(a,"%s",label_DistSensorModeParams[iParam]); displayL2(a);
       conf.midiMode[1]=iParam;
      }            
      
     else if (iSetupMode == setupMode_MotionSensor1RH)       {       iParam= tabNote1[NoteRH_];       sprintf(a,"Note: %d",iParam); displayL2(a);      }      
     else if (iSetupMode == setupMode_MotionSensor1RV)       {       iParam= tabNote1[NoteRV_];       sprintf(a,"Note: %d",iParam); displayL2(a);      }   
     else if (iSetupMode == setupMode_MotionSensor1LH)       {       iParam= tabNote1[NoteLH_];       sprintf(a,"Note: %d",iParam); displayL2(a);      }   
     else if (iSetupMode == setupMode_MotionSensor1LV)       {       iParam= tabNote1[NoteLV_];       sprintf(a,"Note: %d",iParam); displayL2(a);      }         
   }// end button 2
   
   else if (bouton == 3) // Plus button
   {
    if (iSetupMode == setupMode_MotionSensor1RH)          {      conf.motionSensor1RH=tabNote1[NoteRH_]= ++iParam;       sprintf(a,"Note: %d",iParam); displayL2(a);     } 
    else if (iSetupMode == setupMode_MotionSensor1RV)     {      conf.motionSensor1RV=tabNote1[NoteRV_]= ++iParam;       sprintf(a,"Note: %d",iParam); displayL2(a);     } 
    else if (iSetupMode == setupMode_MotionSensor1LH)     {      conf.motionSensor1LH=tabNote1[NoteLH_]= ++iParam;       sprintf(a,"Note: %d",iParam); displayL2(a);     } 
    else if (iSetupMode == setupMode_MotionSensor1LV)     {      conf.motionSensor1LV=tabNote1[NoteLV_]= ++iParam;       sprintf(a,"Note: %d",iParam); displayL2(a);     } 
   }
   
   
   else if (bouton == 4) // Decrement button
   {
    if (iSetupMode == setupMode_MotionSensor1RH)          {      if (iParam>0) conf.motionSensor1RH=tabNote1[NoteRH_]= --iParam;      sprintf(a,"Note: %d",iParam); displayL2(a);    } 
    else if (iSetupMode == setupMode_MotionSensor1RV)     {      if (iParam>0) conf.motionSensor1RV=tabNote1[NoteRV_]= --iParam;      sprintf(a,"Note: %d",iParam); displayL2(a);    } 
    else if (iSetupMode == setupMode_MotionSensor1LH)     {      if (iParam>0) conf.motionSensor1LH=tabNote1[NoteLH_]= --iParam;      sprintf(a,"Note: %d",iParam); displayL2(a);    } 
    else if (iSetupMode == setupMode_MotionSensor1LV)     {      if (iParam>0) conf.motionSensor1LV=tabNote1[NoteLV_]= --iParam;      sprintf(a,"Note: %d",iParam); displayL2(a);    }     
   }
   
   
 }
}

//*********************************
void saveConfig()
{
eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
}

//*********************************
void loadConfig()
{
eeprom_read_block((void*)&conf, (void*)0, sizeof(conf));

 if (conf.signature !=92) // we read shit
  {
   displayL1("No config found..."); displayL2("Default Config"); delay(1000);
   setDefaultConfig();
   saveConfig();
  }
 else // the EEPROM contains a valid config
  {
    displayL1("Dist sensor 1:");
    displayL2(label_DistSensorModeParams[conf.midiMode[0]]); delay(1000);
    
    displayL1("Dist sensor 2:");
    displayL2(label_DistSensorModeParams[conf.midiMode[1]]); delay(1000);
    
    tabNote1[NoteRH_]= conf.motionSensor1RH;
    tabNote1[NoteRV_]= conf.motionSensor1RV;
    tabNote1[NoteLH_]= conf.motionSensor1LH;
    tabNote1[NoteLV_]= conf.motionSensor1LV;
  }
}

//*********************************
void setDefaultConfig()
{
  conf.signature=92;
  conf.midiMode[0] = MIDI_MODE_NOTE;
  conf.midiMode[1] = MIDI_MODE_EX_CTRL_FILTER;
  conf.motionSensor1RH=tabNote1[NoteRH_];
  conf.motionSensor1RV=tabNote1[NoteRV_];
  conf.motionSensor1LH=tabNote1[NoteLH_];
  conf.motionSensor1LV=tabNote1[NoteLV_];  
}

 
//*********************************
void playChord(int cmd, int pitch, int velocity) {
  int shift;

if (pitch >= countChords) return;

 shift= 24;
  Serial.write(cmd);  Serial.write(chordsMajor[pitch][0]+shift);  Serial.write(velocity);
  Serial.write(cmd);  Serial.write(chordsMajor[pitch][1]+shift);  Serial.write(velocity);
  Serial.write(cmd);  Serial.write(chordsMajor[pitch][2]+shift);  Serial.write(velocity);
  Serial.write(cmd);  Serial.write(chordsMajor[pitch][3]+shift);  Serial.write(velocity);
  Serial.write(cmd);  Serial.write(chordsMajor[pitch][4]+shift);  Serial.write(velocity);
  
}


#define thresholdAPlus 3000  
#define thresholdAMinus 3000  

//**************************
int16_t isOver(int16_t i)
{
   if (((i>0) && (i <thresholdAPlus)) || ((i<0) && (i>-thresholdAMinus))) return 0;
   else return i;
}

//**************************
char* displayVal(char *txt, int16_t i)
{
    if (!isOver(i)) sprintf(txt," ");
    else { if (i>0) sprintf(txt,"+"); else sprintf(txt,"-");   }
return txt;    
} 

//*******************************
void actionDirection()
{
char a[32];
int rl; // right, left flag
int bf; // backward forward flag

bf = digitalRead(SensorDirection_pin1);
rl = digitalRead(SensorDirection_pin2);

sprintf(a,"rl: %d, bf: %d", rl, bf);
displayL2(a);

if ((previousFB=='B') && (bf==0)) // sensor was backward and moved forward
 {
  playNote(0x90,DO,0x45);
  previousFB='F';
 }

else
{
  if ((previousFB=='F') && (bf==0)) playNote(0x80,DO,0);
    previousFB=(bf==1)? 'B':'F';
}

}

//*******************************
void actionMotion1()
{
//char a[32];// should be 17 max...we keep more, not clean but good enough
char aa[17];
char b[16];
unsigned long t;

//#define minBetweenDrum_  100
//if (timerLastDrum + minBetweenDrum_ > millis()) return;
//timerLastDrum=millis();

accelgyro1.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

if (az<lastAzMin1) lastAzMin1=az; 

ax+=axDelta1; ay+=ayDelta1; az+=azDelta1; gx+=gxDelta1; gy+=gyDelta1; gz+=gzDelta1;

if ((az>0) && (az>azMax1)) { azMax1= az; } //sprintf (a,"az max=%d",azMax); displayL1(a);}
if ((az<0) && (az<azMin1)) { azMin1= az; }; //sprintf (a,"az min=%d",azMin); displayL2(a);}
//if ((gz>0) && (gz>gzMax)) { gzMax= gz; sprintf (a,"gz max=%d",gzMax); displayL1(a);}
//if ((gz<0) && (gz<gzMin)) { gzMin= gz; sprintf (a,"gz min=%d",gzMin); displayL2(a);}

if (gz>20000) fRight1=false;
else if (gz<-20000) fRight1=true;

t=millis();

// added for synth who need a note off. electronic drums usually dont.
if (tStartNote1 && (t - tStartNote1 >100)) // there is a Note ON since more than xxx milli -> we stop it.
 {
  //midiNoteOff(midiChannelDrum,lastNote,0x45); // note on & off
  tStartNote1 = 0;
 }

if ((az<-accelSensitivity) && !tAzM1) // we detect a start of move down -negative az
 {
   tAzM1=t;
   cMD++;
  }
/*
else if (tAzM && (t-tAzM > 200)) // we reset move down time to ignore the previous move - pas au point...
 {
   cResetMD++;
   tAzM=0; 
   fDisplay=1;
 }
*/

if ((az>accelSensitivity) && !tAzP1 && tAzM1) // we detect a decceleration after an accel Negative (a tap down)
 {
  //computeAzimuth();  //{ "E", "NE", "N", "NW", "W", "SW", "S", "SE" };

  if (lastAzMin1==-32768) lastAzMin1=-32767; // to correct the extreme case cuasing a bug...
  
  fVertical=(ay<-2000)?false:true;
  
  velocity=map(-lastAzMin1,-azMax1,-azMin1,0,126); // set the note velocity depending on negative accel. int16_t is from -32678 to +32767
  if (fRight1) 
   {
    if (fVertical) midiNoteOn(midiChannelDrum,lastNote1= tabNote1[NoteRV_],velocity); //37
    else  midiNoteOn(midiChannelDrum,lastNote1= tabNote1[NoteRH_],velocity); //kick 36
   }
  else 
   {
     if (fVertical) midiNoteOn(midiChannelDrum,lastNote1= tabNote1[NoteLV_],velocity);      // crash:49
     else midiNoteOn(midiChannelDrum,lastNote1= tabNote1[NoteLH_],velocity); //symbal
   }

  tStartNote1 = t;
  cNotes++;
  tAzM1=0;  
  //lastAzMin=0;  //   sprintf(a,"N=%d MD=%d %d",cNotes,cMD,fRight); displayL1(a); 
   aa[0]=(fRight1)?'R':'L';    aa[1]=(fVertical)?'V':'H';   aa[2]=0;

  if (iSetupMode == -1) // we dont distrub the menu text wile editing it...
   {
   sprintf(a,"%d %d",azMin1,azMax1); displayL1(a);  
   sprintf(a,"%s %d %d %d",aa,lastNote1,velocity,ay); displayL2(a);
   }
   lastAzMin1=32767;
 }
}

//*******************************
void actionMotion2()
{
//char a[32];// should be 17 max...we keep more, not clean but good enough
char aa[17];
char b[16];
unsigned long t;

//#define minBetweenDrum_  100
//if (timerLastDrum + minBetweenDrum_ > millis()) return;
//timerLastDrum=millis();

accelgyro2.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

if (az<lastAzMin2) lastAzMin2=az; 

ax+=axDelta2; ay+=ayDelta2; az+=azDelta2; gx+=gxDelta2; gy+=gyDelta2; gz+=gzDelta2;

if ((az>0) && (az>azMax2)) { azMax2= az; } //sprintf (a,"az max=%d",azMax); displayL2(a);}
if ((az<0) && (az<azMin2)) { azMin2= az; }; //sprintf (a,"az min=%d",azMin); displayL2(a);}
//if ((gz>0) && (gz>gzMax)) { gzMax= gz; sprintf (a,"gz max=%d",gzMax); displayL1(a);}
//if ((gz<0) && (gz<gzMin)) { gzMin= gz; sprintf (a,"gz min=%d",gzMin); displayL2(a);}

if (gz>20000) fRight2=false;
else if (gz<-20000) fRight2=true;

t=millis();

// added for synth who need a note off. electronic drums usually dont.
if (tStartNote2 && (t - tStartNote2 >100)) // there is a Note ON since more than xxx milli -> we stop it.
 {
  //midiNoteOff(midiChannelDrum,lastNote,0x45); // note on & off
  tStartNote2 = 0;
 }

if ((az<-accelSensitivity) && !tAzM2) // we detect a start of move down -negative az
 {
   tAzM2=t;
   cMD++;
  }
/*
else if (tAzM && (t-tAzM > 200)) // we reset move down time to ignore the previous move - pas au point...
 {
   cResetMD++;
   tAzM=0; 
   fDisplay=1;
 }
*/

if ((az>accelSensitivity) && !tAzP2 && tAzM2) // we detect a decceleration after an accel Negative (a tap down)
 {
  //computeAzimuth();  //{ "E", "NE", "N", "NW", "W", "SW", "S", "SE" };

  if (lastAzMin2==-32768) lastAzMin2=-32767; // to correct the extreme case cuasing a bug...
  
  fVertical=(ay<-2000)?false:true;
  
  velocity=map(-lastAzMin2,-azMax2,-azMin2,0,126); // set the note velocity depending on negative accel. int16_t is from -32678 to +32767
  if (fRight2) 
   {
    if (fVertical) midiNoteOn(midiChannelDrum,lastNote2= tabNote2[NoteRV_],velocity); //37
    else  midiNoteOn(midiChannelDrum,lastNote2= tabNote2[NoteRH_],velocity); //kick 36
   }
  else 
   {
     if (fVertical) midiNoteOn(midiChannelDrum,lastNote2= tabNote2[NoteLV_],velocity);      // crash:49
     else midiNoteOn(midiChannelDrum,lastNote2= tabNote2[NoteLH_],velocity); //symbal
   }

  tStartNote2 = t;
  cNotes++;
  tAzM2=0;  
  //lastAzMin=0;  //   sprintf(a,"N=%d MD=%d %d",cNotes,cMD,fRight); displayL1(a); 
   aa[0]=(fRight2)?'R':'L';    aa[1]=(fVertical)?'V':'H';   aa[2]=0;

  if (iSetupMode == -1) // we dont distrub the menu text wile editing it...
   {
   sprintf(a,"%d %d",azMin2,azMax2); displayL1(a);  
   sprintf(a,"%s %d %d %d",aa,lastNote2,velocity,ay); displayL2(a);
   }
   lastAzMin2=32767;
 }
}


//********************************
void sensorDistance(int sensorId)
{
int v,cm;
int codeNote,octave;
int distance;
int gMajeur[]={DO,RE,MI,FA,SOL,LA,SI,DO+12,RE+12,MI+12,FA+12,SOL+12,LA+12,SI+12,DO+24,RE+24,MI+24,FA+24,SOL+24,LA+24,SI+24,DO+36,RE+36,MI+36,FA+36,SOL+36,LA+36,SI+36,DO+48,RE+48,MI+48,FA+48,SOL+48,LA+48,SI+48,DO+60,RE+60,MI+60,FA+60,SOL+60,LA+60,SI+60};

  float val=analogRead(sensorId);                      // Read ir distance sensor data. on analog pin zero or one

    distance=6787.0 /(val - 3.0) - 4.0;     // Convert to the distance - cm
    
      if (distance < 90)
      {
       cm = distance;
       distance=distance -6; // zero for minimum distance of the sensor (usually 6 cm)
       distance = distance /4;

       if (distance != currentDistance[sensorId]) // we play new note or chord
        {
         if (conf.midiMode[sensorId] == MIDI_MODE_NOTE) 
          {
           if (millis()-timerLastNoteStart<100) return; // to get slower BPM in the notes
           
           if (currentNote[sensorId] != -1) playNote(0x80,currentNote[sensorId],0);  // stop previous note
           
           currentNote[sensorId] = gMajeur[distance];
           codeNote= currentNote[sensorId]%12;
           octave= currentNote[sensorId]/12-1;
           //sprintf(a,"cm:%d,n:%d %s%d", distance,currentNote[sensorId],aNotesNames[codeNote],octave); 
           sprintf(a,"Playing:%s%d", aNotesNames[codeNote],octave); 
           
           displayL2(a);
           playNote(0x90,currentNote[sensorId],100);
           timerLastNoteStart = millis();
          }      
          
         else if (conf.midiMode[sensorId] == MIDI_MODE_CHORDS) // mode chords
          {
           if (millis()-timerLastNoteStart<100) return;

            if (currentNote[sensorId] !=-1) playChord(0x80,currentNote[sensorId],0); // stop last chord currently playing
           currentNote[sensorId] = distance;
           playChord(0x90,currentNote[sensorId],40);              
           sprintf(a,"pos:%d,ChordId:%d", distance,currentNote[sensorId]); 
           displayL2(a);
           timerLastNoteStart = millis();
          }
          
         else if ((conf.midiMode[sensorId] == MIDI_MODE_EX_CTRL_FILTER) || (conf.midiMode[sensorId] == MIDI_MODE_EX_CTRL_RES) || (conf.midiMode[sensorId] == MIDI_MODE_EX_CTRL_VOL)) // mode MIDI Exclusive to control filter, volume, etc...
          {
            cm-=6;
            v = cm*2;
            if (v>127) v=127;
            midiControler(v,TabControler[conf.midiMode[sensorId]]);
           //sprintf(a,"pos:%d,V:%d", cm,v); 
           sprintf(a,"Midi Control=%d",v);
           displayL2(a);
          }
         
         currentDistance[sensorId]=distance;
       } // end distance changed       
      }
      
     else // distance >90, on arrete la note qui joue si il y en a une
      {
       if (currentNote[sensorId] != -1) 
        {
          if (conf.midiMode[sensorId] == MIDI_MODE_NOTE) playNote(0x80,currentNote[sensorId],0); // stop note avec velocité zero
          else if (conf.midiMode[sensorId] == MIDI_MODE_CHORDS) playChord(0x80,currentNote[sensorId],0x0);
          timerLastNoteStart=0;
          currentNote[sensorId]=-1;
          currentDistance[sensorId] = -1;
          displayL2("Move your body!");   
         }
      }
   }

//********************************
// la boucle principale
void loop()
{
char a[32];
int v,cm;
int codeNote,octave;

//*******************************************
// test si un bouton est appuyé et decide quoi faire en conséquence
  if(millis()-timerButtons>=200) // interval
   {                          
    timerButtons = millis();                              // get the current time of programme
   
     int bouton = isButtonPressed();
     actionBoutonSetup(bouton);
     //if (fModeMotion) actionBoutonMotionMode(bouton);
     //else actionBouton(bouton);
    }
 
 if((fModeMotion||fModeDirection) && (millis()-timerMotion>=5)) //==>>>> should we keep this timer???????????????????????
   {                          
    if (fModeMotion) 
     {
       actionMotion1();
       actionMotion2();
     }
    if (fModeDirection) actionDirection();
    timerMotion = millis();
    }
 
 if (iSetupMode != -1) return; // we are in the menu mode
 
 //********************************
 // measure interval
  if(fModeIR && ((millis() - irTimer > 50) ))
   { 
    sensorDistance(0);
    sensorDistance(1); // comment this line if you have only 1 sensor for distance measures
    irTimer = millis();
   }    
}

