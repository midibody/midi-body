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
MPU6050 accelgyro;

int16_t ax, ay, az,axDelta,ayDelta,azDelta;
int16_t axMax,axMin,ayMax,ayMin,azMax,azMin,gzMax,gzMin;
int16_t gx, gy, gz,gxDelta,gyDelta,gzDelta;
int16_t mx, my, mz;
int16_t var_compass;

char previousAx=0;
char previousAy=0;
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
unsigned long lastAzNegTime=0xFFFFFFFF;
unsigned long lastAzPlusTime=0xFFFFFFFF;
unsigned long tAzP=0;
unsigned long tAzM=0;
unsigned long tStartNote=0;
int16_t accelSensitivity=2000;
unsigned int azimuth=0;
unsigned int velocity;
int lastNote;
boolean up= true;                                 // create a boolean variable 
unsigned long irTimer ;                           // timer for managing the sensor reading flash rate
boolean fRight=true;
boolean fVertical=false;

int16_t lastAzMin=0;

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
accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
axDelta=-ax; ayDelta=-ay; azDelta=-az;
//gxDelta=-gx; gyDelta=-gy; gzDelta=-gz; //no need

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

    displayL1("Init motion...");
    accelgyro.initialize();

    accelgyro.setMotionDetectionThreshold(1);
    accelgyro.setMotionDetectionDuration(1);
    accelgyro.setZeroMotionDetectionThreshold(10);
    accelgyro.setZeroMotionDetectionDuration(1);
    
    // verify connection
    displayL1("Chk motion sens.");
    if (accelgyro.testConnection()) displayL1("MPU9150 Cnx OK"); else displayL1("MPU9150 Cnx FAILED!");

    //attachInterrupt(0, acceleroInterrupt, RISING);
      
    accelgyro.setIntEnabled(0x40);
    //accelgyro.setIntEnabled(MPU6050_INTERRUPT_FF_BIT|MPU6050_INTERRUPT_MOT_BIT|MPU6050_INTERRUPT_ZMOT_BIT);
    //m=accelgyro.getInterruptMode();
    //sprintf(a,"InterruptMode = %d\t",m); Serial.print(a);
    //accelgyro.setIntEnabled(MPU6050_INTERRUPT_FF_BIT);
    //accelgyro.setIntEnabled(MPU6050_INTERRUPT_ZMOT_BIT);

    calibrate();
}


//**************************************
void setup()
//**************************************
{
initLcd();
displayL1("MIDI Body V1");

pinMode(led, OUTPUT);
pinMode(SensorDirection_pin1, INPUT);
pinMode(SensorDirection_pin2, INPUT);

Serial.begin(31250); // midi port speed is 31250
loadConfig();
initializeMotionSensor();
displayL1("Ready"); displayL2("Move your body!");
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
#define setupMode_Sensor1Mode 0
#define setupMode_Sensor2Mode 1
#define setupMode_MotionSensor 2
#define setupMode_QuitSetup 3 // should always be the last index



// labels for menu level 1
#define cParam_DistSensorMode 5 //=> put alays the same amount as values in the array below

char *label_SetupMode[]={"Sensor 1 mode","Sensor 2 mode","Motion Sensor", "Exit Config Mode"};
char *label_DistSensorModeParams[]={"Single Note","Chords","Ctrl Filter Freq", "Ctrl Filter Reso", "Ctrl Volume"};
int TabControler[]={0,0,74,42,7}; //=> CAREFUL: the list mus be synched with the label_DistSensorModeParams. We put 2 zeros first to simplify the management of indexes

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
    displayL2("Change param=>S2");
    iParam=-1;
   }

  else if (bouton == 2) // Setup Level 2 Menu selection
   {
     if (iSetupMode == setupMode_QuitSetup)
      {
       iSetupMode = -1;
       displayL1("Saving Config...");displayL2("");
       saveConfig();
       //loadConfig();
       delay(1000);displayL1("Ready");
      }
      
     else if (iSetupMode == setupMode_Sensor1Mode) // select 
      {
       if (iParam!=-1) {iParam++ ; if (iParam>=cParam_DistSensorMode) iParam = 0; } else iParam = 0;
       sprintf(a,"%s",label_DistSensorModeParams[iParam]); displayL2(a);
       conf.midiMode[0]=iParam;
/*       switch (iParam) {
        case 0: conf.midiMode[0]= MIDI_MODE_NOTE; break;
        case 1: conf.midiMode[0]= MIDI_MODE_CHORDS; break;
        case 2: case 3: case 4:
         conf.midiMode[0]= MIDI_MODE_EX;
         conf.midiControler[0] = iParam-2; 
         
        break; //=> warning: by changing the order of the labels for the motion mode it breaks this index compuutation... not a nice code here 
        }*/
        
      }

     else if (iSetupMode == setupMode_Sensor2Mode) // select 
      {
       if (iParam!=-1) {iParam++ ; if (iParam>=cParam_DistSensorMode) iParam = 0; } else iParam = 0;
       sprintf(a,"%s",label_DistSensorModeParams[iParam]); displayL2(a);
       conf.midiMode[1]=iParam;
/*       switch (iParam) {
        case 0: conf.midiMode[1]= MIDI_MODE_NOTE; break;
        case 1: conf.midiMode[1]= MIDI_MODE_CHORDS; break;
        case 2: case 3: case 4:
         conf.midiMode[0]= MIDI_MODE_EX;
         conf.midiControler[1] = iParam-2; 
        break; //=> warning: by changing the order of the labels for the motion mode it breaks this index compuutation... not a nice code here        
        }*/
        
      }      

/*     else if (iSetupMode == setupMode_MidiCtrl) // select midi controller number
      {
       if (iParam!=-1) {iParam++ ; if (iParam>=cMidiControler) iParam = 0; } else iParam = 0;
       sprintf(a,"%s",label_MidiControler[iParam]); displayL2(a);
       conf.midiControler[0]=iParam;
      }*/
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

 if (conf.signature !=91)
  {
   displayL1("No config found..."); displayL2("Default Config"); delay(1000);
   setDefaultConfig();
   saveConfig();
   // need to set defaults here
  }
 else // the EEPROM contains a valid config
  {
    displayL1("Dist sensor 1:");
    displayL2(label_DistSensorModeParams[conf.midiMode[0]]); delay(1000);
    
    displayL1("Dist sensor 2:");
    displayL2(label_DistSensorModeParams[conf.midiMode[1]]); delay(1000);

//    controler= TabControler[conf.midiControler[0]];// in the conf we dont store the value but the index
//   sprintf(a,"Sensor1:%s",label_NotesModeParams[conf.midiMode[0]]); displayL2(a); delay(1000);
//   sprintf(a,"%s",label_MidiControler[conf.midiControler[0]]); displayL2(a); delay(1000);
  }
}

//*********************************
void setDefaultConfig()
{
  conf.signature=91;
  conf.midiMode[0] = MIDI_MODE_NOTE;
  conf.midiMode[1] = MIDI_MODE_EX_CTRL_FILTER;
//  conf.midiControler[0]=0; // useless variable
//  conf.midiControler[1]=0; // useless variable
}

 
//*********************************
void playChord(int cmd, int pitch, int velocity) {
  int shift;
  
pitch = pitch;

//30,32,34,35,37,39,41, 42,44,46,47,49,51,53, 54,56,58,59,61,63,65, 66,68,70,71,73,75,77, 78,80,82,83,85,87,89, 90,92,94,95,97,99,101, 30,32,33,35,37,40, 42,44,45,47,49,52, 54};
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
void actionMotion()
{
char a[32];// should be 17 max...we keep more, not clean but good enough
char aa[17];
char b[16];
unsigned long t;
int fDisplay = 0;

//#define minBetweenDrum_  100

//if (timerLastDrum + minBetweenDrum_ > millis()) return;
//timerLastDrum=millis();

accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
if (az<lastAzMin) lastAzMin=az; 

ax+=axDelta; ay+=ayDelta; az+=azDelta; gx+=gxDelta; gy+=gyDelta; gz+=gzDelta;

if ((az>0) && (az>azMax)) { azMax= az; } //sprintf (a,"az max=%d",azMax); displayL1(a);}
if ((az<0) && (az<azMin)) { azMin= az; }; //sprintf (a,"az min=%d",azMin); displayL2(a);}
//if ((gz>0) && (gz>gzMax)) { gzMax= gz; sprintf (a,"gz max=%d",gzMax); displayL1(a);}
//if ((gz<0) && (gz<gzMin)) { gzMin= gz; sprintf (a,"gz min=%d",gzMin); displayL2(a);}

if (gz>20000) fRight=false;
else if (gz<-20000) fRight=true;

t=millis();

// added for synth who need a note off. electronic drums usually dont.
if (tStartNote && (t - tStartNote >100)) // there is a Note ON since more than xxx milli -> we stop it.
 {
  //midiNoteOff(midiChannelDrum,lastNote,0x45); // note on & off
  tStartNote = 0;
 }

if ((az<-accelSensitivity) && !tAzM) // we detect a start of move down -negative az
 {
   tAzM=t;
   cMD++;
   fDisplay=0;
  }
/*
else if (tAzM && (t-tAzM > 200)) // we reset move down time to ignore the previous move - pas au point...
 {
   cResetMD++;
   tAzM=0; 
   fDisplay=1;
 }
*/

if ((az>accelSensitivity) && !tAzP && tAzM) // we detect a decceleration after an accel Negative (a tap down)
 {
  //computeAzimuth();  //{ "E", "NE", "N", "NW", "W", "SW", "S", "SE" };

  if (lastAzMin==-32768) lastAzMin=-32767; // to correct the extreme case cuasing a bug...
  
  if (ay<-2000) fVertical=false;
  else fVertical=true;
  
  velocity=map(-lastAzMin,-azMax,-azMin,0,126); // set the note velocity depending on negative accel. int16_t is from -32678 to +32767
  if (fRight) 
   {
    if (fVertical) midiNoteOn(midiChannelDrum,lastNote= 37,velocity); 
    else  midiNoteOn(midiChannelDrum,lastNote= DO,velocity); //kick
   }
  else 
   {
     if (fVertical) midiNoteOn(midiChannelDrum,lastNote=49,velocity);      // crash:49
     else midiNoteOn(midiChannelDrum,lastNote= vNote,velocity); 
   }

  tStartNote = t;
  cNotes++;
  tAzM=0;  
  //lastAzMin=0;

  fDisplay=0;
//   sprintf(a,"N=%d MD=%d %d",cNotes,cMD,fRight); displayL1(a); 
   aa[0]=(fRight)?'R':'L';    aa[1]=(fVertical)?'V':'H';   aa[2]=0;
   
   sprintf(a,"%d %d",azMin,azMax); displayL1(a);  
   sprintf(a,"%s %d %d %d",aa,lastNote,velocity,ay); displayL2(a);

   lastAzMin=32767;

 }

 if (fDisplay==1)    
  {
//   if (  tStartNote == t) lastAzMin=0;
  }
   

}


//*******************************
void actionMotionOld2()
{
char a[32];
char b[16];
#define minBetweenDrum_  100

//if (timerLastDrum + minBetweenDrum_ > millis()) return;
//timerLastDrum=millis();

accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
ax+=axDelta; ay+=ayDelta; az+=azDelta; gx+=gxDelta; gy+=gyDelta; gz+=gzDelta;

if ((az>0) && (az>azMax)) { azMax= az; sprintf (a,"az max=%d",azMax); displayL1(a);}
if ((az<0) && (az<azMin)) { azMin= az; sprintf (a,"az min=%d",azMin); displayL2(a);}


if (az<-3000) 
 {
  if (lastAzPlusTime+200<millis()) //detect note at deceleraton just after accel
   {
  playNote(0x90,SOL,0x45); delay (10); playNote(0x80,SOL,0x45);

  sprintf(a,"c=%d az=%d",++cNotes,az); displayL1(a);
  sprintf(a,"%l",millis()); displayL2(a);
  lastAzNegTime=0xFFFFFFFF;
  }

   else lastAzNegTime= millis();
 }
 
else if (az>3000) 
 {
  if (lastAzNegTime+200<millis()) //detect note at deceleraton just after accel
   {
  playNote(0x90,DO,0x45); delay (10); playNote(0x80,DO,0x45);

  sprintf(a,"c=%d az=%d",++cNotes,az); displayL1(a);
  lastAzPlusTime=0xFFFFFFFF;
   }
  //timerLastDrum=millis();
  else lastAzPlusTime= millis();  
 }


}

//*******************************
void actionMotionOld()
{
char a[32];
char b[16];

accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
ax+=axDelta; ay+=ayDelta; az+=azDelta; gx+=gxDelta; gy+=gyDelta; gz+=gzDelta;

//az -=16000; // is at this value when nothing moves...

//kick on ax

a[0]=0;
//strcat (a,displayVal(b,ax));

displayVal(b,ax);
if ((previousAx=='+') && (b[0]=='-'))
 {
  playNote(0x80,SOL,0);
  playNote(0x90,DO,0x45);
  previousAx='-';
 }
else if ((previousAx=='-') && (b[0]=='+'))
 {
  playNote(0x80,DO,0);
  playNote(0x90,SOL,0x45);
  previousAx='+';
 }

else previousAx=b[0];

displayVal(b,ay);
if ((previousAy=='+') && (b[0]=='-'))
 {
  playNote(0x80,SIb,0);
  playNote(0x90,SIb,0x45);
  previousAy='-';
 }
else if ((previousAy=='-') && (b[0]=='+'))
 {
  playNote(0x80,SIb,0);
  playNote(0x90,SIb,0x45);
  previousAy='+';
 }

else previousAy=b[0];


displayL2(b);
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
       distance=distance -6; // zero for minimum distance of the cqpteru (usually 6 cm)
       distance = distance /4;
       
       if (distance != currentDistance[sensorId]) // we play new note or chord
        {
         currentDistance[sensorId]=distance;
         
         if (conf.midiMode[sensorId] == MIDI_MODE_NOTE) 
          {
           if (currentNote[sensorId] != -1) playNote(0x90,currentNote[sensorId],0);  // stop note avec velocité zero
           currentNote[sensorId] = gMajeur[distance];
           codeNote= currentNote[sensorId]%12;
           octave= currentNote[sensorId]/12-1;
           sprintf(a,"cm:%d,n:%d %s%d", distance,currentNote[sensorId],aNotesNames[codeNote],octave); 
           displayL2(a);
           playNote(0x90,currentNote[sensorId],120);
         //delay (100);
          }      
          
         else if (conf.midiMode[sensorId] == MIDI_MODE_CHORDS) // mode chords
          {
           if (currentNote[sensorId] !=0) playChord(0x90,currentNote[sensorId],0); 
           currentNote[sensorId] = distance;
           playChord(0x90,currentNote[sensorId],30);              
           sprintf(a,"pos:%d,ChordId:%d", distance,currentNote[sensorId]); 
           displayL2(a);
           //delay (100);
          }
          
         else if ((conf.midiMode[sensorId] == MIDI_MODE_EX_CTRL_FILTER) || (conf.midiMode[sensorId] == MIDI_MODE_EX_CTRL_RES) || (conf.midiMode[sensorId] == MIDI_MODE_EX_CTRL_VOL)) // mode MIDI Exclusive to control filter, volume, etc...
          {
            cm-=6;
            v = cm*2;
            if (v>127) v=127;
            midiControler(v,TabControler[conf.midiMode[sensorId]]);
           sprintf(a,"pos:%d,V:%d", cm,v); 
           displayL2(a);
          }
         

       }   
      }
     else // distance >90, on arrete la note qui joue si il y en a une
      {
       if (currentNote[sensorId] != -1) 
        {
          if (conf.midiMode[sensorId] == MIDI_MODE_NOTE) playNote(0x80,currentNote[sensorId],0); // stop note avec velocité zero
          else if (conf.midiMode[sensorId] == MIDI_MODE_CHORDS) playChord(0x80,currentNote[sensorId],0x0);
          
          currentNote[sensorId]=-1;
          currentDistance[sensorId] = -1;
          displayL2("Jouez ...");   
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

 if (iSetupMode != -1) return; // we are in the menu mode
 
 if((fModeMotion||fModeDirection) && (millis()-timerMotion>=5)) //==>>>> should we keep this timer???????????????????????
   {                          
    if (fModeMotion) actionMotion();
    if (fModeDirection) actionDirection();
    timerMotion = millis();
    }
 

 //********************************
 // measure interval
  if(fModeIR && ((millis() - irTimer > 50) ))
   { 
    sensorDistance(0);
    sensorDistance(1); // comment this line if you have only 1 sensor for distance measures
    irTimer = millis();
   }    
}

