
#include <math.h>                              
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#include "I2Cdev.h"
#include "MPU6050.h"

#include <Stdio.h>

//Arduino Leonardo: connect SDA to digital pin 2 and SCL to digital pin 3 on your Arduino.

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

LiquidCrystal_I2C lcd(0x20,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

//*** MODES *****
int fModeIR =0;
int fModeMotion = 1;
int fModeDirection = 0;


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

#define MIDI_MODE_NOTE 0
#define MIDI_MODE_CHORDS 1
#define MIDI_MODE_EX 2

//#define DO   30

#define DO   36
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
#define DO_HIGH 48

#define countChords 5 //=> change if you add chords

int vNote= RE; // default note for snare

  int chordsMajor[][5]={
{DO,MI,SOL,DO-24,DO-12},
{DO,FA,LA,FA-24,FA-12},
{SOL,SI,RE,SOL-24,SOL-12},
{RE,FAd,LA,RE-24,RE-12},
{RE,SOL,LAd,LAd-24,LAd-12},
};

//******************************
// les variables du programme

int led = 13;

#define SensorDirection_pin1  10
#define SensorDirection_pin2  11


int i=0;
char a[17]; // pour stocker le texte à afficher sur le LCD
int note=0;
int currentNote=-1;
int distance=0;
int currentDistance = -1;
int midiMode;
int midiChannelDrum=10; // drums

int iControler = 0;
int controler;

#define cControler 3
char* TabControlerLabel[]= {"Filter Frequency", "Filter Reso.", "Volume"};
int TabControler[]={74,42,7};

int iMotionMode=0;
int motionMode;

#define MOTION_MODE_0  0
#define MOTION_MODE_1  1

#define cMotionMode 2
char* TabMotionModeLabel[]= {"Motion Mode 0", "Motion Mode 1"};
int TabMotionMode[]={MOTION_MODE_0, MOTION_MODE_1};
// for string conversion, if you can't just do e.g. dir.toString():
char * azimuthHeadings[] = { "E", "NE", "N", "NW", "W", "SW", "S", "SE" };


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

//******************************
// fonction pour afficher un texte sur la 2eme ligne du LCD (pour debugger)
void displayL1(char* t)
{
 char a[17];
 int l,i;
 
 for (l=0; l<16; l++) {if (t[l]==0) break; a[l]=t[l];}
 for (i=l; i<16; i++) a[i]=' ';
 a[16]=0;
 lcd.setCursor(0, 0); // curseur sur colonne 0 et ligne 0 (0=ligne haute, 1 =ligne basse)
 lcd.print(a); // affiche texte
}

//******************************
// fonction pour afficher un texte sur la 2eme ligne du LCD (pour debugger)
void displayL2(char* t)
{
 char a[17];
 int l,i;
 
 for (l=0; l<16; l++) {if (t[l]==0) break; a[l]=t[l];}
 for (i=l; i<16; i++) a[i]=' ';
 a[16]=0;
 lcd.setCursor(0, 1); // curseur sur colonne 0 et ligne 1 (0=ligne haute, 1 =ligne basse)
 lcd.print(a); // affiche texte
 }

//******************************
const int key_S1_5 = 7; // pind de la mesure des boutons 1 a 5

int boutons_check(){
// Cette fonction vérifie les boutons et retourne
// 0 si pas de bouton appuyé ou sinon 1 à 5 suivant S1...S5
// Priorités : S1, S2, S3, S4, S5

int w = analogRead(key_S1_5);

#define vS1 0
#define vS2 130
#define vS3 306
#define vS4 478
#define vS5 720
if ( w < vS2/2 ) return 1;
if ( w < (vS3+vS2)/2 ) return 2;
if ( w < (vS4+vS3)/2 ) return 3;
if ( w < (vS5+vS4)/2 ) return 4;
if ( w < (1024+vS5)/2 ) return 5;
return 0;
}

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
{
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.home();
  lcd.print("MIDI Body V1");

  pinMode(led, OUTPUT);
  pinMode(SensorDirection_pin1, INPUT);
  pinMode(SensorDirection_pin2, INPUT);

 Serial.begin(31250); // supprimer pour ne pas faire de midi

 midiMode= MIDI_MODE_CHORDS;
 motionMode = TabMotionMode[MOTION_MODE_0];
 
// displayL1("Play Chords");

 initializeMotionSensor();
 delay(1000);
 displayL1("MidiBody V1");
}


//********************************
// si on veut que les boutons commandent la pince
void actionBoutonMotionMode (int bouton)
{
 if ( bouton >0 ) 
 {
  if (bouton ==1) 
   {
     midiMode= MIDI_MODE_NOTE;
     displayL1("Play Notes");
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

//********************************
// si on veut que les boutons commandent la pince
void actionBouton (int bouton)
{
 if ( bouton >0 ) 
 {
//   sprintf(a,"Bouton %d", bouton);
//   displayL1(a);
  // un bouton est appuyé
  if (bouton ==1) 
   {
     midiMode= MIDI_MODE_NOTE;
     displayL1("Play Notes");
   }
  else if (bouton == 2) 
   {
     midiMode= MIDI_MODE_CHORDS;
     displayL1("Play Chords");
   }
  else if (bouton == 3) 
   {
     midiMode= MIDI_MODE_EX;
     displayL1("MIDI Exclusive");
   }
  else if (bouton == 4)
   {
    if (iControler<cControler-1) iControler++;
    else iControler = 0;
    sprintf(a,"%s", TabControlerLabel[iControler]); 
    controler = TabControler[iControler];
    displayL1(a);
   }
  else if (bouton == 5)
   {
    if (iMotionMode<cMotionMode-1) iMotionMode++;
    else iMotionMode = 0;
    sprintf(a,"%s", TabMotionModeLabel[iMotionMode]); 
    motionMode = TabMotionMode[iMotionMode];
    displayL1(a);
   }
   

 } 
}

//*********************************
void midiNoteOn(int c, int pitch, int velocity) {
  Serial.write(0x90-1+c); // Note on: hexa 9n  n= midi channel
  Serial.write(pitch);
  Serial.write(velocity);
}

//*********************************
void midiNoteOff(int c, int pitch, int velocity) {
  Serial.write(0x80-1+c); // note off: hexa 8n
  Serial.write(pitch);
  Serial.write(velocity);
}

//*********************************
void playNote(int cmd, int pitch, int velocity) {
  Serial.write(cmd);
  Serial.write(pitch);
  Serial.write(velocity);
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

//********************************
void midiControler(int i)
{
// midi controls: Bn , Number, Value (0-127) 

if (i>127) i=127;

  Serial.write(0xB0);
  Serial.write(controler);
  Serial.write(i);
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
// la boucle principale
void loop()
{
char a[32];
int v,cm;

//*******************************************
// test si un bouton est appuyé et decide quoi faire en conséquence
  if(millis()-timerButtons>=200) // interval
   {                          
    timerButtons = millis();                              // get the current time of programme
   
     int bouton = boutons_check();
     if (fModeMotion) actionBoutonMotionMode(bouton);
     else actionBouton(bouton);
    }
 
 if((fModeMotion||fModeDirection) && (millis()-timerMotion>=5)) 
   {                          
    if (fModeMotion) actionMotion();
    if (fModeDirection) actionDirection();
    timerMotion = millis();
    }
 

 //********************************
 // measure interval
  if(fModeIR && ((millis() - irTimer > 50) ))
   {
     int gMajeur[]={30,32,34,35,37,39,41, 42,44,46,47,49,51,53, 54,56,58,59,61,63,65, 66,68,70,71,73,75,77, 78,80,82,83,85,87,89, 90,92,94,95,97,99,101, 30,32,33,35,37,40, 42,44,45,47,49,52, 54};
    
    irTimer = millis();
    float val=analogRead(0);                      // Read ir distance sensor data
    int distance=6787.0 /(val - 3.0) - 4.0;     // Convert to the distance - cm
    
      if (distance < 90)
      {
       cm = distance;
       distance=distance -6; // zero for minimum distance of the cqpteru (usually 6 cm)
       distance = distance /3;
       
       if (distance != currentDistance) // we play new note or chord
        {
         currentDistance=distance;
         
         if (midiMode == MIDI_MODE_NOTE) 
          {
           if (currentNote != -1) playNote(0x90,currentNote,0);  // stop note avec velocité zero
           currentNote = gMajeur[distance];         
           sprintf(a,"pos:%d,note:%d", distance,currentNote); 
           displayL2(a);
           playNote(0x90,currentNote,0x45);
         delay (100);
          }      
          
         else if (midiMode == MIDI_MODE_CHORDS) // mode chords
          {
           if (currentNote !=0) playChord(0x90,currentNote,0); 
           currentNote = distance;
           playChord(0x90,currentNote,0x45);              
           sprintf(a,"pos:%d,ChordIdx:%d", distance,currentNote); 
           displayL2(a);
           delay (100);
          }
          
         else if (midiMode == MIDI_MODE_EX) // mode MIDI Exclusive to control filter, volume, etc...
          {
            cm-=6;
            v = cm*2;
            if (v>127) v=127;
            midiControler(v);
           sprintf(a,"pos:%d,V:%d", cm,v); 
           displayL2(a);
          }
         

       }   
      }
     else // distance >90, on arrete la note qui joue si il y en a une
      {
       if (currentNote != -1) 
        {
          if (midiMode == MIDI_MODE_NOTE) playNote(0x80,currentNote,0); // stop note avec velocité zero
          else if (midiMode == MIDI_MODE_CHORDS) playChord(0x80,currentNote,0x0);
          
          currentNote=-1;
          currentDistance = -1;
          displayL2("Jouez ...");   
         }
      }
   }
}
