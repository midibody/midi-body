/* utilities functions */

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Arduino.h>

LiquidCrystal_I2C lcd(0x20,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//******************************
// this function must be added into SETUP function of the main program
void initLcd()
{
  lcd.init();  lcd.backlight();  lcd.home();
}

//******************************
// displays texts on LCD Line 1
//******************************
void displayL1(char* t)
{
 char a[17];
 int l,i;
 
 // adds spaces until 16 chars in order to delete previous text
 for (l=0; l<16; l++) {if (t[l]==0) break; a[l]=t[l];}
 for (i=l; i<16; i++) a[i]=' ';
 a[16]=0;
 lcd.setCursor(0, 0); // cursor on line 0 et column 0
 lcd.print(a); 
}

//******************************
// displays texts on LCD Line 1
//******************************
void displayL2(char* t)
{
 char a[17];
 int l,i;
 
 for (l=0; l<16; l++) {if (t[l]==0) break; a[l]=t[l];}
 for (i=l; i<16; i++) a[i]=' ';
 a[16]=0;
 lcd.setCursor(0, 1); // curseur sur colonne 0 et ligne 1 (0=ligne haute, 1 =ligne basse)
 lcd.print(a);
 }
 
//******************************
// check if a button was pressed . designed for Arduino Romeo that integrates 5 switchs on the board
// return zero if no button pressed; else, return teh button number
//******************************

const int key_S1_5 = 7; // input analog pin used to measure the buttons

int isButtonPressed()
{

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

//*****************************
// Plays a midi note
// c is the midi channel from 1 to 16
//*****************************
void midiNoteOn(int c, int pitch, int velocity) {
  Serial.write(0x90-1+c); // Note on: hexa 9n  n= midi channel
  Serial.write(pitch);  Serial.write(velocity);
}

//*****************************
// Plays a midi note
//*****************************
void midiNoteOff(int c, int pitch, int velocity) {
  Serial.write(0x80-1+c); // note off: hexa 8n
  Serial.write(pitch);  Serial.write(velocity);
}

//*********************************
// not really used. cmd should be 0x90 for example for note ON for midi channel
//*********************************
void playNote(int cmd, int pitch, int velocity) {
  Serial.write(cmd);  Serial.write(pitch);  Serial.write(velocity);
}

//********************************
// send a midi control command
//********************************
void midiControler(int value,int control)
{
// midi controls: Bn , Number, Value (0-127) 
if (value>127) value=127; // in case the caller passes silly value  
Serial.write(0xB0);  Serial.write(control);  Serial.write(value);
}

