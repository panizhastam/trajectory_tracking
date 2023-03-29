#include <Arduino.h>
#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Butterworth.hpp>
#include <math.h>
//#include <WiFi.h>
//#include <WiFiUdp.h>
#include <string.h>

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                      global value variables                      */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


//Elbow PINs
static const int emgPinEl = A1;
static const int pressurePinEl = 6;
static const int driverPinEla = 17;
static const int driverPinElb = 5;
static const int encoder_data_el = 2;
static const int encoder_clk_el = 3;


int el_count = 0;
int elState = 0;
int elLastState = 0;  
double el_pos = 0;

//Shoulder AA Pins
static const int emgPinShAA = A11;
static const int pressurePinShAA = 27;
static const int driverPinShAAa = 14;
static const int driverPinShAAb = 12;
static const int encoder_data_aa = 21;
static const int encoder_clk_aa = 20;

int aa_count = 0;
int aaState = 0;
int aaLastState = 0;  
double aa_pos = 0;


//Shoulder FE Pins
static const int emgPinShFE = A5;
static const int pressurePinShFE = 22;
static const int driverPinShFEa = 4;
static const int driverPinShFEb = 13;
static const int encoder_data_fe = 18;
static const int encoder_clk_fe = 19;

int fe_count = 0;
int feState = 0;
int feLastState = 0;  
double fe_pos = 0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(emgPinEl, INPUT);
  pinMode(emgPinShFE, INPUT);
  pinMode(emgPinShAA, INPUT);
  
  pinMode(encoder_data_el, INPUT);
  pinMode(encoder_clk_el, INPUT);
  pinMode(encoder_data_fe, INPUT);
  pinMode(encoder_clk_fe, INPUT);
  pinMode(encoder_data_aa, INPUT);
  pinMode(encoder_clk_aa, INPUT);
}



/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                         The main loop                             */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/



void loop() {

  double data[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

//  map input voltages between 0 and 5 volts into integer values between 0 and 1023. 
//  This yields a resolution between readings of: 5 volts / 1024 units or, .0049 volts (4.9 mV) per unit.
  double elEMG;
  elEMG = analogRead(emgPinEl);
  data[0] = elEMG;

  double feEMG;
  feEMG = analogRead(emgPinShFE);
  data[1] = feEMG;

  double aaEMG;
  aaEMG = analogRead(emgPinShAA);
  data[2] = aaEMG;

  // read angular position from encoders in rad
  ElbowPos();
  ShFEPos();
  ShAAPos();

  data[3] = el_pos;
  data[4] = fe_pos;
  data[5] = aa_pos;


  String arrString = "";
  
  for (int i = 0; i < 6; i++) {
    arrString += String(data[i],4);
    if (i<5)
      arrString += ",";
  }

  Serial.println(arrString);
 
  

}


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                           encoder functions                      */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

void ElbowPos(){
  elState = digitalRead(encoder_clk_el);
  if (elState != elLastState){
    if (digitalRead(encoder_data_el) != elState) { 
       el_count ++;
     } else {
       el_count --;
     }
     el_pos = el_count * (PI/1024);    
  }
  elLastState = elState;
}


void ShFEPos(){
  feState = digitalRead(encoder_clk_fe);
  if (feState != feLastState){
    if (digitalRead(encoder_data_fe) != feState) { 
       fe_count ++;
     } else {
       fe_count --;
     }
     fe_pos = fe_count * (PI/1024);
  }
  feLastState = feState;
}

void ShAAPos(){
  aaState = digitalRead(encoder_clk_aa);
  if (aaState != aaLastState){
    if (digitalRead(encoder_data_aa) != aaState) { 
       aa_count ++;
     } else {
       aa_count --;
     }
     aa_pos = aa_count * (PI/1024);
  }
  aaLastState = aaState;
}
