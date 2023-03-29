#include <Arduino.h>
#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Butterworth.hpp>
#include <math.h>
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
int elLastState = 1;  
double el_pos = 0.0;

//Shoulder AA Pins
static const int emgPinShAA = A11;
static const int pressurePinShAA = 8;
static const int driverPinShAAa = 26;
static const int driverPinShAAb = 27;
static const int encoder_data_aa = 21;
static const int encoder_clk_aa = 20;



int aa_count = 0;
int aaState = 0;
int aaLastState = 1;  
double aa_pos = 0.0;

float ShAA_tao;
float ShFE_tao;
float El_tao;


//Shoulder FE Pins
static const int emgPinShFE = A5;
static const int pressurePinShFE = 7;
static const int driverPinShFEa = 44;
static const int driverPinShFEb = 45;
static const int encoder_data_fe = 18;
static const int encoder_clk_fe = 19;

int fe_count = 0;
int feState = 1;
int feLastState = 0;  
double fe_pos = 0.0;



void setup() {
  // put your setup code here, to run once:
  
  pinMode(emgPinEl, INPUT);
  pinMode(emgPinShFE, INPUT);
  pinMode(emgPinShAA, INPUT);
  
  pinMode(encoder_data_el, INPUT);
  pinMode(encoder_clk_el, INPUT);
  pinMode(encoder_data_fe, INPUT);
  pinMode(encoder_clk_fe, INPUT);
  pinMode(encoder_data_aa, INPUT);
  pinMode(encoder_clk_aa, INPUT);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
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

//  Serial.print(el_pos);
//  Serial.print(',');
//  Serial.print(fe_pos);
//  Serial.print(',');



  String arrString = "";
  
  for (int i = 0; i < 6; i++) {
    arrString += String(data[i]);
    if (i<5)
      arrString += ",";
  }

  Serial.println(arrString);

//  delay(5);
  ShAAcontrol();
  ShFEcontrol();
  Elcontrol();
}

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


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                      gravity compensation                      */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
void Elcontrol(){
  
  digitalWrite(driverPinEla,LOW);
  digitalWrite(driverPinElb,HIGH);
  double f = 0.0;
  f = ((0.0815)*pow(el_pos,3) + (-0.2798)*pow(el_pos,2) + (0.05135)*el_pos + 1.85)*(el_pos);
  if (f > 6.0){
        f = 6.0;
      }else if(f < 0){
        f = 0;
      }
  El_tao = (-384.08 * pow(((f*14.5038)/90), 4) + 789.52*pow(((f*14.5038)/90),3) - 618.81* pow(((f*14.5038)/90),2) + 468.18*((((f*14.5038))/90)));
  
  analogWrite(pressurePinEl,El_tao);
}


void ShFEcontrol(){

  digitalWrite(driverPinShFEa,LOW);
  digitalWrite(driverPinShFEb,HIGH);
  double f = 0.0;

    if (f > 6.0){
        f = 6.0;
      }else if(f < 0){
        f = 0;
      }
  f = ((0.06433)*pow(fe_pos,3) + (-0.1573)*pow(fe_pos,2) + (-0.05318)*fe_pos + 2.8)*fe_pos;

  if (f > 6.0){
        f = 6.0;
      }else if(f < 0){
        f = 0;
      }
      
  ShFE_tao = (-384.08 * pow(((f*14.5038)/90), 4) + 789.52*pow(((f*14.5038)/90),3) - 618.81* pow(((f*14.5038)/90),2) + 468.18*((((f*14.5038))/90)));
  
  analogWrite(pressurePinShFE,ShFE_tao);
}
void ShAAcontrol(){
  
  digitalWrite(driverPinShAAa,LOW);
  digitalWrite(driverPinShAAb,HIGH);
  double f = 0.0;
  f = ((1.334)*pow(aa_pos,5)  +  (-5.584 )*pow(aa_pos,4) + (8.238)*pow(aa_pos,3) + (-4.944)*pow(aa_pos,2) + (0.3681)*aa_pos + 2.8)*aa_pos;
  
  if (f > 6 ){
        f = 6;
      }else if(f < 0){
        f = 0;
      }
  
  ShAA_tao = (-384.08 * pow(((f*14.5038)/90), 4) + 789.52*pow(((f*14.5038)/90),3) - 618.81* pow(((f*14.5038)/90),2) + 468.18*((((f*14.5038))/90)));
  
  analogWrite(pressurePinShAA,ShAA_tao);
}
