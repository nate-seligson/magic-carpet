#include "HX711.h"

#define B_KNEE 8
#define B_UP 9
#define B_DOWN 10

#define RED 11
#define BLUE 12
#define GREEN 13

#define R_DOUT 3
#define L_DOUT 4
#define B_DOUT 5

#define BUZZER 7
#define CLK  2

HX711 R_scale, L_scale, B_scale;

int calibration_factor = 100;

bool knee_calibrated = false;
bool up_calibrated = false;
bool down_calibrated = false;

int CALIBRATION_STEPS = 10;

long knee_high = 0;
long knee_low = 0;

long R_high = 0;
long R_static= 0;
long R_low= 0;

long L_high = 0;
long L_static= 0;
long L_low= 0;

int threshold = 10;
long knee_threshold = 400;

void beep(float frequency, int duration){
  tone(BUZZER, frequency);
  delay(duration);
  noTone(BUZZER);
}

void musical_intro(){
  beep(369.99, 500);
  beep(329.63, 250);
  beep(392.00, 500);
  beep(369.99, 250);
  beep(293.66, 500);
  beep(220, 1000);
}
void SetColorLerp(double left, double right, double bottom){
  int red = (double)(left + 1) / (double)2 * 255;
  int green = (double)(right + 1) / (double)2 * 255;
  int blue = (double)(bottom + 1) / (double)2 * 255;
  setColor(red,green,blue);
}
void setColor(int red, int green, int blue) {
  analogWrite(RED, red);
  analogWrite(GREEN, green);
  analogWrite(BLUE, blue);
}

void show_uncalibrated(){
  static unsigned long lastFlash = 0;
  static int idx = 0;
  int interval = 500;
  bool uncalibrated[3] = {!knee_calibrated, !up_calibrated, !down_calibrated};

  if(!(uncalibrated[0] || uncalibrated[1] || uncalibrated[2])){
    setColor(0,0,0);
    idx = 0;
    return;
  }
  long now = millis();
  if(now - lastFlash < interval) return;
  lastFlash = now;

  for(int i = 0; i<3; ++i){
    idx = (idx + 1) % 3;
    if(uncalibrated[idx]) break;
  }

  switch(idx){
    case 0: setColor(255,0,0); break;
    case 1: setColor(255,255,255); break;
    case 2: setColor(0,0,255); break;
  }

}

void CALIBRATE_LOOP(int color, long &L_average, long &R_average, long &K_average) {
  for (int i = 0; i < CALIBRATION_STEPS; i++) {

    switch(color){
      case 0:setColor(255,0,0); break;
      case 1:setColor(255,255,255); break;
      case 2:setColor(0,0,255);break;
    }

    if (i % ((CALIBRATION_STEPS / 3) + 1) == 0) {
      beep(440, 100);
    }

    R_average += R_scale.get_units();
    L_average += L_scale.get_units();
    K_average += B_scale.get_units();
    delay(60);
  }
}

void CALIBRATE_KNEE(){
  long R_average = 0;
  long L_average = 0;
  long K_average = 0;
  CALIBRATE_LOOP(0, L_average, R_average, K_average);

  L_average /= CALIBRATION_STEPS;
  R_average /= CALIBRATION_STEPS;
  K_average /= CALIBRATION_STEPS;

  L_static = L_average;
  R_static = R_average;
  knee_high= K_average;
  
  knee_calibrated = true;

  beep(587.33, 250);
}

void CALIBRATE_UP() {
  long R_average = 0;
  long L_average = 0;
  long K_average = 0;
  CALIBRATE_LOOP(1, L_average, R_average, K_average);

  L_average /= CALIBRATION_STEPS;
  R_average /= CALIBRATION_STEPS;
  K_average /= CALIBRATION_STEPS;

  L_high = L_average;
  R_high = R_average;

  up_calibrated = true;

  beep(587.33, 250);
}

void CALIBRATE_DOWN() {
  long R_average = 0;
  long L_average = 0;
  long K_average = 0;
  CALIBRATE_LOOP(2, L_average, R_average, K_average);

  L_average /= CALIBRATION_STEPS;
  R_average /= CALIBRATION_STEPS;
  K_average /= CALIBRATION_STEPS;

  L_low = L_average;
  R_low = R_average;

  down_calibrated = true;

  beep(587.33, 250);
}

void setup() {
  Serial.begin(115200);
  //init buttons
  pinMode(B_KNEE, INPUT_PULLUP);
  pinMode(B_UP, INPUT_PULLUP);
  pinMode(B_DOWN, INPUT_PULLUP);
  //init rgbled
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  musical_intro();
  //init scales
  R_scale.begin(R_DOUT, CLK);
  R_scale.set_scale(calibration_factor);
  R_scale.set_gain(64);
  R_scale.tare(); //Reset the scale to 0x

  setColor(255,0,0);
  beep(349.23, 250);

  L_scale.begin(L_DOUT, CLK);
  L_scale.set_scale(calibration_factor);
  L_scale.set_gain(64);
  L_scale.tare(); //Reset the scale to 0x

  setColor(255,255,255);
  beep(440.00, 250);

  B_scale.begin(B_DOUT, CLK);
  B_scale.set_scale(calibration_factor);
  B_scale.set_gain(64);
  B_scale.tare(); //Reset the scale to 0x
  knee_low = B_scale.get_units(10);
  setColor(0,0,255);
  beep(523.25, 250);
  setColor(0,0,0);

  delay(300);

  setColor(255,0,255);
  beep(698.46, 100);
  setColor(0,0,0);

  delay(100);

  setColor(255,0,255);
  beep(698.46, 100);
  setColor(0,0,0);
}

void loop() {
  // Check for serial commands: 'k' = knee, 'u' = up, 'd' = down
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'k') {
      CALIBRATE_KNEE();
    } else if (cmd == 'u') {
      CALIBRATE_UP();
    } else if (cmd == 'd') {
      CALIBRATE_DOWN();
    }
      else if(cmd == 'o'){
        threshold++;

      }
      else if(cmd == 'l'){
        threshold--;
        
      }
    else if (cmd=='s'){
      while(true){
          long r_reading = R_scale.get_units();
          long l_reading = L_scale.get_units();
          long b_reading = B_scale.get_units();
          Serial.print(l_reading);
          Serial.print(',');
          Serial.print(r_reading);
          Serial.print(',');
          Serial.print(b_reading);
          Serial.print(',');
          Serial.println();
          delay(10);
          
      }
    }
  }

  long r_reading = R_scale.get_units();
  long l_reading = L_scale.get_units();
  long b_reading = B_scale.get_units();

  if(digitalRead(B_KNEE) == LOW){
    CALIBRATE_KNEE();
  }
  else if(digitalRead(B_UP)==LOW){
    CALIBRATE_UP();
  }
  else if(digitalRead(B_DOWN)==LOW){
    CALIBRATE_DOWN();
  }
  else{
    Serial.println("0,0,-1");
    show_uncalibrated();
  }

  if(knee_calibrated && up_calibrated && down_calibrated){
    double r_value, l_value, k_value;
    if(abs(r_reading - R_static) > threshold){
      double r_ratio = (R_high - R_low) / 2;
      r_value = min(max((r_reading - R_low - r_ratio)/r_ratio,-1),1);
    }
    else{
      r_value = 0;
    }
    if(abs(l_reading - L_static) > threshold){
      double l_ratio = (L_high - L_low) / 2;
      l_value = min(max((l_reading - L_low - l_ratio)/l_ratio,-1),1);
    }
    else{
      l_value = 0;
    }
    if (abs(b_reading - knee_high) < knee_threshold) {
      k_value = min(max(double(b_reading) / double(knee_high), 0), 1);
    } else if (abs(b_reading - knee_low) < knee_threshold) {
      k_value = -min(max(double(b_reading) / double(knee_low), 0), 1);
    } else {
      k_value = 0;
    }

    SetColorLerp(l_value, r_value, k_value);
    Serial.print(l_value);
    Serial.print(',');
    Serial.print(r_value);
    Serial.print(',');
    Serial.print(k_value);
    Serial.println();
  }
}