
#include <avr/power.h>
#include <Wire.h>
#include <Encoder.h>
#include <LiquidCrystal.h>
#include "PD_UFP.h"

// const uint8_t encoderPin1 = 0;
// const uint8_t encoderPin2 = 1;

const uint8_t encoderPin1 = 5;
const uint8_t encoderPin2 = 6;

const uint8_t encoderBtnPin = 4;

class PD_UFP_c PD_UFP;
LiquidCrystal lcd(19, 18, 15, 14, 16, 20);
Encoder mainKnob(encoderPin1, encoderPin2);

unsigned long lastUpdate = 0;

bool isPPS = false;
float maxVoltage = 0.0;
float maxCurrent = 0.0;

float toRealVolt(uint16_t v, bool PPS = false){
    if(PPS){
        return (v-0.01)/50.0;
    }
    return (v-0.01)/20.0;
}

float toRealAmp(uint16_t a, bool PPS = false){
    if(PPS){
        return (a-0.01)/20.0;
    }
    return (a-0.01)/100.0;
}

void displayUpdate(void){
  lastUpdate = millis();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(toRealVolt(PD_UFP.get_voltage(), isPPS));
  //Serial1.print((float)((PD_UFP.get_voltage()-0.01)/50));
  lcd.print("V");
  lcd.setCursor(0, 1);
  lcd.print(toRealAmp(PD_UFP.get_current(), isPPS));
  lcd.print("A");
}

void debugPulse(){
  digitalWrite(9, HIGH);
  delay(1);
  digitalWrite(9, LOW);
}

void testPSU(){
  // test PPS
  PD_UFP.init_PPS(PPS_V(5.0), PPS_A(1.0), PD_POWER_OPTION_MAX_5V);
  unsigned long testMillis = millis();
  while(millis() - testMillis < 1000){
    PD_UFP.run();
    if (PD_UFP.is_PPS_ready()) {
      isPPS = true;
      break;
    }
  }
  // if PPS is available then test max voltage and current
  if(isPPS){
    // check current
    for (uint8_t i = 0; i <= PPS_A(5.0); i+=PPS_A(0.05)) {
      if(PD_UFP.set_PPS(PPS_V(5.0), i)){
        while(PD_UFP.is_ps_transition()){
          PD_UFP.run();
        }
        if (PD_UFP.is_PPS_ready()) {
          maxCurrent = toRealAmp(PD_UFP.get_current(), isPPS);
        }
      }
    }

    // check voltage
    for (uint16_t i = PPS_V(5.0); i <= PPS_V(21.0); i+=PPS_V(1.00)) {
      if(PD_UFP.set_PPS(i, PPS_A(maxCurrent))){
        while(PD_UFP.is_ps_transition()){
          PD_UFP.run();
        }
        if (PD_UFP.is_PPS_ready()) {
          maxVoltage = toRealVolt(PD_UFP.get_voltage(), isPPS);
        }
      }
    }
     
  }else{
    // check voltage
    PD_UFP.set_power_option(PD_POWER_OPTION_MAX_VOLTAGE);
    while(PD_UFP.is_ps_transition()){
      PD_UFP.run();
    }      
    maxVoltage = toRealVolt(PD_UFP.get_voltage(), isPPS);  
    
    // check current
    PD_UFP.set_power_option(PD_POWER_OPTION_MAX_CURRENT);
    while(PD_UFP.is_ps_transition()){
      PD_UFP.run();
    }
    maxCurrent = toRealAmp(PD_UFP.get_current(), isPPS);
  }
}

void setup() {
  pinMode(encoderBtnPin, INPUT_PULLUP);
  pinMode(1, OUTPUT);
  pinMode(9, OUTPUT);
  Serial1.begin(115200);
  // while(!Serial1) {
  //   ;
  // }
  // Serial1.println("Encoder Test:");
  Wire.begin();
  //PD_UFP.init_PPS(PPS_V(5.31), PPS_A(2.5));
  //PD_UFP.init(PD_POWER_OPTION_MAX_20V);


  // PD_UFP.clock_prescale_set(2);
  // clock_prescale_set(clock_div_2);

  //PD_UFP.init_PPS(PPS_V(5.3), PPS_A(1.0));
  PD_UFP.clock_prescale_set(2);
  clock_prescale_set(clock_div_2);

  lcd.begin(8, 2);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Testing");
  lcd.setCursor(0, 1);
  lcd.print("PSU...");
  testPSU();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(maxVoltage);
  //Serial1.print((float)((PD_UFP.get_voltage()-0.01)/50));
  lcd.print("V");
  lcd.setCursor(0, 1);
  lcd.print(maxCurrent);
  lcd.print("A");
  if(isPPS){
    lcd.print("PPS");
  }
  delay(5000);

  PD_UFP.set_PPS(PPS_V(5.0), PPS_A(2.8));
}

long position = -999;

void loop() {
  long newPos;
  newPos = mainKnob.read()/4;
  if (newPos != position) {
    // Serial1.print("Left = ");
    // Serial1.print(newPos);
    // Serial1.println();
    position = newPos;
  }
  // if a character is sent from the Serial1 monitor,
  // reset both back to zero.
  if (Serial1.available()) {
    // Serial1.read();
    // Serial1.println("Reset both knob to zero");
    mainKnob.write(0);
  }

  PD_UFP.run();
  if (PD_UFP.is_PPS_ready()) {          // PPS trigger success
    PD_UFP.set_output(1);               // Turn on load switch 
    PD_UFP.set_led(1);                  // PPS output 4.2V 2.0A ready
  } else if (PD_UFP.is_power_ready()) { // Fail to trigger PPS, fall back
    PD_UFP.set_output(0);               // Turn off load switch
    PD_UFP.blink_led(400);              // blink LED
  }

  if (PD_UFP.is_PPS_ready()) {
    isPPS = true;
    digitalWrite(9, HIGH);
  }else{
    isPPS = false;
    digitalWrite(9, LOW);
  }
  // if (PD_UFP.is_power_ready()) { 
  //   if (PD_UFP.get_voltage() == PD_V(20.0) && PD_UFP.get_current() >= PD_A(1.5)) {
  //     PD_UFP.set_output(1);   // Turn on load switch 
  //     PD_UFP.set_led(1);      // Output reach 20V and 1.5A, set indicators on
  //   } else {
  //     PD_UFP.set_output(0);   // Turn off load switch
  //     PD_UFP.blink_led(400);  // Output less than 20V or 1.5A, blink LED
  //   }
  // }

  if(millis() - lastUpdate > 100) {
    displayUpdate();
  }
}
