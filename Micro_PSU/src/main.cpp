
#include <avr/power.h>
#include <Wire.h>
#include <Encoder.h>
#include <LiquidCrystal.h>
#include "PD_UFP.h"
#include <symbols.h>

//#include <SoftwareSerial.h>// TODO remove this later
//SoftwareSerial swSerial (8, 9);


const uint8_t encoderPin1 = 1;
const uint8_t encoderPin2 = 0;

// const uint8_t encoderPin1 = 4;
// const uint8_t encoderPin2 = 5;

const uint8_t encoderBtnPin = 4;

class PD_UFP_c PD_UFP;
LiquidCrystal lcd(19, 18, 15, 14, 16, 20);
Encoder mainKnob(encoderPin1, encoderPin2);

unsigned long lastUpdate = 0;
unsigned long lastUpdate2 = 0;

bool isPPS = false;
float maxVoltage = 0.0;
float maxCurrent = 0.0;

uint16_t requestedVoltage = 0;
uint16_t requestedCurrent = 0;

bool outputEN = false;

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

// button pooling function, returns 0 if button is not pressed, 1 if button is pressed, 2 if long press
uint8_t btnPooling(bool btn){
  const unsigned int longPressInterval = 500;
  static uint8_t status = 0;
  static uint8_t oldStatus = 0;
  static unsigned long gpTimer = 0;
  uint8_t ret = 0;
  if (btn == LOW && status == 0){
    status = 1;
    gpTimer = millis();
  }else if (!btn && status == 1 && millis() - gpTimer >= 20){
    status = 2;
    gpTimer = millis();
  }else if (!btn && status == 2 && millis() - gpTimer >= longPressInterval){
    status = 3;
  }else if (btn == HIGH){
    status = 0;
  }
  if (status != oldStatus){
    if (status == 3){
      ret = 2;
    }else if (status == 0 && oldStatus != 3){
      ret = 1;
    }else{
      ret = 0;
    }
    oldStatus = status;
  }
  return ret;
}

void printCustomChar(byte arr[], uint8_t slot, uint8_t x, uint8_t y){
  lcd.createChar(slot, arr);
  lcd.setCursor(x, y);
  lcd.write(byte(slot));
}

long position = 0;

void UIUpdate(void){
  static uint16_t lastVoltage = 1;
  static uint16_t lastCurrent = 1;
  static bool editing = false;
  static uint8_t editSubject = 0;

  static uint8_t animationStage = 0;
  if(millis() - lastUpdate2 > 300){
    lastUpdate2 = millis();
    if(outputEN){
      printCustomChar(outEnAnim[animationStage], 1, 7, 0);
      animationStage++;
      if(animationStage >= 5){
        animationStage = 0;
      }
    }else{
      animationStage = 0;
      printCustomChar(outDis, 1, 7, 0);
    }
  }

  if((lastVoltage != requestedVoltage || lastCurrent != requestedCurrent) || (editing && millis() - lastUpdate >= 100)){
    lastVoltage = requestedVoltage;
    lastCurrent = requestedCurrent;
    lastUpdate = millis();
    lcd.clear();
    if(outputEN){
      printCustomChar(outEnAnim[animationStage], 1, 7, 0);
    }else{
      printCustomChar(outDis, 1, 7, 0);
    }
    lcd.setCursor(0, 0);
    lcd.print(toRealVolt(lastVoltage, isPPS));
    lcd.print("V");
    if(editing && editSubject == 0)
      lcd.print("<");
    lcd.setCursor(0, 1);
    lcd.print(toRealAmp(lastCurrent, isPPS));
    lcd.print("A");
    if(editing && editSubject == 1)
      lcd.print("<");
  }

  uint8_t button = btnPooling(digitalRead(encoderBtnPin));

  if(button == 2){
    outputEN = !outputEN;
  }else if(button == 1 && !editing){
    editing = true;
  }else if(button == 1 && editing){
    editSubject++;
    if(editSubject >= 2){
      editSubject = 0;
      editing = false;
      lcd.setCursor(0, 1);
      lcd.print(toRealAmp(lastCurrent, isPPS));
      lcd.print("A ");
    if(editing && editSubject == 1)
      lcd.print("<");
    }else if(editSubject == 0){
      mainKnob.write(lastVoltage);
      position = lastVoltage;
    }else if(editSubject == 1){
      mainKnob.write(lastCurrent);
      position = lastCurrent;
    }
  }

  if(editing && position >= 65535){
    mainKnob.write(0);
    position = 0;
  }
}

void lcdLoading(uint8_t x, uint8_t y){
  static uint8_t i = 0;
  if(millis() - lastUpdate > 300){
    if(i >= 3){
      i = 0;
      lcd.setCursor(x, y);
      lcd.print("   ");
    }else{
      lcd.setCursor(x + i, y);
      lcd.print('.');
      i++;
    }
    lastUpdate = millis();
  }
}

void testPSU(){
  // test PPS
  PD_UFP.init_PPS(PPS_V(5.0), PPS_A(1.0), PD_POWER_OPTION_MAX_5V);
  lastUpdate2 = millis();
  while(millis() - lastUpdate2 < 1000){
    PD_UFP.run();
    lcdLoading(3, 1);
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
          lcdLoading(3, 1);
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
          lcdLoading(3, 1);
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
      lcdLoading(3, 1);
    }      
    maxVoltage = toRealVolt(PD_UFP.get_voltage(), isPPS);  
    
    // check current
    PD_UFP.set_power_option(PD_POWER_OPTION_MAX_CURRENT);
    while(PD_UFP.is_ps_transition()){
      PD_UFP.run();
      lcdLoading(3, 1);
    }
    maxCurrent = toRealAmp(PD_UFP.get_current(), isPPS);
  }
}

void setup() {
  pinMode(encoderBtnPin, INPUT_PULLUP);
  pinMode(1, OUTPUT);
  pinMode(9, OUTPUT);
  //Serial1.begin(9600);

  Wire.begin();
  PD_UFP.clock_prescale_set(2);
  clock_prescale_set(clock_div_2);

  lcd.begin(8, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Testing");
  lcd.setCursor(0, 1);
  lcd.print("PSU");
  lcdLoading(3, 1);

  testPSU();

  lcd.clear();
  char message[] = "Results: Max Volt: ";
  lcd.setCursor(0, 0);
  lcd.print(message);
  lcd.print(maxVoltage);
  lcd.print("V");
  char message2[] = " Max Curr: ";
  lcd.setCursor(0, 1);
  lcd.print("PPS: ");
  if(isPPS)
    lcd.print("Yes");
  else
    lcd.print("No ");
  lcd.print(message2);
  lcd.print(maxCurrent);
  lcd.print("A");
  lastUpdate = millis();
  while (millis() - lastUpdate < 1000)
  {
    if(btnPooling(digitalRead(encoderBtnPin))){
      goto done;
    }
  }
  for (uint8_t i = 0; i < sizeof(message) - 1; i++) {
    lcd.scrollDisplayLeft();
    lastUpdate = millis();
    while (millis() - lastUpdate < 200)
    {
      if(btnPooling(digitalRead(encoderBtnPin))){
        goto done;
      }
    }
  }
  done:
  lastUpdate2 = millis();
  while(millis() - lastUpdate2 < 3000){
    if(millis() - lastUpdate > 100) {
      UIUpdate();
      lastUpdate = millis();
    }
    if(btnPooling(digitalRead(encoderBtnPin))){
        break;
    }
  }
  if(isPPS)
    PD_UFP.set_PPS(PPS_V(5.0), PPS_A(1.0));
}

void loop() {
  position = mainKnob.read()/4;
  UIUpdate();
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
}


