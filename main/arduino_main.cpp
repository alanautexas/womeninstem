/****************************************************************************
Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.


    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
limitations under the License.
****************************************************************************/
#define LED 2

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>

#include <ESP32Servo.h>
#include <ESP32SharpIR.h>
#include <QTRSensors.h>

//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            // Console.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            // GamepadProperties properties = gp->getProperties();
            // Console.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n", gp->getModelName(), properties.vendor_id,
            //                properties.product_id);
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        // Console.println("CALLBACK: Gamepad connected, but could not found empty slot");
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            // Console.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }
    if (!foundGamepad) {
        // Console.println("CALLBACK: Gamepad disconnected, but not found in myGamepads");
    }
}

Servo servoleft;
Servo servoright;
ESP32SharpIR sensor1( ESP32SharpIR::GP2Y0A21YK0F, 36);
QTRSensors qtr;

// Arduino setup function. Runs in CPU 1
void setup() {
servoleft.setPeriodHertz(50);
servoleft.attach(13,1000,2000);
pinMode (LED, OUTPUT);
servoright.setPeriodHertz(50);
servoright.attach(14,1000,2000);




    // Console.printf("Firmware: %s\n", BP32.firmwareVersion());

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

    

    BP32.forgetBluetoothKeys();

     Serial.begin(115200);
    sensor1.setFilterRate(0.1f);
    //pinMode(36, INPUT);

    //qtr.setTypeRC(); // or 
    //setTypeAnalog()
 //   qtr.setSensorPins((const uint8_t[]) {36,39,34}, 3);
   /* for (uint8_t i = 0; i < 250; i++)
    {
      //  Serial.println("calibrating");
      //  qtr.calibrate();
    //  delay(20);
   // }
     //qtr.calibrate();*/
}
void ledloop(){
for (int i= 0; i < 2; i++){
    digitalWrite(LED, HIGH);
    delay(100);
digitalWrite(LED, LOW);
delay(100);
}
}
GamepadPtr controller = myGamepads[0];
void rightservo(){
servoright.write(((((float) controller -> axisY())/512.0f)*500) +1500);
}

void leftservo(){
servoleft.write(((((float) controller -> axisY())/512.0f)*500) +1500);
}


#define DEFAULT_SPEED 2000

void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    
    /* 10/16
    servoleft.write(1000);
    
    servoright.write(2000);
    delay(1000);
    servoleft.write(2000);
    
    servoright.write(1000);
    delay(1000);
    */





    BP32.update();
    // Serial.println( controller -> axisY());
    //servoleft.write(1750);
    // ledloop();

   /* int irValue = digitalRead(36);
if(irValue == HIGH){
    Serial.println("OBJECT");
}
else {
    Serial.println("NO OBJ");
}
 delay(1000);*/
 
    

   
    if(controller && controller ->isConnected()){
        // Console.printf() // << to see values of controller
        //float controller_value_y = ((float) controller -> axisY()/512.0f)*500;
        //float controller_value_x = ((float) controller -> axisX()/512.0f)*500;

       // if((((float) controller -> axisY())/512.0f)*500) {
            
       // }
       // else {}
        digitalWrite(LED, HIGH);
        
        //forwards
        if((((((float) controller -> axisY())/512.0f)*500) > 0)){
            servoright.write(((((float) controller -> axisY())/512.0f)*500) +1500);
            servoleft.write(((((float) controller -> axisY())/512.0f)*500*(-1)) +1500);
        }
        //backwards
        if((((((float) controller -> axisY())/512.0f)*500) < 0)){
            servoright.write(((((float) controller -> axisY())/512.0f)*500) +1500);
            servoleft.write(((((float) controller -> axisY())/512.0f)*500*(-1)) +1500));
        }
        //right
        if((((((float) controller -> axisx())/512.0f)*500) > 0)){
            servoright.write(((((float) controller -> axisX())/512.0f)*500*(-1)) +1500);
            servoleft.write(((((float) controller -> axisX())/512.0f)*500 +1500));
        }
        //left
        if((((((float) controller -> axisX())/512.0f)*500) < 0)){
            servoright.write(((((float) controller -> axisX())/512.0f)*500) +1500);
            servoleft.write(((((float) controller -> axisX()))/512.0f)*500 *(-1) +1500));
        }

        
    }
   

    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    //for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
       // GamepadPtr myGamepad = myGamepads[i];
       // if (myGamepad && myGamepad->isConnected()) {

           // servo.write( ((((float) myGamepad->axisY()) / 512.0f) * 500) + 1500 );

            // Another way to query the buttons, is by calling buttons(), or
            // miscButtons() which return a bitmask.
            // Some gamepads also have DPAD, axis and more.
            // Console.printf(
            //     "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, "
            //     "%4d, brake: %4d, throttle: %4d, misc: 0x%02x\n",
            //     i,                        // Gamepad Index
            //     myGamepad->dpad(),        // DPAD
            //     myGamepad->buttons(),     // bitmask of pressed buttons
            //     myGamepad->axisX(),       // (-511 - 512) left X Axis
            //     myGamepad->axisY(),       // (-511 - 512) left Y axis
            //     myGamepad->axisRX(),      // (-511 - 512) right X axis
            //     myGamepad->axisRY(),      // (-511 - 512) right Y axis
            //     myGamepad->brake(),       // (0 - 1023): brake button
            //     myGamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button
            //     myGamepad->miscButtons()  // bitmak of pressed "misc" buttons
            // );

            // You can query the axis and other properties as well. See Gamepad.h
            // For all the available functions.
       // }
   // }

    Serial.println(sensor1.getDistanceFloat());
    delay(500);

    // if(sensor1.getDistanceFloat() < tooClose //we to determine a vaue for tooClose){
    //     servoright.write(1000);
    //     servoleft.write(2000);
    // }

    //  uint16_t sensors[3];
    // int16_t position = qtr.readLineBlack(sensors);
    // int16_t error = position - 1000;
    // if (error < 0)
    // {
    //     // Serial.println("On the left");
    //     servoright.write(error * (calibrationValue)*(-1)+1500); // we need to figure out the calibrationValue !!
    //     servoleft.write(error * (calibrationValue) + 1500);
    // }
    // if (error > 0)
    // {
    //     // Serial.println("On the right");
    //     servoright.write(error * (calibrationValue) + 1500);
    //     servoleft.write(error * (calibrationValue)*(-1) +1500);
    // }
    // if(error == 0){
    //     // Serial.println("Straight Ahead");  
    // }
    // vTaskDelay(1);
    // delay(100);
}
