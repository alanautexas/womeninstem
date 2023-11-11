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

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Wire.h>
#include <Arduino_APDS9960.h>
 #include <bits/stdc++.h>

#include <Arduino.h>
#include <Bluepad32.h>

#include <ESP32Servo.h>
#include <ESP32SharpIR.h>
#include <QTRSensors.h>

// Definitions for sensors
#define LED 2   // LED output pin
#define APDS9960_INT 2
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000

//Color sensor unit and I2C Unit 
TwoWire I2C_0 =  TwoWire (0);
APDS9960 apds = APDS9960 (I2C_0, APDS9960_INT);




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

// Initialized servos and sensors
Servo servoleft;
Servo servoright;

ESP32SharpIR distance_sensor1(ESP32SharpIR::GP2Y0A21YK0F, 36);
QTRSensors line_sensor; 

// Setup controller
GamepadPtr controller = myGamepads[0];

// Arduino setup function. Runs in CPU 1
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();

    I2C_0.begin (I2C_SDA, I2C_SCL, I2C_FREQ);

apds.setInterruptPin(APDS9960_INT);
apds.begin();

    // Serial setup
    Serial.begin(115200);

    // LED setup
    pinMode(LED, OUTPUT);

    //Servo setup
    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
    servoleft.setPeriodHertz(50);
    servoleft.attach(13,1000,2000);
    
    servoright.setPeriodHertz(50);
    servoright.attach(14,1000,2000);

    // Distance sensor setup
    distance_sensor1.setFilterRate(0.1f);

    // Line sensor setup
    line_sensor.setTypeAnalog();
    line_sensor.setSensorPins((const uint8_t[]) {39,34,35}, 3);
    for (uint8_t i = 0; i < 250; i++)
    {
        Serial.println("calibrating");
        line_sensor.calibrate();
        delay(20);
    }



}

void LED_loop() {
    for (int i= 0; i < 2; i++){
        digitalWrite(LED, HIGH);
        delay(100);
    digitalWrite(LED, LOW);
    delay(100);
    }
}

// void rightservo() {
//     servoright.write(((((float) controller -> axisY())/512.0f)*500) + 1500);
// }

// void leftservo() {
//     servoleft.write(((((float) controller -> axisY())/512.0f)*500) + 1500);
// }

void loop() {

    BP32.update();

    // Brownout code
 

    // Blinky LED
    // LED_loop();

    // Move Chassis (straight and back)
     //servoleft.write(1000);    
     //servoright.write(2000);
    // delay(1000);

     //servoleft.write(1500);
    // servoright.write(1000);
    // delay(1000);

    // Something else for line sensor
    
 

    // Distance sensor
  // Serial.println(distance_sensor1.getDistanceFloat()); // Read from sensor
    // delay(250);

Servo servoArm;
servoArm.setPeriodHertz(50);
servoArm.attach(27,1000,2000);
// bool throws = false;






//      int r,g,b,a;
//      while(!apds.colorAvailable()){
// delay(500);
//      }
// apds.readColor(r, g, b, a);

// Serial.print(" RED:");
// Serial.print(r);
// Serial.println(" GREEN:");
// Serial.print(g);
// Serial.println(" BLUE:");
// Serial.print(b);
// Serial.println(" AMBIENT:");
// Serial.print(a);
// Serial.println();

// if(r > g && r > b){
// for (int i= 0; i < 2; i++){
//         digitalWrite(LED, HIGH);
//         delay(200);
//     digitalWrite(LED, LOW);
//     delay(200);
//     }

// delay(1000);
// }

// if(g > r && g > b){
//     for (int i= 0; i < 3; i++){
//         digitalWrite(LED, HIGH);
//         delay(200);
//     digitalWrite(LED, LOW);
//     delay(200);
//     }
//    delay(1000); 
// }

// if( b > g && b > r){
//     for (int i= 0; i < 4; i++){
//         digitalWrite(LED, HIGH);
//         delay(200);
//     digitalWrite(LED, LOW);
//     delay(200);
//     } 
//     delay(1000);
// }
/*if(distance_sensor1.getDistanceFloat() < 13.0){
         Serial.println(distance_sensor1.getDistanceFloat()); // Read from sensor
        servoleft.write(1000);
       servoright.write(2000);
      
        //servoleft.write(1000);
       //servoright.write(1500);

      // servoleft.write(1000);
      // servoright.write(2000);

        //delay(1000);
    }
  else if (distance_sensor1.getDistanceFloat()  >= 13.0 && distance_sensor1.getDistanceFloat() < 18.0){
Serial.println(distance_sensor1.getDistanceFloat()); // Read from sensor
        servoleft.write(1500);
       servoright.write(2000);
     }
     
    else{
        Serial.println(distance_sensor1.getDistanceFloat()); // Read from sensor
        servoleft.write(2000);
       servoright.write(1000);
        //delay(1000);
    }*/

    GamepadPtr controller = myGamepads[0];
    if(controller && controller ->isConnected()) {
        digitalWrite(LED, HIGH);
        Serial.println("connected");
        servoleft.write(1500);
        servoright.write(1500);

       // back forward
        if(((((float) controller -> axisY())/512.0f)*500) > 0){ //straight forward
            servoleft.write(((((float) controller -> axisY())/512.0f)*500) + 1500);
            servoright.write(((((float) controller -> axisY())/512.0f)*500*(-1)) + 1500);
        }

        if(((((float) controller -> axisY())/512.0f)*500) < 0){ //straight backward
            servoleft.write(((((float) controller -> axisY())/512.0f)*500) + 1500);//
            servoright.write(((((float) controller -> axisY())/512.0f)*500*(-1)) + 1500);
        }

        //left and right 
        if(((((float) controller -> axisX())/512.0f)*500) > 0){//turn right
            servoleft.write(((((float) controller -> axisX()))/512.0f)*500+ 1500);
            servoright.write(((((float) controller -> axisX())/512.0f)*500) + 1500);
        }
        if(((((float) controller -> axisX())/512.0f)*500) < 0){//turn left
            servoleft.write(((((float) controller -> axisX())/512.0f)*500) + 1500);
            servoright.write(((((float) controller -> axisX())/512.0f)*500) + 1500);
        }


        //SERVOARM !!!!
        servoArm.write(1500);
          if(((((float) controller ->axisRY())/512.0f)*500) > 0){ //straight forward
            servoArm.write(((((float) controller -> axisRY())/512.0f)*500*(-1)) + 1500);
        }

        if(((((float) controller ->axisRY())/512.0f)*500) < 0){ //straight backward
            servoArm.write(((((float) controller -> axisRY())/512.0f)*500*(-1)) + 1500);
        }





        // if(controller ->()){
        //     throws = true;
        //     Serial.print("slayed");

        //     if (throws == true){
        //         Serial.print(" RED:");
        //         servoArm.write(1500);
        //         Serial.print("before delay");
        //         //delay(500);
        //        // Serial.print("after delay");
        //         throws = false;
        //     }
        //     Serial.print("left if");


        // }
   }
    
   
    vTaskDelay(1);
    delay(100);





   // STARTER CODE 
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

    // Serial.println(sensor1.getDistanceFloat());

 uint16_t sensors[3];
    int16_t position = line_sensor.readLineBlack(sensors);
    int16_t error = position - 1000;
    Serial.println(error);
    //  if (error < 0 && error >-20)
    // {
    //  Serial.println("On the side");
    //     servoright.write(error * (calibrationValue)*(-1)+1500); // we need to figure out the calibrationValue !!
    //     servoleft.write(error * (calibrationValue) + 1500);
    // servoright.write(1500);
    // servoleft.write(2000);
    
    // }
   
    // else {
    //     Serial.println("On the line");
    // //     servoright.write(error * (calibrationValue) + 1500);
    // //     servoleft.write(error * (calibrationValue)*(-1) +1500);
    // servoright.write(1000);
    // servoleft.write(1500);
    //  }
   
   vTaskDelay(1);
    delay(100);
    }

