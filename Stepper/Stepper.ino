#include <Arduino.h>
#include <ESPStepperMotorServer.h>

ESPStepperMotorServer *stepperMotorServer;

const char *wifiName= "MakeLab-guest2"; // enter the SSID of the wifi network to connect to
const char *wifiSecret = "20182018"; // enter the password of the the existing wifi network here

void setup() 
{
  Serial.begin(115200);
  stepperMotorServer = new ESPStepperMotorServer(ESPServerRestApiEnabled | ESPServerWebserverEnabled | ESPServerSerialEnabled);
  stepperMotorServer->setWifiCredentials(wifiName, wifiSecret);
  stepperMotorServer->setWifiMode(ESPServerWifiModeClient); //start the server as a wifi client (DHCP client of an existing wifi network)
  stepperMotorServer->start();
}

void loop() 
{
}