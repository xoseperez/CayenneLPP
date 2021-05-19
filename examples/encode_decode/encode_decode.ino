/*

Cayenne LPP Encoder / Decoder

Encode-decode Simple example

*/

#include <Arduino.h>
#include "CayenneLPP.h"
#include "ArduinoJson.h"

#if defined(ARDUINO_ARCH_SAMD)
    #define PC_SERIAL SerialUSB
#else
    #define PC_SERIAL Serial
#endif

void setup() {

    PC_SERIAL.begin(115200);
    while (!PC_SERIAL && millis() < 5000);

    PC_SERIAL.println();
    PC_SERIAL.println("CayenneLPP simple encode-decode example");
    PC_SERIAL.println("=======================================");
    PC_SERIAL.println();

    PC_SERIAL.println("Creating payload buffer...");
    CayenneLPP payload(32);
    
    PC_SERIAL.println("Adding fields...");
    payload.addTemperature(1, 22.5);
    payload.addPower(2, 1204);
    payload.addRelativeHumidity(3, 57);

    PC_SERIAL.print("Current buffer size: "); PC_SERIAL.println(payload.getSize());
    PC_SERIAL.print("Current buffer contents: ");
    char byte[6];
    for (uint8_t i=0; i<payload.getSize(); i++) {
        snprintf(byte, sizeof(byte), "%02X ", payload.getBuffer()[i]);
        PC_SERIAL.print(byte);
    }
    PC_SERIAL.println();

    PC_SERIAL.println("Decoding...");
    DynamicJsonDocument jsonBuffer(512);
    JsonArray root = jsonBuffer.createNestedArray();  
    uint8_t fields = payload.decode(payload.getBuffer(), payload.getSize(), root); 
    serializeJsonPretty(root, PC_SERIAL);
    PC_SERIAL.println();

    PC_SERIAL.println();
    for (uint8_t i=0; i<fields; i++) {
        uint8_t channel = root[i]["channel"].as<int>();
        const char * name = root[i]["name"].as<char *>();
        float value = root[i]["value"].as<float>();
        PC_SERIAL.print("Channel "); PC_SERIAL.print(channel); 
        PC_SERIAL.print(" ("); PC_SERIAL.print(name); PC_SERIAL.print("): ");
        PC_SERIAL.print(value);
        PC_SERIAL.println();
    }

    PC_SERIAL.println();

}

void loop() {
    delay(1);
}
