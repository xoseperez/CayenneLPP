/*

CayenneLPP

AUnit Unit Tests

Copyright (C) 2019 by Xose Pérez <xose dot perez at gmail dot com>

The rpnlib library is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

The rpnlib library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with the rpnlib library.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <Arduino.h>
#include <CayenneLPP.h>
#include <ArduinoJson.h>
#include <AUnit.h>

using namespace aunit;

#define LPP_TEST_VERBOSE 0
#define LPP_TEST_ENCODER 1
#define LPP_TEST_DECODER 1

// -----------------------------------------------------------------------------
// Test class
// -----------------------------------------------------------------------------

class EncoderTest: public TestOnce {

    protected:

        virtual void setup() override {
            lpp = new CayenneLPP(100);
            lpp->reset();
        }

        virtual void teardown() override {
            delete lpp;
        }

        virtual void compare(unsigned char depth, uint8_t * expected) {
            
            uint8_t * actual = lpp->getBuffer();
            
            #if LPP_TEST_VERBOSE
                PC_SERIAL.println();
                char buff[6];
                PC_SERIAL.print("Expected: ");
                for (unsigned char i=0; i<depth; i++) {
                    snprintf(buff, sizeof(buff), "%02X ", expected[i]);
                    PC_SERIAL.print(buff);
                }
                PC_SERIAL.println();
                PC_SERIAL.print("Actual  : ");
                for (unsigned char i=0; i<lpp->getSize(); i++) {
                    snprintf(buff, sizeof(buff), "%02X ", actual[i]);
                    PC_SERIAL.print(buff);
                }
                PC_SERIAL.println();
            #endif
            
            assertEqual(depth, lpp->getSize());
            for (unsigned char i=0; i<depth; i++) {
                assertEqual(expected[i], actual[i]);
            }

        }

        CayenneLPP * lpp;

};

class DecoderTest: public TestOnce {

    protected:

        virtual void setup() override {
            lpp = new CayenneLPP(10);
            lpp->reset();
        }

        virtual void teardown() override {
            delete lpp;
        }

        virtual void compare(
            uint8_t * buffer, unsigned char len, uint8_t port, 
            uint8_t fields, uint8_t channel0 = 0, uint8_t type0 = 0,
            float value0 = 0, float precission0 = 0) {
            
            StaticJsonDocument<512> jsonBuffer;
            JsonArray root = jsonBuffer.createNestedArray();    
            assertEqual(fields, lpp->decode(buffer, len, port, root));
            assertEqual(fields, (uint8_t) root.size());

            #if LPP_TEST_VERBOSE
                PC_SERIAL.println();
                serializeJsonPretty(root, PC_SERIAL);
                PC_SERIAL.println();
            #endif

            if (channel0 > 0) assertEqual(channel0, root[0]["channel"]);
            if (type0 > 0) assertEqual(type0, root[0]["type"]);
            if (precission0 > 0) assertNear(value0, root[0]["value"], precission0);

        }

        CayenneLPP * lpp;
        

};

// -----------------------------------------------------------------------------
// Tests
// -----------------------------------------------------------------------------

#if LPP_TEST_ENCODER

testF(EncoderTest, Multichannel) {
    lpp->addTemperature(3, 27.2);
    lpp->addRelativeHumidity(2, 54);
    uint8_t expected[] = {0x03,0x67,0x01,0x10,0x02,0x68,0x6C};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Packed) {
    lpp->setMode(LPP_MODE_PACKED);
    lpp->addTemperature(3, 27.2);
    lpp->addTemperature(5, 25.5);
    uint8_t expected[] = {0x67,0x01,0x10,0x67,0x00,0xFF};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, History) {
    lpp->setMode(LPP_MODE_HISTORY);
    lpp->setDelta(60);
    lpp->addPower(1, 200);
    lpp->setDelta(0);
    lpp->addPower(1, 250);
    uint8_t expected[] = {0x80,0x00,0x3C,0x00,0xC8,0x80,0x00,0x00,0x00,0xFA};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Reset) {
    lpp->addTemperature(3, 27.2);
    lpp->reset();
    lpp->addTemperature(5, 25.5);
    uint8_t expected[] = {0x05,0x67,0x00,0xFF};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Limit) {
    lpp->addPower(1, 0xFFFF);
    uint8_t expected[] = {0x01,0x80,0xFF,0xFF};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Negative_Temperature) {
    lpp->addTemperature(5, -4.7);
    uint8_t expected[] = {0x05,0x67,0xFF,0xD1};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Humidity) {
    lpp->addRelativeHumidity(2, 54);
    uint8_t expected[] = {0x02,0x68,0x6C};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Generic_Sensor) {
    lpp->addGenericSensor(1, 998);
    uint8_t expected[] = {0x01,0x64,0x00,0x00,0x03,0xE6};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Accelerometer) {
    lpp->addAccelerometer(6, 1.234, -1.234, 0);
    uint8_t expected[] = {0x06,0x71,0x04,0xD2,0xFB,0x2E,0x00,0x00};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Voltage) {
    lpp->addVoltage(3, 224.56);
    uint8_t expected[] = {0x03,0x74,0x57,0xB8};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Current) {
    lpp->addCurrent(1, 0.237);
    uint8_t expected[] = {0x01,0x75,0x00,0xED};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Frequency) {
    lpp->addFrequency(1, 868100000);
    uint8_t expected[] = {0x01,0x76,0x33,0xBe,0x27,0xA0};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Altitude) {
    lpp->addAltitude(5, -17);
    uint8_t expected[] = {0x05,0x79,0xFF,0XEF};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Power) {
    lpp->addPower(5, 3450);
    uint8_t expected[] = {0x05,0x80,0x0D,0x7A};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Distance) {
    lpp->addDistance(1, 0.034);
    uint8_t expected[] = {0x01,0x82,0x00,0x00,0x00,0x22};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Energy) {
    lpp->addEnergy(1, 7953.2);
    uint8_t expected[] = {0x01,0x83,0x00,0x79,0x5B,0x30};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, Direction) {
    lpp->addDirection(1, 256);
    uint8_t expected[] = {0x01,0x84,0x01,0x00};
    compare(sizeof(expected), expected);
}

testF(EncoderTest, GPS) {
    lpp->addGPS(1, 42.3519, -87.9094, 10);
    uint8_t expected[] = {0x01,0x88,0x06,0x76,0x5e,0xf2,0x96,0x0a,0x00,0x03,0xe8};
    compare(sizeof(expected), expected);
}
 
testF(EncoderTest, GPS_Full_Scale) {
    lpp->addGPSFull(1, 42.351920, -87.909435, 10.30);
    uint8_t expected[] = {0x01,0x02,0x86,0x3D,0x30,0xFA,0xC2,0x9B,0xC8,0x04,0x06};
    compare(sizeof(expected), expected);
}
 
testF(EncoderTest, Switch) {
    lpp->addSwitch(5, 1);
    uint8_t expected[] = {0x05,0x8E,0x01};
    compare(sizeof(expected), expected);
}

#endif

// -----------------------------------------------------------------------------

#if LPP_TEST_DECODER

testF(DecoderTest, Multichannel) {
    uint8_t buffer[] = {0x03,0x67,0x01,0x10,0x05,0x67,0x00,0xFF};
    compare(buffer, sizeof(buffer), 1, 2, 3, LPP_TEMPERATURE, 27.2, 0.01);
}

testF(DecoderTest, Packed) {
    uint8_t buffer[] = {0x67,0x01,0x10,0x67,0x00,0xFF};
    compare(buffer, sizeof(buffer), 2, 2, 0, LPP_TEMPERATURE, 27.2, 0.01);
}

testF(DecoderTest, History) {
    uint8_t buffer[] = {0x80,0x00,0x3C,0x00,0xC8,0x80,0x00,0x00,0x00,0xFA};
    compare(buffer, sizeof(buffer), 101, 2, 1, LPP_POWER, 200, 0.01);
}

testF(DecoderTest, Negative_Temperature) {
    uint8_t buffer[] = {0x05,0x67,0xFF,0xD1}; 
    compare(buffer, sizeof(buffer), 1, 1, 5, LPP_TEMPERATURE, -4.7, 0.01);
}

testF(DecoderTest, GPS) {
    uint8_t buffer[] = {0x01,0x88,0x06,0x76,0x5e,0xf2,0x96,0x0a,0x00,0x03,0xe8}; 
    compare(buffer, sizeof(buffer), 1, 1, 1, LPP_GPS);
}

testF(DecoderTest, GPS_Full_Scale) {
    uint8_t buffer[] = {0x02,0x02,0x86,0x3D,0x30,0xFA,0xC2,0x9B,0xC8,0x04,0x06}; 
    compare(buffer, sizeof(buffer), 3, 1, 2, LPP_GPS);
}

testF(DecoderTest, Voltage) {
    uint8_t buffer[] = {0x03,0x74,0x57,0xB8}; 
    compare(buffer, sizeof(buffer), 1, 1, 3, LPP_VOLTAGE, 224.56, 0.01);
}

testF(DecoderTest, Distance) {
    uint8_t buffer[] = {0x01,0x82,0x00,0x00,0x00,0x22}; 
    compare(buffer, sizeof(buffer), 1, 1, 1, LPP_DISTANCE, 0.034, 0.001);
}

testF(DecoderTest, Frequency) {
    uint8_t buffer[] = {0x01,0x76,0x33,0xBe,0x27,0xA0};
    compare(buffer, sizeof(buffer), 1, 1, 1, LPP_FREQUENCY, 868100000, 0.1);
}

#endif

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------

void setup() {

    PC_SERIAL.begin(115200);
    while (!PC_SERIAL && millis() < 5000);

    Printer::setPrinter(&PC_SERIAL);
    //TestRunner::setVerbosity(Verbosity::kAll);

}

void loop() {
    TestRunner::run();
    delay(1);
}
