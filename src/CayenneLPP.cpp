// Adapted from https://developer.mbed.org/teams/myDevicesIoT/code/Cayenne-LPP/

// Copyright © 2017 The Things Network
// Use of this source code is governed by the MIT license that can be found in the LICENSE file.

#include "CayenneLPP.h"

CayenneLPP::CayenneLPP(uint8_t size) : _maxsize(size) {
  _buffer = (uint8_t *)malloc(size);
  _cursor = 0;
}

CayenneLPP::~CayenneLPP(void) {
  free(_buffer);
}

void CayenneLPP::reset(void) {
  _cursor = 0;
}

uint8_t CayenneLPP::getSize(void) {
  return _cursor;
}

uint8_t *CayenneLPP::getBuffer(void) {
  return _buffer;
}

uint8_t CayenneLPP::copy(uint8_t *dst) {
  memcpy(dst, _buffer, _cursor);
  return _cursor;
}

void CayenneLPP::setMode(uint8_t mode) {
  if (_mode != mode) {
    _mode = mode;
    if (LPP_MODE_PACKED  == _mode) _headersize = 1;
    if (LPP_MODE_DYNAMIC == _mode) _headersize = 2;
    if (LPP_MODE_HISTORY == _mode) _headersize = 3;
  }
  reset();
}

void CayenneLPP::setDelta(uint16_t seconds) {
  _delta = seconds;
}

// ----------------------------------------------------------------------------

template <typename T> uint8_t CayenneLPP::addValue(uint8_t channel, uint8_t type, T value, uint32_t multiplier, uint8_t size, bool is_signed) {

  // check buffer overflow
  if ((_cursor + size + _headersize) > _maxsize) return 0;

  // check sign  
  bool sign = value < 0;
  if (sign) value = -value;

  // get value to store
  uint32_t v = value * multiplier;

  // format an uint32_t as if it was an int32_t
  if (is_signed & sign) {
    uint32_t mask = (1 << (size * 8)) - 1;
    v = v & mask;
    if (sign) v = mask - v + 1;
  }

  // header
  if (LPP_MODE_DYNAMIC == _mode) {
    _buffer[_cursor++] = channel;
  }
  _buffer[_cursor++] = type;
  if (LPP_MODE_HISTORY == _mode) {
    _buffer[_cursor++] = (_delta >> 8) & 0xFF;
    _buffer[_cursor++] = _delta & 0xFF;
  }
  
  // add bytes (MSB first)
  for (uint8_t i=1; i<=size; i++) {
    _buffer[_cursor + size - i] = (v & 0xFF);
    v >>= 8;
  }

  // update & return _cursor
  _cursor += size;
  return _cursor;

}

// ----------------------------------------------------------------------------

uint8_t CayenneLPP::addDigitalInput(uint8_t channel, uint8_t value) {
  return addValue(channel, LPP_DIGITAL_INPUT, value, LPP_DIGITAL_INPUT_MULT, LPP_DIGITAL_INPUT_SIZE);
}

uint8_t CayenneLPP::addDigitalOutput(uint8_t channel, uint8_t value) {
  return addValue(channel, LPP_DIGITAL_OUTPUT, value, LPP_DIGITAL_OUTPUT_MULT, LPP_DIGITAL_OUTPUT_SIZE);
}

uint8_t CayenneLPP::addAnalogInput(uint8_t channel, float value) {
  return addValue(channel, LPP_ANALOG_INPUT, value, LPP_ANALOG_INPUT_MULT, LPP_ANALOG_INPUT_SIZE, true);
}

uint8_t CayenneLPP::addAnalogOutput(uint8_t channel, float value) {
  return addValue(channel, LPP_ANALOG_OUTPUT, value, LPP_ANALOG_OUTPUT_MULT, LPP_ANALOG_OUTPUT_SIZE, true);
}

uint8_t CayenneLPP::addGenericSensor(uint8_t channel, float value)  {
  return addValue(channel, LPP_GENERIC_SENSOR, value, LPP_GENERIC_SENSOR_MULT, LPP_GENERIC_SENSOR_SIZE);
}

uint8_t CayenneLPP::addLuminosity(uint8_t channel, uint16_t value) {
  return addValue(channel, LPP_LUMINOSITY, value, LPP_LUMINOSITY_MULT, LPP_LUMINOSITY_SIZE);
}

uint8_t CayenneLPP::addPresence(uint8_t channel, uint8_t value) {
  return addValue(channel, LPP_PRESENCE, value, LPP_PRESENCE_MULT, LPP_PRESENCE_SIZE);
}

uint8_t CayenneLPP::addTemperature(uint8_t channel, float value) {
  return addValue(channel, LPP_TEMPERATURE, value, LPP_TEMPERATURE_MULT, LPP_TEMPERATURE_SIZE, true);
}

uint8_t CayenneLPP::addRelativeHumidity(uint8_t channel, float value) {
  return addValue(channel, LPP_RELATIVE_HUMIDITY, value, LPP_RELATIVE_HUMIDITY_MULT, LPP_RELATIVE_HUMIDITY_SIZE);
}

uint8_t CayenneLPP::addVoltage(uint8_t channel, float value) {
  return addValue(channel, LPP_VOLTAGE, value, LPP_VOLTAGE_MULT, LPP_VOLTAGE_SIZE);
}

uint8_t CayenneLPP::addCurrent(uint8_t channel, float value) {
  return addValue(channel, LPP_CURRENT, value, LPP_CURRENT_MULT, LPP_CURRENT_SIZE);
}

uint8_t CayenneLPP::addFrequency(uint8_t channel, uint32_t value) {
  return addValue(channel, LPP_FREQUENCY, value, LPP_FREQUENCY_MULT, LPP_FREQUENCY_SIZE);
}

uint8_t CayenneLPP::addPercentage(uint8_t channel, uint8_t value) {
  return addValue(channel, LPP_PERCENTAGE, value, LPP_PERCENTAGE_MULT, LPP_PERCENTAGE_SIZE);
}

uint8_t CayenneLPP::addAltitude(uint8_t channel, float value) {
  return addValue(channel, LPP_ALTITUDE, value, LPP_ALTITUDE_MULT, LPP_ALTITUDE_SIZE, true);
}

uint8_t CayenneLPP::addPower(uint8_t channel, uint16_t value) {
  return addValue(channel, LPP_POWER, value, LPP_POWER_MULT, LPP_POWER_SIZE);
}

uint8_t CayenneLPP::addDistance(uint8_t channel, float value) {
  return addValue(channel, LPP_DISTANCE, value, LPP_DISTANCE_MULT, LPP_DISTANCE_SIZE);
}

uint8_t CayenneLPP::addEnergy(uint8_t channel, float value) {
  return addValue(channel, LPP_ENERGY, value, LPP_ENERGY_MULT, LPP_ENERGY_SIZE);
}

uint8_t CayenneLPP::addBarometricPressure(uint8_t channel, float value) {
  return addValue(channel, LPP_BAROMETRIC_PRESSURE, value, LPP_BAROMETRIC_PRESSURE_MULT, LPP_BAROMETRIC_PRESSURE_SIZE);
}

uint8_t CayenneLPP::addUnixTime(uint8_t channel, uint32_t value) {
  return addValue(channel, LPP_UNIXTIME, value, LPP_UNIXTIME_MULT, LPP_UNIXTIME_SIZE);
}

uint8_t CayenneLPP::addDirection(uint8_t channel, float value) {
  return addValue(channel, LPP_DIRECTION, value, LPP_DIRECTION_MULT, LPP_DIRECTION_SIZE);
}

uint8_t CayenneLPP::addSwitch(uint8_t channel, uint8_t value) {
  return addValue(channel, LPP_SWITCH, value, LPP_SWITCH_MULT, LPP_SWITCH_SIZE);
}

uint8_t CayenneLPP::addAccelerometer(uint8_t channel, float x, float y, float z) {
  
  if ((_cursor + LPP_ACCELEROMETER_SIZE + _headersize) > _maxsize) return 0;

  int16_t vx = x * LPP_ACCELEROMETER_MULT;
  int16_t vy = y * LPP_ACCELEROMETER_MULT;
  int16_t vz = z * LPP_ACCELEROMETER_MULT;

  if (LPP_MODE_DYNAMIC == _mode) {
    _buffer[_cursor++] = channel;
  }
  _buffer[_cursor++] = LPP_ACCELEROMETER;
  if (LPP_MODE_HISTORY == _mode) {
    _buffer[_cursor++] = (_delta >> 8) & 0xFF;
    _buffer[_cursor++] = _delta & 0xFF;
  }

  _buffer[_cursor++] = vx >> 8;
  _buffer[_cursor++] = vx;
  _buffer[_cursor++] = vy >> 8;
  _buffer[_cursor++] = vy;
  _buffer[_cursor++] = vz >> 8;
  _buffer[_cursor++] = vz;

  return _cursor;

}

uint8_t CayenneLPP::addGyrometer(uint8_t channel, float x, float y, float z) {

  if ((_cursor + LPP_GYROMETER_SIZE + _headersize) > _maxsize) return 0;

  int16_t vx = x * LPP_GYROMETER_MULT;
  int16_t vy = y * LPP_GYROMETER_MULT;
  int16_t vz = z * LPP_GYROMETER_MULT;

  if (LPP_MODE_DYNAMIC == _mode) {
    _buffer[_cursor++] = channel;
  }
  _buffer[_cursor++] = LPP_GYROMETER;
  if (LPP_MODE_HISTORY == _mode) {
    _buffer[_cursor++] = (_delta >> 8) & 0xFF;
    _buffer[_cursor++] = _delta & 0xFF;
  }

  _buffer[_cursor++] = vx >> 8;
  _buffer[_cursor++] = vx;
  _buffer[_cursor++] = vy >> 8;
  _buffer[_cursor++] = vy;
  _buffer[_cursor++] = vz >> 8;
  _buffer[_cursor++] = vz;

  return _cursor;

}

uint8_t CayenneLPP::addGPS(uint8_t channel, float latitude, float longitude, float altitude) {
  
  if ((_cursor + LPP_GPS_SIZE + _headersize) > _maxsize) return 0;

  int32_t lat = latitude * LPP_GPS_LAT_LON_MULT;
  int32_t lon = longitude * LPP_GPS_LAT_LON_MULT;
  int32_t alt = altitude * LPP_GPS_ALT_MULT;

  if (LPP_MODE_DYNAMIC == _mode) {
    _buffer[_cursor++] = channel;
  }
  _buffer[_cursor++] = LPP_GPS;
  if (LPP_MODE_HISTORY == _mode) {
    _buffer[_cursor++] = (_delta >> 8) & 0xFF;
    _buffer[_cursor++] = _delta & 0xFF;
  }

  _buffer[_cursor++] = lat >> 16;
  _buffer[_cursor++] = lat >> 8;
  _buffer[_cursor++] = lat;
  _buffer[_cursor++] = lon >> 16;
  _buffer[_cursor++] = lon >> 8;
  _buffer[_cursor++] = lon;
  _buffer[_cursor++] = alt >> 16;
  _buffer[_cursor++] = alt >> 8;
  _buffer[_cursor++] = alt;

  return _cursor;

}

uint8_t CayenneLPP::addGPSFull(uint8_t channel, float latitude, float longitude, float altitude) {
  
  if ((_cursor + LPP_GPS_FULL_SIZE + 1) > _maxsize) return 0;

  int32_t lat = latitude * LPP_GPS_FULL_LAT_LON_MULT;
  int32_t lon = longitude * LPP_GPS_FULL_LAT_LON_MULT;
  int16_t alt = altitude * LPP_GPS_FULL_ALT_MULT;

  _buffer[_cursor++] = channel;

  _buffer[_cursor++] = lat >> 24;
  _buffer[_cursor++] = lat >> 16;
  _buffer[_cursor++] = lat >> 8;
  _buffer[_cursor++] = lat;
  _buffer[_cursor++] = lon >> 24;
  _buffer[_cursor++] = lon >> 16;
  _buffer[_cursor++] = lon >> 8;
  _buffer[_cursor++] = lon;
  _buffer[_cursor++] = alt >> 8;
  _buffer[_cursor++] = alt;

  return _cursor;

}