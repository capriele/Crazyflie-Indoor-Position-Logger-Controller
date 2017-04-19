/**
 * Authored by Michael Hamer (http://www.mikehamer.info), November 2016.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */


#pragma once

#include "stabilizer_types.h"

void stateControllerInit(void);
bool stateControllerTest(void);
void stateControllerRun(control_t *control, const sensorData_t *sensors, const state_t *state);
void stateControllerUpdateStateWithExternalPosition();

#define CONTROL_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz
#define EXTERNAL_MEASUREMENT_STDDEV (0.005)

#define CONTROLMODE_ACCELERATION(mode) ((0b001 & mode) != 0)
#define CONTROLMODE_VELOCITY(mode)     ((0b010 & mode) != 0)
#define CONTROLMODE_POSITION(mode)     ((0b100 & mode) != 0)

typedef struct {
  bool setEmergency;
  bool resetEmergency;
  uint8_t xmode, ymode, zmode;
  float x[3];
  float y[3];
  float z[3];
  float yaw[2];
} controlReference_t;

typedef struct {
  //uint8_t packetHasExternalReference;
  bool setEmergency;
  bool resetEmergency;
  uint8_t controlModeX;
  uint8_t controlModeY;
  uint8_t controlModeZ;
} __attribute__((packed)) crtpControlPacketHeader_t; // size 2

typedef struct
{
  crtpControlPacketHeader_t header; // size 2
  uint16_t x[3];
  uint16_t y[3];
  uint16_t z[3];
  uint16_t yaw[2];
} __attribute__((packed)) crtpControlPacket_t;

typedef struct
{
  crtpControlPacketHeader_t header; // size 2
  uint16_t x[4];
  uint16_t y[4];
  uint16_t z[4];
  uint16_t yaw[2];
} __attribute__((packed)) crtpControlPacketWithExternalPosition_t;
