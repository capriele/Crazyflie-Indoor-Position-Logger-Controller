/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 * position_estimator_altitude.c: Altitude-only position estimator
 */

#include "log.h"
#include "param.h"
#include "num.h"
#include "position_estimator.h"

#define G 9.81f;

struct selfState_s {
  float estimatedZ; // The current Z estimate, has same offset as asl
  float velocityZ; // Vertical speed (world frame) integrated from vertical acceleration (m/s)
  float estAlpha;
  float velocityFactor;
  float vAccDeadband; // Vertical acceleration deadband
  float velZAlpha;   // Blending factor to avoid vertical speed to accumulate error
  float estimatedVZ;
};

static struct selfState_s state = {
  .estimatedZ = 0.0f,
  .velocityZ = 0.0f,
  .estAlpha = 0.997f,
  .velocityFactor = 1.0f,
  .vAccDeadband = 0.04f,
  .velZAlpha = 0.995f,
  .estimatedVZ = 0.0f,
};

static void positionEstimateInternal(state_t* estimate, float asl, float dt, struct selfState_s* state);
static void positionUpdateVelocityInternal(float accWZ, float dt, struct selfState_s* state);

void positionEstimate(state_t* estimate, float asl, float dt) {
  positionEstimateInternal(estimate, asl, dt, &state);
}

void positionUpdateVelocity(float accWZ, float dt) {
  positionUpdateVelocityInternal(accWZ, dt, &state);
}

static void positionEstimateInternal(state_t* estimate, float asl, float dt, struct selfState_s* state) {
  static float prev_estimatedZ = 0;
  state->estimatedZ = state->estAlpha * state->estimatedZ +
                     (1.0f - state->estAlpha) * asl +
                     state->velocityFactor * state->velocityZ * dt;

  estimate->position.x = 0.0f;
  estimate->position.y = 0.0f;
  estimate->position.z = state->estimatedZ;
  estimate->velocity.z = (state->estimatedZ - prev_estimatedZ) / dt;
  state->estimatedVZ = estimate->velocity.z;
  prev_estimatedZ = state->estimatedZ;
}

static void positionUpdateVelocityInternal(float accWZ, float dt, struct selfState_s* state) {
  state->velocityZ += deadband(accWZ, state->vAccDeadband) * dt * G;
  state->velocityZ *= state->velZAlpha;
}

LOG_GROUP_START(posEstimatorAlt)
LOG_ADD(LOG_FLOAT, estimatedZ, &state.estimatedZ)
LOG_ADD(LOG_FLOAT, estVZ, &state.estimatedVZ)
LOG_ADD(LOG_FLOAT, velocityZ, &state.velocityZ)
LOG_GROUP_STOP(posEstimatorAlt)

PARAM_GROUP_START(posEst)
PARAM_ADD(PARAM_FLOAT, estAlpha, &state.estAlpha)
PARAM_ADD(PARAM_FLOAT, velFactor, &state.velocityFactor)
PARAM_ADD(PARAM_FLOAT, velZAlpha, &state.velZAlpha)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &state.vAccDeadband)
PARAM_GROUP_STOP(posEstimatorAlt)