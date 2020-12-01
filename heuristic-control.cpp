#include "sensor.h"
#include "heuristic-control.h"

Displacement displacement { dx: 0, dy: 0 };

/// finds the temperature maxima's displacement from the center of the image
void maximaDisplacement() {
  size_t maxima = 0;

  for (int j = 0; j < SENSOR_SIZE; ++j) {
    if (pixels[j] >= pixels[maxima]) { maxima = j; }
  }

  // transform index to coordinates
  displacement.dx = ((SENSOR_SIZE / 2) - maxima) % SENSOR_COLUMNS;
  displacement.dy = ((SENSOR_SIZE / 2) - maxima) / SENSOR_COLUMNS;
}

void clampMotorStates() {
  motorState.xa = max(min(motorState.xa, 1.0), -1.0); motorState.ya = max(min(motorState.ya, 1.0), -1.0);
  motorState.xb = max(min(motorState.xb, 1.0), -1.0); motorState.yb = max(min(motorState.yb, 1.0), -1.0);
}

/// manipulates the abstract servo state based on the displacement vector
void seekAlignment() {
  /*
   * This is the only component which is not trivial. 
   * without a rigorous inverse kinematic model, we must provide 
   * a sequence of rapid rough guesses on how to update the motor states, in the hopes that this will reliably guide the
   * state of the arm in a roughly correct (and visually appealing) way.
   * 
   * The motor control code disallows unsafe angular positions that would break the device
   * or cause injury, so physical safety need not be considered here, 
   * however the phase space of motor states is extremely large and we don't want to get lost in it, 
   * e.g. the motor state is configured outside the allowed bounds of the physical system, and changes to this state
   * result in zero change realized in the physical system, and our gradient becomes zero - we are temporarily lost,
   * so we want to avoid this situation by clamping the values here as well.
   * 
   * To develop our guess, let's consider some situations for our displacement:
   * 
   *  1) Maxima is in lower right:
   * 
   *    [            ]
   *    [            ]
   *    [      c--|  ]
   *    [         |  ]
   *    [         m  ]
   * 
   *  2) Maxima is in lower right:
   * 
   *    [            ]
   *    [            ]
   *    [      c     ]
   *    [      |     ]
   *    [      m     ]
   * 
   *  3) Maxima is in upper left:
   * 
   *    [ m-----     ]
   *    [      |     ]
   *    [      c     ]
   *    [            ]
   *    [            ]
   * 
   *     In the final case, we want to adjust the orientation of the end of the arm
   *     to point more up, and also to point more to its left.  To achieve this,
   *     we could adjust the second stage X axis to point left, and the second stage Y axis to point up
   *          a.1) but what happens when the second stage is already at its maximum actuation? 
   *               e.g. an overhead view like:
   *                 [ m  s         ]
   *                 [     \        ]
   *                 [      \       ]
   *                 [       -      ]
   *                 [       |      ]
   *                 [       |      ]
   *                 [       |      ]
   *               in this case, we should not actuate the second stage X axis, we should actuate the
   *               first stage X axis, because the second stage X axis is exhausted.  The same may be true of the
   *               Y axis, and the same may be true with the stages in this example flipped.
   * 
   *    I will be implementing an obvious algorithm, where we attempt to use the first stage to orient the arm, but when the first stage
   *    is exhausted we redistribute the actuation across both stages.
   *
   *    if ∆x is negative, we want to move left.
   *    if ∆x is positive, we want to move right.
   * 
   *    if ∆y is negative, we want to move up.
   *    if ∆y is positive, we want to move down.
   */

  // get f32 values normalized to the size of the dimensions of the image,
  // that is, ∆x ∈ [-1, 1] and ∆y ∈ [-1, 1]
  float dx = (float)displacement.dx / (float)(SENSOR_COLUMNS / 2);
  float dy = (float)displacement.dy / (float)(SENSOR_COLUMNS / 2);

  // THIS MAY BE A BAD MODEL: ∆u = -cu
  float horizontal_movement = RELAXATION_COEFFICIENT * -dx;
  float vertical_movement   = RELAXATION_COEFFICIENT * -dy;

  // get the remaining budgets on all axes for all stages
  float a_budget_x = (1.0 - motorState.xa); float b_budget_x = (1.0 - motorState.xb);
  float a_budget_y = (1.0 - motorState.ya); float b_budget_y = (1.0 - motorState.yb);

  // try to use the first stage, but if there is budget left over
  // we try to distribute it to the second stage. If this too is exhausted
  // we just give up and max out both stages.
  dx = min(horizontal_movement, a_budget_x);
  dy = min(vertical_movement,   a_budget_y);

  // apply the budget-limited actuation to all stages
  motorState.xa += dx; motorState.xb += min(b_budget_x, horizontal_movement - dx);
  motorState.ya += dy; motorState.yb += min(b_budget_y, vertical_movement - dy);

  // update the stepper motor positions based on the motor states
  updateOrientations();
}