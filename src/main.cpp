#include <cstdio>
#include <chrono>
#include <thread>
#include "pico/stdlib.h"

#include "motor2040.hpp"
#include "button.hpp"
#include "pid.hpp"

#include "bsp/board.h"
#include "tusb.h"

/*
A demonstration of driving all four of Motor 2040's motor outputs through a
sequence of velocities, with the help of their attached encoders and PID control.

Press "Boot" to exit the program.
*/

using namespace motor;
using namespace encoder;

enum Wheels {
  FL = 3,
  FR = 0,
  RL = 2,
  RR = 1,
};

// The gear ratio of the motor
constexpr float GEAR_RATIO = 50.0f;

// The counts per revolution of the motor's output shaft
constexpr float COUNTS_PER_REV = MMME_CPR * GEAR_RATIO;

// The scaling to apply to the motor's speed to match its real-world speed
constexpr float SPEED_SCALE = 5.4f;

// How many times to update the motor per second
const uint UPDATES = 100;
constexpr float UPDATE_RATE = 1.0f / (float)UPDATES;

// The time to travel between each random value
constexpr float TIME_FOR_EACH_MOVE = 2.0f;
const uint UPDATES_PER_MOVE = TIME_FOR_EACH_MOVE * UPDATES;

// How many of the updates should be printed (i.e. 2 would be every other update)
const uint PRINT_DIVIDER = 4;

// The speed to drive the wheels at
float DRIVING_SPEED = 2.0f;

// PID values
constexpr float VEL_KP = 30.0f;   // Velocity proportional (P) gain
constexpr float VEL_KI = 0.0f;    // Velocity integral (I) gain
constexpr float VEL_KD = 0.4f;    // Velocity derivative (D) gain

// Create an array of motor pointers
const pin_pair motor_pins[] = {motor2040::MOTOR_A, motor2040::MOTOR_B,
                               motor2040::MOTOR_C, motor2040::MOTOR_D};
const uint NUM_MOTORS = count_of(motor_pins);
Motor *motors[NUM_MOTORS];

// Create an array of encoder pointers
const pin_pair encoder_pins[] = {motor2040::ENCODER_A, motor2040::ENCODER_B,
                                 motor2040::ENCODER_C, motor2040::ENCODER_D};
const char* ENCODER_NAMES[] = {"RR", "RL", "FL", "FR"};
const uint NUM_ENCODERS = count_of(encoder_pins);
Encoder *encoders[NUM_ENCODERS];

// Create the user button
Button user_sw(motor2040::USER_SW);

// Create an array of PID pointers
PID vel_pids[NUM_MOTORS];


// Helper functions for driving in common directions
void drive_forward(float speed) {
  vel_pids[FL].setpoint = speed;
  vel_pids[FR].setpoint = speed;
  vel_pids[RL].setpoint = speed;
  vel_pids[RR].setpoint = speed;
}

void turn_right(float speed) {
  vel_pids[FL].setpoint = speed;
  vel_pids[FR].setpoint = -speed;
  vel_pids[RL].setpoint = speed;
  vel_pids[RR].setpoint = -speed;
}

void strafe_right(float speed) {
  vel_pids[FL].setpoint = speed;
  vel_pids[FR].setpoint = -speed;
  vel_pids[RL].setpoint = -speed;
  vel_pids[RR].setpoint = speed;
}

void stop() {
  vel_pids[FL].setpoint = 0.0f;
  vel_pids[FR].setpoint = 0.0f;
  vel_pids[RL].setpoint = 0.0f;
  vel_pids[RR].setpoint = 0.0f;
}

void motors_init()
{
  // Fill the arrays of motors, encoders, and pids, and initialise them
  for(auto i = 0u; i < NUM_MOTORS; i++) {
    motors[i] = new Motor(motor_pins[i], NORMAL_DIR, SPEED_SCALE);
    motors[i]->init();

    encoders[i] = new Encoder(pio0, i, encoder_pins[i], PIN_UNUSED, NORMAL_DIR, COUNTS_PER_REV, true);
    encoders[i]->init();

    vel_pids[i] = PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE);
  }

  // Reverse the direction of the B and D motors and encoders
  motors[FL]->direction(REVERSED_DIR);
  motors[RL]->direction(REVERSED_DIR);
  encoders[FL]->direction(REVERSED_DIR);
  encoders[RL]->direction(REVERSED_DIR);

  // Enable all motors
  for(auto i = 0u; i < NUM_MOTORS; i++) {
    motors[i]->enable();
  }
}

void motors_update()
{
  Encoder::Capture captures[NUM_MOTORS];

  // Capture the state of all the encoders
  for(auto i = 0u; i < NUM_MOTORS; i++) {
    captures[i] = encoders[i]->capture();
  }

  for(auto i = 0u; i < NUM_MOTORS; i++) {
    // Calculate the acceleration to apply to the motor to move it closer to the velocity setpoint
    float accel = vel_pids[i].calculate(captures[i].revolutions_per_second());

    // Accelerate or decelerate the motor
    motors[i]->speed(motors[i]->speed() + (accel * UPDATE_RATE));
  }
}

void cdc_task(){
  if(tud_cdc_n_connected(0))
  {
    if(tud_cdc_n_available(0))
    {
      // Create a buffer and read bytes into it
      uint8_t buf[64];
      uint32_t count = tud_cdc_n_read(0, buf, sizeof(buf));

      // Convert the 8 bytes into 4 int16_t
      int16_t vels_encoded[NUM_MOTORS];
      for (int i = 0; i < sizeof(NUM_MOTORS); i++) {
          vels_encoded[i] = static_cast<int16_t>((buf[2*i+1] << 8) | buf[2*i]);
      }

      // Normalise the int16_t to floats between -1 and 1
      const float MAX_SIGNED_INT = 32767.0f;
      float vels_norm[NUM_MOTORS];
      for (int i = 0; i < sizeof(NUM_MOTORS); i++) {
          vels_norm[i] = static_cast<float>(vels_encoded[i]) / MAX_SIGNED_INT;
      }
      
      // Update the motor target speeds
      vel_pids[FL].setpoint = vels_norm[FL];
      vel_pids[FR].setpoint = vels_norm[FR];
      vel_pids[RL].setpoint = vels_norm[RL];
      vel_pids[RR].setpoint = vels_norm[RR];

    }
  }
}

int main() {
  // Initialise comms
  stdio_init_all();
  board_init();
  tud_init(BOARD_TUD_RHPORT);

  // Initialise the motors
  motors_init();

  // Continually move the motor until the user button is pressed
  while(!user_sw.raw()) {
    
    // tinyusb device task
    tud_task();

    // Read incoming serial
    cdc_task();

    // Update the motors
    motors_update();
    
  
    sleep_ms(2);

  }

  // Stop all the motors
  for(auto m = 0u; m < NUM_MOTORS; m++) {
    motors[m]->disable();
  }
}