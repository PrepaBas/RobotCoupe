/* Includes ------------------------------------------------------------------*/

/* Arduino specific header files. */
#include "Arduino.h"

/* Helper header files. */
#include "SPI.h"

/* Expansion Board specific header files. */
#include "XNucleoIHM02A1.h"

/* file header file */
#include "RobotCoupe.hpp"


/* Definitions ---------------------------------------------------------------*/

/* Number of steps. */
#define STEPS 200   /* 1 revolution given a 400 steps motor configured at 1/128 microstep mode. */
#define MICRO_STEPS 200 * 128

/* Variables -----------------------------------------------------------------*/

/* Motor Control Expansion Board. */
XNucleoIHM02A1 *x_nucleo_ihm02a1;
SPIClass *dev_spi;
L6470 **motors;
int loops = 0;

/* Initialization parameters of the motors connected to the expansion board. */
L6470_init_t L6470_init[L6470DAISYCHAINSIZE] = {
    /* First Motor. */
    {
        9.0,                           /* Motor supply voltage in V. */
        200,                           /* Min number of steps per revolution for the motor. */
        2.0,                           /* Max motor phase voltage in A. */
        3.06,                          /* Max motor phase voltage in V. */
        300.0,                         /* Motor initial speed [step/s]. */
        500.0,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
        500.0,                         /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
        992.0,                         /* Motor maximum speed [step/s]. */
        0.0,                           /* Motor minimum speed [step/s]. */
        602.7,                         /* Motor full-step speed threshold [step/s]. */
        3.06,                          /* Holding kval [V]. */
        3.06,                          /* Constant speed kval [V]. */
        3.06,                          /* Acceleration starting kval [V]. */
        3.06,                          /* Deceleration starting kval [V]. */
        61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
        392.1569e-6,                   /* Start slope [s/step]. */
        643.1372e-6,                   /* Acceleration final slope [s/step]. */
        643.1372e-6,                   /* Deceleration final slope [s/step]. */
        0,                             /* Thermal compensation factor (range [0, 15]). */
        3.06 * 1000 * 1.10,            /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
        3.06 * 1000 * 1.00,            /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
        StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
        0xFF,                          /* Alarm conditions enable. */
        0x2E88                         /* Ic configuration. */
    },

    /* Second Motor. */
    {
        9.0,                           /* Motor supply voltage in V. */
        200,                           /* Min number of steps per revolution for the motor. */
        2.0,                           /* Max motor phase voltage in A. */
        3.06,                          /* Max motor phase voltage in V. */
        300.0,                         /* Motor initial speed [step/s]. */
        500.0,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
        500.0,                         /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
        992.0,                         /* Motor maximum speed [step/s]. */
        0.0,                           /* Motor minimum speed [step/s]. */
        602.7,                         /* Motor full-step speed threshold [step/s]. */
        3.06,                          /* Holding kval [V]. */
        3.06,                          /* Constant speed kval [V]. */
        3.06,                          /* Acceleration starting kval [V]. */
        3.06,                          /* Deceleration starting kval [V]. */
        61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
        392.1569e-6,                   /* Start slope [s/step]. */
        643.1372e-6,                   /* Acceleration final slope [s/step]. */
        643.1372e-6,                   /* Deceleration final slope [s/step]. */
        0,                             /* Thermal compensation factor (range [0, 15]). */
        3.06 * 1000 * 1.10,            /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
        3.06 * 1000 * 1.00,            /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
        StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
        0xFF,                          /* Alarm conditions enable. */
        0x2E88                         /* Ic configuration. */
    }
};


/* ---------------------------------------------------------------------*/
RobotCoupe::RobotCoupe(){

}

float robotWidth = 245.72;
float wheelRadius = 73.025;

void RobotCoupe::begin(robotWidth, wheelRadius){
    /*----- Initialization. -----*/

    /* Initializing robot width and wheel radius*/
    RobotCoupe::set_baseWidth(robotWidth);
    RobotCoupe::set_wheelRadius(wheelRadius);

    /* Initializing SPI bus. */
    dev_spi = new SPIClass(D11, D12, D3);

    /* Initializing Motor Control Expansion Board. */
    x_nucleo_ihm02a1 = new XNucleoIHM02A1(&L6470_init[0], &L6470_init[1], A4, A5, D4, A2, dev_spi);

    /* Building a list of motor control components. */
    motors = x_nucleo_ihm02a1->get_components();
}

/* Setters and Getters -------------------------------------------------------*/

void RobotCoupe::set_x(float x){
    _x = x;
}

void RobotCoupe::set_y(float y){
    _y = y;
}

void RobotCoupe::set_theta(float theta){
    _theta = theta;
}

void RobotCoupe::set_baseWidth(float baseWidth){
    _baseWidth = baseWidth;
}

void RobotCoupe::set_wheelRadius(float wheelRadius){
    _wheelRadius = wheelRadius;
}

/* loop ----------------------------------------------------------------------*/

void RobotCoupe::move_straight (char direction, float distance){
    /* Move robot in a straight line
     * direction :  1 for Forward
     *              0 for Backward */
    long m_steps = MICRO_STEPS * distance / (2 * 3.14 * wheelRadius); // Implicit conversion
    motors[0]->prepare_move(direction?(StepperMotor::FWD):(StepperMotor::BWD), m_steps);
    motors[1]->prepare_move(direction?(StepperMotor::FWD):(StepperMotor::BWD), m_steps);
    x_nucleo_ihm02a1->perform_prepared_actions();
}

void RobotCoupe::rotate (int direction, float angle){
    /* Rotate robot in place
     * direction :  0 for left rotation (anti-clockwise)
     *              1 for right rotation (clockwise) */
    float distance = PI * angle * robotWidth / 360;
    long m_steps = MICRO_STEPS * distance / (2 * wheelRadius); // Implicit conversion
    motors[0]->prepare_move(direction?(StepperMotor::FWD):(StepperMotor::BWD), m_steps);
    motors[1]->prepare_move(direction?(StepperMotor::BWD):(StepperMotor::FWD), m_steps);
    x_nucleo_ihm02a1->perform_prepared_actions();
}

void RobotCoupe::wait_stop (){
    motors[0]->wait_while_active();
    motors[1]->wait_while_active();
}

