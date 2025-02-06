/* Includes ------------------------------------------------------------------*/

/* Arduino specific header files. */
#include "Arduino.h"

/* Helper header files. */
#include "SPI.h"

/* Expansion Board specific header files. */
#include "XNucleoIHM02A1.h"

/* file header file */
#include "RobotCoupe.h"


/* Definitions ---------------------------------------------------------------*/

/* Number of steps. */
#define STEPS 200   /* 1 revolution given a 400 steps motor configured at 1/128 microstep mode. */
#define MICRO_STEPS STEPS * 16

/* Variables -----------------------------------------------------------------*/

/* Motor Control Expansion Board. */
XNucleoIHM02A1 *x_nucleo_ihm02a1;
SPIClass *dev_spi;
L6470 **motors;

/* Initialization parameters of the motors connected to the expansion board. */
L6470_init_t L6470_init[L6470DAISYCHAINSIZE] = {
    /* First Motor. */
    {
        9.0,                           /* Motor supply voltage in V. */
        400,                           /* Min number of steps per revolution for the motor. */
        1.7,                           /* Max motor phase voltage in A. */
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
        StepperMotor::STEP_MODE_1_16, /* Step mode selection. */
        0xFF,                          /* Alarm conditions enable. */
        0x2E88                         /* Ic configuration. */
    },

    /* Second Motor. */
    {
        9.0,                           /* Motor supply voltage in V. */
        400,                           /* Min number of steps per revolution for the motor. */
        1.7,                           /* Max motor phase voltage in A. */
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
        StepperMotor::STEP_MODE_1_16, /* Step mode selection. */
        0xFF,                          /* Alarm conditions enable. */
        0x2E88                         /* Ic configuration. */
    }
};


/* ---------------------------------------------------------------------*/
RobotCoupe::RobotCoupe(){

}

void RobotCoupe::begin(float baseWidth, float wheelRadius){
    /*----- Initialization. -----*/
    Serial.begin(9600);
    /* Initializing robot width and wheel radius*/
    RobotCoupe::set_baseWidth(baseWidth);
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
    _position.x = x;
}

void RobotCoupe::set_y(float y){
    _position.y = y;
}

void RobotCoupe::set_theta(float theta){
    _position.theta = theta;
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
    float m_steps = MICRO_STEPS * distance / (2 * PI * _wheelRadius); // Implicit conversion
    motors[0]->prepare_move(direction?(StepperMotor::FWD):(StepperMotor::BWD), m_steps);
    motors[1]->prepare_move(direction?(StepperMotor::FWD):(StepperMotor::BWD), m_steps);
    x_nucleo_ihm02a1->perform_prepared_actions();
}

void RobotCoupe::rotate (int direction, float angle){
    /* Rotate robot in place
     * direction :  0 for left rotation (anti-clockwise)
     *              1 for right rotation (clockwise) */
    float distance = angle * _baseWidth / 360;
    float m_steps = MICRO_STEPS * distance / (2 * _wheelRadius);
    motors[0]->prepare_move(direction?(StepperMotor::FWD):(StepperMotor::BWD), m_steps);
    motors[1]->prepare_move(direction?(StepperMotor::BWD):(StepperMotor::FWD), m_steps);
    x_nucleo_ihm02a1->perform_prepared_actions();
}

void RobotCoupe::wait_stop (){
    motors[0]->wait_while_active();
    motors[1]->wait_while_active();
}

void RobotCoupe::go_to(struct position pos){
    /* Move robot to coordinates.
     * First rotate then move in straight line */
    Serial.print("Starting from : "); Serial.print(_position.x); Serial.print(" ");  Serial.print(_position.y); Serial.print(" "); Serial.print(_position.theta);
    Serial.print("go_to\n");
    Serial.print(pos.x); Serial.print("\n");
    Serial.print(pos.y); Serial.print("\n");
    float x = pos.x - _position.x;
    float y = pos.y - _position.y;
    float radius = sqrt(x*x+y*y);
    float angle = atan2(y, x)*360/(2 * PI);
    Serial.print("valeur de l'angle : "); Serial.print(angle); Serial.print("radius : "); Serial.print(radius); Serial.print("\n");
    float move_angle = angle - _position.theta;
    if(move_angle >= 0){
        RobotCoupe::rotate(1, move_angle);
    }
    else{
        RobotCoupe::rotate(0, - move_angle);
    }
    RobotCoupe::wait_stop();
    RobotCoupe::move_straight(0, radius);
    RobotCoupe::wait_stop();

    // Update new position
    RobotCoupe::set_x(pos.x);
    RobotCoupe::set_y(pos.y);
    RobotCoupe::set_theta(angle);
}

struct table {
    struct position bl;
    struct position tl;
    struct position br;
    struct position tr;
    struct position extreme;
};


table table_coupe = {{800, 800}, {800, 1200}, {2200, 800}, {2200, 1200}, {3000, 2000}};

position corner_positon(int c){
    switch(c){
        case 0 :
            return table_coupe.bl;
        case 2 :
            return table_coupe.br;
        case 4 :
            return table_coupe.tr;
        case 6 :
            return table_coupe.tl;
    }
}

int what_zone (struct position pos) {
    
    if (pos.x < table_coupe.bl.x){
        if(pos.y < table_coupe.bl.y) 
            return 0; // left_bot
        if(pos.y < table_coupe.tl.y)
            return 7; // left_mid
        else 
            return 6; // left_top
    }
    if (pos.x < table_coupe.br.x){
        if(pos.y < table_coupe.bl.y) 
            return 1; // mid_bot
        if(pos.y < table_coupe.tl.y){ // middle section
            if(pos.x<pos.y){
                if(pos.x<table_coupe.extreme.y - pos.y) 
                    return 7; // left_mid
                else 
                    return 5; // mid_top
            }
            else{
                if(pos.x<table_coupe.extreme.y - pos.y) 
                    return 1; // mid_bot
                else 
                    return 3; // right_middle
            }
        }
        else 
            return 5; // mid_top
    }
    else{
        if(pos.y < table_coupe.bl.y) 
            return 2; // right_bot
        if(pos.y < table_coupe.tl.y)
            return 3; // right_mid
        else 
            return 4; // right_top
    }
}



void RobotCoupe::follow_to(struct position pos){
    /* Zones finding */
    int start_zone = what_zone(_position);
    Serial.print("\n*** Start zone is "); Serial.print(start_zone); Serial.print(" *** \n");
    int end_zone = what_zone(pos);
    Serial.print("\n*** End zone is "); Serial.print(end_zone); Serial.print(" *** \n");


    /* Place robot to nearest point in circulation path*/
    if(start_zone== 0){
        RobotCoupe::go_to(table_coupe.bl);
    }
    else if(start_zone== 1){
        struct position destination = {_position.x, table_coupe.bl.y};
        RobotCoupe::go_to(destination);
        RobotCoupe::wait_stop();
    }
    else if(start_zone== 2){
        RobotCoupe::go_to(table_coupe.br);
        RobotCoupe::wait_stop();
    }
    else if(start_zone== 3){
        struct position destination = {table_coupe.br.x, _position.y};
        RobotCoupe::go_to(destination);
        RobotCoupe::wait_stop();
    }
    else if(start_zone== 4){
        RobotCoupe::go_to(table_coupe.tr);
        RobotCoupe::wait_stop();
    }
    else if(start_zone== 5){
        struct position destination = {_position.x, table_coupe.tr.y};
        RobotCoupe::go_to(destination);
        RobotCoupe::wait_stop();
    }
    else if(start_zone== 6){
        RobotCoupe::go_to(table_coupe.tl);
        RobotCoupe::wait_stop();
    }
    else if(start_zone== 7){
        struct position destination = {table_coupe.bl.x, _position.y};
        RobotCoupe::go_to(destination);
        RobotCoupe::wait_stop();
    }
    /* Robot is now on circulation path */

    /* Path rotation direction finding*/
    // Imaginary line crossing starting position and center of stage
    float a = (_position.y - table_coupe.extreme.y/2)/(_position.x - table_coupe.extreme.x/2);
    float b = _position.y - a * _position.x;

    // Checking if destination point is above or under this line
    int rotation_direction;
    if(a*pos.x + b > pos.y){ // point under the line
        if(_position.x < table_coupe.extreme.x /2){
            rotation_direction = -1;
        }
        else{
            rotation_direction = 1;
        }
    }
    else{ // point above the line
        if(_position.x < table_coupe.extreme.x /2){
            rotation_direction = 1;
        }
        else{
            rotation_direction = -1;
        }
    }
    /* rotation direction found */
    Serial.print("\n*** Circulation direction is "); Serial.print(rotation_direction); Serial.print(" ***\n");
    /* circulating robot following circulation path */
    int x_zone = start_zone;
    while(x_zone != end_zone){
        x_zone = (x_zone - rotation_direction)%8;
        if(x_zone%2 == 0){ // next zone is a corner
            RobotCoupe::go_to(corner_positon(x_zone));
            RobotCoupe::wait_stop();
        }
        Serial.print("\n*** Now in zone "); Serial.print(x_zone); Serial.print(" ***\n");
    }
    Serial.print("\n*** End of circulation zone is"); Serial.print(x_zone); Serial.print(" ***\n");
    /* Robot stopped at nearest corner from desired zone */

    /* last movement is to go to end position*/
    if(end_zone== 0){
        RobotCoupe::go_to(pos);
    }
    else if(end_zone== 1){
        struct position destination = {pos.x, table_coupe.bl.y};
        RobotCoupe::go_to(destination);
        RobotCoupe::wait_stop();
        RobotCoupe::go_to(pos);
    }
    else if(end_zone== 2){
        RobotCoupe::go_to(pos);
    }
    else if(end_zone== 3){
        struct position destination = {table_coupe.br.x, pos.y};
        RobotCoupe::go_to(destination);
        RobotCoupe::wait_stop();
        RobotCoupe::go_to(pos);
    }
    else if(end_zone== 4){
        RobotCoupe::go_to(pos);
    }
    else if(end_zone== 5){
        struct position destination = {pos.x, table_coupe.tr.y};
        RobotCoupe::go_to(destination);
        RobotCoupe::wait_stop();
        RobotCoupe::go_to(pos);
    }
    else if(end_zone== 6){
        RobotCoupe::go_to(pos);
    }
    else if(end_zone== 7){
        struct position destination = {table_coupe.bl.x, pos.y};
        RobotCoupe::go_to(destination);
        RobotCoupe::wait_stop();
        RobotCoupe::go_to(pos);
    }
    /* Robot is at desired position */
}
