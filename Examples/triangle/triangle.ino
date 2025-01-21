#include <RobotCoupe.h>
#include "XNucleoIHM02A1.h"

RobotCoupe robot;

void setup(){
    robot.begin();
}

void loop()
{   
    robot.move_straight(1, 1000);
    robot.wait_stop();


    robot.rotate(1, 60);
    robot.wait_stop();

    robot.move_straight(1, 1000);
    robot.wait_stop();


    robot.rotate(1, 60);
    robot.wait_stop();


    robot.move_straight(1, 1000);
    robot.wait_stop();


    robot.rotate(1, 60);
    robot.wait_stop();
    delay(1000);
}