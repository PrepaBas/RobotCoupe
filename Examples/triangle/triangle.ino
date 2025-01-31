#include "RobotCoupe.h"
#include "XNucleoIHM02A1.h"

RobotCoupe robot;

void setup(){
    robot.begin(263, 36);
    
}

void loop()
{   
    for(int i=0; i<3; i++){
        robot.move_straight(1, 700);
        robot.wait_stop();

        robot.rotate(1, 120);
        robot.wait_stop();
    }
    delay(2000);
}
