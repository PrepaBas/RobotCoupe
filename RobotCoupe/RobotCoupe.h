/*
  RobotCoupe.h - Library for controling 2 weeled robots using XNucleoIHM02A1.
  Created by Bastien T. France, january, 2025.
  Released into the public domain.
*/

#ifndef RobotCoupe_h
#define RobotCoupe_h

#include "Arduino.h"


class RobotCoupe {

  public:
    RobotCoupe();
    void begin();
    void move_straight(char direction, float distance);
    void rotate(int direction, float angle);
    void wait_stop();
};


#endif