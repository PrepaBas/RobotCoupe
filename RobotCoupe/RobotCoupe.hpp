/*
  RobotCoupe.h - Library for controling 2 weeled robots using XNucleoIHM02A1.
  Created by Bastien T. France, january, 2025.
  Released into the public domain.
*/

#ifndef RobotCoupe_h
#define RobotCoupe_h

#include "Arduino.h"


class RobotCoupe {

  // Methodes : public par defaut:
  public:

  RobotCoupe();
  void begin(float baseWidth, float wheelRadius);
  // Setters
  void set_x(float x);
  void set_y(float y);
  void set_theta(float theta);
  void set_baseWidth(float baseWidth);
  void set_wheelRadius(float wheelRadius);
  // Autres methodes
  void move_straight(char direction, float distance);
  void rotate(int direction, float angle);
  void wait_stop();
  void go_to()

  // Attributs : private par defaut
  private:

  float _x;
  float _y;
  float _theta;
  float _baseWidth;
  float _wheelRadius;

  
};


#endif