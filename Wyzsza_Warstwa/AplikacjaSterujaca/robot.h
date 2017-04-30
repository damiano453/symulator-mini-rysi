#ifndef ROBOT_H
#define ROBOT_H
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

class robot
{
private:
  struct paramsToFile{
    std::string left;     //Velocity of left wheel
    std::string right;    //Velocity of right wheel
    std::string angle;    //angle of robot position
    std::string run;      //start simulation = 0; stop simulation = 1
  };
  std::string ControlFile;
  std::string PathToFile;
  paramsToFile paramsOfRobot;
public:
  void saveControlFile();
public:
  robot(std::string &controlFile);
};

#endif // ROBOT_H
