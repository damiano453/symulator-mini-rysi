#ifndef ROBOT_H
#define ROBOT_H
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <cstdlib>
#include <cmath>


#include "robotsimswarmapibody.h"

//#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164;

//Struktura pomocnicza
/// TODO trzeba zrobić porządek w plikach
struct paramsToFile{
  std::string left;     //Velocity of left wheel
  std::string right;    //Velocity of right wheel
  std::string angle;    //angle of robot position
  std::string run;      //start simulation = 0; stop simulation = 1
};

enum direction {prawo=0, lewo=1, back=2};

class robot
{
private:
  float pi;
  std::string ControlFile;
  std::string OutputFile;
  paramsToFile paramsOfRobot;
  irys::RobotStatus RobotStat;
  void saveControlFile();
  void readOutputFile();
public:
  // obroc: patrz enum direction
  void turn(int dir);
  // jedź o numberOfBlock do przodu, drugi parametr zawsze 0 !!
  float goStraight(int numberOfBlocks, int length);   //0 jedź o cały; 1 jedź o pół; 2 jedź do klocka
  // jedź do środka klocka
  void goMiddle(void);
  // go to obstacle (when lying)
  void goToObstacle(void);
  // go from Obstacle (when lying)
  void goFromObstacle(void);
  // start symulacji
  void stopSimulation(int stop);    ///0 - start;  1 - stop.
  //informacje do sterowania ||*(&)->x || *(&+1)->y
  float* getAbsolutePosition();
  float getOrientation();
  //informacje i kontrola
  int getDistance(int sensornumber);
  void setSpeed(int left, int right);
  //kładzie i wstaje
  void standUp();
  void layDown();
  robot(std::string &pathToFile,int robotID);
};

#endif // ROBOT_H
