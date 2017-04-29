#include <iostream>
#include "robotsimswarmapibody.h"

using namespace std;

irys::RobotStatus Robot_1_Stat;
string ControlFile = "/home/damian/git-repos/symulator-mini-rysi/vrep/Commands.txt";
simulator::paramsToFile paramsOfRobot;

int main(int argc, char *argv[])
{
  paramsOfRobot.angle = "90";
  paramsOfRobot.left = "1";
  paramsOfRobot.right = "1";
  paramsOfRobot.run = "1";
  irys::setSpeed(100,100);
  return 0;
}

