#include <iostream>
#include "robotsimswarmapibody.h"
#include "robot.h"

using namespace std;

string ControlFile = "/home/damian/git-repos/symulator-mini-rysi/vrep/Commands.txt";

int main(int argc, char *argv[])
{
  robot robocik(ControlFile);
  robocik.saveControlFile();
  return 0;
}

