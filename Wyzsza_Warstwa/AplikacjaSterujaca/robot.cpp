#include "robot.h"

robot::robot(std::string &ControlLocation)
{
  ControlFile = ControlLocation;    // location of control file
  paramsOfRobot.angle = "0";        // angle of robot
  paramsOfRobot.left = "0";         // velocity of left wheel
  paramsOfRobot.right = "0";        // velocity of right wheel
  paramsOfRobot.run = "1";          // stop simulation
}

void robot::saveControlFile()
{
  std::cout << ControlFile << std::endl << std::endl<<std::endl;
  std::ofstream file;// (ControlFile,PathToFile);
  file.open(ControlFile.c_str());
  //file.open(ControlFile);//,std::ios::out);
  if (file.is_open())
  {
    std::cout << "This is a line.\n";
    std::cout << "This is another line.\n";
  }
  else std::cout << "Unable to open file";

  file << paramsOfRobot.left <<"\t"<< paramsOfRobot.right <<"\t"<< paramsOfRobot.angle <<std::endl<< paramsOfRobot.run <<std::endl;

  std::cout << paramsOfRobot.left <<"\t"<< paramsOfRobot.right <<"\t"<< paramsOfRobot.angle <<std::endl<< paramsOfRobot.run <<std::endl;

  file.close();
}
