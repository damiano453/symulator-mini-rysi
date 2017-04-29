#include "robotsimswarmapibody.h"
#include <iostream>
#include <fstream>
#include <sstream>

/*
 * Method of communication with simulator
 */
void simulator::saveControlFile(simulator::paramsToFile *RobotStruct){
  std::ofstream file;
  file.open("/home/damian/git-repos/symulator-mini-rysi/vrep/Commands.txt");//,std::ios::out);

  file << RobotStruct->left <<"\t"<< RobotStruct->right <<"\t"<< RobotStruct->angle <<std::endl<< RobotStruct->run <<std::endl;

  std::cout << RobotStruct->left <<"\t"<< RobotStruct->right <<"\t"<< RobotStruct->angle <<std::endl<< RobotStruct->run <<std::endl;

  file.close();
}

/*
 * Set speed; Uses method of communication
 */
void irys::setSpeed(int left, int right){
  // To file
  std::stringstream value[2];
  std::string rowToSting;
  value[0] << left/100.0;
  value[1] << right/100.0;

  rowToSting = value[0].str();
  paramsOfRobot.left = rowToSting;

  rowToSting = value[1].str();
  paramsOfRobot.right = rowToSting;

  simulator::saveControlFile(&paramsOfRobot);

}
