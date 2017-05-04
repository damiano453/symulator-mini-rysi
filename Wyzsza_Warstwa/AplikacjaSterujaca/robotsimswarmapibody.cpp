#include "robotsimswarmapibody.h"
#include <iostream>
#include <fstream>
#include <sstream>

void irys::sleepcp(int milliseconds) // cross-platform sleep function
{
    #ifdef WIN32
    Sleep(milliseconds);
    #else
    usleep(milliseconds * 1000); ///conversion na ms
    #endif // win32
}

/*
 * Set speed; Uses method of communication
 */
/*
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
*/
