#include "robot.h"
/*
 * konstruktur, inicjalizacja -> robot nie rusza się, symulacja stop
 */
robot::robot(std::string &ControlLocation,std::string &OutputLocation)
{
  ControlFile = ControlLocation;    // location of control file
  OutputFile = OutputLocation;      // location of Output file
  paramsOfRobot.angle = "0";        // angle of robot
  paramsOfRobot.left = "0";         // velocity of left wheel
  paramsOfRobot.right = "0";        // velocity of right wheel
  paramsOfRobot.run = "1";          // stop simulation
}

/*
 * zapis komunikacji do pliku (odczyt w symulatorze)
 */
void robot::saveControlFile()
{
  std::ofstream file;
  file.open(ControlFile.c_str());
  if (file.is_open())
  {
      file << paramsOfRobot.left <<"\t"<< paramsOfRobot.right <<"\t"<< paramsOfRobot.angle <<std::endl<< paramsOfRobot.run <<std::endl;
      std::cout << paramsOfRobot.left <<"\t"<< paramsOfRobot.right <<"\t"<< paramsOfRobot.angle <<std::endl<< paramsOfRobot.run <<std::endl;
      file.close();
  }
  else std::cout << "Unable to open file";
}

/*
 * Ustawia prędkość kół robota
 */
/// TODO ma być private, przesuwamy się o kwadrat (jako jednostka odległości)
void robot::setSpeed(int left, int right)
{
  // To file
  std::stringstream value[2];
  std::string rowToSting;
  value[0] << left/100.0;
  value[1] << right/100.0;

  rowToSting = value[0].str();
  paramsOfRobot.left = rowToSting;

  rowToSting = value[1].str();
  paramsOfRobot.right = rowToSting;

  saveControlFile();
}

/*
 * Robot wstaje do pionu (angle = 0)
 */
void robot::standUp()
{
  paramsOfRobot.angle = "0";
  saveControlFile();
}

/*
 * Robot się kładzie, założono angle = 90*
 */
void robot::layDown()
{
  paramsOfRobot.angle = "90";
  saveControlFile();
}

/*
 * Czyta parametry robota z symulatora
 */
void robot::readOutputFile()
{
  std::string musiByc;
  int ifMeasure[3];
  std::ifstream file;
  file.open(OutputFile.c_str());
  if (file.is_open())
  {
      file >> musiByc >> RobotStat.positionXYZ[0] >> RobotStat.positionXYZ[1] >> RobotStat.orientation;
      file >> musiByc >> RobotStat.sensorsData[0] >> RobotStat.sensorsData[1] >> RobotStat.sensorsData[2];
      file >> musiByc >> ifMeasure[0] >> ifMeasure[1] >> ifMeasure[2];
      if(ifMeasure[0] == 0)
        RobotStat.sensorsData[0] = 0;
      if(ifMeasure[1] == 0)
        RobotStat.sensorsData[1] = 0;
      if(ifMeasure[2] == 0)
        RobotStat.sensorsData[2] = 0;
      file.close();
  }
  else std::cout << "Unable to open file";

}

/*
 * Pobierz odległość od przeszkody
 */
int robot::getDistance(int sensornumber)
{
  readOutputFile();
  return RobotStat.sensorsData[sensornumber];
}

/*
 * Pozycja robota
 */
float* robot::getAbsolutePosition()
{
  readOutputFile();
  return RobotStat.positionXYZ;
}
