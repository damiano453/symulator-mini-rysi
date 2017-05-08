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
  pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164;
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
 * Jedzie do przodu o liczbę bloków
 */

float robot::goStraight(int numberOfBlocks)
{
  int i=0;
  ///PID params
  float T = 0.05;
  float K  = 10000;
  float Td = 0;
  float Ti = 10000000000000;

  float Tv = 10000000000000;

  float r2 = 0.0;
  float r1 = 0.0;
  float r0 = 0.0;

  float uk_1 =  0.0;
  float ek_0 = 0.0;
  float ek_1 = 0.0;
  float ek_2 = 0.0;

  float u = 0.0;
  float up = 0.0;
  float ui = 0.0;
  float ui_1 =0.0;
  float ud = 0.0;
  float uw_1 = 0.0;
  r2 = K*Td/T;
  r1 = K*(T/2/Ti - 2*Td/T - 1);
  r0 = K*(1+ T/2/Ti + Td/T);
  ///PID params
  float distanceToGet = 0.0;
  int position;
  readOutputFile();
  distanceToGet = 0.5*numberOfBlocks;
  if(int((RobotStat.orientation*180/pi))%180==0){
      position = 0;
    }
  else if (int((RobotStat.orientation*180/pi))%180==0) {
      position = 1;
    }
  distanceToGet += RobotStat.positionXYZ[position];
  do {
      std::cout<<"direction = "<<position<<std::endl;
      std::cout<<"control error = "<<ek_0<<std::endl;
      std::cout<<"Position = "<<RobotStat.positionXYZ[position]<<std::endl;
      readOutputFile();
      i++;
      ek_2 = ek_1;
      ek_1 = ek_0;
      ek_0 = (distanceToGet - RobotStat.positionXYZ[position]);

      up = K* ek_0;
      ud = K*Td*((ek_0-ek_1)/T);
      ui = ui_1+K/Ti*T*((ek_1+ek_0)/2);


      // anty windup
      //ui= ui + T/Tv*(uw_1 - uk_1);

      u = up + ui + ud;

      ui_1 = ui;
      u = r2*ek_2 + r1*ek_1 + r0*ek_0 + uk_1; // <-- tutaj algorytm regulacji
      uk_1 = u;
      //u windup
      if (uk_1 > 2000) uw_1 = 2000;
      else if (uk_1 < -2000) uw_1 = -2000;
      else uw_1 = uk_1;

      //only u
      if (u > 2000) u = 2000;
      else if (u < -2000) u = -2000;
      else u = u;

      setSpeed(u,u);
      std::cout <<"u = "<< u<<std::endl<<"ek = "<<ek_0<<std::endl<<std::endl;
      irys::sleepcp(50); //50ms
    } while (ek_0!=0);
  setSpeed(0,0);
}

/*
 * Start symulacji
 * 0 - start;  1 - stop.
 */
void robot::stopSimulation(int stop)
{
  std::stringstream value;
  value<<stop;
  paramsOfRobot.run = value.str();    ///Start simulation
  saveControlFile();
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
  value[0] << left/1000.0;
  value[1] << right/1000.0;

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
