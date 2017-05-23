#include "robot.h"
#define SLOWDOWN 5000.0

/*
 * konstruktur, inicjalizacja -> robot nie rusza się, symulacja stop
 */
robot::robot(std::string &pathToFile,int robotID)
{
  std::ostringstream ID;
  ID << robotID;
  ControlFile = pathToFile+std::string("Commands")+ID.str()+std::string(".txt");    // location of control file
  OutputFile = pathToFile+std::string("Output")+ID.str()+std::string(".txt");      // location of Output file
  std::cout << ControlFile<<std::endl;
  paramsOfRobot.angle = "0";        // angle of robot
  paramsOfRobot.left = "0";         // velocity of left wheel
  paramsOfRobot.right = "0";        // velocity of right wheel
  paramsOfRobot.run = "1";          // stop simulation
  pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164;
  this->saveControlFile();
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
      file.close();
  }
  else std::cout << "Unable to open file";
}

/*
 * Obraca:
 * 0 -> w prawo
 * 1 -> w lewo
 * 2 -> w tył
 * TODO   czasem odwraca źle, ale zawsze do pozycji
 */
void robot::turn(int dir)
{
  ///PID var
  float T = 0.05;
  float K  = 50;
  float Td = 0;
  float Ti = 450;

  int ogr = 6000;

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
  ///PID params
  r2 = K*Td/T;
  r1 = K*(T/2/Ti - 2*Td/T - 1);
  r0 = K*(1+ T/2/Ti + Td/T);

  //Find Position
  int directionToGet = 0.0;
  if(dir == 0)  dir = -1; // do algorytmu
  directionToGet = dir*90;
  readOutputFile();
  directionToGet += RobotStat.orientation;
  if(abs(directionToGet)<=2)  directionToGet = 0;
  else if(abs(90-directionToGet)<=2)  directionToGet = 90;
  else if(abs(180-directionToGet)<=2)  directionToGet = 180;
  else if(abs(270-directionToGet)<=2)  directionToGet = 270;
  else if(abs(360-directionToGet)<=2)  directionToGet = 0;
  if(directionToGet >= 0)
    directionToGet = directionToGet%360;
  else
    directionToGet = (360+directionToGet)%360;
  if(dir == 2)
    dir = 1; // to algorithm
  // PID control
  do {
      ek_2 = ek_1;
      ek_1 = ek_0;
      readOutputFile();
      ek_0 = (dir*directionToGet - dir*RobotStat.orientation);

      up = K* ek_0;
      ud = K*Td*((ek_0-ek_1)/T);
      ui = ui_1+K/Ti*T*((ek_1+ek_0)/2);

      u = up + ui + ud;

      // PID alg
      ui_1 = ui;
      u = r2*ek_2 + r1*ek_1 + r0*ek_0 + uk_1; // <-- tutaj algorytm regulacji
      uk_1 = u;

      // limits
      if (u > ogr) u = ogr;
      else if (u < -ogr) u = -ogr;
      else u = u;

      //to Robot
      setSpeed(-dir*u,dir*u);
      irys::sleepcp(45); //5ms
    } while (abs(ek_0*10)>=1);
  setSpeed(0,0);
}

/*
 * Jedzie do przodu o liczbę bloków
 */
float robot::goStraight(int numberOfBlocks, int length)
{
  int mul= 0;
  ///PID var
  float T = 0.05;
  float K  = 22000;
  float Td = 0;
  float Ti = 110;

  int ogr = 2000;

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

  //PID params
  r2 = K*Td/T;
  r1 = K*(T/2/Ti - 2*Td/T - 1);
  r0 = K*(1+ T/2/Ti + Td/T);

  /// position to get
  float distanceToGet = 0.0;
  float posXY = 0;
  int position;
  readOutputFile();
  if (length == 0)
    distanceToGet = 0.5*numberOfBlocks;
  else if (length == 1)
    distanceToGet = 0.25;
  else if (length == 2)
    distanceToGet = 0.05;
  else if (length == 3)
    distanceToGet = -0.05;
  else length = 0;

  if(abs(180-abs(int(RobotStat.orientation)))<=2){
      position = 0;
      if (length != 3)
        distanceToGet *= -1;
      mul = -1;
    }
  else if(abs(360-abs(int(RobotStat.orientation)))<=2){
      position = 0;
      mul = 1;
    }
  else if(abs(int(RobotStat.orientation))<=2){
      position = 0;
      mul = 1;
    }
  else if(abs(270-abs(int(RobotStat.orientation)))<=2){
      position = 1;
      if (length != 3)
        distanceToGet *= -1;
      mul = -1;
    }
  else if(abs(90-abs(int(RobotStat.orientation)))<=2){
      position = 1;
      mul = 1;
    }
  std::cout << "Yzad = "<<distanceToGet <<std::endl;
  switch (length) {
    case 0:     // incrementing 0.5 from 0.25
      if(int((RobotStat.positionXYZ[position]-0.25)*10/5*10)%10 >= 6){
              posXY = (int(RobotStat.positionXYZ[position]*10/5)+1)*0.5+0.25;
        }
      else    posXY = int(RobotStat.positionXYZ[position]*10/5)*0.5+0.25;
      break;
    case 1:     // 0.25 from the beginning of square
      posXY = 0;
      break;
    case 2:     // 0.3 position from beggining of square
      if(int((RobotStat.positionXYZ[position]-0.25)*10/5*10)%10 >= 6){
          if(RobotStat.positionXYZ[position]>=0)
              posXY = (int(RobotStat.positionXYZ[position]*10/5)+1)*0.5+0.25;
          else
              posXY = (int(RobotStat.positionXYZ[position]*10/5)-1)*0.5-0.25;
        }
      else{
          if(RobotStat.positionXYZ[position]>=0)
              posXY = int(RobotStat.positionXYZ[position]*10/5)*0.5+0.25;
          else
              posXY = int(RobotStat.positionXYZ[position]*10/5)*0.5-0.25;
      }
      ogr = 200;
      std::cout<<"jestemTu"<<std::endl;
      break;
    case 3:     // 0.3 position from beggining of square
      if(int((RobotStat.positionXYZ[position]-0.25)*10/5*10)%10 >= 6){
          if(RobotStat.positionXYZ[position]>=0)
              posXY = (int(RobotStat.positionXYZ[position]*10/5)+mul*1)*0.5+0.3;
          else
              posXY = (int(RobotStat.positionXYZ[position]*10/5)-mul*1)*0.5-0.2;
        }
      else {
          if(RobotStat.positionXYZ[position]>=0)
              posXY = int(RobotStat.positionXYZ[position]*10/5)*0.5+0.3;
          else
              posXY = int(RobotStat.positionXYZ[position]*10/5)*0.5-0.2;
      }
      ogr = 200;
      break;
    }
  distanceToGet += posXY;

  // PID control
  do {
      ek_2 = ek_1;
      ek_1 = ek_0;
      readOutputFile();
      ek_0 = (distanceToGet - RobotStat.positionXYZ[position]);
      std::cout<<"Uchyb = "<<RobotStat.positionXYZ[position]<<std::endl<<std::endl;

      up = K* ek_0;
      ud = K*Td*((ek_0-ek_1)/T);
      ui = ui_1+K/Ti*T*((ek_1+ek_0)/2);

      u = up + ui + ud;

      //PID algorithm
      ui_1 = ui;
      u = r2*ek_2 + r1*ek_1 + r0*ek_0 + uk_1;
      uk_1 = u;

      // limits oof control val
      if (u > ogr) u = ogr;
      else if (u < -ogr) u = -ogr;
      else u = u;

      setSpeed(mul*u,mul*u);
      irys::sleepcp(45); //5ms
    } while (abs(ek_0*100)>=1);
  setSpeed(0,0);
  return 0;
}

/*
 * Jedzie do środka klocka
 */
void robot::goMiddle(void){
  this->goStraight(1,1);
  this->turn(direction(lewo));
  this->goStraight(1,1);
}

/*
 * Jedzie do przeszkody
 */
void robot::goToObstacle(void){
  this->goStraight(1,2);
}

/*
 * Odjeżdza od przeszkody
 */
void robot::goFromObstacle(void){
  this->goStraight(1,3);
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
  value[0] << left/SLOWDOWN;
  value[1] << right/SLOWDOWN;

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
      RobotStat.orientation = RobotStat.orientation*180/pi;
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
/*
 * Kierunek robota
 */
float robot::getOrientation()
{
  readOutputFile();
  return RobotStat.orientation;
}
