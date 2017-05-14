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
 * Obraca:
 * 0 -> w prawo
 * 1 -> w lewo
 * 2 -> w tył
 * TODO   czasem odwraca źle, ale zawsze do pozycji
 */
void robot::turn(int dir)
{
  int i=0;
  ///PID params
  float T = 0.05;
  float K  = 50;//4;
  float Td = 0;
  float Ti = 450;//2600;//100000000000000000;     2500

  int ogr = 6000;

  float Tv = 100000000000000000;//12000;

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
    dir = 1; // do algorytmu
  do {
      readOutputFile();
      i++;
      ek_2 = ek_1;
      ek_1 = ek_0;
      ek_0 = (dir*directionToGet - dir*RobotStat.orientation);
      if (ek_0 < 0 && i ==1)
          dir *= -1;

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
      if (uk_1 > ogr) uw_1 = ogr;
      else if (uk_1 < -ogr) uw_1 = -ogr;
      else uw_1 = uk_1;

      //only u
      if (u > ogr) u = ogr;
      else if (u < -ogr) u = -ogr;
      else u = u;

      setSpeed(-dir*u,dir*u);
      irys::sleepcp(45); //5ms
      std::cout<< std::endl<<std::endl<<"Uchyb = "<<ek_0<<std::endl<<"U = "<<u<<std::endl<<"PosToGet = "<<directionToGet<<std::endl;
    } while (abs(ek_0*100)>=4);
  setSpeed(0,0);
  readOutputFile();
  std::cout<<"Orientation = "<<RobotStat.orientation<<"   ENDEND\n";
}

/*
 * Jedzie do przodu o liczbę bloków
 */
float robot::goStraight(int numberOfBlocks)
{
  int i=0;
  int mul= 0;
  ///PID params
  float T = 0.05;
  float K  = 20000;
  float Td = 0;
  float Ti = 100;

  int ogr = 2000;

  float Tv = 1000000000000000000;

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
  //std::cout<<"to get = "<<abs(180-(int(RobotStat.orientation)%180))<<std::endl;
  if(180-abs(int(RobotStat.orientation))<=2){         //jedź, gdy w okolica 180 stopni
      position = 0;
      distanceToGet *= -1;
      mul = -1;
    }
  else if(360-abs(int(RobotStat.orientation))<=2){    //jedź, gdy w okolica 0 stopni
      position = 0;
      mul = 1;
    }
  else if(abs(int(RobotStat.orientation))<=2){        //jedź, gdy w okolica 0 stopni
      position = 0;
      mul = 1;
    }
  else if(270-abs(int(RobotStat.orientation))<=2){    //jedź, gdy w okolica 270 stopni
      position = 1;
      distanceToGet *= -1;
      mul = -1;
    }
  else if(90-abs(int(RobotStat.orientation))<=2){     //jedź, gdy w okolica 90 stopni
      position = 1;
      mul = 1;
    }
  distanceToGet += RobotStat.positionXYZ[position];
  std::cout<<"\n\nto get = "<<distanceToGet<<std::endl;
  //pause();
  do {
      std::cout<<"direction = "<<position<<std::endl;

      std::cout<<"Position = "<<RobotStat.positionXYZ[position]<<std::endl;
      readOutputFile();
      i++;
      ek_2 = ek_1;
      ek_1 = ek_0;
      ek_0 = (distanceToGet - mul*RobotStat.positionXYZ[position]);

      std::cout<<"control error = "<<ek_0<<std::endl;
 pause();
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
      if (uk_1 > ogr) uw_1 = ogr;
      else if (uk_1 < -ogr) uw_1 = -ogr;
      else uw_1 = uk_1;

      //only u
      if (u > ogr) u = ogr;
      else if (u < -ogr) u = -ogr;
      else u = u;

      setSpeed(u,u);
      std::cout <<"u = "<< u<<std::endl<<"ek = "<<ek_0<<std::endl<<std::endl;
      irys::sleepcp(10); //5ms
    //  setSpeed(0,0);
    //  pause();
    } while (abs(ek_0*100000)>=1);
  setSpeed(0,0);
  readOutputFile();
  std::cout<<"direction = "<<position<<std::endl;
  std::cout<<"control error = "<<ek_0<<std::endl;
  std::cout<<"Position = "<<RobotStat.positionXYZ[position]<<std::endl;
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
