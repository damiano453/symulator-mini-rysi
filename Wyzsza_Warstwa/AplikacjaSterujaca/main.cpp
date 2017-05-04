#include <iostream>
#include "robot.h"
#include <unistd.h>

/// TODO zrobić obsługę wyjotków, bo się pogubisz!!

using namespace std;

string ControlFile = "/home/damian/git-repos/symulator-mini-rysi/vrep/matlab/Commands.txt";
string OutputFile = "/home/damian/git-repos/symulator-mini-rysi/vrep/matlab/Output.txt";

int main(int argc, char *argv[])
{
  robot robocik(ControlFile,OutputFile);    ///Tworzymy pojedynczego robota (mozna stworzyc tablice, czyli kilka)
  robocik.setSpeed(10,10);
//  cout << endl << 10%180 << endl;
//  while (1) {
//      robocik.stopSimulation(0);
//      cout<<*robocik.getAbsolutePosition()<<"  "<<*(robocik.getAbsolutePosition()+1)<<endl;
//      getchar();
//    }
  robocik.stopSimulation(0);//start
  irys::sleepcp(50); //500ms
  cout<< robocik.goStraight(3)<<endl;
  //robocik.stopSimulation(1);//start
  return 0;
}

