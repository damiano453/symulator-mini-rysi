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
 //robocik.setSpeed(10,10);
 robocik.stopSimulation(0);//start
 irys::sleepcp(50); //50ms
 robocik.turn(1);
 //cout<< robocik.goStraight(1)<<endl;
 //robocik.stopSimulation(1);//start
  return 0;
}

