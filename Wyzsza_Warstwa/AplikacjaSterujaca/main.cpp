#include <iostream>
#include "robot.h"

/// TODO zrobić obsługę wyjotków, bo się pogubisz!!

using namespace std;

string ControlFile = "/home/damian/git-repos/symulator-mini-rysi/vrep/Commands.txt";
string OutputFile = "/home/damian/git-repos/symulator-mini-rysi/vrep/Output.txt";

int main(int argc, char *argv[])
{
  robot robocik(ControlFile,OutputFile);
  robocik.setSpeed(0,0);
  robocik.layDown();
  cout << endl<<endl<<*robocik.getAbsolutePosition()<<endl<<*(robocik.getAbsolutePosition()+1);   //Tak zwraca wartość przez wskaźnik jbc.

  return 0;
}

