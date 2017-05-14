#include <iostream>
#include "robot.h"
#include <unistd.h>


/// TODO zrobić obsługę wyjotków, bo się pogubisz!!

using namespace std;

//Podaj ścieżkę do plików
string pathToFile = "/home/damian/git-repos/symulator-mini-rysi/vrep/c/";
string path;
int num = 0;

int main(int argc, char *argv[])
{

  int i=0;
 robot robocik(pathToFile,0);    ///Tworzymy pojedynczego robota (mozna stworzyc tablice, czyli kilka)
  robocik.stopSimulation(0);//start
  irys::sleepcp(50); //50ms
//  robocik.layDown();
//  robocik.turn(2);
//  robocik.goToObstacle();
//  irys::sleepcp(1000);
//  robocik.goFromObstacle();
//  robocik.standUp();


  ////// Zacznij z pozycji 0;0
    robocik.goMiddle();
  ////// Zacznij z pozycji 0.25; 0.25
  // Przykładowa trajektoria:
    robocik.turn(direction(prawo));
    robocik.goStraight(1,0);
    robocik.turn(direction(prawo));
    robocik.goStraight(2,0);
    robocik.turn(direction(prawo));
    robocik.goStraight(3,0);
    robocik.turn(direction(prawo));
    robocik.goStraight(1,0);
    robocik.turn(direction(back));
    robocik.goStraight(1,0);
  //Dojazd i odjazd od przeszkody, gdy leży, przykład
    robocik.layDown();
    robocik.goToObstacle();
    irys::sleepcp(1000);
    robocik.goFromObstacle();
    robocik.standUp();

  return 0;
}

