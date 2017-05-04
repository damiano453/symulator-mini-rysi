#ifndef ROBOTSIMSWARMAPIBODY_H
#define ROBOTSIMSWARMAPIBODY_H
#include <string>

namespace irys {

const std::size_t STATUS_TEXT_MESSAGE_LENGTH = 31;
const std::size_t MAP_X_SIZE = 12; // rozmiar mapy x
const std::size_t MAP_Y_SIZE = 12; //rozmiar mapy y

/// Structure describing robot's status
///
/// It is sent to the whole swarm from each robot
struct RobotStatus {
    float inclination;      /// robot's tilt angle
    float sensorsData[3];   /// radius distances from sonars
    float linearVelocity;   /// robot's horizontal velocity
    float angularVelocity;  /// robot's wheel's angular velocity
    float positionXYZ[3];   /// robot's position in the area
    float orientation;      /// orientation relative to the XY-axes
    float batteryState;     /// battery state in %
    int id;                 /// robot's ID
    bool busy;              /// tells whether the robot is performing some task
    char textMessage[STATUS_TEXT_MESSAGE_LENGTH];  /// additional message
};

//int getDistance(int sensornumber); // 3 czujniki tak jak w robicie zakres 0 - 300 mm 1-przód, 2-tył, 3-góra
//void standUp(); // to chyba jasne
//void layDown(); // to też
//void setSpeed(int left, int right); // jednak zostaniemy przy zadawanu prędkości na silniki od -1000 do 1000
//float* getAbsolutePosition(); // tak jak się umawialiśmy

// funkcje operujące na mapie jak i sposób jej reprezanetacji pozostawiam wam do okreslenia


} // namespace irys


#endif // ROBOTSIMSWARMAPIBODY_H
