#ifndef _MAIN_H_
#define _MAIN_H_

#define STATUS_TEXT_MESSAGE_LENGTH 255

typedef struct RobotStatus {
    float inclination;      /// robot's tilt angle
    float sensorsData[3];   /// radius distances from sonars
    float linearVelocity;   /// robot's horizontal velocity
    float angularVelocity;  /// robot's wheel's angular velocity
    float positionXYZ[3];   /// robot's position in the area
    float orientation;      /// orientation relative to the XY-axes
    float batteryState;     /// battery state in %
    int id;                 /// robot's ID
    int busy;              /// tells whether the robot is performing some task
    char textMessage[STATUS_TEXT_MESSAGE_LENGTH];  /// additional message
} message;

#endif
