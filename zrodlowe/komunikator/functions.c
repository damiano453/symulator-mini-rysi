#include <stdio.h>
#include <stdlib.h>

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

#define NUMPOS 4
#define MAXNAMELENGTH 255
#define BOXESDIRECTORY "../comm_robots/"

FILE *inputFile, *outputFile;

void sendMessageToSwarm(int myId,message myMessage)
{
	char outputname[MAXNAMELENGTH]=BOXESDIRECTORY;

	char number[NUMPOS];
	char extension[]=".dat";
	snprintf(number,NUMPOS,"%d",myId);

	strcat(outputname,"OutBox");
	strcat(outputname,number);
	strcat(outputname,extension);

	outputFile=fopen(outputname,"ab");

	fwrite(&myMessage,sizeof(myMessage),1,outputFile);
	fclose(outputFile);
}

message receiveDatagram(int myId)
{
	static long int filePosition=0;

	char inputname[MAXNAMELENGTH]=BOXESDIRECTORY;

	char number[NUMPOS];
	char extension[]=".dat";
	snprintf(number,NUMPOS,"%d",myId);

	strcat(inputname,"InBox");
	strcat(inputname,number);
	strcat(inputname,extension);

	inputFile=fopen(inputname,"rb");

	fseek(inputFile,0,SEEK_END);
	long int fileLength=ftell(inputFile);
	long int newData=fileLength-filePosition;
	fseek(inputFile,filePosition,SEEK_SET);

	if(newData>=sizeof(message)) {
		message tempMessage;
		fread(&tempMessage,sizeof(tempMessage),1,inputFile);
		filePosition=ftell(inputFile);
		
		fclose(inputFile);

		return tempMessage;
	} else
		return;


}
