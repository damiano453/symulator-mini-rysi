#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
//#include <conio.h>
//#include <Windows.h>

//#include <unistd.h>

#include "extApi.h"
#include "extApiPlatform.h"
#include "extApiInternal.h"

#define MODELPATH "E:/Robot.ttm"
#define RADIUS 0.15
#define SPACESIZE 3
#define SHAPEMINID 2
#define MOVEDISTANCE 0.05

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

float norm(float* vector);

int main(int argc, char *argv[])
{
	float spawn[2];
	int robotid;

	if (argc==1) {
		printf("Brak argumentow wywolania. Uzywane dane domyslne\n");
		spawn[0]=0;
		spawn[1]=0;
		robotid=0;
	} else if (argc==4) {
		sscanf(argv[1],"%d",&robotid);
		sscanf(argv[2],"%f",spawn);
		sscanf(argv[3],"%f",spawn+1);
	}
	//simxFinish(-1);
	int cid=simxStart("127.0.0.1",19997+robotid,true,true,5000,5);

	if(cid>-1) {
		//KOD programu
		printf("Podlaczono do serwera RemoteAPI :)\n");
		int responseCode;
		int robotcsys;

		responseCode=simxLoadModel(cid,MODELPATH,0,&robotcsys,simx_opmode_blocking);
		if(responseCode!=-1)
			printf("Zaladowano model robota\n");
		else
			exit(EXIT_FAILURE);

		printf("Niezerowa komendaw pliku, by zakonczyc!\n");

		int robot,korpus,ultraG,ultraP,ultraT;
		simxGetObjectChild(cid,robotcsys,0,&robot,simx_opmode_blocking);
		simxGetObjectChild(cid,robot,0,&korpus,simx_opmode_blocking);

		simxGetObjectChild(cid,korpus,0,&ultraG,simx_opmode_blocking);
		simxGetObjectChild(cid,korpus,1,&ultraP,simx_opmode_blocking);
		simxGetObjectChild(cid,korpus,2,&ultraT,simx_opmode_blocking);

		char suffix[]=".txt";
		char robotnumbertext[4];
		
		sprintf(robotnumbertext,"%d",robotid);

		char commandfilename[255]="Commands";
		char outputfilename[255]="Output";
		
		strcat(commandfilename,robotnumbertext);
		strcat(outputfilename,robotnumbertext);
		
		strcat(commandfilename,suffix);
		strcat(outputfilename,suffix);

		FILE *commandfile,*outputfile;
		commandfile=fopen(commandfilename,"r");
		outputfile=fopen(outputfilename,"wt");

		float position[]= {0.0,0.0,0.0};
		float orientation[]= {0.0,M_PI/2,0.0};
		float tilt[]= {0.0,0.0,0.0};

		simxGetObjectPosition(cid,robotcsys,-1,position,simx_opmode_blocking);
		position[0]=spawn[0];
		position[1]=spawn[1];
		simxSetObjectPosition(cid,robotcsys,-1,position,simx_opmode_blocking);
		simxGetObjectPosition(cid,robotcsys,-1,position,simx_opmode_streaming);

		simxGetObjectOrientation(cid,robotcsys,-1,orientation,simx_opmode_streaming);
		simxGetObjectOrientation(cid,robot,robotcsys,tilt,simx_opmode_streaming);

		simxReadProximitySensor(cid,ultraG,NULL,NULL,NULL,NULL,simx_opmode_streaming);
		simxReadProximitySensor(cid,ultraP,NULL,NULL,NULL,NULL,simx_opmode_streaming);
		simxReadProximitySensor(cid,ultraT,NULL,NULL,NULL,NULL,simx_opmode_streaming);

		responseCode=simx_return_novalue_flag;
		while(responseCode!=simx_return_ok)
			responseCode=simxGetObjectPosition(cid,robotcsys,-1,position,simx_opmode_buffer);

		responseCode=simx_return_novalue_flag;
		while(responseCode!=simx_return_ok)
			responseCode=simxGetObjectOrientation(cid,robotcsys,-1,orientation,simx_opmode_buffer);

		responseCode=simx_return_novalue_flag;
		while(responseCode!=simx_return_ok)
			responseCode=simxGetObjectOrientation(cid,robot,robotcsys,tilt,simx_opmode_buffer);

		float dt;
		simxGetFloatingParameter(cid,sim_floatparam_simulation_time_step,&dt,simx_opmode_blocking);
		printf("dt=%f\n",dt);
		simxSetBooleanParameter(cid,sim_boolparam_realtime_simulation,true,simx_opmode_oneshot);

		//simxStopSimulation(cid,simx_opmode_blocking);
		//simxStartSimulation(cid,simx_opmode_blocking);

		while(1) {

			float speeds[2];
			int cmd;

			//Odczyt rozkazów

			rewind(commandfile);
			fscanf(commandfile,"%f\t%f\t%f\n%d",speeds,speeds+1,tilt+2,&cmd);

			if(cmd!=0/*||kbhit()*/) {
				responseCode=simxStopSimulation(cid,simx_opmode_blocking);
				printf("stop=%d\n",responseCode);
				responseCode=simxRemoveModel(cid,robotcsys,simx_opmode_blocking);
				printf("remove=%d\n",responseCode);
				simxFinish(cid);
				break;
			}

			//Fizyka robota

			float v=(speeds[0]+speeds[1])/2;
			float w=(speeds[1]-speeds[0])/(2*RADIUS);

			position[0]+=cos(orientation[2])*v*dt;
			position[1]+=sin(orientation[2])*v*dt;

			orientation[2]=fmod(orientation[2]+w*dt,2*M_PI);

			tilt[2]=M_PI/2+tilt[2]*2*M_PI/360.0;

			simxSetObjectPosition(cid,robotcsys,-1,&position[0],simx_opmode_oneshot);
			simxSetObjectOrientation(cid,robotcsys,-1,&orientation[0],simx_opmode_oneshot);
			simxSetObjectOrientation(cid,robot,robotcsys,&tilt[0],simx_opmode_oneshot);

			char bG,bP,bT; //binarnie
			//int hG,hP,hT; //wskaŸniki na namierzone obiekty
			float pG[SPACESIZE],pP[SPACESIZE],pT[SPACESIZE]; //namierzone punkty
			//float normalG[SPACESIZE],normalP[SPACESIZE],normalT[SPACESIZE]; //normalne namierzonych powierzchni

			//Percepcja robota
			simxReadProximitySensor(cid,ultraG,&bG,pG,NULL,NULL,simx_opmode_buffer);
			simxReadProximitySensor(cid,ultraP,&bP,pP,NULL,NULL,simx_opmode_buffer);
			simxReadProximitySensor(cid,ultraT,&bT,pT,NULL,NULL,simx_opmode_buffer);

			//float temppos[3];

			/*if (bG&&hG>=SHAPEMINID&&norm(pG)<=MOVEDISTANCE) {
				simxGetObjectPosition(cid,hG,-1,temppos,simx_opmode_blocking);
				temppos[0]+=cos(orientation[2])*v*dt;//*normalG[0];
				temppos[1]+=sin(orientation[2])*v*dt;//*normalG[1];
				simxSetObjectPosition(cid,hG,-1,temppos,simx_opmode_oneshot);
			}
			if (bT&&hT>=SHAPEMINID&&norm(pT)<=MOVEDISTANCE) {
				simxGetObjectPosition(cid,hT,-1,temppos,simx_opmode_blocking);
				temppos[0]+=cos(orientation[2])*v*dt;//*normalT[0];
				temppos[1]+=sin(orientation[2])*v*dt;//*normalT[1];
				simxSetObjectPosition(cid,hT,-1,temppos,simx_opmode_oneshot);
			}
			if (bP&&hP>=SHAPEMINID&&norm(pP)<=MOVEDISTANCE) {
				simxGetObjectPosition(cid,hP,-1,temppos,simx_opmode_blocking);
				temppos[0]+=cos(orientation[2])*v*dt;//*normalP[0];
				temppos[1]+=sin(orientation[2])*v*dt;//*normalP[1];
				simxSetObjectPosition(cid,hP,-1,temppos,simx_opmode_oneshot);
			}*/

			//Zapis pomiarów
			rewind(outputfile);
			fprintf(outputfile,"pos+ori:\t%f\t%f\t%f\n",position[0],position[1],orientation[2]);
			fprintf(outputfile,"prox:\t%f\t%f\t%f\n",norm(pP),norm(pG),norm(pT));
			fprintf(outputfile,"valid:\t%d\t%d\t%d\n",bP,bG,bT);
			//fprintf(outputfile,"objs:\t%d\t%d\t%d\n",hP,hG,hT);
			fflush(outputfile);

			extApi_sleepMs(dt*1000);
		}

		fclose(commandfile);
		fclose(outputfile);
		//simxFinish(cid);
		exit(EXIT_SUCCESS);
	} else {
		printf("Nie mozna nawiazac polaczenia z VREPem :(\n");
		exit(EXIT_FAILURE);
	}

	return 0;
}

float norm(float* vector)//obliczanie drugiej normy wektora
{
	float sum=0;

	for (int i=0; i<SPACESIZE; ++i)
		sum+=vector[i]*vector[i];

	return sqrt(sum);
}
