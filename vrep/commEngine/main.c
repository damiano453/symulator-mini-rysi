#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"

#ifdef __linux__
#include <unistd.h>
#elif _WIN32
#include <Windows.h>
#else
#error Nie zdefiniowano systemu!
#endif

#define NUMPOS 4
#define MAXNAMELENGTH 255
#define BOXESDIRECTORY "E:/blabla/"
#define SLEEPMS 500

void mysleep(int ms);

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

int main(int argc, char *argv[])
{

	int clientNum=1;

	if(argc>1)
		sscanf(argv[1],"%d",&clientNum);

	long int *filePositions;
	filePositions=malloc(sizeof(*filePositions)*clientNum);

	for (int i=0; i<clientNum; ++i) {
		filePositions[i]=0;
	}

	FILE *inputFile,*outputFile;


	while(1) {
		for(int i=0; i<clientNum; ++i) {
			char inputname[MAXNAMELENGTH]=BOXESDIRECTORY;
			char number[NUMPOS];
			char extension[]=".dat";
			snprintf(number,NUMPOS,"%d",i);
			
			strcat(inputname,"OutBox");
			strcat(inputname,number);
			strcat(inputname,extension);

			inputFile=fopen(inputname,"rb");

			fseek(inputFile,0,SEEK_END);
			long int fileLength=ftell(inputFile);
			long int newData=fileLength-filePositions[i];
			fseek(inputFile,filePositions[i],SEEK_SET);

			if(newData>=sizeof(message)) {

				message tempMessage;
				fread(&tempMessage,sizeof(tempMessage),1,inputFile);
				filePositions[i]=ftell(inputFile);
				
				for (int j=0; j<clientNum; ++j) {
					if(i==j) continue;

					char outputname[MAXNAMELENGTH]=BOXESDIRECTORY;
					snprintf(number,NUMPOS,"%d",j);
					
					strcat(outputname,"InBox");
					
					strcat(outputname,number);
					strcat(outputname,extension);

					outputFile=fopen(outputname,"ab");

					fwrite(&tempMessage,sizeof(tempMessage),1,outputFile);

					fclose(outputFile);
				}
			} else
				fclose(inputFile);

		}
		mysleep(SLEEPMS);
	}
	return 0;
}

void mysleep(int ms)
{
#ifdef __linux__
	usleep(ms*1000);
#elif _WIN32
	Sleep(ms);
#else
#error Nie zdefiniowano systemu!
#endif
}
