Symulator: extApiPlatform.o extApi.o main.o
	gcc -lm -g++ -o Symulator extApiPlatform.o extApi.o main.o
	
extApi.o:extApi.c
	gcc -cpp extApi.c
	
plik2.o:extApiPlatform.c
	gcc -cpp extApiPlatform.c
	
main.o:main.c
	gcc -cpp main.c
