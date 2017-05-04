clear all
clc

%% U mnie tylko tak działa ciągłe sterowanie z pliku
%  Bez ciągłego zamyknia i otwierania, nie matlab nie odczytuje

outputFile = 'Output.txt';
[fidout,msg]=fopen('/home/damian/git-repos/symulator-mini-rysi/vrep/matlab/Output.txt','w')
fprintf(fidout,'pos+ori')

fclose('all');
fclose all;