clear all
clc

%% U mnie tylko tak działa ciągłe sterowanie z pliku
%  Bez ciągłego zamyknia i otwierania, nie matlab nie odczytuje

commandFile = '/Commands.txt'
fid=fopen(commandFile,'r');

i = 0;
while 1
    fid=fopen(commandFile,'r');
    i = i+1
    frewind(fid);
    speeds=fscanf(fid,'%f\t%f\t%f\n%d')
    pause(0.5)
    fclose(fid)
end


fclose('all');