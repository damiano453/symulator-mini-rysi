%% Test pomostu MATLAB VREP
% Kuba Sawulski
% 02.04.2017
%%
clear variables
clc

%% DP: Dodałem ścieżkę do pliku jako zmienną, na linuhu nie banglało
% commandFile = '/Commands.txt'   % For linux
% outputFile = '/Output.txt'      % For linux
commandFile = 'Commands.txt'   % For Windows
outputFile = 'Output.txt'       % For Windows

%%                                                                      %DP- Ścieżka moja
addpath('vrepPort');                                                    %/home/damian/git-repos/symulator-mini-rysi/vrep
modelpath='/home/damian/git-repos/symulator-mini-rysi/vrep/Robot.ttm';
vrep=remApi('remoteApi');
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

%%
if (clientID>-1)
    disp('Connected to remote API server');
    
    [res,csyshandle]=vrep.simxLoadModel(clientID,modelpath,0,vrep.simx_opmode_blocking);
    %%[res,~,~,~,stringData]=vrep.simxGetObjectGroupData(clientID,vrep.sim_object_shape_type,0,vrep.simx_opmode_blocking);
    %[res,robothandle]=vrep.simxGetObjectHandle(clientID,'Robot1',vrep.simx_opmode_blocking);
    [res,robothandle]=vrep.simxGetObjectChild(clientID,csyshandle,0,vrep.simx_opmode_blocking);
    %[res,csyshandle]=vrep.simxGetObjectHandle(clientID,'CSYS_Robot1',vrep.simx_opmode_blocking);
    [res,korpus]=vrep.simxGetObjectChild(clientID,robothandle,0,vrep.simx_opmode_blocking);
    [res,ultraG]=vrep.simxGetObjectChild(clientID,korpus,0,vrep.simx_opmode_blocking);
    [res,ultraP]=vrep.simxGetObjectChild(clientID,korpus,1,vrep.simx_opmode_blocking);
    [res,ultraT]=vrep.simxGetObjectChild(clientID,korpus,2,vrep.simx_opmode_blocking);
    
    fid=fopen(commandFile,'r');
    
    while 1
        
        frewind(fid);
        speeds=fscanf(fid,'%f\t%f\t%f\n%d')
        
        if(speeds(4)==0)
            [res]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
            [res]=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
            %%pause(4);
            
            [res,startposition]=vrep.simxGetObjectPosition(clientID,csyshandle,-1,vrep.simx_opmode_streaming);
            [res,startorientation]=vrep.simxGetObjectOrientation(clientID,csyshandle,-1,vrep.simx_opmode_streaming);
            [res,starttilt]=vrep.simxGetObjectOrientation(clientID,robothandle,csyshandle,vrep.simx_opmode_streaming);
            [res,~,~,~,~]=vrep.simxReadProximitySensor(clientID,ultraG,vrep.simx_opmode_streaming);
            [res,~,~,~,~]=vrep.simxReadProximitySensor(clientID,ultraT,vrep.simx_opmode_streaming);
            [res,~,~,~,~]=vrep.simxReadProximitySensor(clientID,ultraP,vrep.simx_opmode_streaming);
            
            pause(1);
            
            res=vrep.simx_return_novalue_flag;
            while(res~=vrep.simx_return_ok)
                [res,position]=vrep.simxGetObjectPosition(clientID,csyshandle,-1,vrep.simx_opmode_buffer);
                %[res,startorientation]=vrep.simxGetObjectOrientation(clientID,handle,-1,vrep.simx_opmode_buffer);
            end
            
            res=vrep.simx_return_novalue_flag;
            while(res~=vrep.simx_return_ok)
                %[res,startposition]=vrep.simxGetObjectPosition(clientID,handle,-1,vrep.simx_opmode_buffer);
                [res,orientation]=vrep.simxGetObjectOrientation(clientID,csyshandle,-1,vrep.simx_opmode_buffer);
            end
            
            res=vrep.simx_return_novalue_flag;
            while(res~=vrep.simx_return_ok)
                %[res,startposition]=vrep.simxGetObjectPosition(clientID,handle,-1,vrep.simx_opmode_buffer);
                [res,starttilt]=vrep.simxGetObjectOrientation(clientID,robothandle,csyshandle,vrep.simx_opmode_buffer);
            end
            
            tilt=starttilt;
            
            [res,dt]=vrep.simxGetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,vrep.simx_opmode_blocking);
            
            r=0.15;
            %%pause(1);
            while 1
                %%disp(i);i
                frewind(fid);
                speeds=fscanf(fid,'%f\t%f\t%f\n%d')
                
                if(speeds(4)==1)
                    [res]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
                    break;
                end
                
                V=mean(speeds(1:2));
                W=diff(speeds(1:2))/(2*r);
                position(1)=position(1)+cos(orientation(3))*V*dt;
                position(2)=position(2)+sin(orientation(3))*V*dt;
                orientation(3)=mod(orientation(3)+W*dt,2*pi);
                tilt(3)=pi/2+speeds(3)*2*pi/360;
                
                [res]=vrep.simxSetObjectPosition(clientID,csyshandle,-1,position,vrep.simx_opmode_oneshot);
                [res]=vrep.simxSetObjectOrientation(clientID,csyshandle,-1,orientation,vrep.simx_opmode_oneshot);
                [res]=vrep.simxSetObjectOrientation(clientID,robothandle,csyshandle,tilt,vrep.simx_opmode_oneshot);
                
                
                [res,bG,ptG,~,~]=vrep.simxReadProximitySensor(clientID,ultraG,vrep.simx_opmode_buffer);
                [res,bP,ptP,~,~]=vrep.simxReadProximitySensor(clientID,ultraP,vrep.simx_opmode_buffer);
                [res,bT,ptT,~,~]=vrep.simxReadProximitySensor(clientID,ultraT,vrep.simx_opmode_buffer);
                
                fidout=fopen(outputFile,'w');
                
                if(fidout~=-1)
                    fprintf(fidout,'pos+ori:\t%f\t%f\t%f\n',position(1),position(2),orientation(3));
                    fprintf(fidout,'prox:\t%f\t%f\t%f\n',norm(ptP),norm(ptG),norm(ptT));
                    fprintf(fidout,'valid:\t%d\t%d\t%d\n',bP,bG,bT);
                    fclose(fidout);
                end
                
                pause(dt);
            end
        else
            pause(1);
        end
    end
    %[res]=vrep.simxSetObjectPosition(clientID,handle,-1,position,vrep.simx_opmode_oneshot);
end
fclose('all');
