clc;
clear;
xlength=3;
ylength=3;
MapMat=zeros(ylength,xlength);
MapMat(2,2)=1;
%allPath=CBS(MapMat,[1 1 1;5 1 1],[5 5 1;1 5 1]);
temp=zeros(ylength,xlength);
temp(1,:)=ones(1,3);
temp(2,:)=3*ones(1,3);
temp(3,:)=ones(1,3);
Highway{1,1}=temp;
temp=zeros(ylength,xlength);
temp(:,1)=2*ones(3,1);
temp(:,2)=4*ones(3,1);
temp(:,3)=4*ones(3,1);
Highway{2,1}=temp;
%% Astar test
path=AStarSTDiffHWY(MapMat,Highway,[1 1 1],[2 3 1],[])