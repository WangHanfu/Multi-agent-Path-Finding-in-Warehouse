%% Initialize
clear;
clc;

xlength=61;
ylength=29;
robotNum=10;
podNum=800;
depotNum = 8;

xy2rc=@(x,y)[ylength+1-y;x];
rc2xy=@(r,c)[c;ylength+1-r];

sz=get(0,'screensize');
sz(1,2) = 80;
sz(1,4) = 950;
h=figure('outerposition',sz);
assignin('base','h',h); %in case of any callback errors.
hold on;
grid on;
set(gca,'xtick',0:1:xlength);
set(gca,'ytick',0:1:ylength);
axis equal;
axis([0 xlength+1 0 ylength+1]);
axis manual;

% stores current states
globalTime = 1;
GlobalRobotXYA = zeros(robotNum,3);
GlobalPodXY = zeros(podNum,2);

MapOccupancy = zeros(ylength,xlength);
RobotOccupancy = zeros(ylength,xlength);
PodOccupancy = zeros(ylength,xlength);
StorageOccupancy = zeros(ylength,xlength);

[RobotXYA,PodXY,DepotXY,StorageOccupancy]=initialize(xlength,ylength,robotNum,podNum,depotNum);
PodOccupancy=StorageOccupancy;

plotAll(RobotXYA,PodXY,DepotXY);

MapOccupancy = MapOccupancy+PodOccupancy;
Highway=cell(2,1);
% horizontal directions
mat=zeros(ylength,xlength);
mat(1,:)=ones(1,xlength);
mat(7,:)=3*ones(1,xlength);
mat(13,:)=ones(1,xlength);
mat(19,:)=3*ones(1,xlength);
mat(25,:)=ones(1,xlength);
mat(26,:)=3*ones(1,xlength);
mat(27,:)=ones(1,xlength);
mat(28,:)=3*ones(1,xlength);
mat(29,:)=ones(1,xlength);
Highway{1,1}=mat;
% vertical directions
mat=zeros(ylength,xlength);
mat(:,1)=2*ones(ylength,1);
mat(:,4)=4*ones(ylength,1);
mat(:,7)=2*ones(ylength,1);
mat(:,10)=4*ones(ylength,1);
mat(:,13)=2*ones(ylength,1);
mat(:,16)=4*ones(ylength,1);
mat(:,19)=2*ones(ylength,1);
mat(:,22)=4*ones(ylength,1);
mat(:,25)=2*ones(ylength,1);
mat(:,28)=4*ones(ylength,1);
mat(:,31)=2*ones(ylength,1);
mat(:,34)=4*ones(ylength,1);
mat(:,37)=2*ones(ylength,1);
mat(:,40)=4*ones(ylength,1);
mat(:,43)=2*ones(ylength,1);
mat(:,46)=4*ones(ylength,1);
mat(:,49)=2*ones(ylength,1);
mat(:,52)=4*ones(ylength,1);
mat(:,55)=2*ones(ylength,1);
mat(:,58)=4*ones(ylength,1);
mat(:,61)=2*ones(ylength,1);
Highway{2,1}=mat;

%% path planning problem
StartXYA = RobotXYA;
GoalXYA = zeros(robotNum,3);
StartRCA = zeros(robotNum,3);
GoalRCA = zeros(robotNum,3);

temp= randperm(podNum);
for i=1:robotNum
    StartRCA(i,1:2) = xy2rc(StartXYA(i,1),StartXYA(i,2));
    StartRCA(i,3) = StartXYA(i,3);
    GoalXYA(i,1:2) = PodXY(temp(i),:);
    GoalRCA(i,1:2) = xy2rc(GoalXYA(i,1),GoalXYA(i,2));
    GoalRCA(i,3) = randi([1 4]);    
end

% StartRCA=[5 1 1];
% GoalRCA=[2 1 1];
tic;
AllPathCell=MRPP_CBSHWY(MapOccupancy,Highway,StartRCA,GoalRCA,0);
toc;
save('allPath_CBS_100.mat','AllPathCell','StartRCA','GoalRCA');
% load('allPath80.mat');
futureSize=3;
HeatMap=cell(futureSize,1);
for i=1:futureSize
    HeatMap{i,1}=zeros(robotNum,3);
end
%% plot results
video = VideoWriter('simulation');
video.FrameRate=2;
open(video);
for loop=1:100
    frame = getframe;
    writeVideo(video,frame);
    pause(0.5);
    cla;
    
    for i=1:robotNum
        if  ~isempty(AllPathCell{i,1})
            path = AllPathCell{i,1};
            if globalTime<=size(path,1)
                state = path(globalTime,:);
                GlobalRobotXYA(i,1:2)=rc2xy(state(1,1),state(1,2));
                GlobalRobotXYA(i,3)=state(1,3);
            end
            
            for j=1:futureSize
                if globalTime+j<=size(path,1)
                    state = path(globalTime+j,:);
                    temp = HeatMap{j,1};
                    temp(i,1:2)=rc2xy(state(1,1),state(1,2));
                    temp(i,3)=state(1,3);
                    HeatMap{j,1} = temp;
                else
                    state = path(end,:);
                    temp = HeatMap{j,1};
                    temp(i,1:2)=rc2xy(state(1,1),state(1,2));
                    temp(i,3)=state(1,3);
                    HeatMap{j,1} = temp;
                end
            end                       
        end
    end
    GlobalPodXY=PodXY;
    plotAll(GlobalRobotXYA,GlobalPodXY,DepotXY);
    globalTime = globalTime + 1;
    loop=loop+1;
end

close(video);