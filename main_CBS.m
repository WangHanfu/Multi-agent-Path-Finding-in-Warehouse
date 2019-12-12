%% Initialize
clear;
clc;

xlength=61;
ylength=29;
robotNum=10;
podNum=800;
depotNum = 8;
taskNum = robotNum;

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
AllRobotState = zeros(robotNum,3);
AllPodState = zeros(podNum,3);

MapOccupancy = zeros(ylength,xlength);
RobotOccupancy = zeros(ylength,xlength);

[RobotStates,PodStates,DepotStates,StorageOccupancy]=initialize(xlength,ylength,robotNum,podNum,depotNum);
PodOccupancy = StorageOccupancy;

MapOccupancy = MapOccupancy+PodOccupancy;

%generate random tasks
TaskCell=cell(taskNum,1);
for i=1:taskNum
    task=Task;
    task.PodID=randi([1 podNum]);
    task.StationID=randi([1 depotNum]);
    task.ProcessTime=randi([5 20]);
    task.ReturnState = zeros(1,3);
    TaskCell{i,1}=task;
end

save('taskSet.mat','TaskCell');
load('taskSet.mat');

%% Task Allocation


%% path planning problem
StartXYA = RobotStates;
GoalXYA = zeros(robotNum,3);
StartRCA = zeros(robotNum,3);
GoalRCA = zeros(robotNum,3);

temp= randperm(podNum);
for i=1:robotNum
    StartRCA(i,1:2) = xy2rc(StartXYA(i,1),StartXYA(i,2));
    StartRCA(i,3) = StartXYA(i,3);
    GoalXYA(i,1:2) = PodStates(temp(i),1:2);
    GoalRCA(i,1:2) = xy2rc(GoalXYA(i,1),GoalXYA(i,2));
    GoalRCA(i,3) = randi([1 4]);    
end
tic;
AllPathCell=MRPP_CBS(MapOccupancy,StartRCA,GoalRCA,0);
toc;
save('allPath.mat','AllPathCell','StartRCA','GoalRCA');

load('allPath.mat');
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
                AllRobotState(i,1:2)=rc2xy(state(1,1),state(1,2));
                AllRobotState(i,3)=state(1,3);
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
    AllPodState=PodStates;
    plotAll(AllRobotState,AllPodState,DepotStates);
    globalTime = globalTime + 1;
    loop=loop+1;
end

close(video);