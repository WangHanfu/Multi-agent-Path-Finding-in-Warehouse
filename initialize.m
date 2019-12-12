function  [RobotStates,PodStates,DepotStates,StorageOccupancy]=initialize(xlength,ylength,robotNum,podNum,depotNum)
%x,y,angle
xlength=61;
ylength=29;
RobotStates = zeros(robotNum,3);
%x,y
PodStates = zeros(podNum,3);
%x,y
DepotStates = zeros(depotNum,3);
%storage locations
StorageOccupancy = zeros(ylength,xlength);

xy2rc=@(x,y)[ylength+1-y;x];
rc2xy=@(r,c)[c;ylength+1-r];

for i=1:20
    StorageOccupancy(2:6,2+3*(i-1):3+3*(i-1))=1;
    StorageOccupancy(8:12,2+3*(i-1):3+3*(i-1))=1;
    StorageOccupancy(14:18,2+3*(i-1):3+3*(i-1))=1;
    StorageOccupancy(20:24,2+3*(i-1):3+3*(i-1))=1;
end

[R,C]=find(StorageOccupancy==1);

for i=1:podNum
    PodStates(i,1:2)=rc2xy(R(i,1),C(i,1));
    PodStates(i,3)=0;
end

[R,C]=find(StorageOccupancy==0);
sb=size(R,1);
sb=randperm(sb);
for i=1:robotNum
    index=sb(i);
    RobotStates(i,1:2)=rc2xy(R(index,1),C(index,1));
    RobotStates(i,3)=randi([1 4]);
end

DepotStates(:,1) = [4 11 18 25 32 39 46 53]';
DepotStates(:,2) = ones(depotNum,1);
DepotStates(:,3) = zeros(depotNum,1);
end

