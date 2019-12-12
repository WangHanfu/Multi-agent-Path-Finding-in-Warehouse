function OptimalPath=AStarSTDiffHWY(MapMat,Highway,startState,goalState,constraints)
%Algorithm: space-time A* for nonholonomic robots
%Author:  Hanfucius
%Input:   Occupancy Map matrix (zero position:upper left),
% start and goal row-colum-direction
[height,width] = size(MapMat);
actions = 4;       %actions: 1=forward,2=turn left,3=turn right,4=stay
%pre-assign memory
OPEN_COUNT=1;
MAX_OPEN_SIZE=200;
OPEN=zeros(MAX_OPEN_SIZE,13);
OPEN_CHECK=zeros(MAX_OPEN_SIZE,1);

CLOSED_COUNT=1;
MAX_CLOSED_SIZE=200;
CLOSED=zeros(MAX_CLOSED_SIZE,13);

%Initialize start node with FValue and open first node.
dist=abs(startState(1,1)-goalState(1,1))+abs(startState(1,2)-goalState(1,2));
%Node definition:R,C,A,T,parentR,parentC,parentA,parentT,F,G,H values,energy,actions
Root(1,1:13)=[startState 0 0 0 0 0 dist 0 dist 0 0];
OPEN(1,:)=Root;
OPEN_CHECK(1,1)=1;
while ~isempty(find(OPEN_CHECK==1, 1))
    %best-first search and tie-breaking policy
    uncheckedIndex=OPEN_CHECK==1;
    TEMPOPEN=OPEN(uncheckedIndex,:);
    minFScore=min(TEMPOPEN(:,9));
    index = find(TEMPOPEN(:,9)==minFScore);
    energy=TEMPOPEN(index,12);%tie-breaking
    [~,index2] = min(energy,[],1);
    currentNodeIndex=index(index2);
    currentNode=TEMPOPEN(currentNodeIndex,:);
    if all(currentNode(1,1:3)==goalState)
        break;
    end
    
    %Removing node from OpenList to ClosedList
    [~,temp]=ismember(currentNode,OPEN,'rows');
    OPEN_CHECK(temp,1) = 0;
    CLOSED(CLOSED_COUNT,:) = currentNode;
    CLOSED_COUNT=CLOSED_COUNT+1;
    currentRCA=currentNode(1,1:3);
    %find out highways
    mat1=Highway{1,1};
    hdirection=mat1(currentRCA(1,1),currentRCA(1,2));
    mat2=Highway{2,1};
    vdirection=mat2(currentRCA(1,1),currentRCA(1,2));
    if hdirection~=0
        if vdirection==0
            feasible=[currentRCA(1,1) currentRCA(1,2) hdirection];
        else
            feasible=[currentRCA(1,1) currentRCA(1,2) hdirection;currentRCA(1,1) currentRCA(1,2) vdirection];
        end
    else
        if vdirection~=0
            feasible=[currentRCA(1,1) currentRCA(1,2) vdirection];
        else
            r=currentRCA(1,1);
            c=currentRCA(1,2);
            feasible=[r c 1;r c 2;r c 3;r c 4];
        end
    end
    
    if currentRCA(1,1)==goalState(1,1) || currentRCA(1,2)==goalState(1,2)
        r=currentRCA(1,1);
        c=currentRCA(1,2);
        feasible=[r c 1;r c 2;r c 3;r c 4];
    end
    
    for i=1:actions
        newNode=zeros(1,13);
        %% state machine for the next state
        newState=currentRCA;
        switch i
            case 1
                switch currentRCA(1,3)
                    case 1
                        newState(1,2)=newState(1,2)+1;
                    case 2
                        newState(1,1)=newState(1,1)-1;
                    case 3
                        newState(1,2)=newState(1,2)-1;
                    case 4
                        newState(1,1)=newState(1,1)+1;
                end
            case 2  %turn left
                
                switch currentRCA(1,3)
                    case 1
                        newState(1,3)=2;
                    case 2
                        newState(1,3)=3;
                    case 3
                        newState(1,3)=4;
                    case 4
                        newState(1,3)=1;
                end
                
                if size(feasible,1)==1
                    if abs(feasible(1,3)-newState(1,3))==2 || feasible(1,3)==currentRCA(1,3)
                        continue;
                    end
                elseif size(feasible,1)==2
                    if ~ismember(newState(1,1:3),feasible,'rows')
                        continue;
                    end
                end
                
            case 3
                switch currentRCA(1,3)
                    case 1
                        newState(1,3)=4;
                    case 2
                        newState(1,3)=1;
                    case 3
                        newState(1,3)=2;
                    case 4
                        newState(1,3)=3;
                end
                
                if size(feasible,1)==1
                    if abs(feasible(1,3)-newState(1,3))==2 || feasible(1,3)==currentRCA(1,3)
                        continue;
                    end
                elseif size(feasible,1)==2
                    if ~ismember(newState(1,1:3),feasible,'rows')
                        continue;
                    end
                end
                
            case 4
        end
        
        newNode(1,1:3)=newState;
        newNode(1,4)=currentNode(1,4)+1;
        %Ignore the out of borders and obstacles.
        if newNode(1,1)<1||newNode(1,1)>height||newNode(1,2)<1||newNode(1,2)>width||MapMat(newNode(1,1),newNode(1,2))==1
            continue;
        end
        
        %Ignore the nodes in CLOSED lists.
        if ismember(newNode(1,1:4),CLOSED(:,1:4),'rows')
            continue;
        end
        
        %Ignore the nodes in constraints lists.
        if ~isempty(constraints)
            temp=newNode(1,1:3);
            temp(1,3)=newNode(1,4);
            if ismember(temp,constraints,'rows')
                continue;
            end
        end
        
        %Discover a new node and update its tentative F,G,H values
        newNode(1,5:8)=currentNode(1,1:4);
        dist=abs(newNode(1,1)-goalState(1,1))+abs(newNode(1,2)-goalState(1,2));
        newNode(1,9:11)=[dist+currentNode(1,10)+1 currentNode(1,10)+1 dist];
        if  i==4
            newNode(1,12)=currentNode(1,12);
        else
            newNode(1,12)=currentNode(1,12)+1;
        end
        [flag,index]=ismember(newNode(1,1:4),OPEN(:,1:4),'rows');
        if  flag==1     % if this node is already in the OPEN list
            if newNode(1,9)>OPEN(index,9)
                continue;
            else
                OPEN(index,:)=newNode;
            end
        else   % if this node is not in the OPEN list
            OPEN_COUNT=OPEN_COUNT+1;
            OPEN_CHECK(OPEN_COUNT,1)=1;
            OPEN(OPEN_COUNT,:)=newNode;
        end
    end
end

%backward reconstruct path
k=1;
OptimalPath=zeros(200,4);
while ~all(Root==currentNode)
    OptimalPath(k,:)=currentNode(1,1:4);
    [~,parentIndex]=ismember(currentNode(1,5:8),CLOSED(:,1:4),'rows');
    currentNode = CLOSED(parentIndex,:);
    k=k+1;
end
OptimalPath=OptimalPath(all(OptimalPath,2),:);
OptimalPath=[OptimalPath;Root(1,1:4)];
OptimalPath=flipud(OptimalPath);
end