function PathCell = MRPP_CBSHWY(MapMat,Highway,StartStates,GoalStates,ctime)
robotNum=size(StartStates,1);
PathCell=cell(robotNum,1);

OPEN_COUNT=1;
MAX_OPEN_SIZE=1000;
MAX_TIME_STEP=300;
OPEN=cell(MAX_OPEN_SIZE,1);
OPEN_CHECK=zeros(MAX_OPEN_SIZE,1);

%% initialize
Root.ID=1;
Root.parent=0;
Root.constraints=[];
solution=cell(robotNum,1);
AllPath=cell(robotNum,1);
costVec=zeros(robotNum,1);
for i=1:robotNum
    tempMat=MapMat;
    tempMat(GoalStates(i,1),GoalStates(i,2))=0;
    path=AStarSTDiffHWY(tempMat,Highway,StartStates(i,:),GoalStates(i,:),[]);
    path(:,4)=path(:,4)+ctime;
    
    %make sure robots at goal positions will stay forever
    pathSize=size(path,1);
    newPath=zeros(MAX_TIME_STEP,4);
    newPath(1:pathSize,:)=path;
    newPath(pathSize+1:MAX_TIME_STEP,1:3)=repmat(path(end,1:3),[MAX_TIME_STEP-pathSize,1]);
    newPath(pathSize+1:MAX_TIME_STEP,4)=(path(pathSize,4)+1:MAX_TIME_STEP-1)';
    AllPath{i,1}=path;
    solution{i,1}=newPath;
    costVec=size(path,1)-1;
end
Root.AllPath=AllPath;
Root.solution = solution;
Root.costVec=costVec;
Root.cost=sum(costVec);

OPEN{1,1}=Root;
OPEN_CHECK(1,1)=0;

while ~isempty(find(OPEN_CHECK==0, 1))
    bestCost=0;
    bestNodeID=0;
    for i=1:OPEN_COUNT
        if  OPEN_CHECK(i,1)==0
            node=OPEN{i,1};
            if node.cost>bestCost
                bestNodeID=node.ID;
                bestCost=node.cost;
            end
        end
    end
    P=OPEN{bestNodeID,1};
    
    %% conflict detection
    solution=P.solution;
    conflictFoundFlag=0;
    constraints=zeros(2,4);
    for i=1:robotNum
        iPath=solution{i,1};
        iPath(:,3)=[];
        for j=1:robotNum
            if j<=i
                continue;
            end
            
            jPath=solution{j,1};
            jPath(:,3)=[];
            iLen=size(iPath,1);
            jLen=size(jPath,1);
            if iLen>1 && jLen>1
                iMat=zeros(iLen-1,6);
                iMat(:,1:3)=iPath(1:iLen-1,:);
                iMat(:,4:6)=iPath(2:iLen,:);
                jMat=zeros(jLen-1,6);
                jMat(:,4:6)=jPath(1:jLen-1,:);
                jMat(:,1:3)=jPath(2:jLen,:);
            end
            temp=jMat(:,6);
            jMat(:,6)=jMat(:,3);
            jMat(:,3)=temp;
            
            %vertex and edge conflict detection
            [vertexKind,~]=ismember(iPath,jPath,'rows');
            [edgeKind,~]=ismember(iMat,jMat,'rows');
            vertex=find(vertexKind==1, 1);
            edge=find(edgeKind==1, 1);
            if ~isempty(vertex) && ~isempty(edge) %both conflicts exist
                vertexConflict=iPath(vertex(1),:);
                edgeConflict=iMat(edge(1),:);
                if vertexConflict(1,3)<=edgeConflict(1,6)
                    constraints(1,:)=[i vertexConflict];
                    constraints(2,:)=[j vertexConflict];
                else
                    constraints(1,:)=[i edgeConflict(1,4:6)];
                    constraints(2,:)=[j edgeConflict(1,1:2) edgeConflict(1,6)];
                end
                conflictFoundFlag=1;
                break;
                
            elseif ~isempty(vertex) && isempty(edge) %only vertex conflict
                conflictFoundFlag=1;
                vertexConflict=iPath(vertex(1),:);
                constraints(1,:)=[i vertexConflict];
                constraints(2,:)=[j vertexConflict];
                break;
            elseif  isempty(vertex) && ~isempty(edge) %only edge conflict
                conflictFoundFlag=1;
                edgeConflict=iMat(edge(1),:);
                constraints(1,:)=[i edgeConflict(1,4:6)];
                constraints(2,:)=[j edgeConflict(1,1:2) edgeConflict(1,6)];
                break;
            else
                continue;
            end
        end
        if conflictFoundFlag == 1
            break;
        end
    end
    
    if conflictFoundFlag == 0
        PathCell=P.AllPath;
        break;
    end
    
    %% branching and expanding the tree
    OPEN_CHECK(bestNodeID,1)=1;
    for i=1:2
        agentID=constraints(i,1);
        A.ID=OPEN_COUNT+1;
        A.parent=P.ID;
        A.constraints=[P.constraints;constraints(i,:)];
        pSolution=P.solution;
        startRCA=StartStates(agentID,:);
        goalRCA=GoalStates(agentID,:);
        AConstraints=A.constraints;
        AConstraints=AConstraints(AConstraints(:,1)==agentID,2:end);
        tempMat=MapMat;
        tempMat(goalRCA(1,1),goalRCA(1,2))=0;
        path=AStarSTDiffHWY(tempMat,Highway,startRCA,goalRCA,AConstraints);
        %make sure robots at goal positions will stay forever
        pathSize=size(path,1);
        newPath=zeros(MAX_TIME_STEP,4);
        newPath(1:pathSize,:)=path;
        newPath(pathSize+1:MAX_TIME_STEP,1:3)=repmat(path(end,1:3),[MAX_TIME_STEP-pathSize,1]);
        newPath(pathSize+1:MAX_TIME_STEP,4)=(path(pathSize,4)+1:MAX_TIME_STEP-1)';
        
        pSolution{agentID,1}=newPath;
        A.solution=pSolution;
        pSolution{agentID,1}=path;
        A.AllPath=pSolution;
        costVec=P.cost;
        costVec(agentID,1)=size(path,1)-1;
        A.costVec=costVec;
        A.cost=sum(A.costVec);
        
        OPEN_COUNT=OPEN_COUNT+1;
        OPEN{OPEN_COUNT,1}=A;
        OPEN_CHECK(OPEN_COUNT,1)=0;
    end
    
end % while loop

end