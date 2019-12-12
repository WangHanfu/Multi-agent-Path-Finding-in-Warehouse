function plotAll(RobotStates,PodStates,DepotStates)
xlength=61;
ylength=29;
rectangle('Position', [0,0,xlength+1,ylength+1],'lineWidth',5);
plot(DepotStates(:,1),DepotStates(:,2),'square','MarkerEdgeColor',[0.5 0.5 0.5],'MarkerFaceColor',[0.7 0.7 0.7],'MarkerSize',30);
plot(PodStates(:,1),PodStates(:,2),'square','MarkerEdgeColor','k','MarkerFaceColor',[1 1 1],'MarkerSize',20);
plot(RobotStates(:,1),RobotStates(:,2),'o','MarkerEdgeColor','k','MarkerFaceColor',[1 0 0],'MarkerSize',15);
%draw robot direction
for i=1:size(RobotStates,1)
    temp = 0.3;
    x=RobotStates(i,1);
    y=RobotStates(i,2);
    a=RobotStates(i,3);
    switch a
        case 1
            xx = x+temp;
            yy = y;
        case 2
            xx = x;
            yy = y+temp;
        case 3
            xx = x-temp;
            yy = y;
        case 4
            xx = x;
            yy = y-temp;            
    end
    line([x,xx],[y,yy],'color','k','linestyle','-','lineWidth',4);
end
end

