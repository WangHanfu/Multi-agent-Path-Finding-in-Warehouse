classdef Robot<handle    
    properties
        ID        
        XYA %[x,y,theta]
        ActionSequence %actions: 0=wait,1=forward,2=backward,3=turn left,4=turn right
        StateSequence
        TaskID
        PodID
        Path
    end
end

