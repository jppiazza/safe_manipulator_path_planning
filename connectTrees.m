function [ Q2, status ] = connectTrees(Q2, robot, collisionArray, obsCollisionArray, ...
                                    qNew, alpha, metric)

isCollision = false;
qOldBar = NaN(Q2.numJoints, 1);

while ~isCollision && ~isequal(qNew, qOldBar)
    
    % Note that for the connect procedure, we do not consider the CDF
    [ qOld, parentInd ] = Q2.nearestNode(robot, qNew, alpha, metric, 0);
    qOldBar = Q2.steer(qOld, qNew);
    isCollision = Q2.checkPathCollision(robot, qOld, qOldBar, collisionArray, obsCollisionArray);
        
    if ~isCollision
        Q2 = Q2.extend(robot, obsCollisionArray, qOldBar, parentInd);
    else
        status = false;
        return;
    end       
        
end
    
status = true; % qNew = qOldBar

end

