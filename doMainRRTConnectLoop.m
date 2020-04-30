function [ Q1, Q2, status ] = doMainRRTConnectLoop(Q1, Q2, robot, collisionArray, ...
        obsCollisionArray, qRand, alpha, metric, dangerFlag)

% Get the nearest node
[ qOld, parentInd ] = Q1.nearestNode(robot, qRand, alpha, metric, dangerFlag);
qNew = Q1.steer(qOld, qRand);

% Check if the new node is collision free
isCollision = Q1.checkPathCollision(robot, qOld, qNew, collisionArray, obsCollisionArray);
    
if ~isCollision
    % Add the new node to the tree
    Q1 = Q1.extend(robot, obsCollisionArray, qNew, parentInd);
    [ Q2, status ] = connectTrees(Q2, robot, collisionArray, obsCollisionArray, qNew,...
                        alpha, metric);
else    
    status = false;
    return;
end
        
end

