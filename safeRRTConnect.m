function [ path, pathCDF, Q1, Q2 ] = safeRRTConnect(robot, collisionArray, obsCollisionArray,...
    qS, qG, K, alpha, metric, dangerFlag, randSeed)

numNodes = 1000;
numJoints = size(qS, 1);

Q1 = Tree(robot, numNodes, numJoints, obsCollisionArray, qS);
Q2 = Tree(robot, numNodes, numJoints, obsCollisionArray, qG);

flag = 1;

rng(randSeed); % Set the random seed 
fprintf("Trial %i\n", randSeed);

for k = 1:K    
 
   qRand = getValidConfig(robot, collisionArray, obsCollisionArray);
       
   if flag
       [ Q1, Q2, status ] = doMainRRTConnectLoop(Q1, Q2, robot, collisionArray, ...
                                obsCollisionArray, qRand, alpha, metric, dangerFlag);
   else
       [ Q2, Q1, status ] = doMainRRTConnectLoop(Q2, Q1, robot, collisionArray, ... 
                                obsCollisionArray, qRand, alpha, metric, dangerFlag); 
   end
   fprintf("Finished loop %i\n", k);
   
   if status
       [ pathQ1, pathCDF1 ] = getTreePath(Q1);
       [ pathQ2, pathCDF2 ] = getTreePath(Q2);
       pathQ2 = flip(pathQ2(:,1:end-1), 2);
       path = [qS pathQ1 pathQ2 qG];
       pathCDF2 = flip(pathCDF2(1:end-1), 2);
       pathCDF = [pathCDF1 pathCDF2];
       return;
   end
   
   if flag
       flag = 0;
   else
       flag = 1;
   end
   
end

path = false; % Failure
pathCDF = [];

end

