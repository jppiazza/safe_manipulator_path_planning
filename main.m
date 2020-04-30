clear ALL; close ALL; clc;

% Change as desired
robot = loadrobot('kinovaGen3','DataFormat','column'); 

% Create the environment
ob1 = collisionSphere(0.25);
ob1.Pose = trvec2tform([0.5 0 0.3]);
ob2 = collisionSphere(0.25);
ob2.Pose = trvec2tform([-0.5 0 0.3]);
%ob3 = collisionSphere(0.5);
%ob3.Pose = trvec2tform([-0.75 0.75 0]);
%ob4 = collisionSphere(0.5);
%ob4.Pose = trvec2tform([0.75 -0.75 0]);
% ob5 = collisionSphere(0.1);
% ob5.Pose = trvec2tform([-0.3 0.3 0.3]);
% ob6 = collisionSphere(0.2);
% ob6.Pose = trvec2tform([-0.25, 0, 0.5]);

obsCollisionArray = {ob1, ob2};
collisionArray = collisionsFromVisuals(robot);

% Uncomment below to get the start/goal configurations using inverse
% kinematics based on end-effector positions
% ik = inverseKinematics('RigidBodyTree', robot);
% weights = [0.25 0.25 0.25 1 1 1];
% tform = [eye(3); zeros(1,3)];
% tformS = [tform [0; 0.8; 0; 1]];
% tformG = [tform [0.4; -0.6; 0; 1]];
% [qS, ~] = ik('EndEffector_Link', tformS, weights, robot.homeConfiguration);
% [qG, ~] = ik('EndEffector_Link', tformG, weights, robot.homeConfiguration);

qS = [1.61; -2.09; 0; 0; 0; 1.99; -1.61];
qG = [-2.12; -2.13; 0.63; 0.03; -0.61; 2.05; 2.13];
numJoints = size(qS, 1);

%qS = getValidConfig(robot, collisionArray, obsCollisionArray);
%qG = getValidConfig(robot, collisionArray, obsCollisionArray);

K = 250; % Max iterations, can change
alpha = 5; % Tuning parameter, can change
metric = 'angDist'; % Change to 'eeDist' as desired
dangerFlag = 1; % Change to 0 for the regular algorithm
drawPath = false;
plotCDF = false;
numTrials = 50;
environment = 2;

runTime = zeros(numTrials, 1);
treeNodes = NaN(numTrials, 1);
pathNodes = NaN(numTrials, 1);
cdfTrials = NaN(numTrials, 1000);
pathTrials = NaN(numJoints*numTrials, 1000);

filename = strcat('results_dangerFlag_', string(dangerFlag), '_alpha_', ...
    string(alpha), '_env_', string(environment), '_numTrials_', ...
    string(numTrials));

for i = 1:numTrials
    
    tic;
    % Safe RRT-Connect planner
    [path, pathCDF, tree1, tree2] = safeRRTConnect(robot, collisionArray, obsCollisionArray, qS, qG, ...
        K, alpha, metric, dangerFlag, i); 

    elapsedTime = toc;
    
    if ~path
         fprintf('No path found.\n');
         runTime(i) = elapsedTime;
        %return;
    else
        runTime(i) = elapsedTime;
        treeNodes(i) = tree1.nodePos + tree2.nodePos;
        pathNodes(i) = size(path, 2);
        cdfTrials(i, 1:size(pathCDF,2)) = pathCDF;
        pathTrials(((i-1)*numJoints+1):i*numJoints, 1:size(path, 2)) = path;
    end 

end

save(strcat('results/',filename));
    
if drawPath

    % Plot the trajectory
    ax = visualizeCollisions(obsCollisionArray);
    
    tS = getTransform(robot, qS, 'EndEffector_Link');
    tG = getTransform(robot, qG, 'EndEffector_Link');
    transPoses = NaN(4, size(path,2));

    plot3(tS(1,4), tS(2,4), tS(3,4), 'g.', 'MarkerSize', 35); % Plot start marker

    for i = 1:length(path)
        pose = getTransform(robot, path(:,i), "EndEffector_Link");
        transPoses(:,i) = pose(:,4);
        show(robot, path(:,i), 'PreservePlot', false, 'Frames', 'off', 'Parent', ax);
        plot3(transPoses(1,:), transPoses(2,:), transPoses(3,:), '-b', 'MarkerSize', 20);   
        plot3(transPoses(1,:), transPoses(2,:), transPoses(3,:), 'b.', 'MarkerSize', 20);
        drawnow;
    end

    plot3(tG(1,4), tG(2,4), tG(3,4), 'r.', 'MarkerSize', 35); % Plot goal marker

    show(robot, qS, 'Parent', ax, 'Frames', 'off');
    show(robot, qG, 'Parent', ax, 'Frames', 'off');

end
    
if plotCDF
    figure;
    numPathNodes = size(path, 2);
    s = linspace(0, 1, numPathNodes);
    plot(s, pathCDF);
    xlabel('s'); ylabel('CDF(s)'); title('CDF Evolution of the Path');
end
