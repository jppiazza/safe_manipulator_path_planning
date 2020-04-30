classdef Tree
    
    properties
        nodes
        parents
        children
        nodePos
        deltaAngMax
        numJoints
        cdf
        steerSteps
    end
    
    methods
        function obj = Tree(robot, numNodes, numJoints, obsCollisionArray, q)
            obj.nodes = zeros(numJoints, numNodes);
            obj.parents = zeros(1, numNodes);
            obj.children = zeros(1, numNodes);
            obj.nodePos = 1;
            obj.nodes(:,obj.nodePos) = q;
            obj.parents(obj.nodePos) = NaN;    
            obj.deltaAngMax = 10*(pi/180); % 10 degrees
            obj.numJoints = numJoints;
            obj.cdf = NaN(1, numNodes);
            obj = obj.cdfNode(robot, obsCollisionArray, q);
            obj.steerSteps = 4;
        end
        
        % Find the nearest node according to the metric (and danger field)
        function [ qOld, minInd ] = nearestNode(obj, robot, q, alpha, metric, dangerFlag)
            rho = zeros(1, obj.nodePos);
            
            for k = 1:obj.nodePos
                rho(k) = obj.calcRawMetric(robot, obj.nodes(:,k), q, metric);
            end
            
            cdfTemp = obj.cdf(~isnan(obj.cdf));
            
            if dangerFlag
                [~, minInd] = min(rho + alpha*cdfTemp);
            else
                [~, minInd] = min(rho);
            end
            qOld = obj.nodes(:, minInd);
        end
        
        % Calculate the appropriate standard metric
        function [ rho ] = calcRawMetric(obj, robot, qTreeNode, q, metric)
            switch metric
                case 'angDist'
                    weights = [1;1;1;1;0.01;0.01;0.01];
                    if obj.numJoints ~= size(weights,1)
                        fprintf('Amend the weights matrix to match the number of joints!\n');
                        return;
                    end
                    rho = sqrt(weights'*((qTreeNode - q).^2));
                
                case 'eeDist'
                    rho = norm(tform2trvec(getTransform(robot, qTreeNode, 'EndEffector_Link')) - ...
                    tform2trvec(getTransform(robot, q, 'EndEffector_Link')));
            end        
        end    
        
        % Calculate the CDF for configuration q
        function obj = cdfNode(obj, robot, obsCollisionArray, q)
            
            obs_cdf = zeros(1, size(obsCollisionArray, 2));
            
            for i = 1:size(obsCollisionArray, 2)
                d = 0;
                for j = 1:obj.numJoints
                    % Do the danger field calculation 
                    if j == 1
                        linkS = [0;0;0];
                    else
                        linkS = tform2trvec(getTransform(robot, q, char(robot.BodyNames(j))))';
                    end
                    linkE = tform2trvec(getTransform(robot, q, char(robot.BodyNames(j+1))))';
                    r_obs = obj.obsClosestPoint(obsCollisionArray{1,i}, linkS, linkE);
                    d = d + obj.cdfLink(r_obs, linkS, linkE);
                 end
                 obs_cdf(i) = d;
            end
            
            obj.cdf(obj.nodePos) = max(obs_cdf);
        end
        
        % Calculate the CDF for link i
        function [ d ] = cdfLink(obj, r_obs, linkS, linkE)        
            k1 = 1; % Can change if necessary
            
            alp1 = r_obs(1) - linkS(1);
            bet1 = r_obs(2) - linkS(2);
            gam1 = r_obs(3) - linkS(3);
            
            alp2 = linkE(1) - linkS(1);
            bet2 = linkE(2) - linkS(2);
            gam2 = linkE(3) - linkS(3);
            
            a = alp2^2 + bet2^2 + gam2^2;
            b = 2*(alp1*alp2 + bet1*bet2 + gam1*gam2);
            c = alp1^2 + bet1^2 + gam1^2;
            
            % This is the integral 1/(sqrt(ax^2 + bx + c) between 0 and 1
            d = k1*(1/sqrt(a))*(log(((2*a+b)/sqrt(a)) + 2*sqrt(a+b+c)) - ...
                log(b/sqrt(a) + 2*sqrt(c)));
        end
        
        % Get the closest point on the obstacle from link i
        function [ r_obs ] = obsClosestPoint(obj, obstacle, linkS, linkE)
            linkDir = linkE - linkS;
            obsCenter = obstacle.Pose(1:3,4);
            minPoint = linkS;
            minDist = norm(linkS - obsCenter);
            
            for k = 0:0.1:1
                point = linkS + linkDir*k;
                dist = norm(point - obsCenter);
                    if dist < minDist
                        minDist = dist;
                        minPoint = point;
                    end
            end
            dir = minPoint - obsCenter;
            r_obs = obsCenter + dir*obstacle.Radius;      
        end
        
        % Steer in the direction of qRand according to the max angle change
        function [ qNew ] = steer(obj, qOld, qRand)
            qDiff = qRand - qOld;
            qDiffInds = abs(qDiff) > obj.deltaAngMax;
         
            qNew = zeros(obj.numJoints, 1);
            qNew(qDiffInds) = qOld(qDiffInds) + sign(qDiff(qDiffInds)) * obj.deltaAngMax;
            qNew(~qDiffInds) = qRand(~qDiffInds);
        end
        
        % Grow the tree
        function obj = extend(obj, robot, obsCollisionArray, q, parentInd)
            obj.nodePos = obj.nodePos + 1;
            obj.nodes(:,obj.nodePos) = q;
            obj = obj.cdfNode(robot, obsCollisionArray, q);
            obj.parents(obj.nodePos) = parentInd;
            obj.children(parentInd) = obj.children(parentInd) + 1;
        end
        
        % Check if the path from q to qNew has a collision
        function isCollision = checkPathCollision(obj, robot, q, qNew, collisionArray,...
                obsCollisionArray)
           
            intStates = zeros(obj.numJoints, obj.steerSteps);           
            
            for i = 1:obj.numJoints
                intStates(i, :) = linspace(q(i), qNew(i), obj.steerSteps);
            end
            
            for j = 1:obj.steerSteps
                [isCollision, ~, ~] = checkCollisions(robot, collisionArray, ...
                        obsCollisionArray, intStates(:,j), false);
                if isCollision
                    return;                    
                end
            end
            
            isCollision = false;
            
        end    
        
    end
end

