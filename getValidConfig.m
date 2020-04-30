function [ q ] = getValidConfig(robot, collisionArray, obsCollisionArray)

% Generate a collision free config

isCollision = true;

while isCollision
    
    q = randomConfiguration(robot);
    [isCollision,~,~] = checkCollisions(robot, collisionArray, ...
        obsCollisionArray, q, false);

end

end

