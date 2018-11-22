map = loadmap('example_map.txt');

start_pos = [pi/3,-pi/4,-pi/3,0,0,0]; 
goal_pos = [0,pi/4,pi/4,pi/3,0,0]; 

lynxTraj = zeros(1,6,1000);

%intitally current position is the starting position
cur_pos = start_pos;

zeta = 0.01; 
eta = 10000; 
rho_knot = 500; 
epsilon = 10;
alph = 0.003; 

% attractive force - parabolic 
% Fatt(q) = -zeta * (distance to box) 

% repulsive force
% zero if greater than rho_knot away 
% Frep(q) = eta * (1/(rho*o(q) - 1/(rho_knot)) * (1/(rho^2 * o(q)) *
% gradient of rho(o(q))

%gradient of rho(o(q)) is (o(q) - b)/ norm(o(i) - b)

% xyz posiiton 
%this is a 6x3 of each joint position 
cur_pos_xyz = calculateFK_sol(cur_pos);
goal_pos_xyz = calculateFK_sol(goal_pos);

globalCounter = 1; 

%total force at each joint is Fatt + Frep for that joint 
while (norm(cur_pos_xyz - goal_pos_xyz) > epsilon)
    
    %based on the current q, compute the position for each of the joints
    %using forward kinematics 
    
    %The next position in joint space 
    new_pos = cur_pos;
    
    %this is a 6x3 giving you the xyz position of each joint
    new_pos_xyz = calculateFK_sol(new_pos); 
    
    %%Populate this with each jacobian
    jacobianVector = zeros(3,5,6);
    for k = 0:5
        jacobianVector(:,:,k+1) = numCalcJacobian(cur_pos,k);
    end
    
    %for each joint position in list of joints
    for i = 1:size(cur_pos_xyz,1)
        
        %calculate distance to goal
        goalDist = norm(cur_pos_xyz - goal_pos_xyz);
        
        %calculate attractive force from goal 
        Fatt = -zeta * (cur_pos_xyz(i,:)-goal_pos_xyz(i,:));
        
        %Add this to a total torque calc for the joint
        jointTorque = jacobianVector(:,:,i)' * Fatt';
        
        %calculate repulsive force for each obstacle
        for j = 1:size(map.obstacles,1)
          
          [obstacleDist,closestPoint] = distPointToBox(cur_pos_xyz(i,:),map.obstacles(j,:));
          
          if(obstacleDist < rho_knot)
              %figure out the closest point on the box
              gradRho = (cur_pos_xyz(i,:) - closestPoint)/obstacleDist; 
              Frep = eta * (1/obstacleDist - 1/rho_knot) * (1/(obstacleDist^2)) * gradRho;
              jointTorque = jointTorque + jacobianVector(:,:,i)' * Frep';
          end
          
        end
        
        %based on the total joint effort, take a step in joint space
        normJT = jointTorque/norm(jointTorque);
        normJT(isnan(normJT)) = 0;
        new_pos = new_pos + alph * [normJT', 0]; 
    end
    
    lynxTraj(:,:,globalCounter) = new_pos;
    globalCounter = globalCounter + 1;
    %now set the current position to be the new position 
    new_pos_xyz = calculateFK_sol(new_pos);
    cur_pos = new_pos;
    cur_pos_xyz = new_pos_xyz;
    
end
lynxTraj = lynxTraj(:,:,1:globalCounter-1);

%now plot the lynxTrajectory 
lynxStart
plotmap(map);

for i = 1:globalCounter-1
    lynxServoSim(lynxTraj(:,:,i));
    hold on;
    pos = calculateFK_sol(lynxTraj(:,:,i));
    pos = pos(6,:); 
    scatter3(pos(1),pos(2),pos(3));
    pause(0.1);
end

