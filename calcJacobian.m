function J = calcJacobian(q)

% Lynx ADL5 constants in mm
d1 = 76.2; % base height (table to center of joint 2)
a2 = 146.05; % shoulder to elbow length
a3 = 187.325; %elbow to wrist length
d5 = 76.2; %wrist to base of gripper
lg = 28.575; %length of gripper

% relevant info from position FK
[T0] = calculateFK_transforms(q);

%% ANGULAR VELOCITY JACOBIAN
% Approach: grab z vectors
Jw = zeros(3,5);
for i = 1:5
    Jw(:,i) = T0(1:3,3,i);
end


%% LINEAR VELOCITY JACOBIAN
% Approach: cross products
Jv = zeros(3,5);
for i = 1:5
    Jv(:,i) = cross(T0(1:3,3,i), T0(1:3,4,end)-T0(1:3,4,i));
end

%% Compose Jacobian Matrix and calc end effector velocities
J = [Jv; Jw]; 

%% SUPPORTING FUNCTIONS
    function [T0] = calculateFK_transforms(q)
        % Input: q - 1 x 5 vector of joint inputs [q1,q2,q3,q4,q5]
        
        % Outputs:  T0 = 6 transformation matrices T_i^0 for transforming from frame
        %                 i-1 to frame 0
        %           Tint = 5 transformation matrices T_i^i-1 for
        %                 transforming between intermediate frames
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % CODE TAKEN FROM calculateFK_sol %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Frame 1 w.r.t Frame 0
        T1 = [cos(q(1))  0  -sin(q(1))  0;
            sin(q(1))  0   cos(q(1))  0;
            0    -1       0      d1;
            0     0       0      1];
        
        %Frame 2 w.r.t Frame 1
        T2 = [sin(q(2)) cos(q(2))  0   a2*sin(q(2));
            -cos(q(2))  sin(q(2))  0   -a2*cos(q(2));
            0              0       1          0;
            0              0       0          1];
        
        %Frame 3 w.r.t Frame 2
        T3 = [cos(q(3)+(pi/2)) -sin(q(3)+(pi/2))  0   a3*cos(q(3)+(pi/2));
            sin(q(3)+(pi/2))  cos(q(3)+(pi/2))  0   a3*sin(q(3)+(pi/2));
            0                        0  1                     0;
            0                        0  0                     1];
        
        %Frame 4 w.r.t Frame 3
        T4 = [cos(q(4)-(pi/2)) -sin(q(4)-(pi/2))*cos(-pi/2)   sin(q(4)-(pi/2))*sin(-pi/2)   0;
            sin(q(4)-(pi/2))  cos(q(4)-(pi/2))*cos(-pi/2)  -cos(q(4)-(pi/2))*sin(-pi/2)   0;
            0                          sin(-pi/2)                    cos(-pi/2)   0;
            0                                   0                             0   1];
        %Frame 5 w.r.t Frame 4
        T5 = [cos(q(5)) -sin(q(5))  0        0;
            sin(q(5))  cos(q(5))  0        0;
            0          0  1       d5 + lg;
            0          0  0        1];
        
        % NEW CODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        T0 = eye(4);
        T0(:,:,2) = T1;
        T0(:,:,3) = T1*T2;
        T0(:,:,4) = T1*T2*T3;
        T0(:,:,5) = T1*T2*T3*T4;
        T0(:,:,6) = T1*T2*T3*T4*T5;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

end
