%% Code Description
    %This code represents the main operating file for an 8-wire parallel
%robot. This script requires a few user inputs being as follows: Robot
%Initial and Final Position, Robot Initial and Final Rotation, End Effector Size and Desired
%Rotation Lock. 

    %With these inputs, the robot will iterate through a
%straight line path between the two points while displaying the tension
%within each wire; with minimum tenstion of zero represented by the color
%green and maximum tension being the color red.

    %This code also calls several external functions stored within other
%MATLAB files, a description of each of these functions can be seen below.

%q(8x1 Vector) - Returns Joint Values Cable Vectors
%InverseKinematics(Frames, EE_Att, Anchors) - Returns robot inverse
    %kinematics in the form of cable vectors and EE location. 
%DrawRobot(Frames, Bounds, EE_Size, Anchors, EE_Att, T) - Returns a Figure
    %showing the robot, supporting cables, and associated tensions.


clear; clc; close all;

%% User Inputs/Robot Parameters
disp('Hello, please answer some questions about your desired 8-wire parallel robot:')

%Bounds
x_bound = input('Environment X-Bound:');
y_bound = input('Environment Y-Bound:');
z_bound = input('Environment Z-Bound:');

%End Effector Size
valid = 0;
while valid == 0
    EE_Size = input('Robot Initial Position as [L, W, H]: ');
    if abs(EE_Size(1)) < abs(x_bound/5) && abs(EE_Size(2)) < abs(y_bound/5) && abs(EE_Size(3)) < abs(z_bound/5) && all(EE_Size > 0)
        valid = 1;
    else
        disp('Inavlid input or Too Big (recommended maximum 1/10th size of bounds), please try again!')
    end
end

%Robot initial and final positions
valid = 0;
while valid == 0
    initialPosition = input('Robot Initial Position as [x; y; z]: ');
    if abs(initialPosition(1)) < abs(x_bound/2) && abs(initialPosition(2)) < abs(y_bound/2) && abs(initialPosition(3)) < abs(z_bound/2)
        valid = 1;
    else
        disp('Inavlid input, please try again!')
    end
end

valid = 0;
while valid == 0
    finalPosition = input('Robot Final Position as [x; y; z]: ');
    if abs(finalPosition(1)) < abs(x_bound/2) && abs(finalPosition(2)) < abs(y_bound/2) && abs(finalPosition(3)) < abs(z_bound/2)
        valid = 1;
    else
        disp('Inavlid input, please try again!')
    end
end

%Robot Rotation
valid = 0;
while valid == 0
    temp = input('End Effector Rotation as a 3x3 matrix or type auto: ', 's');
    if strcmp(temp, 'auto')
        rotation = eye(3);
        valid = 1;
    else
        if ~isempty(temp) && isequal(size(mat), [3 3])
            rotation = temp;
            valid = 1;
        else
            disp('Invalid input, please try again!')
        end
    end
end

%Enable/Disable End Effector Rotation
valid = 0;
while valid == 0
    temp = input('Would you like the rotation of the end effector to be locked? (y/n)', 's')
    if strcmp (temp, 'y')
        rotation_lock = 1;
        valid = 1;
    elseif strcmp (temp, 'n')
        rotation_lock = 1;
        valid = 1;
    else
        disp('Inavlid input, please try again!')
    end
end

%% Variable Initialization

%Workspace bounds centered at world origin
Bounds = [x_bound y_bound z_bound];   %[Xb, Yb, Zb]
Xb = Bounds(1);
Yb = Bounds(2);
Zb = Bounds(3);

% end effector size
L = EE_Size(1);
W = EE_Size(2);
H = EE_Size(3);

%Make anchor points from workspace corner coords
%World cube goes from -1/2Bound to +1/2Bound
xv = [-Xb/2, Xb/2];
yv = [-Yb/2, Yb/2];
zv = [-Zb/2, Zb/2];

%Build every combination, Minus 1 Wire
Anchors = [
    xv(2) yv(2) zv(2);   % 1: +x +y +z (top)
    xv(1) yv(2) zv(2);   % 2: -x +y +z
    xv(1) yv(1) zv(2);   % 3: -x -y +z
    xv(2) yv(1) zv(2);   % 4: +x -y +z

    xv(2) yv(2) zv(1);   % 5: +x +y -z (bottom)
    xv(1) yv(2) zv(1);   % 6: -x +y -z
    xv(1) yv(1) zv(1)];  % 7: -x -y -z
    %xv(2) yv(1) zv(1); This is the removed wire

%EE attachment points (corners of rectangular prism)
%EE frame origin is at center of bottom face
EE_Att = [
    +L/2, +W/2, H;   % 1: +x +y +z (top)
    -L/2, +W/2, H;   % 2: -x +y +z
    -L/2, -W/2, H;   % 3: -x -y +z
    +L/2, -W/2, H;   % 4: +x -y +z

    +L/2, +W/2, 0;   % 5: +x +y -z (bottom)
    -L/2, +W/2, 0;   % 6: -x +y -z
    -L/2, -W/2, 0;   % 7: -x -y -z
    %+L/2, -W/2, 0    % 8: +x -y -z
];


%% Resolved Rates
to = initialPosition;
Initial_Frame = [rotation to; 0 0 0 1];

%Target Position
tf = finalPosition;
Targ_Frame = [rotation tf; 0 0 0 1];
[vecs, ~] = InverseKinematics(Targ_Frame, EE_Att, Anchors); 
qh = q(vecs);

x = [to; 0; 0; 0; 0]; % Makes a 7x1 to match jacobian size

tol = 0.1;  % convergence tolerance in mm
maxIter = 100; %Maximum Number of Iterations to prevent infinite loops
iter = 0;
epsilon = 1e-6; %step for numerical Jacobian


%Initialize Animation File (Same as in Homework 2)
    Animation = VideoWriter('Parallel_Robot.mp4', 'MPEG-4');
    Animation.FrameRate = 10;
    open(Animation);


% Primary Logic Loop
Curr_Frame = Initial_Frame;
while iter < maxIter
    iter = iter + 1;
    
    % Current Location
    t = x(1:3);
    R = eul2rotm(x(4:6)'); %converts Euler to rotation
    Curr_Frame = [R t; 0 0 0 1];

    [P, b] = InverseKinematics(Curr_Frame, EE_Att, Anchors); %Gets Vertex Locations
    
    %Current joint values
    q_curr = q(P);
    
    %Joint Value Error
    dq = qh - q_curr;
    
    %Check if tolerance is met
    if norm(dq, 2) < tol
        fprintf('Converged in %d iterations\n', iter);
        break;
    end
    
    %Numerical Jacobian - Functions not setup properly for Implicit Jacobian  
    J = zeros(7,7);
    for j = 1:7
        x_p = x;
        x_p(j) = x_p(j) + epsilon; %Increments Position

        t_p = x_p(1:3);

        if rotation_lock == 1 && j > 3
            Curr_Frame(1:3,1:3) = tr;
        else
            R_p = eul2rotm(x_p(4:6)');
        end
        p_Frame = [R_p t_p; 0 0 0 1];

            %Base location isn't needed as it's static
        [P_p, ~] = InverseKinematics(p_Frame, EE_Att, Anchors);
        q_pert = q(P_p);
        
        J(:,j) = (q_pert - q_curr) / epsilon;
    end
    
    %% Wire Tension Formulation
    r = zeros(3,7);
    U = zeros(3,7);

    %Formulation of Unit Force Vectors
    for i = 1:7
        U(:,i) = P(i,:)' / norm(P(i,:));

        att_world = Anchors(i,:)' + P(i,:)';
        
        t_world = Curr_Frame(1:3,4);
        r(:,i) = att_world - t_world;
    end

    %Formulation of Wrench Matrix
    W = zeros(5,7);
    for i = 1:7
        W(1:3,i) = U(:,i);
        W(4:6,i) = cross(r(:,i), U(:,i));
    end

    % Definition of Weight/Force on End Effector
    mass = 5; %kg
    g = [0; 0; -9.81];
    Force = mass*g;
    tau_req = [0; 0; 0];

    Wreq = [Force; tau_req];
    
    %Iterative Solver for Positive Wire Tenstions
    opts = optimoptions('lsqlin','Display','none');  %Supress disply output of status
    T = lsqlin(W, Wreq, [], [], [], [], zeros(7,1), [],[],opts); %In Newtons

    %Formulation of Tau Matrix per Paper Documentation
    tau = zeros(3,3);
    for i = 1:7
        tau(:,i) = cross(r(:,i), T(i)*U(:,i));
    end

    %% Draw Robot & Record Animation

    %Generate Figure using new frame data
        cla %Clears figure for next animation frame/figure
        view(64,20);
        DrawRobot(Curr_Frame, Bounds, EE_Size, Anchors, EE_Att, T) %Using the Provided Draw  
        axis equal;
        set(gcf, 'Position', [50, 50, 1500, 500]);
        grid on;

    %Records Frame/Figure to Animation
        drawnow;
        current_frame = getframe(gcf); 
        writeVideo(Animation, current_frame);
    
    %Increment Platform
    dx = pinv(J) * dq * 0.1; %Multiplied by Arbitary Gain to Stop Overstepping
    x = x + dx;
end

%% Cleanup & File Recording
close(Animation); %Animation File is Saved Within Project Folder

if iter == maxIter
    warning('Did Not Reach Exact Location');
end