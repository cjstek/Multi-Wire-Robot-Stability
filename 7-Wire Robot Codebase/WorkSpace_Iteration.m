clc
clear all

%% Variable Initialization
x_bound = 200; y_bound = 200; z_bound = 200;

EE_Size = [10, 10, 10];
to = [0; 0; 0];
rotation = eye(3);
rotation_lock = 1;

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

%% Iterative Logic Loop

%Final Position Sets
x_fin = {80, -80, -20};
y_fin = {80, -80, 20};
z_fin = {80, -80, -20};

rot_temp = [1 0 0; 0 0.7071 -0.7071; 0 0.7071 0.7071]; %45 Degree Rotation about X-axis
rot_fin = {eye(3), [rot_temp]};

Initial_Frame = [rotation to; 0 0 0 1];
iteration_count = 0;

for aa = 1:length(x_fin)
    for bb = 1:length(y_fin)
        for cc = 1:length(z_fin)
            for dd = 1:length(rot_fin)
                
iteration_count = iteration_count+1;
%Target Position
tx = x_fin{aa};
ty = y_fin{bb};
yz = z_fin{cc};
tr = rot_fin{dd};
tf = [tx; ty; yz];
Targ_Frame = [tr tf; 0 0 0 1];
[vecs, ~] = InverseKinematics(Targ_Frame, EE_Att, Anchors); 
qh = q(vecs);

x = [to; 0; 0; 0; 0]; % Makes a 7x1 to match jacobian size

tol = 0.1;  % convergence tolerance in mm
maxIter = 100; %Maximum Number of Iterations to prevent infinite loops
iter = 0;
epsilon = 1e-6; %step for numerical Jacobian


%Initialize Animation File (Same as in Homework 2)
    File_Name = ['7_Wire_Robot_Iteration_',num2str(iteration_count),'.mp4'];
    Animation = VideoWriter(File_Name, 'MPEG-4');
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
    T = lsqlin(W, Wreq, [], [], [], [], zeros(7,1), [], [], opts); %In Newtons

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
progress = ['Finished Iteration: ',num2str(iteration_count),'/',num2str(length(x_fin)*length(y_fin)*length(z_fin)*length(rot_fin))];
disp(progress)

if iter == maxIter
    warning('Did Not Reach Exact Location');
end
            end
        end
    end
end

