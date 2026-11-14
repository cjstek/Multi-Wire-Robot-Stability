function DrawRobot(Frames, Bounds, EE_Size, Anchors, EE_Att, T)
%DrawRobot, produces a drawing of the robot
%Frames: 4x4xK transform stack, last one is EE pose
%Bounds: [Xbound, Ybound, Zbound]
%EE_Size: [L, W, H]
%Anchors: Nx3 anchor locations (probably 8)
%EE_Att: Nx3 EE corner attachment pts (local EE coords)

%Plot workspace box and end effector and cables

cla; hold on; grid on; axis equal;

Xb = Bounds(1);
Yb = Bounds(2);
Zb = Bounds(3);

%Draw workspace (wireframe cube)
plotcube([Xb Yb Zb], [-Xb/2 -Yb/2 -Zb/2], 0.05);

%Get cable vectors & global EE corner positions
[Cables, C_global] = InverseKinematics(Frames, EE_Att, Anchors);

%Draw each cable
T_min = min(T);
T_max = max(T);

for i = 1:size(Anchors,1)
    A = Anchors(i,:);
    C = C_global(i,:);

    if T(i) < 0
        cable_color = [0 0 0];
    else
        Ti = min(max(T(i), T_min), T_max); 
        cable_color = [Ti/T_max, 1-Ti/T_max, 0];

    %Cable lines
    plot3([A(1) C(1)], [A(2) C(2)], [A(3) C(3)], 'k-', 'color', cable_color, 'LineWidth', 5);

    midpoint = (A + C) / 2;     % midpoint of the cable segment
    offset = 0.02 * (Bounds);   % small offset proportional to workspace size
    text_pos = midpoint + offset;  

    text(text_pos(1), text_pos(2), text_pos(3), sprintf('T = %.2f', T(i)), 'FontSize', 12, 'Color', cable_color, 'FontWeight', 'bold');
end

%Draw the end effector box
drawEE(Frames, EE_Size);

R = Frames(1:3,1:3,end);
p = Frames(1:3,4,end);

L = EE_Size(1);
W = EE_Size(2);
H = EE_Size(3);
ax_len = max([L W H]) * 0.6;

quiver3(p(1),p(2),p(3), ax_len*R(1,1), ax_len*R(2,1), ax_len*R(3,1), 'r','LineWidth',2);
quiver3(p(1),p(2),p(3), ax_len*R(1,2), ax_len*R(2,2), ax_len*R(3,2), 'g','LineWidth',2);
quiver3(p(1),p(2),p(3), ax_len*R(1,3), ax_len*R(2,3), ax_len*R(3,3), 'b','LineWidth',2);

%optional labels:
text(p(1)+ax_len*R(1,1), p(2)+ax_len*R(2,1), p(3)+ax_len*R(3,1), 'x_1');
text(p(1)+ax_len*R(1,2), p(2)+ax_len*R(2,2), p(3)+ax_len*R(3,2), 'y_1');
text(p(1)+ax_len*R(1,3), p(2)+ax_len*R(2,3), p(3)+ax_len*R(3,3), 'z_1');


axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Cable Robot View\n(Tension Values in Newtons)');

end