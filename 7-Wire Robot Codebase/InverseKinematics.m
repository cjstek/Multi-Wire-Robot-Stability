function [cable_vecs, C_global] = InverseKinematics(Frames, EE_Att, Anchors)
%Computes cables for a cable-driven robot as vectors

%Frames = 4x4xK stack of world-to-frame transforms
%EE_Att = Nx3 matrix of EE attachment points in EE coordinates
%Anchors = Nx3 matrix of cable anchor points in world coordinates

%Outputs:
%C_global = Nx3 positions of EE attachment points, corners, in world frame
%cable_vecs = Nx3 vectors from anchor to EE attachment pts

%Rotation
R = Frames(1:3,1:3,end);
%Translation
p = Frames(1:3,4,end);

n = size(Anchors,1);

C_global = zeros(n,3);
cable_vecs = zeros(n,3);

for i = 1:n
    % Convert EE attachment point into world coordinates
    C_global(i,:) = (R * EE_Att(i,:)' + p)';

    % Cable vector: anchor to EE point
    cable_vecs(i,:) = C_global(i,:) - Anchors(i,:);
end

end