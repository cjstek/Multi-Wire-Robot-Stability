function drawEE(Frames, EE_Size)
% draws the end effector as a box centered at its frame

R = Frames(1:3,1:3,end);
p = Frames(1:3,4,end);

L = EE_Size(1);
W = EE_Size(2);
H = EE_Size(3);

% EE corners in local frame (8 corners)
corners_local = [
    +L/2, +W/2, 0;
    +L/2, -W/2, 0;
    -L/2, -W/2, 0;
    -L/2, +W/2, 0;
    +L/2, +W/2, H;
    +L/2, -W/2, H;
    -L/2, -W/2, H;
    -L/2, +W/2, H
];

% map to world
for i=1:8
    corners_world(i,:) = (R * corners_local(i,:)' + p)';
end

% faces of the box
faces = [
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8
];

patch('Vertices', corners_world, ...
      'Faces', faces, ...
      'FaceColor',[0.7 0.8 1], ...
      'FaceAlpha',0.4, ...
      'EdgeColor','b');

end