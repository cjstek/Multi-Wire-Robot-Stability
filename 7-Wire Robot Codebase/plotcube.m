function plotcube(Size, origin, alpha)
%Size = [L W H], origin = [x0 y0 z0]

L=Size(1); W=Size(2); H=Size(3);
x0=origin(1); y0=origin(2); z0=origin(3);

[X,Y,Z] = ndgrid([0 L] + x0, [0 W] + y0, [0 H] + z0);

verts = [X(:) Y(:) Z(:)];

faces = [
    1 2 4 3;
    5 6 8 7;
    1 2 6 5;
    2 4 8 6;
    4 3 7 8;
    3 1 5 7
];

patch('Vertices', verts, ...
      'Faces', faces, ...
      'FaceColor',[0.9 0.9 0.9], ...
      'FaceAlpha',alpha, ...
      'EdgeColor','k');
end