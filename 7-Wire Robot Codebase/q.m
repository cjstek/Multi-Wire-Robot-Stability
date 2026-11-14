%Returns Joint Values Cable Vectors

function [q] = q(cable_vecs)
    
    %Calculates Magnitude of each Vector & formulates q
    q1 = norm(cable_vecs(1,:));
    q2 = norm(cable_vecs(2,:));
    q3 = norm(cable_vecs(3,:));
    q4 = norm(cable_vecs(4,:));
    q5 = norm(cable_vecs(5,:));
    q6 = norm(cable_vecs(6,:));
    q7 = norm(cable_vecs(7,:));


    q = [q1 q2 q3 q4 q5 q6 q7]';
end