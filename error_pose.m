function [e, J_e_rob] = error_pose(rob, y)
%in:
%   rob: robot pose
%   y: measured pose

%out:
%   e: estimation error
%   J_e_rob: Jacobian of e wrt. robot pose

e=rob-y;

J_e_rob=eye(3);


end