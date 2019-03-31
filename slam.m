% State and measurement sizes
pose = 3;
lmk_size = 2;

%
num_poses = 3;
num_lmks = 4;

% Factor list

factors{1} = struct('type' , 'pose','index', [0,0],'measurement', [0;0;0],'covariance', diag([1e-8, 1e-8, 1e-8]));

factors{2} = struct('type' , 'motion','index', [0,1],'measurement', [1; 0.05; deg2rad(5)],'covariance', diag([5e-3, 5e-3, (deg2rad(5))^2]));

factors{3} = struct('type' , 'motion','index', [1,2],'measurement', [1.1; -0.3; -deg2rad(15)],'covariance', diag([5e-3, 5e-3, (deg2rad(5))^2]));

factors{4} = struct('type' , 'lmk','index', [0,3],'measurement', [1.5; deg2rad(45)],'covariance', diag([5e-3,(deg2rad(5))^2]));

factors{5} = struct('type' , 'lmk','index', [1,3],'measurement', [0.8; deg2rad(95)],'covariance', diag([5e-3,(deg2rad(5))^2]));

factors{6} = struct('type' , 'lmk','index', [1,4],'measurement', [1.5; deg2rad(25)],'covariance', diag([5e-3,(deg2rad(5))^2]));

factors{7} = struct('type' , 'lmk','index', [0,5],'measurement', [1.5; -deg2rad(50)],'covariance', diag([5e-3,(deg2rad(5))^2]));

factors{8} = struct('type' , 'lmk','index', [1,5],'measurement', [0.9; -deg2rad(75)],'covariance', diag([5e-3,(deg2rad(5))^2]));

factors{9} = struct('type' , 'lmk','index', [0,6],'measurement', [2.8; -deg2rad(20)],'covariance', diag([5e-3,(deg2rad(5))^2]));

factors{10} = struct('type' , 'lmk','index', [1,6],'measurement', [1.8; -deg2rad(35)],'covariance', diag([5e-3,(deg2rad(5))^2]));

factors{11} = struct('type' , 'lmk','index', [2,6],'measurement', [0.8; -deg2rad(79)],'covariance', diag([5e-3,(deg2rad(5))^2]));

% Random states for SLAM to create the picture

states{1} = struct(...
    'type' , 'pose',...
    'range', [1,2,3],...
    'value', rand(3,1)); 

states{2} = struct(...
    'type' , 'pose',...
    'range', [4,5,6],...
    'value', rand(3,1));

states{3} = struct(...
    'type' , 'pose',...
    'range', [7,8,9],...
    'value', rand(3,1));

states{4} = struct(...
    'type' , 'lmk',...
    'range', [10,11],...
    'value', rand(2,1));

states{5} = struct(...
    'type' , 'lmk',...
    'range', [12,13],...
    'value', rand(2,1));

states{6} = struct(...
    'type' , 'lmk',...
'range', [14,15],...
    'value', rand(2,1));

states{7} = struct(...
    'type' , 'lmk',...
    'range', [16,17],...
    'value', rand(2,1));
 
dx = ones(17,1)*1e6;

eps = 1e-3;

while norm(dx) > eps
    
    [A,r] = buildproblem(states,factors);
    dx = solvelinearized(A,r);  
    
    states = updatestates(states,dx);
   
    drawmap(states,factors);
end
