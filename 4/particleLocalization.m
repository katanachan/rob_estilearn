% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2); %remember ranges is a bunch of readings per time step
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N); %going to output x, y, theta
myMap = map;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
myResolution = param.resol;
% % the origin of the map in pixels
myOrigin = transpose(param.origin); 
lo_occ = +10;
lo_free = +1;
% The initial pose is given
myPose(:,1) = param.init_pose;
previous_t = -1;
K = size(scanAngles);

% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 5;                            % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);

for i = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
	%     % 1) Propagate the particles
%I'm using a Kalman filter here to estimate myPose! Do not want to /know how to construct random noise for pose
    [ predictx, predicty, state, param.P ] = kalmanFilter( t(i), param.pose(1,i), param.pose(2,i), state, param.P, previous_t )
    previous_t = t(i);
    P = P + [ state(3), 0, 0; 0 state(4) 0; 0 0 0 ]*(randn([size(P,1), M]); %I'm not adding any noise to my angles
    x = state(1);
    y = state(2);
    theta = param.pose(3,i);	
%       
%     % 2) Measurement Update 
%     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame) 
    for j = 1:K
    	angle = bsxfun(@plus,scanAngles,theta));
    	a = [ranges(:,j).*cos(angle) -ranges(:,j).*sin(angle)]; b = [x y];
    	pos_occ =  bsxfun(@plus,a,b);
    	%occupied position cell calculated from the ray from the robot and its pose
    	%We include all K range scans in pos_occ 
    	%discretization
    	grid_occ = bsxfun(@plus,ceil(myResolution*pos_occ),myOrigin);
    	pose_dis = ceil(myResolution*b) + myOrigin;
    	for k = 1:size(pos_occ,1)	
    		[freex, freey] = bresenham(grid_occ(k,1),grid_occ(k,2),pose_dis(1), pose_dis(2));
    		free = sub2ind(size(myMap),freey,freex);
%     %   2-2) For each particle, calculate the correlation scores of the particles
%
    		corrP = map(grid_occ(k,2),grid_occ(k,1))* 
    		corrMap(grid_occ(k,2),grid_occ(k,1)) = corrMap(grid_occ(k,2),grid_occ(k,1)) + lo_occ;
    		corrMap(free) = corrMap(free) - lo_free;
    	end
    end


%     %   2-3) Update the particle weights         
%  
%     %   2-4) Choose the best particle to update the pose
%     
%     % 3) Resample if the effective number of particles is smaller than a threshold
% 
%     % 4) Visualize the pose on the map as needed
%    
% 
% end

end

