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
% The initial pose is given
myPose(:,1) = param.init_pose;
K = size(scanAngles,1);
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 500;                            % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
corrP = zeros(M,1);
delta_new = zeros(M,K);
for i = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
	%     % 1) Propagate the particles
    corrP = zeros(M,1); 
    W = ones(M,1) * 1/M; %initial weights have to be normalized
    sigma_m = diag([0.25, 0.25, 0.10]);
    sigma_u = [0,0,0];   
%     % 2) Measurement Update 
%     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame) 
    for j = 1:M
    	%This is only a discretization from the local to global LIDARframe
    	P(:,j) = myPose(:,i-1) + mvnrnd(sigma_u,sigma_m)';
    	
        angle = bsxfun(@plus, scanAngles, P(3,j)');
        a = [ ranges(:,i).*cos(angle), -ranges(:,i).*sin(angle) ]; b = [P(1,j)', P(2,j)'];
        pos_occ = bsxfun(@plus, a,b);
        grid_occ = bsxfun(@plus,ceil(myResolution*pos_occ),myOrigin);

        grid_occ(grid_occ(:,1) < 1 | grid_occ(:,2) < 1 | grid_occ(:,1) > size(myMap,2) | grid_occ(:,1) > size(myMap,1)) = [];

        corrP(j) = sum(sum(myMap(grid_occ(:,2),grid_occ(:,1)) >= 0.5))*10;  %This will give the occupied cells that match with the LIDAR scans
        corrP(j) = corrP(j) - sum(sum(myMap(grid_occ(:,1),grid_occ(:,1)) < 0))*2 %This will give the free cells that do not match with the LIDAR scans ie. the cells are free but the LIDAR scans insist that they're occupied
    end
        
        W = W + corrP;
        W = (1/sum(W))*W;
        n_eff = (sum(W))^2/sum(W.^2);
       
        %if n_eff < M/5
        %	sum_of_weights = cumsum(W);

        %end
        [value,index] = max(W(:));
        index = ind2sub(size(W(:)),index)
        myPose(:,i) = P(:,index(1));
        
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