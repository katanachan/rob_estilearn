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
M = 100;                            % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
W = ones(M,1) * 1/M; %initial weights have to be normalized
corrP = zeros(M,1);
delta_new = zeros(M,K);
for i = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
	%     % 1) Propagate the particles
    corrP = zeros(M,1); 
    sigma_m = diag([0.025, 0.025, 0.030]);
    sigma_u = [0,0,0];   
%     % 2) Measurement Update 
%     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame) 
    for j = 1:K
    	%This is only a discretization from the local to global LIDARframe
    	for k = 1:M
    	   	P(:,k) = myPose(:,i-1) + mvnrnd(sigma_u,sigma_m)';
    	end

        angle = bsxfun(@plus, P(3,:)',scanAngles(j));
        a = [ ranges(j,i)*cos(angle), -ranges(j,i)*sin(angle) ]; b = [P(1,:)', P(2,:)'];
        pos_occ = bsxfun(@plus, b,a);
        grid_occ = bsxfun(@plus,ceil(myResolution*pos_occ),myOrigin);
        if orig_grid(1) > size(myMap,2) | orig_grid(2) > size(myMap,1) | orig_grid(1) < 1 | orig_grid(2) < 1
        	map_value = 0;
        else
            map_value = myMap(orig_grid(2), orig_grid(1)); 
        end

        delta_new(:,j) = map_value*delta_metric(:,1);
        
    end
        for j = 1:K
            corrP = corrP + delta_new(:,j);
        end
        W = W.*corrP;
        n_eff = (sum(W))^2/sum(W.^2);
       
        if n_eff < M/5
        	W = (1/sum(W))*W;
        end
        [value,index] = max(W(:));
        index = ind2sub(size(W(:)),index)
        myPose(:,i) = P(:,index(1));
        i
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