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
previous_t = -1;
para = {};
t = 1; 
state = [0,0,0,0];

para_theta = {};
t = 1; 
state_theta = [0,0,0,0]; 
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 100;                            % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
for i = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
	%     % 1) Propagate the particles
    corrP = zeros(M,1); 
    W = ones(M,1) * 1/M; %initial weights have to be normalized
    [ ~, ~, state, para ] = kalmanFilter( t, myPose(1,i-1), myPose(2,i-1), state, para, previous_t );
    [ ~, ~, state_theta, para_theta ] = kalmanFilter( t, myPose(3,i-1), 0, state_theta, para_theta, previous_t );
    
    %previous_t = t(i);
    state(3), state(4)
    previous_t = 1-0.0249;
    if i < 30
        sigma_m = diag([0.1 0.1 0.035]);
    else
    	
    	sigma_m = diag([abs(state(3)) abs(state(4)) abs(state_theta(2))]);
    end
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
        
        null_grid = find(grid_occ(:,1) < 1 | grid_occ(:,2) < 1 | grid_occ(:,1) > size(myMap,2) | grid_occ(:,2) > size(myMap,1));
        grid_x = grid_occ(:,1);
        grid_y = grid_occ(:,2);
        grid_x(null_grid) = [];
        grid_y(null_grid) = [];

        corrP(j) = sum(sum(myMap(grid_y, grid_x) >= 0.5))*10;  %This will give the occupied cells that match with the LIDAR scans
        corrP(j) = corrP(j) - sum(sum(myMap(grid_y, grid_x) < -0.2))*5; %This will give the free cells that do not match with the LIDAR scans ie. the cells are free but the LIDAR scans insist that they're occupied
        W(j) = W(j) + corrP(j);
        
%     %   2-3) Update the particle weights         
%  
    end
        
        
        W = (1/sum(W))*W;
        
        n_eff = (sum(W))^2/sum(W.^2);
        disp(n_eff);

   
%     % 3) Resample if the effective number of particles is smaller than a threshold
%        
        while n_eff < 0.8*M
            W_new = zeros(size(W));
            P_new = zeros(size(P));
        	sum_of_weights = cumsum(W);
        	thresh = 1/size(sum_of_weights,1);
        	for k = 1:size(sum_of_weights,1)
        	   [row_index, ~] = ind2sub(size(W),find(sum_of_weights>thresh,1,'first'));
        	   if isempty(row_index) == 1
        	   	P_new(:,k) =  P(:,k);
        	   	W_new(k) = W(k);
        	   else
        	   	P_new(:,k) = P(:,row_index);
        	   	W_new(k) = W(row_index);

        	   end
        	   
        	   thresh = thresh + 1/size(sum_of_weights,1);
        	end

        	P = P_new;
        	W = (1/sum(W_new))*W_new; 
        	n_eff = (sum(W))^2/sum(W.^2);
        end

%     %   2-4) Choose the best particle to update the pose
%  
        [~,index] = max(W(:));
        index = ind2sub(size(W(:)),index);
        myPose(:,i) = P(:,index(1));
        i
        
end
       
          
end

%     % 4) Visualize the pose on the map as needed
%    
% 
% end