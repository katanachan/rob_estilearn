% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the initial map size in pixels
% Here we are initialization log odds as zero
myMap = zeros(param.size);
% % the origin of the map in pixels
myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;
poss_occ = zeros(size(ranges,1),2);
grid_occ = zeros(size(ranges,1),2);
K = size(pose,2);
for j = 1:K
	angle = bsxfun(@plus,scanAngles,pose(3,j));
	a = [ranges(:,j).*cos(angle) -ranges(:,j).*sin(angle)]; b = [pose(1,j) pose(2,j)];
	pos_occ =  bsxfun(@plus,a,b);
    %We include all K range scans in pos_occ 
    %discretization
    a = ceil((1/myResol)*[poss_occ]); b = myorigin';
    grid_occ = bsxfun(@plus,a,b);
    pose_dis = (1/myResol)*[pose(1,j), pose(2,j)];

    for i = 1:size(grid_occ,2)
    	[freex, freey] = bresenham(pose_dis(1), pose_dis(2), grid_occ(i,1),grid_occ(i,2));
        free = sub2ind(size(myMap),freey,freex);
        %Doing the update after getting the indices of the measured values

        a=myMap(grid_occ(2),grid_occ(1));
        a =+ lo_cc;

        % set free cell values
        b = myMap(free);
        b =- lo_free;

        if a > lo_max
        	a = lo_max;
        elseif a < lo_min
           a = lo_min;


        if b > lo_max
        	b = lo_max;
        elseif b < lo_min
           b = lo_min;
        end

    end
end
% for each time,
% 
%       
%     % Find grids hit by the rays (in the gird map coordinate)
%   
% 
%     % Find occupied-measurement cells and free-measurement cells
%    
% 
%     % Update the log-odds
%   
% 
%     % Saturate the log-odd values
%     
% 
%     % Visualize the map as needed
%    
% 
% end

end

