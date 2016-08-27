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
myorigin = transpose(param.origin); 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;
K = size(pose,2);
for j = 1:K
	angle = bsxfun(@plus,scanAngles,pose(3,j));
	a = [ranges(:,j).*cos(angle) -ranges(:,j).*sin(angle)]; b = [pose(1,j) pose(2,j)];
	pos_occ =  bsxfun(@plus,a,b); 
	%occupied position cell calculated from the ray from the robot and its pose
    %We include all K range scans in pos_occ 
    %discretization
    grid_occ = bsxfun(@plus,ceil(myResol*pos_occ),myorigin);
    pose_dis = ceil(myResol*b) + myorigin;

    for i = 1:size(pos_occ,1)
    	[freex, freey] = bresenham(grid_occ(i,1),grid_occ(i,2),pose_dis(1), pose_dis(2));
        free = sub2ind(size(myMap),freey,freex);
        %Doing the update after getting the indices of the measured values

        
        myMap(grid_occ(i,2),grid_occ(i,1)) = myMap(grid_occ(i,2),grid_occ(i,1)) + lo_occ;

        myMap(free) = myMap(free) - lo_free;

    end

  end
  myMap = min(myMap,lo_max);
  myMap = max(myMap,lo_min); 

end

