% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
load mu.mat
load sig.mat 
thresh = 0.01;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
I = rgb2hsv(I);
imshow(I);
title('HSV Image');

px = zeros(size(I(:,:,2)));
        
for i=1:size(I,1)
    for j = 1:size(I,2)
        k = [];
        for l = 1:size(I,3)
            k = [k, (bsxfun(@minus,I(i,j,l),mu(:,l)))];
        end
        px(i,j) = k*inv(sig)*k' + log(det(sig));        
    end
end

figure; imshow(px);
title('Gaussian probability');
px = px < thresh;
figure; imshow(px);
title('Binary image');
bw_biggest = false(size(px));

% http://www.mathworks.com/help/images/ref/bwconncomp.html
CC = bwconncomp(px);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
figure,
imshow(bw_biggest); hold on;

% show the centroid
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
plot(loc(1), loc(2),'r+');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

segI = bw_biggest;
loc = loc;
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
