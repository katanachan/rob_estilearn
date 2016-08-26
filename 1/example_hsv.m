load('Samples_HSV.mat', '-mat'); 
% visualize the sample distribution
figure, 
scatter3(Samples(:,1),Samples(:,2),Samples(:,3),'.');
title('Pixel Color Distribubtion');
xlabel('Hue');
ylabel('Saturation');
zlabel('Value');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [IMPORTANT]
%
% Now choose you model type and estimate the parameters (mu and Sigma) from
% the sample data.
%3D Gaussian has to be found out in this case!

mu = [0; 0; 0];
sig = zeros(3,3);
x = Samples;
a = size(x);
mu = sum(x)/a(1,1);
sig = (1/a(1,1))*bsxfun(@minus,x,mu)'*(bsxfun(@minus,x,mu));

