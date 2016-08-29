function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %Defining the State model.
    % 1. A - state transition matrix of 4x4.
    % 2. sigma_m - noise due to all properties not being captured in linearity. Is a 4x4 diagonal matrix  
    %Defining the Measurement model. Note: We only have TWO measurements x and y. But 4 states.  
    % 1. C - observation matrix of 2x4
    % 2. sigma_k - sensor noise. It is a 2x2 diagonal matrix.     
    dt = t-previous_t
    sigma_m = [(2*dt)^2, 0, dt^2, 0 ; 0, (2*dt)^2, 0, dt^2; dt, 0, (4*dt)^4, (4*dt)^4; 0, 0.25*dt, 0, 0.25*dt]; 

    sigma_k = [0.01,0; 0,0.01]; 
    
    A = [1, 0, dt, 0 ; 0, 1, 0, dt; 0, 0, 1, 0; 0, 0, 0, 1];
    C = [1, 0, 0, 0; 0, 1, 0, 0];

    % Check if the first time running this function
    if previous_t<0
        state = [param.init_pose(1), param.init_pose(2), 0, 0]';
        param.P = 4 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    Pt = param.P;
    P = A*Pt*transpose(A) + sigma_m;
    %Q = C*P*transpose(C) + sigma_k; %Not using this because the uncertainty in the observation model is supposed to be lower than that of the motion model
    Q = sigma_k;
    K = P*transpose(C)*inv(Q + C*P*transpose(C));
    state = (A-K*C*A)*state + K*[x, y]';
    % Predict 330ms into the future
    predictx = state(1,1) + state(3,1) * dt;
    predicty = state(2,1) + state(4,1) * dt;
    param.P = P - K*C*P; %The estimate of covariance in the state also changes. Find out why?     
    % State is a four dimensional element
    
end
