function lab2()
%% LAB2 demonstrates the SICK LMS291 LIDAR
%
%   [] = LAB2( )
%
%   This is a skeleton file for the LIDAR lab. It assumes you have already
%   called rosinit(URI) in the workspace, i.e., 
%
%   >> rosinit('http://192.168.0.123:11311');
%
% See also ROSINIT
%
% Copyright (C) 2019, by JRS

% Pseudo code for plotting
% rho = ranges
% alpha = bearing angles
% gamma = intensities
% indices = alpha >= thresh
% rhoHat = rho(indices)
% alphaHat = alpha(indices)
% [x,y] = pol2cart(alphaHat, rhoHat)
% plot(x, y, 'r')

%% Global variables
% These implement the functionality of C's kbhit.
global kbhit;
kbhit = false;
%% Local variables
alpha = (-pi/4:pi/360:pi/4)';       % Bearing angles are fixed by hardware
%% Figure setup
close all; 
figure('Name','SICK LMS291 LIDAR Stream','KeyPressFcn',@mykbhit);
% Ranges

h = plot(0,0,'ro');
title('Plot of the reflector location');
axis([-3 3 -3 3]);
xlabel('x-axis');
ylabel('y-axis');

%% Subscribes to the ROS /scan topic
laser = rossubscriber('/scan');

% These two variables are to track the effective scan rate
i=0;
tic;
% Main loop. Hit any key to break out of (with the figure in focus)
originRho = 2.2467;
originAlpha = -0.2356;
[originX, originY] = pol2cart(originAlpha, originRho); % X and Y translation of our frame with Lidar's frame
theta = -0.7914; %rotation of our frame with lidar's frame

while ~kbhit
    % Waits for scan data with timeout of 10 seconds
    scan_data = receive(laser,10);
    i = i+1;
    %origin: x = 107.2cm y= 189; P2: x = same; y +200
    
    myGamma = scan_data.Intensities >= 250; %The reflectivity values we are interested in
    rhoHat = scan_data.Ranges(myGamma); %The range values we are interested in
    alphaHat = alpha(myGamma); %The alpha values we are interested in
    
    myRho = mean(rhoHat); %calculating the mean of alpha vlues
    myAlpha = mean(alphaHat);%calculating the mean of alpha vlues
         
    [x,y] = pol2cart(myAlpha, myRho); 
     
    %theta = atan2(y - originY, x - originX); 
    %was used to calculate the rotation of our frame compared to lidar's frame

    %creating the rotation matrix
    A = [cos(theta) -sin(theta) originX; sin(theta) cos(theta) originY; 0 0 1];
    invA = inv(A); 
    frame = [x;y;1];
    resolved = invA * frame;
    
    %resolved value of x and y wrt our frame
    resolvedX = resolved(1);
    resolvedY = resolved(2); 
    
    % updated the value of the x and y on the plot
    set(h,'xdata',resolvedX, 'ydata', resolvedY);
    drawnow;

end
% Calculate and display the frame rate
toc;
fprintf('Effective scan rate was %0.1f Hz\n',i/toc);



function mykbhit( ~, ~ )
%% MYKBHIT simulates kbhit in 'C'
global kbhit;
kbhit = true;
