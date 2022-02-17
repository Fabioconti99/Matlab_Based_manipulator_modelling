%% Main function - MOCOM LAB 1
clc
clear
close all

addpath('include')

%initialization of the transformation matrix from link i to link i+1 for
%qi=0
biTri(1,1,1) = 1; biTri(1,2,1) = 0; biTri(1,3,1) = 0; biTri(1,4,1) = 0;
biTri(2,1,1) = 0; biTri(2,2,1) = 1; biTri(2,3,1) = 0; biTri(2,4,1) = 0;
biTri(3,1,1) = 0; biTri(3,2,1) = 0; biTri(3,3,1) = 1; biTri(3,4,1) = 0.175;
biTri(4,1,1) = 0; biTri(4,2,1) = 0; biTri(4,3,1) = 0; biTri(4,4,1) = 1;

biTri(1,1,2) = -1; biTri(1,2,2) = 0;  biTri(1,3,2) = 0; biTri(1,4,2) = 0;
biTri(2,1,2) = 0; biTri(2,2,2) = 0;  biTri(2,3,2) = 1; biTri(2,4,2) = 0;
biTri(3,1,2) = 0; biTri(3,2,2) = 1;  biTri(3,3,2) = 0; biTri(3,4,2) = 0.108;
biTri(4,1,2) = 0; biTri(4,2,2) = 0;  biTri(4,3,2) = 0; biTri(4,4,2) = 1;

biTri(1,1,3) = 0; biTri(1,2,3) = 0; biTri(1,3,3) = 1;  biTri(1,4,3) = 0.105;
biTri(2,1,3) = -1; biTri(2,2,3) = 0; biTri(2,3,3) = 0;  biTri(2,4,3) = 0;
biTri(3,1,3) = 0; biTri(3,2,3) = -1; biTri(3,3,3) = 0;  biTri(3,4,3) = 0;
biTri(4,1,3) = 0; biTri(4,2,3) = 0; biTri(4,3,3) = 0;  biTri(4,4,3) = 1;

biTri(1,1,4) = 0; biTri(1,2,4) = 1;  biTri(1,3,4) = 0; biTri(1,4,4) = -0.1455;
biTri(2,1,4) = 0; biTri(2,2,4) = 0;  biTri(2,3,4) = -1; biTri(2,4,4) = 0;
biTri(3,1,4) = -1; biTri(3,2,4) = 0;  biTri(3,3,4) = 0; biTri(3,4,4) = 0.3265;
biTri(4,1,4) = 0; biTri(4,2,4) = 0;  biTri(4,3,4) = 0; biTri(4,4,4) = 1;

biTri(1,1,5) = 0; biTri(1,2,5) = 0; biTri(1,3,5) = 1;  biTri(1,4,5) = 0.095;
biTri(2,1,5) = 0; biTri(2,2,5) = -1; biTri(2,3,5) = 0; biTri(2,4,5) = 0;
biTri(3,1,5) = 1; biTri(3,2,5) = 0; biTri(3,3,5) = 0;  biTri(3,4,5) = 0;
biTri(4,1,5) = 0; biTri(4,2,5) = 0; biTri(4,3,5) = 0;  biTri(4,4,5) = 1;

biTri(1,1,6) = 0; biTri(1,2,6) = 0;  biTri(1,3,6) = 1; biTri(1,4,6) = 0;
biTri(2,1,6) = 0; biTri(2,2,6) = -1;  biTri(2,3,6) = 0; biTri(2,4,6) = 0;
biTri(3,1,6) = 1; biTri(3,2,6) = 0;  biTri(3,3,6) = 0; biTri(3,4,6) = 0.325;
biTri(4,1,6) = 0; biTri(4,2,6) = 0;  biTri(4,3,6) = 0; biTri(4,4,6) = 1;

biTri(1,1,7) = 0; biTri(1,2,7) = 0;  biTri(1,3,7) = 1; biTri(1,4,7) = 0.132;
biTri(2,1,7) = 0; biTri(2,2,7) = -1; biTri(2,3,7) = 0; biTri(2,4,7) = 0;
biTri(3,1,7) = 1; biTri(3,2,7) = 0;  biTri(3,3,7) = 0; biTri(3,4,7) = 0;
biTri(4,1,7) = 0; biTri(4,2,7) = 0;  biTri(4,3,7) = 0; biTri(4,4,7) = 1;

%number of links of the manipulator
nLinks = size (biTri, 3);

%Initial configuration of the manipulator
qi = [0; 0; 0; 0; 0; 0; 0]; 
%We only have revolute joint
linkType = zeros(nLinks,1);

%initial transformation matrix
current_biTri = GetDirectGeometry(qi, biTri, linkType);

% Transformation matrix of the Goal frame with respect to the base
Tgoal = [1, 0, 0, 0.2;
         0, 1, 0, 0.2;
         0, 0, 1, 0.4;
         0, 0, 0,  1]; 
%Angular gain
gamma_a = 0.7;
%Linear gain
gamma_l = 0.8; 

%Variables initialization
basicVectors = zeros(3, nLinks);
distA = zeros(3, 1);
J = zeros(6, nLinks);
q = qi;
qmin = zeros(1, nLinks);
qmax = zeros(1, nLinks);
xA = zeros(3, 1);
xL = zeros(3, 1);
ts = 0.1;
t_end = 7.0;

for i = 1 : nLinks
   if(linkType(i) == 0) %Revolute
       qmin(i) = -pi;
       qmax(i) = pi;
   else %Prismatic
       qmin(i) = 0;
       qmax(i) = 0.1;
   end
end

 %% MOCOM LAB 2 - Kinematic Simulation %%

for i = 0.0:ts:t_end
    
    
    %Distance between the goal and the end-effector
    distL = Tgoal(1:3, 4) - GetBasicVectorWrtBase(current_biTri, nLinks);
    %Linear velocity of the end-effector
    xL = gamma_l * distL;
    
    %Transformation of the end effector with respect to the base
    bTe = GetTransformationWrtBase(current_biTri, nLinks);
    %Angular distance between the goal and the end-effector
    distA = VersorLemma(bTe(1:3, 1:3), Tgoal(1:3, 1:3));
    %Angular velocity of the end-effector
    xA = gamma_a * distA; 
    
    %Angular and linear velocity in one vector
    x = [xA; xL];
    %The jacobian matrix 
    J = GetJacobian(current_biTri, bTe, linkType);
    %Joints's velocities corresponding to the desired velocities
    %of the end-effector
    q_dot = pinv(J)*x;
    
    %Simulation
    q = KinematicSimulation(q, q_dot,ts, qmin, qmax);
    
    current_biTri = GetDirectGeometry(q, biTri, linkType);
    
    % Retrieving the basic vector from biTri
    for linkNumber = 1 : nLinks
    basicVectors(:, linkNumber) = GetBasicVectorWrtBase(current_biTri, linkNumber);
    end

    hold on;
    
    azimuth = 50;
    elevation = 25;

    % Defining the point of view

    view(azimuth,elevation)
    %Plot basic vectors
    plot3(Tgoal(1,4), Tgoal(2,4),Tgoal(3,4),'red.', 'MarkerSize', 6); 
    
    xPlot = [0, basicVectors(1,:)];
    yPlot = [0, basicVectors(2,:)];
    zPlot = [0, basicVectors(3,:)];

    plot3(xPlot, yPlot, zPlot, 'LineWidth',3,'color','g');
    xlabel("X");
    ylabel("Y");
    zlabel("Z");
    grid();
    axis([-1 1 -1 1 -1 1]);

    getframe;

    if i<t_end
        %deleting previous plots
        cla();
    end

end
