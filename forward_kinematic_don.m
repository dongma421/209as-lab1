%forward kinematics
theta=[30*pi/180;45*pi/180;100*pi/180;100*pi/180];   %define the initial state
%initialize the parameters from DH-table
a=[0.2;0.3;0.3;0.1];
d=[0.3;0;0;0];                                
alpha=[pi/2;0;0;0];
%define the constraints of each angle
constraints=[0,2*pi;0.083*pi,0.416*pi;0.1667*pi,0.667*pi;0.25*pi,0.667*pi];
%define origin in 3 dimention frame
p=[0,0,0];
%find out the transfer function for each i
A1 = individual_trans(1,a,alpha,d,theta);
A2 = individual_trans(2,a,alpha,d,theta);
A3 = individual_trans(3,a,alpha,d,theta);
A4 = individual_trans(4,a,alpha,d,theta);
T=A1*A2*A3*A4;
%set the homogeneous matrix for x
homo=[p(1);p(2);p(3);1];
%frame for x
w_frame=T*homo;
%get the initial position of x
x_ini = [w_frame(1);w_frame(2);w_frame(3)];
subplot(1,1,1);
plot3(x_ini(1),x_ini(2),x_ini(3),'b--o');
grid on;
title('Frame for x');
