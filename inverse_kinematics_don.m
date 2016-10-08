% inverse kinematic
% define array to store the value of x 
x1=[];x2=[];x3=[];
a=[0;0.3;0.3;0.1];
% define the target positions
target = [0.1;0.1;0.3];
% define the origin state
q=[30*pi/180;45*pi/180;100*pi/180;100*pi/180];
theta1=q(1); theta(2)=q(2); theta(3)=q(3); theta(4)=q(4);
constraints=[0,2*pi;0.083*pi,0.416*pi;0.1667*pi,0.667*pi;0.25*pi,0.667*pi];
% the length of each linkage
a(2)=0.3;a(3)=0.3;a(4)=0.1;
x = [cos(theta1)*(a(2)*cos(theta(2))+a(3)*cos(theta(2)+theta(3))+a(4)*cos(theta(2)+theta(3)+theta(4)));
    sin(theta1)*(a(2)*cos(theta(2))+a(3)*cos(theta(2)+theta(3))+a(4)*cos(theta(2)+theta(3)+theta(4)));
    a(2)*sin(theta(2))+a(3)*sin(theta(2)+theta(3))+a(4)*sin(theta(2)+theta(3)+theta(4))];
% calculate the distance-how far away the object is from the target
diff = sqrt((target(1)-x(1))^2 + (target(2)-x(2))^2 +(target(3)-x(3))^2);
% store the value of x and theta in predefined arrays
x1(end+1)=x(1);x2(end+1)=x(2);x3(end+1)=x(3);
count = 0;
while(count<500 && diff>0.0001)
% jocobian matrix
J = [-sin(theta1)*(a(2)*cos(theta(2))+a(3)*cos(theta(2)+theta(3))+a(4)*cos(theta(2)+theta(3)+theta(4))),...
    cos(theta1)*(-a(2)*sin(theta(2))-a(3)*sin(theta(2)+theta(3))-a(4)*sin(theta(2)+theta(3)+theta(4))),...
    -cos(theta1)*(a(3)*sin(theta(2)+theta(3))+a(4)*sin(theta(2)+theta(3)+theta(4))),...
    -cos(theta1)*a(4)*sin(theta(2)+theta(3)+theta(4));cos(theta1)*(a(2)*cos(theta(2))+a(3)*cos(theta(2)+theta(3))+a(4)*cos(theta(2)+theta(3)+theta(4))),...
    sin(theta1)*(-a(2)*sin(theta(2))-a(3)*sin(theta(2)+theta(3))-a(4)*sin(theta(2)+theta(3)+theta(4))),...
    -sin(theta1)*(a(3)*sin(theta(2)+theta(3))+a(4)*sin(theta(2)+theta(3)+theta(4))),...
    -sin(theta1)*a(4)*sin(theta(2)+theta(3)+theta(4));0,a(2)*cos(theta(2))+a(3)*cos(theta(2)+theta(3))+a(4)*cos(theta(2)+theta(3)+theta(4)),...
    a(3)*cos(theta(2)+theta(3))+a(4)*cos(theta(2)+theta(3)+theta(4)),a(4)*cos(theta(2)+theta(3)+theta(4))];
% update the value of q
dq = pinv(J)*(target - x);
q=mod(q+dq,2*pi);
% ajust the angle according to the constraints
        if q(1)< constraints(1,1)                 
            q(1) = constraints(1,1);
        end
        if q(1) > constraints(1,2)
            q(1) = constraints(1,2);
        end
        if q(2)< constraints(2,1)
            q(2) = constraints(2,1);
        end
        if q(2) > constraints(2,2)
            q(2) = constraints(2,2);
        end
        if q(3) < constraints(3,1)
            q(3) = constraints(3,1);
        end
        if q(3) > constraints(3,2)
            q(3) = constraints(3,2);
        end
        if q(4) < constraints(4,1)
            q(4) = constraints(4,1);
        end
        if q(4) > constraints(4,2)
            q(4) = constraints(4,2);
        end
theta1=q(1); theta(2)=q(2); theta(3)=q(3); theta(4)=q(4);
% convert back to x
x = [cos(theta1)*(a(2)*cos(theta(2))+a(3)*cos(theta(2)+theta(3))+a(4)*cos(theta(2)+theta(3)+theta(4)));
    sin(theta1)*(a(2)*cos(theta(2))+a(3)*cos(theta(2)+theta(3))+a(4)*cos(theta(2)+theta(3)+theta(4)));
    a(2)*sin(theta(2))+a(3)*sin(theta(2)+theta(3))+a(4)*sin(theta(2)+theta(3)+theta(4))];
disp(x);
diff = sqrt((target(1)-x(1))^2 + (target(2)-x(2))^2 +(target(3)-x(3))^2);
count = count + 1;
x1(end+1)=x(1);
x2(end+1)=x(2);
x3(end+1)=x(3);
end

if diff >0.0001
disp(0);
end
if diff <= 0.0001
disp(1);
end;
plot3(x1,x2,x3,'b--o');
grid on











