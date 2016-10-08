function Tr = individual_trans(i,a,alpha,d,theta)
    Tr = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)),a(i)*cos(theta(i));
    sin(theta(i)), cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)),a(i)*sin(theta(i));       0,sin(alpha(i)), cos(alpha(i)), d(i); 0,0,0,1];
end
