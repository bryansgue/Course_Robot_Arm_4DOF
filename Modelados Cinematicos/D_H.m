function M = D_H(q,d,a,alpha)
   


M = [cos(q) -cos(sym(alpha))*sin(q) sin(sym(alpha))*sin(q) a*cos(q);
     sin(q) cos(sym(alpha))*cos(q) -sin(sym(alpha))*cos(q) a*sin(q);
     0 sin(sym(alpha)) cos(sym(alpha)) d;
     0 0 0 1];
 
% M = [cos(q) -cos(alpha)*sin(q) sin(alpha)*sin(q) a*cos(q);
%      sin(q) cos(alpha)*cos(q) -sin(alpha)*cos(q) a*sin(q);
%      0 sin(alpha) cos(alpha) d;
%      0 0 0 1];

% M = Rotz*TrasZ*TrasX*Rotx;

