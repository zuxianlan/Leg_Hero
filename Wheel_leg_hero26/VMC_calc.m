% 本部分通过五连杆解算得到l0和phi0与phi1与phi4的关系，
% 之后通过VMC腿部力控将末端F Tp转化为关节电机扭矩T1 T2
% 通过phi1和phi4得到l0和phi0

% 定义所需符号变量
syms phi0 phi1 phi2 phi3 phi4;
syms l0 l1 l2 l3 l4 l5;
syms xb yb xc yc xd yd;
syms dphi1 dphi4;
syms F Tp;

% 赋值初始变量
l1 = 0.216;
l2 = 0.252;
l3 = l2;
l4 = l1;
l5 = 0.0;

% 定义B D点坐标 进行几何运算
xb = l1 * cos(phi1);
yb = l1 * sin(phi1);
xd = l5 + l4 * cos(phi4);
yd = l4 * sin(phi4);


A0 = 2 * l2 * (xd - xb);
B0 = 2 * l2 * (yd - yb);
lbd = sqrt((xd-xb)^2 + (yd-yb)^2);
C0 = l2^2 + lbd^2 - l3^2;
phi2 = 2 * atan((B0+sqrt(A0^2+B0^2-C0^2))/(A0+C0));
phi3 = atan2(yb-yd+l2*sin(phi2),xb-xd+l2*cos(phi2));

xc = xb + l2 * cos(phi2);
yc = yb + l2 * sin(phi2);

l0 = sqrt((xc-l5/2)^2+yc^2);
phi0 = atan2(yc, (xc-l5/2));% 得到l0和phi0

% 建立腿部姿态方程 [l0; phi0] = lge_pos(phi1 , phi4)
position = [l0; phi0];
matlabFunction(position, 'File', 'leg_position');

% 建立雅可比矩阵
j11 = diff(l0, phi1);
j12 = diff(l0, phi4);
j21 = diff(phi0, phi1);
j22 = diff(phi0, phi4);
J = [j11 j12; j21 j22];

% 关节处dphi1 dphi4通过速度雅可比矩阵变换得到末端速度dl0 dphi0
% [dl0; dphi0] = leg_speed(phi1, phi4, dphi1, dphi4)
speed = J * [dphi1; dphi4];
matlabFunction(speed, 'File', 'leg_speed');

% 末端F Tp通过力雅可比矩阵变换得到关节扭矩T1 T2
% [T1; T2] = leg_convert(F, Tp, phi1, phi4)
T = J' * [F; Tp];
matlabFunction(T, 'File', 'leg_convert');





% syms phi1(t) phi2(t) phi3(t) phi4(t) phi_dot_1 phi_dot_4 l1 l2 l3 l4 l5
% x_B = l1*cos(phi1);
% y_B = l1*sin(phi1);
% x_C = x_B+l2*cos(phi2);
% y_C = y_B+l2*sin(phi2);
% x_D = l5+l4*cos(phi4);
% y_D = l4*sin(phi4);
% x_dot_B = diff(x_B,t);
% y_dot_B = diff(y_B,t);
% x_dot_C = diff(x_C,t);
% y_dot_C = diff(y_C,t);
% x_dot_D = diff(x_D,t);
% y_dot_D = diff(y_D,t);
% 
% phi_dot_2 = ((x_dot_D-x_dot_B)*cos(phi3)+(y_dot_D-y_dot_B)*sin(phi3))/l2/sin(phi3-phi2);
% 
% x_dot_C = subs(x_dot_C,diff(phi2,t),phi_dot_2);
% x_dot_C = subs(x_dot_C,...
%     [diff(phi1,t),diff(phi4,t)],...
%     [phi_dot_1,phi_dot_4]);
% y_dot_C = subs(y_dot_C,diff(phi2,t),phi_dot_2);
% y_dot_C = subs(y_dot_C,...
%     [diff(phi1,t),diff(phi4,t)],...
%     [phi_dot_1,phi_dot_4]);
% 
% x_dot = [x_dot_C; y_dot_C];
% q_dot = [phi_dot_1; phi_dot_4];
% x_dot = simplify(collect(x_dot,q_dot));
% J = simplify(jacobian(x_dot,q_dot));
% 
% matlabFunction(J, 'File', 'VMC_J');
% 
% disp(J);
