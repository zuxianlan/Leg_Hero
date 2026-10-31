% 本程序用于获得系统的动力学方程
clear

    
% 所需符号量
syms theta theta1 theta2; % theta1=dTheta, theta2=ddTheta
syms x x1 x2 xd;
syms phi phi1 phi2;
syms T Tp N P Nm Pm Nf t L0;

% 机器人结构参数
R=0.033; L=L0/2; Lm=L0/2; l=0; mw=0.0351327; mp=0.028; M=0.5; Iw=5.02655e-05; Ip=(9.66667e-07+7.9975e-06)*2; Im=0.000338542;
g=9.8;

% 进行物理计算
Nm=M*(x2+(L+Lm)*(theta2*cos(theta)-theta1^2*sin(theta))-l*(phi2*cos(phi)-phi1^2*sin(phi)));
Pm=M*g+M*((L+Lm)*(-theta1^2*cos(theta)-theta2*sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));
N=Nm+mp*(x2+L*(theta2*cos(theta)-theta1^2*sin(theta)));
P=Pm+mp*g+mp*L*(-theta1^2*cos(theta)-theta2*sin(theta));

equ1=x2-(T-N*R)/(Iw/R+mw*R);
equ2=(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp-Ip*theta2;
equ3=Tp+Nm*l*cos(phi)+Pm*l*sin(phi)-Im*phi2;
[x2,theta2,phi2]=solve(equ1,equ2,equ3,x2,theta2,phi2);

matlabFunction(x2, "File", "x_ddot");
matlabFunction(theta2, "File", "theta_ddot");
matlabFunction(phi2, "File", "phi_ddot");
    

