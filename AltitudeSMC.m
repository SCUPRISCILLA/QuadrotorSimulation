% When I wrote this, only God and I understood what I was doing
% Now, only god knows
% Quadrotor altitude controller based on sliding mode
function [sys,x0,str,ts] = AltitudeSMC(t, x, u, flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % 调用初始化子函数
  case 1,
    sys=[];
  case 2,
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u);    %计算输出子函数
  case 4,
    sys=[];   %计算下一仿真时刻子函数
  case 9,
    sys=[];    %终止仿真子函数
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes   %初始化子函数

sizes = simsizes;

sizes.NumContStates  = 0;  %连续状态变量个数
sizes.NumDiscStates  = 0;  %离散状态变量个数
sizes.NumOutputs     = 1;  %输出变量个数
sizes.NumInputs      = 2;   %输入变量个数
sizes.DirFeedthrough = 1;   %输入信号是否在输出子函数中出现
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %初始值
str = [];   
ts  = [];   %[0 0]用于连续系统，[-1 0]表示继承其前的采样时间设置
simStateCompliance = 'UnknownSimState';

function sys=mdlOutputs(t,x,u)   %计算输出子函数
x1 = u(1);  %position input
x2 = u(2);  %velocity input
%desired
% xd = 1 + sin(t);
% dxd = cos(t);
% ddxd = -szin(t);
xd = 3;
dxd = 0;
ddxd = 0;
e = xd - x1;
de = dxd - x2;

c = 10;
s = c*e + de;

g = 9.81;
ita = 0.1;
k = 1;
m = 0.3108;

Tth = m*(ddxd + g + c*(dxd - x2) + ita*tanh(s/0.1) + k*s);

sys(1) = Tth;