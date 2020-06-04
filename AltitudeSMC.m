% When I wrote this, only God and I understood what I was doing
% Now, only god knows
% Quadrotor altitude controller based on sliding mode
function [sys,x0,str,ts] = AltitudeSMC(t, x, u, flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % ���ó�ʼ���Ӻ���
  case 1,
    sys=[];
  case 2,
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u);    %��������Ӻ���
  case 4,
    sys=[];   %������һ����ʱ���Ӻ���
  case 9,
    sys=[];    %��ֹ�����Ӻ���
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes   %��ʼ���Ӻ���

sizes = simsizes;

sizes.NumContStates  = 0;  %����״̬��������
sizes.NumDiscStates  = 0;  %��ɢ״̬��������
sizes.NumOutputs     = 1;  %�����������
sizes.NumInputs      = 2;   %�����������
sizes.DirFeedthrough = 1;   %�����ź��Ƿ�������Ӻ����г���
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %��ʼֵ
str = [];   
ts  = [];   %[0 0]��������ϵͳ��[-1 0]��ʾ�̳���ǰ�Ĳ���ʱ������
simStateCompliance = 'UnknownSimState';

function sys=mdlOutputs(t,x,u)   %��������Ӻ���
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