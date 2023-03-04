function [sys,x0,str,ts] = csfuncLagrangianModel(t,x,u,flag, q0, dq0)
%CSFUNC An example MATLAB file S-function for defining a continuous system.  
%   Example MATLAB file S-function implementing continuous equations: 
%      x' = Ax + Bu
%      y  = Cx + Du
%   
%   See sfuntmpl.m for a general S-function template.
%
%   See also SFUNTMPL.
  
%   Copyright 1990-2009 The MathWorks, Inc.

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(q0, dq0);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(q0,dq0)

sizes = simsizes;
sizes.NumContStates  = 2*size(q0,1); %3 for position and 3 for velocity
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2*size(q0,1); %position and velocity
sizes.NumInputs      = size(q0,1); %number of generalised forces, one for each link
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [q0; dq0];
str = [];
ts  = [0 0]; %Continuous time

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)
% sys = dx = [dq; ddq]
% x = [q; dq]
nJoints = size(x,1)/2;
q = x(1:nJoints);
dq = x(nJoints+1:2*nJoints);
tau = u;

% NB: I*ddq + F*dq + G*sinq = tau 

I = get_B_one_dof();
G = get_G_one_dof();
F = get_F_one_dof();
C = 0; % coriolis if dof > 1

ddq = I \ (tau - C*dq - G*sin(q) - F*dq);

sys = [dq;ddq];

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
% sys = y = x
sys = x;

% end mdlOutputs
