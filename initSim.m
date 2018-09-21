% MIT License
%
% Copyright (c) 2018 - Matteo Cocetti, Matteo Ragni
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
%
% The Hybrid Integrator in Simulink example is inspired by the:
%   HyEQ Toolbox (Copyright @ Hybrid Systes Laboratory - HSL)

%% The script runs before Running the simulation (it is the initFnc)! %%

close('all');
clear('data');
clc;

%% Declarations

% Configurations for hybrid integrator
data.T_horizon = inf;
data.J_horizon = 1e4; % This cannot be inf, because of possible Zeno solution
% Simulation limit time
data.T_end     = 60;
% Simulink file name
data.example = 'example';

%% Model definition

% In the model definition, please provide the system A, B, C inside the 
% data structure. Also provide the initial conditions (must be consistent
% with the model) in data.sys_ics.
%
% dx = A x + B theta_hat' * Yd

wn   = 3;
k    = 3;
zeta = 0.2;

data.A = [       0,              1  ; ...
           -(wn)^2, -2 * zeta * wn ];
data.B = [ 0  ; ...
           1 ];
data.C = [ k, 0 ];

clear('wn', 'k', 'zeta');

data.sys_ics = [3; 4];

%% Reference definition

% Please provide a differentiable reference here. It will be prepared
% and used inside the simulink system automatically. Use the variable 't'
% for reference description in function of time. It requires the Symbolic
% Toolbox. Save the reference in the data.ref
% Reference vector will be as follows:
%
% Yd' = [ r(t), diff(r(t),t), ..., diff(r(t),t,n) ]

t = sym('t');

reference = 5 * cos(t / 5) + cos(t + pi/3) + sin(2 * t/5);


%% Right inverse definition

% Right inverse configuration. Please provide the eigenvalues for the
% Lambda matrix (filters) and the update time domain. Gamma will contain
% the update coefficient. For an implementative reason there is an
% additional state that select a jumping time in [tau_min, tau_max].
%
%  (cont. dyn.) for tau in [0, tau_max]
%      dsigma = Lambda * sigma + Phi * y
%        deta = Lambda * eta + Phi * theta_hat' * Yd
%  dtheta_hat = 0
%        dtau = 1                                      
%    dtau_set = 0                                   
%  (disc. dyn.) for tau >= tau_set
%      sigma+ = sigma
%        eta+ = eta
%  theta_hat+ = theta_hat - gamma * sigma/(max(sigma' * sigma, epsilon^2)) * 
%                                           (sigma' * theta_hat - eta(1))
%        tau+ = 0
%    tau_set+ = tau_min + (tau_max - tau_min) * random([0,1])
% 
% Initial condition for theta_hat are in theta_ics. Dimensions will be
% checked before starting the simulation.
% The epsilon value must be a small real number.

data.eig = [-50, -75, -100];

data.tau_min = 0.25;
data.tau_max = 3.0;

data.gamma = 0.6;
data.epsilon = 1e-6;

data.theta_ics = [1; 0.2; 0.1];

%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Do not edi below this point                                            %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %%

%% Simulation preparation

clear('t');

if (~iscolumn(data.sys_ics))
  data.sys_ics = data.sys_ics';
  if (~iscolumn(data.sys_ics))
    error('Cannot create column vector from system initial conditions');
  end
end
data.sys_n = length(data.sys_ics);

if (~iscolumn(data.theta_ics))
  data.theta_ics = data.theta_ics';
  if (~iscolumn(data.theta_ics))
    error('Cannot create column vector from theta_hat initial conditions');
  end
end

if (size(data.eig) ~= size(data.theta_ics'))
  error('Eigenvalues for Lambda and theta_hat size are not consistent');
end

data.theta_n = length(data.theta_ics);
if data.theta_n ~= data.sys_n + 1
  error('Invalid dimension for theta_hat initial conditions.');
end

tmp = struct;
[tmp.A, tmp.B] = FilterMatrices(data.theta_ics);
tmp.CL = place(tmp.A, tmp.B, data.eig);

data.Lambda = tmp.A - tmp.B * tmp.CL;
data.Phi    = tmp.B;

tmp.tau_set = data.tau_min + (data.tau_max - data.tau_min) * 0.33; % Should be rand, this to have deterministic ICS

data.ics = [ data.sys_ics; ...
             zeros(data.theta_n, 1); ...
             zeros(data.theta_n, 1); ...
             data.theta_ics; ...
             0; tmp.tau_set]; 
          
reference = ReferenceGeneration(reference, data.theta_n);
matlabFunctionBlock(sprintf('%s/ref_block', data.example), reference, 'Outputs', {'Yd'});


data.index = struct;
data.index.x         = 1:data.sys_n;
data.index.sigma     = (1:data.theta_n) + data.index.x(end);
data.index.eta       = (1:data.theta_n) + data.index.sigma(end);
data.index.theta_hat = (1:data.theta_n) + data.index.eta(end);
data.index.tau       = 1 + data.index.theta_hat(end);
data.index.tau_set   = 1 + data.index.tau;

tmp.O =  obsv(data.A, data.C);
data.theta_real = (data.C * data.A * data.B)^(-1) * [-data.C * data.A^2 * pinv(tmp.O),1];

clear('tmp', 'reference');

%% Utility Functions

function [A, B] = FilterMatrices(ics)
  dim = length(ics);
  
  A = [zeros(dim, 1) eye(dim)];
  A = A(:,1:end-1);
  B = zeros(dim, 1);
  B(end) = 1;
end

function ref_out = ReferenceGeneration(ref_in, n)
  ref_out = sym('ref%d', [n, 1]);
  t = sym('t');
  for i = 1:(n)
    ref_ins = diff(ref_in, t, i - 1);
    ref_out = subs(ref_out, ref_out(i), ref_ins);  
  end
end