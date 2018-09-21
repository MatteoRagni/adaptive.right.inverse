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
%   HyEQ Toolbox (Copyright @ Hybrid Systems Laboratory - HSL)

%% The script runs at the End of the simulation (it is the stopFnc)! %%

close('all');

sim_s = SimulationDataToStruct(sim, tout, data);
PlotFigure1(sim_s, 'img/figure1.png');
PlotFigure2(sim_s, data, 'img/figure2.png');

%% Util Functions

function [sim_s] = SimulationDataToStruct(sim_a, t, data)
  sim_s = struct;
  
  sim_s.t         = t;
  sim_s.x         = sim_a(:, data.index.x);
  sim_s.sigma     = sim_a(:, data.index.sigma);
  sim_s.eta       = sim_a(:, data.index.eta);
  sim_s.theta_hat = sim_a(:, data.index.theta_hat);
  sim_s.tau       = sim_a(:, data.index.tau);
  sim_s.Yd        = sim_a(:, (data.index.tau_set + 1):end);
  sim_s.r         = sim_s.Yd(:, 1);
  sim_s.y         = sim_s.x * data.C';
  
  sim_s.theta_real = ones(size(sim_s.theta_hat));
  for i = 1:length(data.theta_real)
    sim_s.theta_real(:,i) = sim_s.theta_real(:,i) * data.theta_real(i);
  end
end

function f = PlotFigure1(sim, out_file)
  f = figure();
  
  f.Color = 'white';
    ax = subplot(3,1,1:2);
      plot(sim.t, sim.y, 'k', sim.t, sim.r, 'b--');
      legend({'$y$','$y_\mathrm{d}$'}, 'Interpreter', 'latex');
      ylabel('$y$', 'Interpreter', 'latex');
      grid('on');
    bx = subplot(3,1,3);
      plot(sim.t, sim.y - sim.r, 'k');
      xlabel('$t$', 'Interpreter', 'latex');
      ylabel('$y - y_\mathrm{d}$', 'Interpreter', 'latex');
      grid('on');
    linkaxes([ax, bx], 'x'); 
    
  saveas(f, out_file);
end

function f = PlotFigure2(sim, data, out_file)
  f = figure();
  color = ColorList(data.theta_n);
  legend_cell = cell(data.theta_n * 2, 1);
  
  f.Color = 'white';
    ax = subplot(3,1,1:2);
      for i = 1:data.theta_n
        plot(sim.t, sim.theta_hat(:,i), 'Color', color(i,:));
        hold('on');
        plot(sim.t, sim.theta_real(:,i), 'Color', color(i,:), 'LineStyle', '--');
        legend_cell{2*i - 1} = sprintf('$\\hat{\\theta}_{%d}$', i);
        legend_cell{2*i} = sprintf('$\\theta_{%d}$', i);
      end
      legend(legend_cell, 'Interpreter', 'latex');
      ylabel('$\hat{\theta}_{i}$', 'Interpreter', 'latex');
      grid('on');
    bx = subplot(3,1,3);
      plot(sim.t, sim.tau, 'k');
      xlabel('$t$', 'Interpreter', 'latex');
      ylabel('$\tau$', 'Interpreter', 'latex');
      grid('on');
    linkaxes([ax, bx], 'x');  
   
  saveas(f, out_file);
end

function cl_rgb = ColorList(dim)
  cl_rgb = hsv2rgb([linspace(0, 0.75, dim)', ones(dim, 2)]);
end