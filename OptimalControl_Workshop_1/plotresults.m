function h1 = plotresults(trajt, trajX, trajU)
%%
% PLOTRESULTS plots the results from the Optimal Control workshop.
% 
% H1 = PLOTRESULTS(TRAJT,TRJAX,TRAJU) plots the results of an optimally 
% controlled trajectory. 
%       - TRAJT is the time vector of size 1 x N, with N the number of
%         points along the trajectory.
%       - TRAJX is the state matrix of size 4 X N containing the state at
%         each time in TRAJT.
%       - TRAJU is the command matrix of size P X N (with P = 1 or 2)
%         containing the optimal control at each time in TRAJT.
% This routine returns the handle to the generated figure.

%--------------------------------------------------------------------------
% Preliminary checks
%--------------------------------------------------------------------------
% Number of inputs
narginchk(3,3);
% Dimension consistency
if(~ismatrix(trajX))
   error('TRAJX must be a matrix.'); 
end
if(~isvector(trajt))
   error('TRAJT must be a vector.'); 
end
if(size(trajX, 1) > size(trajX, 2) || size(trajX, 1) ~= 4)
   error('TRAJX must be of size 4xN.');
end

%--------------------------------------------------------------------------
% Define figure, position and size
%--------------------------------------------------------------------------
h1 = figure;
set(h1, 'Position', [680 540 560*2 420*2]);

%--------------------------------------------------------------------------
% Evolution of the state
%--------------------------------------------------------------------------
subplot(2,2,1); 
plot(trajt, trajX'*diag([1,1,100,100])); 
grid on; 
title('States trajectory');
legend('z (m)','x (m)','zdot (cm/s)','xdot (cm/s)', 'Location', 'best');
xlabel('Time (s)'); 
ylabel('States X'); 

%--------------------------------------------------------------------------
% Command
%--------------------------------------------------------------------------
subplot(2,2,2); 
plot(trajt, trajU); 
grid on; 
title('Control trajectory (m/s^2)');
legend('\phi_z','\phi_x', 'Location', 'best');
xlabel('Time (s)'); 
ylabel('Control u'); 

%--------------------------------------------------------------------------
% Trajectory in orbital plane
%--------------------------------------------------------------------------
subplot(2,2,4); 
plot(trajX(1,:),trajX(2,:)); 
hold on;
axis([-1100 1100 -2300 2200]); 
plot([0 500],[0 0]); plot(500,0,'>'); 
text(600,0,'Z');
plot([0 0],[0 500]); plot(0,500,'^'); 
text(0,600,'X');
title('Trajectory of M in the local orbital plane')
xlabel('Z-axis (m)'); 
ylabel('X-axis (m)'); 
grid on
