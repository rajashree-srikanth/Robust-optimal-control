% Exercise on Hinf control design
close 
clear
%% Hinf Design
s = tf('s')
Gs = 1/(s^2-1)
% Additive Uncertainity
% Hinfnorm(Wa*K*(I+G*K)^-1) < 1

