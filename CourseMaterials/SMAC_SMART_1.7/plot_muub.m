%%
% PLOT_MUUB      Plot a (skew-)mu upper bound on frequency intervals
% ------------------------------------------------------------------
%
% This function plots a (skew-)mu upper bound  on a set of frequency
% intervals.
%
% CALL
% plot_muub(tab{,type});
% plot_muub(tab{,wmax});
% plot_muub(tab{,type,wmax});
%
% INPUT ARGUMENTS
% - tab is a structured variable with fields:
%   * ubnd: (skew-)mu upper bounds (Nx1 array).
%   * int: freq intervals on which the bounds are valid (Nx2 array).
%   it is usually obtained with muub_mixed.m.
% - type: x-axis type ('lin'<>linear,'log'<>logarithmic,def='lin').
% - wmax: x-axis upper limit (def=max(max(tab.int))).
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/plot_muub for more info!
