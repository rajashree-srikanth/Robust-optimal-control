%%
% CALC_FREQ_RESP     Compute the frequency response of an LTI system
% ------------------------------------------------------------------
%
% This function computes the frequency response of a continuous-time
% LTI system on a frequency grid with respect to either the imagina-
% ry axis  or the boundary of  a truncated  sector. When  handling a
% sector, the  frequency response  associated to  the kth grid point
% grid(k) is computed  at the point sk of  the sector boundary which
% satisfies Im(sk)=grid(k).
%
% CALL
% M=calc_freq_resp(sys,grid{,sector});
%
% INPUT ARGUMENTS
% - sys: lti object.
% - grid: frequency grid in rad/s.
% - sector: truncated sector  [alpha xi ws].  alpha is the  relative
%           stability degree. xi is the  damping factor  (if ws=[]).
%           ws is optional and corresponds to  the imaginary part of
%           the intersection point  between the two segments defined
%           by alpha and xi (def=[0 0]). see also display_sector.m.
%
% OUTPUT ARGUMENT
% - M: 3-D array where M(:,:,k) is the frequency response associated
%      to grid(k).
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/calc_freq_resp for more info!
