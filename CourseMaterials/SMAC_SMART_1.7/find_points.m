%%
% FIND_POINTS      Find points on the boundary of a truncated sector
% ------------------------------------------------------------------
%
% This function determines the points on the boundary of a truncated
% sector,  whose imaginary parts  are equal to  the elements  in the
% vector w.
%
% CALL
% pts=find_points(w,sector);
%
% INPUT ARGUMENTS
% - w: vector of real numbers.
% - sector: truncated sector  [alpha xi ws].  alpha is the  relative
%           stability degree. xi is the  damping factor  (if ws=[]).
%           ws is optional and corresponds to  the imaginary part of
%           the intersection point  between the two segments defined
%           by alpha and xi (def=[0 0]). see also display_sector.m.
%
% OUTPUT ARGUMENT
% - pts: points on the boundary of sector, whose imaginary parts are
%        equal to the elements in w (vector with same size as w).
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/find_points for more info!
