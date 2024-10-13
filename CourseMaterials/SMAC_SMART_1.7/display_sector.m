%%
% DISPLAY_SECTOR                          Display a truncated sector
% ------------------------------------------------------------------
%
% This function  displays a truncated sector (in red) in the complex
% plane.  If requested  by the user, the  eigenvalues of a given LTI
% system can be displayed too.
%
% CALL
% display_sector(sector{,sys});
%
% INPUT ARGUMENTS
% - sector: truncated sector  [alpha xi ws].  alpha is the  relative
%           stability degree. xi is the  damping factor  (if ws=[]).
%           ws is optional and corresponds to  the imaginary part of
%           the intersection point  between the two segments defined
%           by alpha and xi.
% - sys: lti object  or square  matrix whose  eigenvalues are  to be
%        plotted. a list of  eigenvalues can also be  directly given
%        (optional argument, def=[]).
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/display_sector for more info!
