%%
% CHECK_STAB                   Check whether an LTI system is stable
% ------------------------------------------------------------------
%
% This function checks  whether an LTI system is stable in the sense
% that all its eigenvalues are inside a given truncated sector.
%
% CALL
% [stable,w]=check_stab(sys{,sector});
%
% INPUT ARGUMENTS
% - sys: lti object or square  matrix whose stability has to be tes-
%        ted. a vector of eigenvalues can also be directly given.
% - sector: truncated sector  [alpha xi ws].  alpha is the  relative
%           stability degree. xi is the  damping factor  (if ws=[]).
%           ws is optional and corresponds to  the imaginary part of
%           the intersection point  between the two segments defined
%           by alpha and xi (def=[0 0]). see also display_sector.m.
%
% OUTPUT ARGUMENTS
% - stable: indicates whether sys is stable (1) or not (0).
% - w: imaginary part of the most unstable eigenvalue.
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/check_stab for more info!
