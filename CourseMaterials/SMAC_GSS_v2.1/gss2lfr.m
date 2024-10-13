%%
% GSS2LFR                       Convert gss objects into lfr objects
% ------------------------------------------------------------------
%
% This function  converts a gss object into an lfr object compatible
% with the LFR Toolbox. 
%
% CALL
% lfrsys = gss2lfr(sys{,warn})
%
% INPUT ARGUMENTS
% - sys: gss object.
% - warn: warnings display (1=>yes, 0=>no, default=1).
%
% OUTPUT ARGUMENT
% - lfrsys: lfr object.
%
% CAUTION
% - the sample time is ignored if sys is a discrete-time gss object.
% - polytopic blocks (Type=POL) are not allowed.
% - the fields NomValue and Bounds are sometimes ignored, or automa-
%   tically fixed if undefined, depending on the block type.
% - the field RateBounds  is set to [-Inf Inf] if nonzero, since lfr
%   objects only handle TI and infinitely fast TV blocks.
% - the fields Distribution and Misc are always ignored.
%
% SEE ALSO
%  gss, lfr2gss
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
