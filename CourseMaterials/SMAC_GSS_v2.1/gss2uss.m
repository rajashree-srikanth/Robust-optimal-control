%%
% GSS2USS                       Convert gss objects into uss objects
% ------------------------------------------------------------------
%
% This function  converts a  gss object into a uss object compatible
% with the Robust Control Toolbox. 
%
% CALL
% usys = gss2uss(sys{,warn})
%
% INPUT ARGUMENT
% - sys: gss object.
% - warn: warnings display (1=>yes, 0=>no, default=1).
%
% OUTPUT ARGUMENT
% - usys: uss object.
%
% CAUTION
% - polytopic blocks (Type=POL) are not allowed.
% - the fields NomValue and Bounds are sometimes ignored, or automa-
%   tically fixed if undefined, depending on the block type.
% - the fields  Distribution, RateBounds, Normalization and Misc are
%   always ignored.
%
% SEE ALSO
%  gss, uss2gss
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
