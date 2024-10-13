%%
% GSS/VERTCAT                  Vertical concatenation of gss objects
% ------------------------------------------------------------------
%
% Overloaded VERTCAT function for gss objects.
%
%                     --->| sys1 |---\
% [sys1 sys2]   <=>                   +--->
%                     --->| sys2 |---/
%
% CALL
% sys=vertcat(sys1,sys2,...,sysN)
% sys=[sys1;sys2;...;sysN]
%
% INPUT ARGUMENTS
% - sys1,...,sysN: gss objects.
%
% OUTPUT ARGUMENT
% - sys: gss object representing [sys1;sys2;...;sysN].
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
