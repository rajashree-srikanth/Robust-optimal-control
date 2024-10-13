%%
% GSS/MPOWER                        Repeated product of a gss object
% ------------------------------------------------------------------
%
% Overloaded MPOWER function for gss objects.
%
% sys1^2   <=>   --->| sys1 |--->| sys1 |--->
%
% CALL
% sys=mpower(sys1,k)
% sys=sys1^k
%
% INPUT ARGUMENTS
% - sys1: gss object.
% - k: positive or negative integer.
%
% OUTPUT ARGUMENT
% - sys: gss object representing sys^k.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
