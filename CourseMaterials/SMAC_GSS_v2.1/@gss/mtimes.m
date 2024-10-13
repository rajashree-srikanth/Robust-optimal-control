%%
% GSS/MTIMES                       Multiplication of two gss objects
% ------------------------------------------------------------------
%
% Overloaded MTIMES function for gss objects.
%
% sys1*sys2   <=>   --->| sys2 |--->| sys1 |--->
%
% CALL
% sys=mtimes(sys1,sys2)
% sys=sys1*sys2
%
% INPUT ARGUMENTS
% - sys1,sys2: gss objects.
%
% OUTPUT ARGUMENT
% - sys: gss object representing sys1*sys2.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
