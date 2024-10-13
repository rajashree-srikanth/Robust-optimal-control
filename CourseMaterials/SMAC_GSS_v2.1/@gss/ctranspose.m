%%
% GSS/CTRANSPOSE                 Conjugate transpose of a gss object
% ------------------------------------------------------------------
%
% Overloaded CTRANSPOSE function  for gss objects.  This routine can
% only be applied  when Delta is composed of real parametric blocks.
% In this case, the state-space representation  (a,b,c,d) of the gss
% object is transformed into (-a',-c',b',d'), while the Delta opera-
% tor remains unchanged.
%
% CALL
% sys=ctranspose(sys1)
% sys=sys1'
%
% INPUT ARGUMENT
% - sys1: gss object.
%
% OUTPUT ARGUMENT
% - sys: gss object representing sys1'.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
