%%
% GSS/TRANSPOSE                            Transpose of a gss object
% ------------------------------------------------------------------
%
% Overloaded TRANSPOSE  function for gss  objects. This  routine can
% only be  applied when  Delta is composed of  parametric blocks. In
% this case,  the state-space  representation  (a,b,c,d) of  the gss
% object is transformed into (a.',c.',b.',d.'), while the Delta ope-
% rator remains unchanged.
%
% CALL
% sys=transpose(sys1)
% sys=sys1.'
%
% INPUT ARGUMENT
% - sys1: gss object.
%
% OUTPUT ARGUMENT
% - sys: gss object representing sys1.'.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
