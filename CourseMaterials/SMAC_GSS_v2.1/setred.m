%%
% SETRED                    Set preference for gss objects reduction
% ------------------------------------------------------------------
%
% This function sets the preference for gss objects order reduction:
%  1. systematic call to mingss with a specified tolerance each time
%     a gss object is created or an elementary  operation is applied
%     to an existing gss object (addition, multiplication, division,
%     concatenation...),
%  2. no order reduction.
% The default choice is a systematic call to mingss with a tolerance
% of 1e-8.
%
% CALL
% setred(tol)         systematic order reduction with tolerance tol
% setred('default')   systematic order reduction with tolerance 1e-8
% setred('no')        no order reduction
% setred('info')      display current preference
%
% INPUT ARGUMENT
% - tol: tolerance  for order  reduction (corresponds  to the second
%        input argument of mingss).
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
