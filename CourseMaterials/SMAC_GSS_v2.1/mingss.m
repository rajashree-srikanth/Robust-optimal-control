%%
% MINGSS                            Reduce the order of a gss object
% ------------------------------------------------------------------
%
% This function calculates a minimal multi-D realization by elimina-
% ting unreachable/unobservable  subspaces. It  implements a  method
% based on n-D Kalman decomposition and developed by R. D'Andrea and
% S. Khatri (ACC 1997, pp 3557-3461).
%
% CALL
% sysr=mingss(sys{,tol})
%
% WARNING
% By default, mingss is used to perform a systematic order reduction
% each time a gss  object is created  or an elementary  operation is
% applied to an existing gss object (addition, multiplication, divi-
% sion,  concatenation...). This  setting can be  changed using  the
% function setred.
%
% INPUT ARGUMENTS
% - sys: gss object.
% - tol: optional tolerance  (default=1e-8). it should  sometimes be
%        increased as the default value is often too small.
%
% OUTPUT ARGUMENT
% - sysr: reduced gss object.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
