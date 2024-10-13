%%
% GSS/SS            Construct a gss object from state-space matrices
% ------------------------------------------------------------------
%
% Overloaded SS function for gss objects.
%
% CALL
% sys=ss(a,b,c,d{,st})
%
% INPUT ARGUMENTS
% - a,b,c,d: state-space matrices of compatible dimensions. each ma-
%            trix can be a gss object.
% - st: sample time in seconds.  set st=-1 to create a discrete-time
%       gss object with  unspecified sample time or st=0 to create a
%       continuous-time gss object. the default value is st=0.
%
% OUTPUT ARGUMENT
% - sys: gss object  representing  the continuous- or  discrete-time
%        system  dx/dt=ax+bu, y=cx+du or x(n+1)=ax+bu, y=cx+du.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
