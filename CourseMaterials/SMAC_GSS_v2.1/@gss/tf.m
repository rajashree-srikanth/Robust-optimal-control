%%
% GSS/TF       Construct a gss object from numerator and denominator
% ------------------------------------------------------------------
%
% Overloaded TF function for gss objects.
%
% CALL
% sys=tf(num,den{,st})
%
% INPUT ARGUMENTS
% - num,den: row vectors listing  the numerator and  the denominator
%            coefficients of  a SISO transfer function in descending
%            powers of s or z. each coefficient can be a gss object.
% - st: sample time in seconds.  set st=-1 to create a discrete-time
%       gss object with  unspecified sample time or st=0 to create a
%       continuous-time gss object. the default value is st=0.
%
% OUTPUT ARGUMENT
% - sys: gss object  representing the  continuous- or  discrete-time
%        transfer function of numerator num and denominator den.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
