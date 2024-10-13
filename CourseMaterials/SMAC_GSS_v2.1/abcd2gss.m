%%
% ABCD2GSS            From static gss objects to dynamic gss objects
% ------------------------------------------------------------------
%
% This function  converts a  static gss  object representing  state-
% space matrices [a b;c d] into a continuous-time or a discrete-time
% dynamic gss  object representing the nth order system dx/dt=ax+bu,
% y=cx+du or x(n+1)=ax+bu, y=cx+du. The conversion is performed with
% the  overloaded feedback  function. The order of the resulting gss
% object is then reduced with the  function mingss, unless the func-
% tion setred is used to change this setting.
%
% CALL
% sys=abcd2gss(abcd,n{,st})
%
% INPUT ARGUMENTS
% - abcd: static gss object  with inputs [x;u]  and output [dx/dt;y]
%         representing state-space matrices [a b;c d].
% - n: number of states, i.e. dimension of vector x.
% - st: sample time in seconds.  set st=-1 to create a discrete-time
%       gss object with  unspecified sample time or st=0 to create a
%       continuous-time gss object. the default value is st=0.
%
% OUTPUT ARGUMENT
% - sys: dynamic gss object with inputs u and outputs y representing
%        the system dx/dt=ax+bu, y=cx+du or x(n+1)=ax+bu, y=cx+du.
%
% EXAMPLE
%
% 1. Realization using abcd2gss 
% p=gss('p');
% q=gss('q');
% abcd=[-2+p q 2;3*p*q -1 p+q;0 p q^2];
% sys1=abcd2gss(abcd,2);
%
% 2. Direct realization
% sys2=ss([-2+p q;3*p*q -1],[2;p+q],[0 p],q^2);
%
% 3. Comparison
% distgss(sys1,sys2)
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
