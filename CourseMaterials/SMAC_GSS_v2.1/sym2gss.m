%%
% SYM2GSS     Create low-order gss objects from symbolic expressions
% ------------------------------------------------------------------
%
% This function computes a  gss object  from a polynomial  matrix in
% symbolic  form using the tree decomposition approach  described in
% J.C. Cockburn and  B.G. Morton, Automatica, 33(7):1263-1271, 1997.
% The order  of the resulting  gss object  is further  reduced  with
% mingss, unless the function setred is used to change this setting.
%
% CALL
% sys=sym2gss(symbex{,reorder,trace})
%
% INPUT ARGUMENTS
% - symbex: symbolic polynomial matrix.  the reserved names  for 1/s
%           and 1/z are  Int and Delay respectively.  variable names
%           with trailing underscore are not allowed.
% - reorder: cell of  strings of  length less than  or equal  to the
%            number of  symbolic variables  in  symbex.  each string
%            must contain  the name of a variable.  this argument is
%            used to reorder the variables  during the tree decompo-
%            sition process. it can be important in certain cases to
%            obtain lower order gss objects.
% - trace: trace of execution(1=yes,0=no,default=0).
%
% OUTPUT ARGUMENT
% - sys: gss object.
%
% EXAMPLE
%
% setred('no')      % systematic reduction with mingss is turned off
%
% 1. Direct realization
% Int=gss('Int');
% x=gss('x');
% y=gss('y');
% z=gss('z');
% sys1=[ Int*x^2*z^3+3*x*y*z   x^2*y^2*z^3          1        ;...
%                 0           Int^2*x^3*y*z  x*y^2*z-x^2*y*z ];
% size(sys1)
% sys1=mingss(sys1);
% size(sys1)
% 
% 2. Realization using an elementary structured tree decomposition
% syms Int x y z
% symbex=[ Int*x^2*z^3+3*x*y*z   x^2*y^2*z^3          1        ;...
%                   0           Int^2*x^3*y*z  x*y^2*z-x^2*y*z ];
% sys2=sym2gss(symbex);
% size(sys2)
% sys2=mingss(sys2);
% size(sys2)
% 
% 3. Same as before but with parameters reordering
% sys3=sym2gss(symbex,{'z','y','x'});
% size(sys3)
% 
% 4. Comparison
% distgss(sys1,sys2)
% distgss(sys2,sys3)
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
