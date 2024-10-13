%%
% GSS/NE                      Check if two gss objects are not equal
% ------------------------------------------------------------------
%
% Overloaded NE function  for gss objects.  Two gss objects  are not
% equal if the distance computed with distgss is more than 1e-10.
%
% CALL
% result=ne(sys1,sys2)
% result=(sys1~=sys2)
%
% INPUT ARGUMENTS
% - sys1,sys2: gss objects.
%
% OUTPUT ARGUMENT
% - result: true if sys1 and sys2 are not equal, false otherwise.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
