%%
% GSS/EQ                          Check if two gss objects are equal
% ------------------------------------------------------------------
%
% Overloaded EQ function for gss objects.  Two gss objects are equal
% if the distance computed with distgss is less than 1e-10.
%
% CALL
% result=eq(sys1,sys2)
% result=(sys1==sys2)
%
% INPUT ARGUMENTS
% - sys1,sys2: gss objects.
%
% OUTPUT ARGUMENT
% - result: true if sys1 and sys2 are equal, false otherwise.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
