%%
% GSS/SET                     Change some properties of a gss object
% ------------------------------------------------------------------
%
% Overloaded SET function  for gss objects.  This function allows to
% modify one or several properties of a selected Delta block.
%
% CALL
% sys=set(sys,name,field1,value1,...,fieldn,valuen)
% set(sys,name,field1,value1,...,fieldn,valuen)
%
% INPUT ARGUMENTS
% - sys: gss object.
% - name: block name.
% - fieldk: property to be modified ('Name','Type','Size','NomValue'
%           'Bounds', 'Distribution', 'RateBounds', 'Normalization',
%           'Misc').
% - valuek: corresponding new value.
%
% OUTPUT ARGUMENT
% - sys: gss object (optional).
%
% WARNING
% The consistency of the  requested changes is not  checked. Use the
% function checkgss to determine  whether the modified gss object is
% defined correctly.
%
% REMARK
% To change the  value of the field Distribution, a string  with the
% distribution type or a 1x2 cell  array with  the distribution type
% and the corresponding parameters can be passed to the function set
% instead of a structure array  with fields Type and Parameters. For
% example in the call set(sys,name,'Distribution',dis) it is equiva-
% lent to define dis={'normal' [0 1]} or dis=struct('Type','normal',
% 'Parameters',[0 1]) for a normal distribution of mean 0 and var 1.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
