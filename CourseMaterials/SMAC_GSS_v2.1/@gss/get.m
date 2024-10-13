%%
% GSS/GET                     Obtain some properties of a gss object
% ------------------------------------------------------------------
%
% Overloaded GET function  for gss objects.  This function allows to
% obtain one or several properties of a selected Delta block.
%
% CALL
% values=get(sys,name)
% value=get(sys,name,field)
%
% INPUT ARGUMENTS
% - sys: gss object.
% - name: block name.
% - field: requested  property  ('Name', 'Type', 'Size', 'NomValue',
%          'Bounds', 'Distribution',  'RateBounds', 'Normalization',
%          'Misc').
%
% OUTPUT ARGUMENT
% - values: structured array with all the properties of the conside-
%           red block.
% - value: requested property of the considered block.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
