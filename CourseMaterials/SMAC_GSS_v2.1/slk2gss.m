%%
% SLK2GSS                From Simulink block diagrams to gss objects
% ------------------------------------------------------------------
%
% This function converts any Simulink block diagram describing a li-
% near interconnection  of gss objects into a single gss object. The
% Simulink diagram must be built using the dedicated library GSSLIB.
% The order  of the  resulting  gss object is  then reduced with the
% function mingss, unless the function setred is used to change this
% setting.
%
% CALL
% sys=slk2gss(filename)
%
% INPUT ARGUMENT
% - filename: name of the Simulink file without extension (character
%             array).
%        
% OUTPUT ARGUMENT
% - sys: gss object.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
