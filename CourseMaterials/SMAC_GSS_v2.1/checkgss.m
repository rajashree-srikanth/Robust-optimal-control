%%
% CHECKGSS                  Check whether a gss object is consistent
% ------------------------------------------------------------------
%
% This function checks whether a gss object is defined correctly and
% provides information about  possible inconsistencies. It is useful
% when errors are  encountered after a gss object  has been modified
% without using the function gss.
%
% CALL
% pb = checkgss(sys)
%
% INPUT ARGUMENT
% - sys: gss object.
%
% OUTPUT ARGUMENT
% - pb: boolean equal to 0 if sys is consistent, and to 1 otherwise.
%       if checkgss is called with  no output argument,  the list of
%       inconsistencies is displayed on the screen.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
