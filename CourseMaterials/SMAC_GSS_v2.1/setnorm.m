%%
% SETNORM               Set preference for gss objects normalization
% ------------------------------------------------------------------
%
% This function sets the preference for gss objects normalization:
%  1. systematic call to dbnorm to normalize all the Delta blocks of
%     type PAR and LTI
%  2. call to dbnorm only in case of  inversion problem (for example
%     1/a with 1<a<3 cannot be realized but 1/(2+a) with -1<a<1 can)
%     or to ensure Delta blocks consistency (for example if both the
%     normalized & the unnormalized versions of the same Delta block
%     appear in the same gss object)
%  3. no normalization,  an error occurs in case of inversion or in-
%     consistency problem
% The default choice is to perform normalization only in case of in-
% version or inconsistency problem.
%
% CALL
% setnorm('yes')       systematic normalization
% setnorm('default')   normalization if inversion/inconsistency prob
% setnorm('no')        no normalization
% setnorm('info')      display current preference
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
