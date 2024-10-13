%%
% GSS2GSS                      Convert gss objects from v1.x to v2.x
% ------------------------------------------------------------------
%
% This function converts a gss object  created with  the GSS Library
% v1.x into a  gss object compatible with the GSS Library v2.x.  The
% latter is then reduced with the function mingss,  unless the func-
% tion setred is used to change this setting.
%
% CALL
% sys = gss2gss(gsys{,dist})
% sys = gss2gss(gsys{,uall})
% sys = gss2gss(gsys{,dist,uall})
% The output argument sys can be replaced with 2 arguments [M,D].
%
% INPUT ARGUMENTS
% - gsys: gss object computed with the GSS Library v1.x.
% - dist: this optional kx3 cell array  allows to specify the proba-
%         bility  distribution for  1<=k<=N Delta  blocks.  each row
%         contains the name  of the considered Delta block, the type
%         of  distribution, and the associated numerical parameters.
%         for example dist={'a' 'uniform' [-2 2];'b' 'normal' [1 3]}
%         sets a  uniform distribution  on the interval [-2 2] for a
%         and a normal distribution of mean 1 and variance 3 for b.
% - uall: if nonzero a uniform distribution on the interval given by
%         Bounds is set for all parameters with unknown distribution
%         (default = 0).
%
% OUTPUT ARGUMENTS
% - sys: gss object compatible with the GSS Library v2.x.
% - M,D: fields of the gss object.
%
% SEE ALSO
%  gss
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
