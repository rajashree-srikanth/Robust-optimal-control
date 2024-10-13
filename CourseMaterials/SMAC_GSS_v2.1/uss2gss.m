%%
% USS2GSS                       Convert uss objects into gss objects
% ------------------------------------------------------------------
%
% This function converts a ureal/ucomplex/ucomplexm/ultidyn/udyn/uss
% /umat object computed with  the Robust Control Toolbox  into a gss
% object. The latter is then reduced with the function mingss unless
% the function setred is used to change this setting.
%
% CALL
% sys   = uss2gss(usys{,Dinfo})
% [M,D] = uss2gss(usys{,Dinfo})
%
% INPUT ARGUMENTS
% - usys: ureal/ucomplex/ucomplexm/ultidyn/udyn/uss/umat object.
% - Dinfo: this optional 1xk structure array in gss format allows to
%          specify for 1<=k<=N Delta blocks some additional informa-
%          tion which is not available in lfrsys:
%          * the field Name is mandatory,
%          * the fields  Type, NomValue, Bounds, Distribution, Rate-
%            Bounds, Normalization and Misc are optional.
%          type 'help gss/description'  to find  out how to  specify
%          these fields.  note also that a shortcut can  be used for
%          Distribution, which can  be reduced to  a string with the
%          distribution type  or a 1x2 cell array with the distribu-
%          tion type and the corresponding parameters.
%          if  contradictory  information  is provided  by usys  and
%          Dinfo, the later prevails.
%
% OUTPUT ARGUMENTS
% - sys: gss object.
% - M,D: fields of the gss object.
%
% SEE ALSO
%  gss, gss2uss
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
