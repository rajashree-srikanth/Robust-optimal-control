%%
% GSS/SUBSREF                  Subscripted reference for gss objects
% ------------------------------------------------------------------
%
% Overloaded SUBSREF function for gss objects.
%
% CALL
% sys=subsref(sys1,S)
% sysij=sys1(I,J)
% sysi=sys1(I)
% M=sys1.M
% dkl=sys1(I,J).M.d(k,l)
% D=sys1.D
% namek=sys1.D(k).Name
%
% INPUT ARGUMENTS
% - sys1: gss object.
% - S: structured array with fields:
%      type: string '()' or '.' specifying the subscript type.
%      subs: cell array or string containing the actual subscripts.
%
% OUTPUT ARGUMENT
% => sysij=sys1(I,J) corresponds to S.type/subs='()'/{I J}. sysij is
%    a gss object  obtained from sys1 by  selecting only  the inputs
%    and the outputs of indices J and I respectively.
% => sysi=sys1(I) corresponds to S.type/subs='()'/{I}. when sys1 has
%    a single input  or output, sysi  is a gss object  obtained from
%    sys1 by selecting only the inputs or the outputs of indices I.
% => M=sys1.M corresponds  to S.type/subs='.'/'M'. M is a  ss object
%    representing the interconnection plant of sys1.
% => dkl=sys1(I,J).M.d(k,l) corresponds to S(1).type/subs='()'/{I J}
%    S(2).type/subs='.'/'M',  S(3).type/subs='.'/'d'  and S(4).type/
%    subs='()'/{k l}. dkl is the coefficient on the kth line and the
%    lth column of the d matrix of M.
% => D=sys1.D corresponds  to S.type/subs='.'/'D'. D is a structured
%    array describing the Delta operator of sys1.
% => namek=sys1.D(k).Name  corresponds  to  S(1).type/subs='.'/'D' ,
%    S(2).type/subs='()'/{k} and S(3).type/subs='.'/'Name'. namek is
%    the name of the kth block in Delta.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
