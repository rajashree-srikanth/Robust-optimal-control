%%
% DBNORM                                      Normalize a gss object
% ------------------------------------------------------------------
%
% This function normalizes PAR and  LTI blocks of a gss object sys1.
% The resulting gss object sys is such that:
% - the field Bounds of real PAR blocks is equal to [-1 1],
% - the field Bounds of complex PAR blocks is equal to [0 0 1],
% - the field Bounds of LTI blocks is equal to 1.
% For each normalized block:
% - the field NomValue  is computed so that  eval(sys,'nominal') and
%   eval(sys1,'nominal') are  identical from an I/O  point of view ;
%   only linear transformations are performed, so it is not equal to
%   0 after  normalization if the  nominal value is not  centered in
%   the initial object sys1.
% - the field Distribution is adapted only in case of a PAR block if
%   Distribution.Type='uniform' or 'normal',
% - the field RateBounds is adapted only in case of a PAR block,
% - the field Normalization  contains all the  necessary information
%   to unnormalize the block with dbunorm,
% - the other fields are left unchanged.
%
% CALL
% sys = dbnorm(sys1{,names})
%
% INPUT ARGUMENTS
% - sys1: gss object.
% - names: cell array of strings  with the names of the Delta blocks
%          to be normalized  (by default, all blocks of type PAR and
%          LTI are normalized). block names with trailing underscore
%          are not allowed.
%
% OUTPUT ARGUMENT
% - sys: normalized gss object.
%
% WARNING
% Normalization is only possible for blocks of type PAR and LTI.
%
% SEE ALSO
%  dbunorm
%
% EXAMPLE
%
% 1. Creation of a gss object
% a=gss('a',5,[2 6],{'uniform' [0 8]},[-2 4]);
% b=gss('b',3+4i,[2 3 2],{'uniform' [1 2 3]});
% c=gss('c','LTI',[1 1],tf(5,[1 1]),tf(10,[1 1]));
% sys1=(1/a+b)*c;
% size(sys1)
%
% 2. Normalization
% sysn=dbnorm(sys1);
% size(sysn)
% s1=eval(sys1,'nominal');
% sn=eval(sysn,'nominal');
% norm(s1-sn,inf)
%
% 3. Unnormalization
% sys2=dbunorm(sysn);
% size(sys2)
% distgss(sys1,sys2)
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
