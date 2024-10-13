%%
% DBUNORM                                   Unnormalize a gss object
% ------------------------------------------------------------------
%
% This function unnormalizes PAR and LTI blocks of a gss object sys1
% For each unnormalized block of the resulting gss object sys:
% - the field Bounds is derived from the  fields Bounds and Normali-
%   zation of the corresponding normalized block in sys1,
% - the field NomValue  is computed so that  eval(sys,'nominal') and
%   eval(sys1,'nominal') are identical from an I/O point of view.
% For example, consider a real PAR block with NomValue=5 and Bounds=
% [2 6]. After normalization  (using dbnorm), NomValue=0.5,  Bounds=
% [-1 1] and Normalization=[2 6].
% - If these values are not modified, then 5 and [2 6] are recovered
%   after unnormalization (using dbunorm).
% - If they are modified, e.g. NomValue=-0.5 and Bounds=[-2 3], then
%   3 and [0 10] are obtained after unnormalization.
% Moreover, for each unnormalized block of sys:
% - the field Distribution is set to  its value before normalization
%   in case of a PAR block if Distribution.Type='uniform'/'normal',
% - the field RateBounds is set to its value before normalization in
%   case of a PAR block,
% - the field Normalization is set to [],
% - the other fields are left unchanged.
%
% CALL
% sys = dbunorm(sys1{,names})
%
% INPUT ARGUMENTS
% - sys1: gss object.
% - names: cell array of strings  with the names of the Delta blocks
%          to be  unnormalized (by default,  all blocks of  type PAR
%          and LTI are unnormalized).  block names with trailing un-
%          derscore are not allowed.
%
% OUTPUT ARGUMENT
% - sys: unnormalized gss object.
%
% WARNING
% Unnormalization is only possible for blocks of type PAR and LTI.
%
% SEE ALSO
%  dbnorm
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
