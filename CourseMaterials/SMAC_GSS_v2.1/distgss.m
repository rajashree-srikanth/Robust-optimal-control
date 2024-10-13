%%
% DISTGSS               Compute the distance between two gss objects
% ------------------------------------------------------------------
%
% This function evaluates the distance  between two gss objects. Two
% sets of n samples corresponding to random values of both Delta and
% the frequency are first  computed. They are stacked in two noxnixn
% matrices  M1 and M2, where no and ni denote the  number of outputs
% and inputs of the gss  objects respectively. Several distances can
% then be defined for each entry:
% - maximum relative error
%   => mrerr(i,j)=max_k(dM(i,j,k)./mM(i,j,k))
%      dist=max_i,j(mrerr(i,j))
% - global relative error
%   => grerr(i,j)=norm(dM(i,j,:))/norm(mM(i,j,:))
% - maximum absolute error
%   => maerr(i,j)=max_k(dM(i,j,k))
% where dM=abs(M1-M2) and mM=max(abs(M1),abs(M2)). If dist~0, it can
% be concluded  that the two gss objects are  equivalent from an I/O
% perspective.
%
% WARNING
% mM(i,j,k) can sometimes be very close to zero for some values of i
% j and k. This results in large values of mrerr(i,j) and dist, even
% if the two gss  objects are almost equivalent. In this case, it is
% worth considering grerr and maerr instead.
%
% CALL
% [dist,mrerr,grerr,maerr]=distgss(sys1,sys2,npts,freq)
%
% INPUT ARGUMENTS
% - sys1,sys2: gss objects.
% - n: number of samples (def=200).
% - freq: frequency interval in rad/s (def=computed according to the
%         systems poles).
%
% OUTPUT ARGUMENTS
% - dist: maximum relative error over all entries = max(max(mrerr)).
% - mrerr: maximum relative error entry by entry (noxni array).
% - grerr: global relative error entry by entry (noxni array).
% - maerr: maximum absolute error entry by entry (noxni array).
%
% NOTE
% At least one input argument must be a gss object. The other can be
% a matrix or a ss/tf/lfr/uss/gss object.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
