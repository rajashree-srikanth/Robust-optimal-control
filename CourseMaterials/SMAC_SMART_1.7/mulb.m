%%
% MULB                                Compute (skew-)mu lower bounds
% ------------------------------------------------------------------
%
% This function computes (skew-)mu lower bounds  for an interconnec-
% tion between an LTI  system P and a block-diagonal operator Delta.
% Destabilizing values of Delta are also obtained and the considered
% stability region can be bounded either by the imaginary axis or by
% a truncated sector.
%
% If P has exogenous  inputs e and  outputs y,  a robust performance
% problem is  considered: lower  bounds are computed  on the highest
% Hinfinity norm of  the transfer function Fu(P,Delta) between e and
% y when Delta takes any admissible value.
%
%               -------                       -------
%          +---| Delta |<--+             +---| Delta |<--+
%          |    -------    |             |    -------    |
%          |               |             |               |
%          |    -------    |             |    -------    |
%          |   |       |   |             +-->|       |---+
%          +-->|   P   |---+                 |   P   |
%              |       |               e --->|       |---> y
%               -------                       -------
%
%          (skew-)mu problem            performance problem
%
% The initial interconnection is NORMALIZED  with convert_data.m be-
% fore the problem is solved.
%
% CALL
% [lbnd,wc,pert,iodesc]=mulb(usys{,skew/perf,options});
% [lbnd,wc,pert,iodesc]=mulb(gsys{,skew/perf,options});
% [lbnd,wc,pert,iodesc]=mulb(lsys{,skew/perf,options});
% [lbnd,wc,pert,iodesc]=mulb(sys,blk{,skew/perf,options});
%
% INPUT ARGUMENTS
% The interconnection between P and Delta can be described by:
% - a uss object usys
% - a gss object gsys
% - an lfr object lsys
% - an lti object sys  describing the  LTI system P and an nblk-by-2
%   matrix blk defining the structure of the block-diagonal operator
%   Delta=diag(Delta_1,...,Delta_nblk):
%   * blk(i,:)=[-ni 0] => Delta_i=di*eye(ni) with di real
%   * blk(i,:)=[ni 0]  => Delta_i=di*eye(ni) with di complex
%   * blk(i,:)=[ni mi] => Delta_i is a ni-by-mi LTI system
% For a skew-mu problem, an additional argument skew can be defined:
% * skew(i)=0 => Delta_i must remain below user-defined bound
% * skew(i)=1 => the size of Delta_i must be minimized
% If skew is undefined,  skew(i) is set to 1 for all i,  which means
% that a classical mu problem is considered.
% For a performance problem,  an additional argument perf can be de-
% fined to specify how  the transfer between e  and y is structured.
% Each line of  perf corresponds to a performance channel. For exam-
% ple, if perf=[2 1;1 3], the transfer between inputs 1-2 and output
% 1 is considered independently  of the one between input 3 and out-
% puts 2-3-4.  If perf is  undefined, it is set to [ne ny], where ne
% and ny denote the size of the signals e and y.
%
% The variable options is an optional structure with fields:
% * freq: frequency interval in rad/s on which the lower bound is to
%         be computed (def=[0 10*max(abs(eig(P)))]).
% * grid: frequency grid  in rad/s  (1xN array). it  can also be a
%         negative integer -N, in which case a N-point grid is au-
%         tomatically  generated (def=-10  if all  uncertainties are
%         real and -50 otherwise).
% * sector: truncated sector  [alpha xi ws].  alpha is the  relative
%           stability degree. xi is the  damping factor  (if ws=[]).
%           ws is optional and corresponds to  the imaginary part of
%           the intersection point  between the two segments defined
%           by alpha and xi (def=[0 0]). see also display_sector.m.
% * target: the algorithm  is interrupted if  a lower bound is found
%           which is greater than target (def=Inf).
% * trace: trace of execution (def=1).
% * warn: warnings display (def=1).
% Additional  fields can be defined  (see mulb_mixed.m, mulb_1real.m
% mulb_nreal.m  and hinflb_real.m), but  the ones  listed above  are
% usually sufficient for non-expert users.
%        
% OUTPUT ARGUMENTS
% - lbnd: (skew-)mu lower bound  for the NORMALIZED  interconnection
%         ((skew-)mu problem), or lower bound on the worst-case Hin-
%         finity norm for each transfer (performance problem).
% - wc: frequency in rad/s for which lbnd has been computed.
% - pert: associated  value of  Delta. unless  lbnd=0, at  least one
%         eigenvalue of the NORMALIZED interconnection between P and
%         pert is unstable ((skew-)mu  problem), or the largest sin-
%         gular value  of Fu(P,pert) at frequency wc (or at the cor-
%         responding point on  the boundary of the truncated  sector
%         defined  by options.sector)  is equal to lbnd (performance
%         problem).
% - iodesc: same as pert, but for the INITIAL interconnection.
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/mulb for more info!
