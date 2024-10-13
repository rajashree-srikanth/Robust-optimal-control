%%
% MUBB                Compute (skew-)mu bounds with desired accuracy
% ------------------------------------------------------------------
%
% This function  computes both (skew-)mu lower  and upper bounds for
% an interconnection  between an LTI  system P and a  block-diagonal
% operator Delta. The upper bound is GUARANTEED on a whole frequency
% interval and  satisfies  ubnd<(1+maxgap)*lbnd,  where maxgap  is a
% user-defined  threshold. Destabilizing  values of  Delta  are also
% obtained. The considered stability region can be bounded either by
% the imaginary axis or by a truncated sector.
%
% If P has exogenous  inputs e and  outputs y,  a robust performance
% problem is considered: GUARANTEED bounds are computed on the high-
% est Hinfinity norm of  the transfer function Fu(P,Delta) between e
% and y when Delta takes any admissible value.
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
% [bnds,wc,pert,iodesc]=mubb(usys{,skew/perf,options});
% [bnds,wc,pert,iodesc]=mubb(gsys{,skew/perf,options});
% [bnds,wc,pert,iodesc]=mubb(lsys{,skew/perf,options});
% [bnds,wc,pert,iodesc]=mubb(sys,blk{,skew/perf,options});
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
% * skew(i)=1 => the size of Delta_i is unconstrained
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
% * maxgap: maximum gap  between the bounds  (def=0.05, i.e. 5%). in
%           case of a  mu-test  (options.umax>0),  maxgap is used to
%           set options.tol in muub_mixed.
% * maxcut: maximum number  of times the real  parametric domain can
%           be bisected (def=100).
% * maxtime: maximum computational time (def=Inf).
% * freq: frequency interval  in rad/s on which the bounds are to be
%         computed (def=[0 10*max(abs(eig(P)))]).
% * sector: truncated sector  [alpha xi ws].  alpha is the  relative
%           stability degree. xi is the  damping factor  (if ws=[]).
%           ws is optional and corresponds to  the imaginary part of
%           the intersection point  between the two segments defined
%           by alpha and xi (def=[0 0]). see also display_sector.m.
% * trace: trace of execution (def=1).
% * warn: warnings display (def=1).
% * umax: if nonzero,  the algorithm  checks whether  ubnd<umax (mu-
%         test). in this case, no lower bound is computed (def=0).
% Additional fields can be defined (see mubb_mixed.m),  but the ones
% listed above are usually sufficient for non-expert users.
%        
% OUTPUT ARGUMENTS
% - bnds: (skew-)mu lower and upper bounds for the NORMALIZED inter-
%         connection ((skew-)mu problem) or bounds on the worst-case
%         Hinfinity norm for each transfer (performance problem).
% - wc: frequency in rad/s for which lower bound has been computed.
% - pert: associated value of Delta. unless the lower bound is equal
%         to zero, at  least one eigenvalue of the NORMALIZED inter-
%         connection between P and pert  is unstable ((skew-)mu pro-
%         blem), or the largest singular value of Fu(P,pert) at fre-
%         quency wc (or at the corresponding point  on  the boundary
%         of  the truncated  sector  defined  by options.sector)  is
%         equal to bnds(1) (performance problem).
% - iodesc is a structured variable which contains:
%   * the values  of Delta  for which  the stability  of the INITIAL
%     interconnection  is guaranteed ((skew)-mu problem) or the Hin-
%     finity norm  of the transfer between e and y  is guaranteed to
%     be less than bnds(2) (performance problem).
%   * the same data as pert, but for the INITIAL interconnection.
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/mubb for more info!
