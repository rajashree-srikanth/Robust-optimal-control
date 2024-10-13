%%
% IOMARGINS   Compute bounds on gain, phase, modulus & delay margins
% ------------------------------------------------------------------
%
% This function  computes both lower and  upper bounds on the worst-
% case gain, phase, modulus and delay margins for an interconnection
% between an LTI  system P and a block-diagonal  operator Delta. The
% lower bound is guaranteed  on a whole frequency interval. A branch
% and bound scheme can be applied, in which case the  bounds satisfy
% ubnd<(1+maxgap)*lbnd,  where maxgap is a user-defined threshold. A
% value of  Delta for which the upper bound is  obtained is also de-
% termined. The  considered stability region  can be bounded  either
% by the imaginary axis or by a truncated sector.
%
%                              -------
%                         +---| Delta |<--+
%                         |    -------    |
%                         |               |
%                         |    -------    |
%                         +-->|       |---+
%                             |   P   |
%                       e --->|       |---> y
%                              -------
%
% More precisely, some  additional fictitious uncertainties delta_M=
% diag(delta_Mi) corresponding to gain, phase, modulus or time-delay
% variations are connected between y and e:
%  - gain margin => delta_Mi=1+delta_i, where delta_i is real
%  - phase margin => delta_Mi=exp(j*delta_i), where delta_i is real
%  - modulus margin => delta_Mi=1/(1+delta_i), where delta_i is cplx
%  - delay margin => delta_Mi=exp(-delta_i*s), where delta_i>=0
% In this context, the worst-case margin  corresponds to the maximum
% value of |delta_i| for which the interconnection is stable for any
% admissible value of Delta.
%
% The initial interconnection is NORMALIZED  with convert_data.m be-
% fore the problem is solved.
%
% CALL
% [bnds,wc,pert,iodesc]=iomargins(usys,margin{,options});
% [bnds,wc,pert,iodesc]=iomargins(gsys,margin{,options});
% [bnds,wc,pert,iodesc]=iomargins(lsys,margin{,options});
% [bnds,wc,pert,iodesc]=iomargins(sys,blk,margin{,options});
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
% The second (or third)  input argument margin  is a character array
% which allows to specify the considered problem:
%   * it must contain exactly one of the following characters : 'g',
%     'p', 'm' or 'd' (gain, phase, modulus or delay margin),
%   * it must also contain 'l', 'u', 'lu' or 'b'  (lower bound only,
%     upper bound only,  both lower and  upper bounds, or both lower
%     and upper bounds with desired accuracy).
% The variable options is an optional structure with fields:
% * freq: frequency interval  in rad/s on which the bounds are to be
%         computed (def=[0 10*max(abs(eig(P)))]).
% * sector: truncated sector  [alpha xi ws].  alpha is the  relative
%           stability degree. xi is the  damping factor  (if ws=[]).
%           ws is optional and corresponds to  the imaginary part of
%           the intersection point  between the two segments defined
%           by alpha and xi (def=[0 0]). see also display_sector.m.
% * target: value of the lower/upper bound below which the algorithm
%           is interrupted (def=0, ignored if margin contains 'b').
% * lmi: switch to LMI (if lmi=1) for  better accuracy (def=0, igno-
%        red if margin does not contains 'l' or 'b').
% * maxgap: maximum gap  between the bounds (def=0.05, i.e. 5%, con-
%           sidered only if margin contains 'b'). 
% * trace: trace of execution (def=1).
% * warn: warnings display (def=1).
% Additional fields can  be defined (see muub_mixed for lower bound,
% mulb_mixed or mulb_nreal for upper bound and mubb_mixed for branch
% and bound), but the ones listed  above are usually  sufficient for
% non-expert users.
%
% OUTPUT ARGUMENTS
% - bnds: lower and upper bounds  on the worst-case gain, phase, mo-
%         dulus or delay margin.
% - wc: frequency in rad/s for which upper bound has been computed.
% - pert: value of Delta for which upper bound has been computed for
%         the NORMALIZED interconnection.
% - iodesc is a structured variable which contains:
%   * the values of Delta for which  the gain, phase, modulus or de-
%     lay margin of the INITIAL interconnection is  guaranteed to be
%     more than bnds(1).
%   * the same data as pert, but for the INITIAL interconnection.
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/iomargins for more info!
