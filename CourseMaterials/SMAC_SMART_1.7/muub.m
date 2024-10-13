%%
% MUUB                                Compute (skew-)mu upper bounds
% ------------------------------------------------------------------
%
% This function computes a (skew-)mu upper bound for an interconnec-
% tion between an LTI  system P and a block-diagonal operator Delta.
% The bound is GUARANTEED on a whole frequency interval and the con-
% sidered stability  region can be  bounded either  by the imaginary 
% axis or by a truncated sector.
%
% If P has exogenous  inputs e and  outputs y,  a robust performance
% problem is considered: a GUARANTEED upper bound is computed on the
% highest Hinfinity norm of  the transfer function  Fu(P,Delta) bet-
% ween e and y when Delta takes any admissible value.
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
% [ubnd,wc,iodesc]=muub(usys{,skew/perf,options});
% [ubnd,wc,iodesc]=muub(gsys{,skew/perf,options});
% [ubnd,wc,iodesc]=muub(lsys{,skew/perf,options});
% [ubnd,wc,iodesc]=muub(sys,blk{,skew/perf,options});
%
% INPUT ARGUMENTS
% The interconnection between P and Delta can be described by:
% - a uss object usys
% - a gss object usys
% - an lfr object lsys
% - an lti object sys  describing the  LTI system P and an nblk-by-2
%   matrix blk defining the structure of the block-diagonal operator
%   Delta=diag(Delta_1,...,Delta_nblk):
%   * blk(i,:)=[-ni 0] => Delta_i=di*eye(ni) with di real
%   * blk(i,:)=[ni 0]  => Delta_i=di*eye(ni) with di complex
%   * blk(i,:)=[ni mi] => Delta_i is a ni-by-mi LTI system
% For a skew-mu problem, an additional argument skew can be defined:
% * skew(i)=0 => Delta_i must remain below user-defined bound
% * skew(i)=1 => the size of Delta_i must be maximized
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
% * freq: frequency interval in rad/s on which the upper bound is to
%         be computed (def=[0 10*max(abs(eig(P)))]).
% * sector: truncated sector  [alpha xi ws].  alpha is the  relative
%           stability degree. xi is the  damping factor  (if ws=[]).
%           ws is optional and corresponds to  the imaginary part of
%           the intersection point  between the two segments defined
%           by alpha and xi (def=[0 0]). see also display_sector.m.
% * lmi: switch to LMI (if lmi=1) for better accuracy (def=0).
% * target: value of  the upper bound  above which the  algorithm is
%           interrupted (def=Inf).
% * trace: trace of execution (def=1).
% * warn: warnings display (def=1).
% Additional fields can be defined (see muub_mixed.m),  but the ones
% listed above are usually sufficient for non-expert users.
%        
% OUTPUT ARGUMENTS
% - ubnd: (skew-)mu upper bound  for the NORMALIZED  interconnection
%         ((skew-)mu problem), or upper bound on the worst-case Hin-
%         finity norm for each transfer (performance problem).
% - wc: frequency in rad/s for which ubnd has been computed.
% - iodesc: values of  Delta for which the stability  of the INITIAL
%           interconnection is guaranteed ((skew)-mu problem) or the
%           Hinfinity  norm of the transfer between e and  y is gua-
%           ranteed to be less than ubnd (performance problem).
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/muub for more info!
