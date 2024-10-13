%%
% HINFLB_REAL       Compute lower bounds on the worst-case Hinf norm
% ------------------------------------------------------------------
%
% This function  computes lower bounds  on the worst-case  Hinfinity
% norm for an interconnection  between an LTI  system P and a block-
% diagonal operator Delta=diag(Delta_1,...,Delta_nblk) with Delta_i=
% di*eye(ni) and di REAL. Lower bounds are first computed on a rough
% frequency grid. A series of quadratic programming problems is then
% solved using these results as an initialization.
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
% Let B(Delta) be the set of all block-diagonal operators Delta with
% |di|<1 for all i.  Assume that the  interconnection  between P and
% Delta is stable for all Delta in B(Delta).  Let Fu(P,Delta) be the
% transfer function between e and y, which must be square. The worst
% case Hinfinity norm is defined by:
%
%                     max          ||Fu(P,Delta)||
%               Delta C B(Delta)                  inf
%
% where the Hinfinity  norm can be computed  either on the imaginary
% axis or on the boundary of a truncated sector.
%
% NOTE
% The aim is not to compute a lower bound as a function of frequency
% but to compute a lower  bound over the whole frequency  range. The
% frequency  for which the largest singular  value of Fu(P,Delta) is
% maximum is left free during the optimization. Thus a rough initial
% grid usually allows to capture the most critical frequencies.
%
% CALL
% [lbnd,wc,pert,tab]=hinflb_real(sys,blk{,options})
%
% INPUT ARGUMENTS
% - sys: lti object describing the LTI system P.
% - blk: matrix defining the structure  of the block-diagonal opera-
%        tor Delta=diag(Delta_1,...,Delta_nblk). its first 2 columns
%        must be defined as follows for all i=1,...,nblk:
%        * blk(i,1:2)=[-ni 0] => Delta_i=di*eye(ni), di real
% - options is an optional structured variable with fields:
%   * freq: frequency interval in rad/s  on which the lower bound is
%           to be computed (def=[0 10*max(abs(eig(P)))]).
%   * grid: frequency grid  in rad/s  (1xN array). it  can also be a
%           negative integer -N, in which case a N-point grid is au-
%           tomatically generated (def=-10).
%   * sector: truncated sector [alpha xi ws].  alpha is the relative
%             stability degree. xi is the damping factor (if ws=[]).
%             ws is optional  and corresponds to the  imaginary part
%             of the intersection point between the two segments de-
%             fined by alpha and xi (def=[0 0]). see display_sector.
%   * target: the algorithm is interrupted if a lower bound is found
%             which is greater than target (def=Inf).
%   * trace: trace of execution (def=1).
%   * warn: warnings display (def=1).
%   * wpar: if nonzero, frequency is treated as an additional parame
%           ter in Delta and the problem  is solved on the intervals
%           [grid(1) grid(2)],...,[grid(end-1) grid(end)] (def=0).
%   * prec: level of precision of the final result (2 <> good preci-
%           sion, 1 <> faster but potentially less precise, def=1).
%   * local: if zero, the algorithm focuses on the global worst-case
%            configuration. otherwise, it also tries to detect local
%            maxima,  but the  probability  to determine  the global
%            worst-case configuration is lower (def=0).
%   * method: 1x2 array defining the method used for the line search
%             [1 tol] means  that the optimum  is computed  from the
%             real eigenvalues of a hamiltonian matrix with a preci-
%             sion of tol. [2 n] means  that the optimum is computed
%             on a grid composed of n points.  [1 tol] is often more
%             accurate but  CPU time is  higher (def = [1 0.001]  or
%             [2 6] depending on the size of Delta (< or > 50)).
%     
% OUTPUT ARGUMENTS
% - lbnd: best lower bound on the worst-case Hinfinity norm.
% - wc: associated frequency in rad/s.
% - pert: associated value of Delta. unless lbnd=0, the largest sin-
%         gular value of Fu(P,pert) at frequency wc is equal to lbnd
% - tab is a structured variable with fields:
%   * lbnd: other lower bounds on the Hinfinity norm (Nx1 array).
%   * wc: associated frequencies in rad/s (Nx1 array).
%   * pert: associated values of Delta (nxnxN array where n=sum(ni))
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/hinflb_real for more info!
