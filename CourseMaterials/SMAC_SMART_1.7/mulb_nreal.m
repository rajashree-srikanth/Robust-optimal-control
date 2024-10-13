%%
% MULB_NREAL   Compute (skew-)mu lower bounds for real perturbations
% ------------------------------------------------------------------
%
% This function computes (skew-)mu lower bounds  for an interconnec-
% tion between an LTI  system P and a block-diagonal operator Delta=
% diag(Delta_1,...,Delta_nblk), with Delta_i=di*eye(ni) and di REAL.
% At each point of a  rough frequency grid, a power algorithm is run
% on a  regularized problem  and a series  of LP problems  is solved
% starting from the real part Delta_R of the resulting destabilizing
% perturbation (which  usually brings  the interconnection  close to
% instability). The considered  stability region  can be bounded ei-
% ther by the imaginary axis or by a truncated sector.
%
%                              -------
%                         +---| Delta |<--+
%                         |    -------    |
%                         |               |
%                         |    -------    |
%                         |   |       |   |
%                         +-->|   P   |---+
%                             |       |
%                              -------
%
% NOTE
% The aim is not to compute a (skew-)mu lower bound as a function of
% frequency,  but to compute a lower  bound over the whole frequency
% range. The frequency at which the interconnection becomes unstable
% is left free during the optimization  and can be very far from the
% initial one.  Thus, a rough initial grid is  usually sufficient to
% capture the most critical frequencies.
%
% CALL
% [lbnd,wc,pert,tab]=mulb_nreal(sys,blk{,options});
%
% INPUT ARGUMENTS
% - sys: lti object describing the LTI system P.
% - blk: matrix defining the structure  of the block-diagonal opera-
%        tor Delta=diag(Delta_1,...,Delta_nblk). its first 2 columns
%        must be defined as follows for all i=1,...,nblk:
%        * blk(i,1:2)=[-ni 0] => Delta_i=di*eye(ni) with di real
%        a third column can be used to specify skew uncertainties:
%        * blk(i,3)=0 => svds(Delta_i,1)<=1
%        * blk(i,3)=1 => svds(Delta_i,1)<=1/lbnd with lbnd maximized
%        if none, blk(i,3) is set to 1 for all i.
% - options is an optional structured variable with fields:
%   * freq: frequency interval in rad/s  on which the lower bound is
%           to be computed (def=[0 10*max(abs(eig(P)))]).
%   * grid: frequency grid in rad/s (1xN array). it can also be:
%           > a negative integer -N, in which case a N-point grid is
%             automatically generated (def=-10),
%           > empty, in which case the LP problem is directly solved
%             without being initialized with the power alogrithm.
%   * sector: truncated sector [alpha xi ws].  alpha is the relative
%             stability degree. xi is the damping factor (if ws=[]).
%             ws is optional  and corresponds to the  imaginary part
%             of the intersection point between the two segments de-
%             fined by alpha and xi (def=[0 0]). see display_sector.
%   * target: the algorithm is interrupted if a lower bound is found
%             which is greater than target (def=Inf).
%   * epsilon: a (skew-)mu lower bound lbnd is  acceptable if a per-
%              turbation pert  of norm 1/lbnd and a frequency wc are
%              found which satisfy abs(det(I-M*pert))<epsilon, where
%              M is the frequency response  of P at frequency wc (or
%              at the  corresponding  point on  the boundary  of the
%              truncated sector options.sector) (def=1e-8).
%   * trace: trace of execution (def=1).
%   * warn: warnings display (def=1).
%   * method: type of LP problem.  if method=1, the  scalar alpha is
%             increased or  decreased until the interconnection bet-
%             ween P and  Delta=alpha*Delta_R is marginally  stable.
%             if  method=2, the  eigenvalue  of the  interconnection
%             between P and Delta_R is identified, which is the clo-
%             sest  to the considered point  on the boundary  of the
%             stability region. when shifting this eigenvalue toward
%             instability, the infinity  norm of the model perturba-
%             tion Delta is minimized (def=2).
%   * reg: regularization to  ensure convergence  of the power algo-
%          rithm (def=0.05, i.e. 5% of complex uncertainty added).
%   * improve: if nonzero, additional computations  are performed to
%              find more accurate  mu lower bounds (def=0). this can
%              only be applied to classical mu problems.
%
% OUTPUT ARGUMENTS
% - lbnd: best (skew-)mu lower bound.
% - wc: associated frequency in rad/s.
% - pert: associated  destabilizing value  of Delta.  unless lbnd=0,
%         det(I-M*pert)~0, where M is the frequency response of P at
%         frequency wc (or at the corresponding point on the bounda-
%         ry of the truncated sector defined by options.sector).
% - tab is a structured variable with fields:
%   * lbnd: other (skew-)mu lower bounds (Nx1 array).
%   * wc: associated frequencies in rad/s (Nx1 array).
%   * pert: associated  destabilizing values of  Delta (nxnxN array,
%           where n=sum(ni)).
%   * pb indicates a convergence problem when non-zero (Nx1 array):
%     > 1 => the power algorithm did not converge
%     > 2 => the LP problem could not be solved
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/mulb_nreal for more info!
