%%
% MULB_1REAL  Compute mu lower bounds for a single real perturbation
% ------------------------------------------------------------------
%
% This function  computes mu  lower  bounds  for an  interconnection
% between  an LTI  system  P and  a block-diagonal  operator  Delta=
% diag(Delta_1,...,Delta_nblk), with Delta_i=di*eye(ni) and di REAL.
% Only one uncertainty is allowed to vary at a time. nblk bounds are
% thus computed  and the ith one corresponds  to the inverse  of the
% smallest destabilizing value of di  when  dk=0  for all k~=i.  The
% considered stability region can be bounded either by the imaginary
% axis or by a truncated sector.
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
% The aim is not to compute a mu lower bound as a function of frequ-
% ency, but to compute a lower bound over the whole frequency range.
%
% CALL
% [lbnd,wc,pert,tab]=mulb_1real(sys,blk{,options});
%
% INPUT ARGUMENTS
% - sys: lti object describing the LTI system P.
% - blk: matrix defining the structure  of the block-diagonal opera-
%        tor Delta=diag(Delta_1,...,Delta_nblk). its first 2 columns
%        must be defined as follows for all i=1,...,nblk:
%        * blk(i,1:2)=[-ni 0] => Delta_i=di*eye(ni) with di real
% - options is an optional structured variable with fields:
%   * freq: frequency interval in rad/s  on which the lower bound is
%           to be computed (def=[0 10*max(abs(eig(P)))]).
%   * sector: truncated sector [alpha xi ws].  alpha is the relative
%             stability degree. xi is the damping factor (if ws=[]).
%             ws is optional  and corresponds to the  imaginary part
%             of the intersection point between the two segments de-
%             fined by alpha and xi (def=[0 0]). see display_sector.
%   * target: the algorithm is interrupted if a lower bound is found
%             which is greater than target (def=Inf).
%   * epsilon: a mu lower bound lbnd is acceptable if a perturbation
%              pert  of norm 1/lbnd  and a  frequency wc  are found,
%              which satisfy abs(det(I-M*pert))<epsilon,  where M is
%              the frequency response  of P at  frequency wc  (or at
%              the corresponding  point on the boundary of the trun-
%              cated sector options.sector) (def=1e-8).
%   * trace: trace of execution (def=1).
%   * warn: warnings display (def=1).
%   * kmax: largest considered perturbation (def=1000).
%   * npts: maximum number  of equally-spaced  values of each  di in
%           [0 options.kmax] for  which the stability  of the inter-
%           connection is checked (def=100).
%
% OUTPUT ARGUMENTS
% - lbnd: best mu lower bound.
% - wc: associated frequency in rad/s.
% - pert: associated  destabilizing value  of Delta.  unless lbnd=0,
%         det(I-M*pert)~0, where M is the frequency response of P at
%         frequency wc (or at the corresponding point on the bounda-
%         ry of the truncated sector defined by options.sector).
% - tab is a structured variable with fields:
%   * lbnd: other mu lower bounds (nblkx1 array).
%   * wc: associated frequencies in rad/s (nblkx1 array).
%   * pert: associated destabilizing values of Delta (nxnxnblk array
%           where n=sum(ni)).
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/mulb_1real for more info!
