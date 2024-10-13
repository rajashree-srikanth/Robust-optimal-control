%%
% MUBB_MIXED          Compute (skew-)mu bounds with desired accuracy
% ------------------------------------------------------------------
%
% This function  computes both (skew-)mu lower  and upper bounds for
% an interconnection  between an LTI  system P and a  block-diagonal
% operator  Delta=diag(Delta_1,...,Delta_nblk).  A branch  and bound
% scheme is implemented:  the real parametric domain  is partitioned
% in more  and more subsets  until the highest lower bound  lbnd and
% the highest upper bound ubnd computed on all subsets satisfy ubnd<
% (1+maxgap)*lbnd,  where  maxgap is  a user-defined  thresohld. The
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
% This algorithm converges for systems with only real uncertainties,
% i.e. maxgap can be arbitrarily small. However, it usually exhibits
% an exponential growth of computational complexity as a function of
% the number  of real uncertainties.  The threshold maxgap allows to
% handle the tradeoff between  accuracy and  computational time. The
% mu-sensitivities  can also be used to get the same accuracy with a
% reduced number of iterations (see options.musen).
%
% CALL
% [bnds,wc,pert,tab,elts]=mubb_mixed(sys,blk{,options});
%
% INPUT ARGUMENTS
% - sys: lti object describing the LTI system P.
% - blk: matrix defining the structure  of the block-diagonal opera-
%        tor Delta=diag(Delta_1,...,Delta_nblk). its first 2 columns
%        must be defined as follows for all i=1,...,nblk:
%        * blk(i,1:2)=[-ni 0] => Delta_i=di*eye(ni) with di real
%        * blk(i,1:2)=[ni 0]  => Delta_i=di*eye(ni) with di complex
%        * blk(i,1:2)=[ni ni] => Delta_i is a ni-by-ni LTI system
%        a third column can be used to specify skew uncertainties:
%        * blk(i,3)=0 => svds(Delta_i,1)<=1
%        * blk(i,3)=1 => svds(Delta_i,1) is unconstrained
%        if none, blk(i,3) is set to 1 for all i.
% - options is an optional structured variable with fields:
%   * maxgap: maximum gap between the bounds (def=0.05, i.e. 5%). in
%             case of a mu-test (options.umax>0),  maxgap is used to
%             set options.tol in muub_mixed.
%   * maxcut: maximum number of times the real parametric domain can
%             be bisected (def=100).
%   * maxtime: maximum computational time (def=Inf).
%   * musen: if musen=0, the parametric domain is bisected along its
%            longest edge. if musen=Inf, the parametric domain is bi
%            sected alond the edge  with the highest mu-sensitivity.
%            if musen is an integer m>0, the algorithm switches bet-
%            ween  the two strategies every m iterations  in case of
%            slow progress (def=10).
%   * freq: frequency interval  in rad/s on which  the bounds are to
%           be computed (def=[0 10*max(abs(eig(P)))]).
%   * sector: truncated sector [alpha xi ws].  alpha is the relative
%             stability degree. xi is the damping factor (if ws=[]).
%             ws is optional  and corresponds to the  imaginary part
%             of the intersection point between the two segments de-
%             fined by alpha and xi (def=[0 0]). see display_sector.
%   * trace: trace of execution (def=1).  an 's' in the first column
%            indicates that the mu-sensitivities have been used.
%   * warn: warnings display (def=1).
%   * umax: if nonzero,  the algorithm checks  whether the (skew-)mu
%           upper bound is  less than umax  (mu-test). in this case,
%           no lower bound is computed (def=0).
%   Additional fields can be defined if necessary: lmi, reg, maxiter
%   for upper bound  computation (see muub_mixed) ; method, improve,
%   grid for lower bound computation (see mulb_mixed or mulb_nreal).
%
% OUTPUT ARGUMENTS
% - bnds: (skew-)mu lower  and upper bounds  lbnd and ubnd.  ubnd is
%         valid only if the algorithm succeeds, i.e. if elts.invalid
%         is empty.
% - wc: frequency in rad/s for which lbnd has been computed.
% - pert: associated  destabilizing value  of Delta.  unless lbnd=0,
%         det(I-pert*M)~0, where M is the frequency response of P at
%         frequency wc (or at the corresponding point on the bounda-
%         ry of the truncated sector defined by options.sector).
% - tab is a structured variable with fields:
%   * lbnd: other (skew-)mu lower bounds (Nx1 array).
%   * wc: associated frequencies in rad/s (Nx1 array).
%   * pert: associated  destabilizing values of  Delta (nxnxN array,
%           where n=sum(ni)).
% - elts is a structured variable with fields valid and invalid:
%   * elts.valid(k) describes the kth validated subset and is compo-
%     sed of fields:
%     => freq: frequency interval.
%     => domain: real parametric domain (one line per parameter).
%     => ncut: number of bisections needed to obtain the subset.
%     => utest: upper bound considered during validation.
%   * elts.invalid(k) describes  the kth  invalidated  subset. it is
%     empty if the algorithm succeeds.
%
% See http://w3.onera.fr/smac/mubb_mixed for more info!
