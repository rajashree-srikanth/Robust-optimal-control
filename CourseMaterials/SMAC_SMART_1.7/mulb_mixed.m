%%
% MULB_MIXED  Compute (skew-)mu lower bounds for mixed perturbations
% ------------------------------------------------------------------
%
% This function computes (skew-)mu lower bounds  for an interconnec-
% tion between an LTI  system P and a block-diagonal operator Delta=
% diag(Delta_1,...,Delta_nblk), where Delta_i=di*eye(ni) (di real or
% complex), or Delta_i is a stable and proper real-rational unstruc-
% tured transfer function. A power algorithm is run at each point of
% a frequency grid. The  considered stability region  can be bounded
% either by the imaginary axis or by a truncated sector.
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
% CALL
% [lbnd,wc,pert,tab]=mulb_mixed(sys,blk{,options});
%
% INPUT ARGUMENTS
% - sys: lti object describing the LTI system P. it can also be a 2D
%        or 3D numeric array corresponding to the frequency response
% - blk: matrix defining the structure  of the block-diagonal opera-
%        tor Delta=diag(Delta_1,...,Delta_nblk). its first 2 columns
%        must be defined as follows for all i=1,...,nblk:
%        * blk(i,1:2)=[-ni 0] => Delta_i=di*eye(ni) with di real
%        * blk(i,1:2)=[ni 0]  => Delta_i=di*eye(ni) with di complex
%        * blk(i,1:2)=[ni ni] => Delta_i is a ni-by-ni LTI system
%        a third column can be used to specify skew uncertainties:
%        * blk(i,3)=0 => svds(Delta_i,1)<=1
%        * blk(i,3)=1 => svds(Delta_i,1)<=1/lbnd with lbnd maximized
%        if none, blk(i,3) is set to 1 for all i.
% - options is an optional structured variable with fields:
%   * freq: frequency interval in rad/s  on which the lower bound is
%           to be computed (def=[0 10*max(abs(eig(P)))]). this para-
%           meter is ignored if sys is a numeric array.
%   * grid: frequency grid  in rad/s  (1xN array). it  can also be a
%           negative integer -N, in which case a N-point grid is au-
%           tomatically generated (def=-50). this parameter is igno-
%           red if sys is a numeric array.
%   * sector: truncated sector [alpha xi ws].  alpha is the relative
%             stability degree. xi is the damping factor (if ws=[]).
%             ws is optional  and corresponds to the  imaginary part
%             of the intersection point between the two segments de-
%             fined by alpha and xi (def=[0 0]). see display_sector.
%             this parameter is ignored if sys is a numeric array.
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
%   * power: tuning parameters of the power algorithm (1x4 array). at
%            each frequency:
%            > a run using previous results  as an initialization is
%              performed if power(1) is nonzero (def=1).
%            > a run with deterministic  initialization is performed
%              if power(2) is nonzero (def=0).
%            > power(3) runs  are performed with  random initializa-
%              tion (def=1).
%            > at most power(4) iterations are performed at each run
%              (def=100).
%   * refine: if nonzero, the initial frequency grid is refined near
%             peak values of the (skew-)mu lower bound (def=1). this
%             parameter is ignored if sys is a numeric array.
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
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/mulb_mixed for more info!
