%%
% MUUB_MIXED  Compute (skew-)mu upper bounds for mixed perturbations
% ------------------------------------------------------------------
%
% This function computes a (skew-)mu upper bound for an interconnec-
% tion between an LTI  system P and a block-diagonal operator Delta=
% diag(Delta_1,...,Delta_nblk). A bound and associated  D,G scalings
% are first computed at a given frequency. The largest frequency in-
% terval on which they remain valid  is then obtained. This strategy
% is repeated until the whole frequency range is covered which leads
% to a  GUARANTEED upper bound. The  considered stability region can
% be bounded either by the imaginary axis or by a truncated sector.
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
% [ubnd,wc,tab,pb]=muub_mixed(sys,blk{,options});
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
%        * blk(i,3)=1 => svds(Delta_i,1)<=1/ubnd with ubnd minimized
%        if none, blk(i,3) is set to 1 for all i.
% - options is an optional structured variable with fields:
%   * w0: starting frequency point in rad/s (def=0).
%   * freq: frequency interval in rad/s  on which the upper bound is
%           to be computed (def=[0 max([10*abs(eig(P)) w0])]).
%   * sector: truncated sector [alpha xi ws].  alpha is the relative
%             stability degree. xi is the damping factor (if ws=[]).
%             ws is optional  and corresponds to the  imaginary part
%             of the intersection point between the two segments de-
%             fined by alpha and xi (def=[0 0]). see display_sector.
%   * lmi: if lmi=0, no LMI solver is used (default value,  fast but
%          sometimes not very accurate).  if lmi=1, an LMI solver is
%          used with tuning parameters [.02 100 1e10 5 1], see gevp.
%          lmi can also be a 1x5 array with used-defined parameters.
%   * trace: trace of execution (def=1).
%   * warn: warnings display (def=1).
%   * target: value of the upper bound  above which the algorithm is
%             interrupted (def=Inf).
%   * init: initial guess (e.g. (skew)-mu lower bound) (def=0).
%   * reg: regularization for real-mu problems (def=0.01, i.e. 1% of
%          complex uncertainty is added if convergence problems).
%   * tol: at each iteration the (skew-)mu upper  bound is increased
%          by a  factor of  1+tol before  the interval on which  the
%          scaling matrices are valid is computed (def=0.01).
%   * maxiter: maximum number of iterations (def=1000).
%   * elim: a pawl effect  on mu is used  to fasten  the elimination
%           technique if elim=1. better estimates of secondary peaks
%           are obtained if elim=0 (def=1).
%   * smm: method for the iterative calls to mussv in case of a skew
%          mu  problem  (1 <> fixed-point algorithm,  2 <> dichotomy
%          search, def=1). this option is ignored if options.lmi~=0.
%
% OUTPUT ARGUMENTS
% - ubnd: (skew-)mu upper bound on the considered frequency interval
% - wc: frequency in rad/s for which ubnd has been computed.
% - tab is a structured variable with fields (see plot_muub.m):
%   * ubnd: (skew-)mu upper bounds.
%   * int: frequency intervals on which the upper bounds are valid.
% - pb indicates a convergence problem when non-zero:
%   > 1 => impossible to further eliminate intervals
%   > 2 => maximum number of iterations reached
%   > 3 => skew-mu problem infeasible
%   > 4 => mu-test failed
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/muub_mixed for more info!
