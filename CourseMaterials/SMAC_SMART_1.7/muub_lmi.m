%%
% MUUB_LMI   Compute (skew-)mu upper bounds on a grid with LMI tools
% ------------------------------------------------------------------
%
% This function computes a (skew-)mu upper bound for an interconnec-
% tion between an LTI  system P and a block-diagonal operator Delta=
% diag(Delta_1,...,Delta_nblk). The bound is computed on a frequency
% grid and  the resulting D/G scaling  matrices have the same values
% for all frequency points.
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
% In case of a classical mu problem, the output arguments satisfy:
% Mk* D Mk + j [G Mk - Mk* G] <= ubnd^2 D    for all k
% where the mu upper bound ubnd is minimized and Mk=M(:,:,k) denotes
% the frequency response of P at frequency wk.
%
% In case of a skew-mu problem, the output arguments satisfy:
% Mk* D Mk + j [G Mk - Mk* G] <= S(ubnd) D   for all k
% where the skew-mu upper  bound ubnd is minimized, and S(ubnd)=diag
% (S_1(ubnd),...,S_nblk(ubnd)) is a block-diagonal matrix such that:
% * S_i(ubnd) = I         if svds(Delta_i,1) must remain less than 1
% * S_i(ubnd) = ubnd^2 I  if svds(Delta_i,1) is to be maximized
%
% CALL
% [ubnd,D,G,S]=muub_lmi(M,blk{,options});
%
% INPUT ARGUMENTS
% - M: 2D or 3D numeric array  describing the LTI system P. M(:,:,k)
%      corresponds to the frequency response of P at frequency wk.
% - blk: matrix defining the structure  of the block-diagonal opera-
%        tor Delta=diag(Delta_1,...,Delta_nblk). its first 2 columns
%        must be defined as follows for all i=1,...,nblk:
%        * blk(i,1:2)=[-ni 0] => Delta_i=di*eye(ni) with di real
%        * blk(i,1:2)=[ni 0]  => Delta_i=di*eye(ni) with di complex
%        * blk(i,1:2)=[ni ni] => Delta_i is a ni-by-mi LTI system
%        a third column can be used to specify skew uncertainties:
%        * blk(i,3)=0 => svds(Delta_i,1)<=1
%        * blk(i,3)=1 => svds(Delta_i,1)<=1/ubnd with ubnd minimized
%        if none, blk(i,3) is set to 1 for all i.
% - options is an optional structured variable with fields:
%   * target: the LMI solver stops  if the (skew-)mu upper bound be-
%             comes lower than target (def=0).
%   * epsilon: the constraint D>epsilon*I  is considered  instead of
%              D>0 (def=1e-8).
%   * lmiopt: tuning parameters  of the LMI solver  (see gevp/mincx)
%             (def=[0.02 100 1e10 5 1]).
%
% OUTPUT ARGUMENTS
% - ubnd: (skew-)mu upper bound.
% - D: D scaling matrix.
% - G: G scaling matrix.
% - S: S matrix (for skew-mu problems only).
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/muub_lmi for more info!
