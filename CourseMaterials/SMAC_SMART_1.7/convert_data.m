%%
% CONVERT_DATA   Convert gss/lfr/uss objects into standard variables
% ------------------------------------------------------------------
%
% This function converts a gss/lfr/uss object  describing one of the
% interconnections below into an ss object representing the LTI sys-
% tem P and  a matrix defining  the structure of  the block-diagonal
% operator Delta. The resulting interconnection is NORMALIZED.
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
% CALL
% [sys,blk]=convert_data(usys{,skew/perf});
% [sys,blk]=convert_data(gsys{,skew/perf});
% [sys,blk]=convert_data(lsys{,skew/perf});
% [sys,blk]=convert_data(sys,blk{,skew/perf});
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
% * skew(i)=1 => the size of Delta_i must be minimized or maximized
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
% OUTPUT ARGUMENTS
% - sys: ss object describing the NORMALIZED LTI system P.
% - blk: nblk-by-3 matrix defining the structure of Delta. its first
%        2 columns are defined as follows for all i=1,...,nblk:
%        * blk(i,1:2)=[-ni 0] => Delta_i=di*eye(ni) with di real
%        * blk(i,1:2)=[ni 0]  => Delta_i=di*eye(ni) with di complex
%        * blk(i,1:2)=[ni mi] => Delta_i is a ni-by-mi LTI system
%        the third column is used to specify skew uncertainties:
%        * blk(i,3)=0 => svds(Delta_i,1) must remain below 1
%        * blk(i,3)=1 => svds(Delta_i,1) must be minimized/maximized
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/convert_data for more info!
