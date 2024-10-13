%%
% MAKE_SQUARE                     Make all uncertainty blocks square
% ------------------------------------------------------------------
%
% This function transforms  the interconnection  between P and Delta
% (see below) so that the block-diagonal operator Delta is only com-
% posed of square blocks.
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
% CALL
% [sys2,blk2]=make_square(sys,blk{,perfo});
%
% INPUT ARGUMENTS
% - sys: lti object describing the LTI system P. it can also be a 2D
%        or 3D numeric array corresponding to the frequency response
% - blk: matrix defining the structure  of the block-diagonal opera-
%        tor Delta=diag(Delta_1,...,Delta_nblk). its first 2 columns
%        must be defined as follows for all i=1,...,nblk:
%        * blk(i,1:2)=[-ni 0] => Delta_i=di*eye(ni) with di real
%        * blk(i,1:2)=[ni 0]  => Delta_i=di*eye(ni) with di complex
%        * blk(i,1:2)=[ni mi] => Delta_i is a ni-by-mi LTI system
% - perfo: if nonzero,  the performance channel  between e and y (if
%          any) is also made square (optional argument, def=0). 
%
% OUTPUT ARGUMENTS
% The interconnection defined by sys2 and blk2  is equivalent to the
% one described  by sys and blk from an input/output  point of view,
% but Delta is now only composed of square blocks.
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/make_square for more info!
