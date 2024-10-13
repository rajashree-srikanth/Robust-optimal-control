%%
% QUADCONV                           Transform a quadratic criterion
% ------------------------------------------------------------------
%
% This function computes  the matrix H such that the  quadratic term
% v'*delta'*delta*v is transformed into xi'*H*xi, where xi and delta
% are the vector form and the matrix form of a block-diagonal opera-
% tor Delta respectively, i.e. xi=delta2xi(delta,blk).
%
% CALL
% H=quadconv(v,blk);
%
% INPUT ARGUMENTS
% - v: column vector.
% - blk: matrix defining the structure  of the block-diagonal opera-
%        tor Delta=diag(Delta_1,...,Delta_nblk). its first 2 columns
%        must be defined as follows for all i=1,...,nblk:
%        * blk(i,1:2)=[-ni 0] => Delta_i=di*eye(ni) with di real
%        * blk(i,1:2)=[ni 0]  => Delta_i=di*eye(ni) with di complex
%        * blk(i,1:2)=[ni mi] => Delta_i full ni-by-mi complex block
%
% OUTPUT ARGUMENT
% - H: matrix such that v'*delta'*delta*v=xi'*H*xi.
%
% EXAMPLE
% blk=[-3 0;2 0;2 3];
% v=rand(8,1)+j*rand(8,1);
% H=quadconv(v,blk);
% xi=(1:15)';
% delta=xi2delta(xi,blk);
% v'*delta'*delta*v-xi'*H*xi
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/quadconv for more info!
