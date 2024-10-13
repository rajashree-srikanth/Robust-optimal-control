%%
% LINCONV                               Transform a linear criterion
% ------------------------------------------------------------------
%
% This function computes  the row vector A such that the linear term
% u*delta*v  is transformed into A*xi,  where xi  and delta  are the
% vector form and the matrix form of a block-diagonal operator Delta
% respectively, i.e. xi=delta2xi(delta,blk).
%
% CALL
% A=linconv(u,v,blk);
%
% INPUT ARGUMENTS
% - u: row vector.
% - v: column vector.
% - blk: matrix defining the structure  of the block-diagonal opera-
%        tor Delta=diag(Delta_1,...,Delta_nblk). its first 2 columns
%        must be defined as follows for all i=1,...,nblk:
%        * blk(i,1:2)=[-ni 0] => Delta_i=di*eye(ni) with di real
%        * blk(i,1:2)=[ni 0]  => Delta_i=di*eye(ni) with di complex
%        * blk(i,1:2)=[ni mi] => Delta_i full ni-by-mi complex block
%
% OUTPUT ARGUMENT
% - A: row vector such that u*delta*v=A*xi.
%
% EXAMPLE
% blk=[-3 0;2 0;2 3];
% u=rand(1,7)+j*rand(1,7);
% v=rand(8,1)+j*rand(8,1);
% A=linconv(u,v,blk);
% xi=(1:15)';
% delta=xi2delta(xi,blk);
% u*delta*v-A*xi
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/linconv for more info!
