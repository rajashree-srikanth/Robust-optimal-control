%%
% XI2DELTA                           From vector form to matrix form
% ------------------------------------------------------------------
%
% This function converts a block-diagonal operator  from vector form
% to matrix form.
%
% CALL
% delta=xi2delta(xi,blk);
%
% INPUT ARGUMENTS
% - xi: vector form of Delta=diag(Delta_1,...,Delta_nblk).
% - blk: matrix defining the structure of Delta. its first 2 columns
%        must be defined as follows for all i=1,...,nblk:
%        * blk(i,1:2)=[-ni 0] => Delta_i=di*eye(ni) with di real
%        * blk(i,1:2)=[ni 0]  => Delta_i=di*eye(ni) with di complex
%        * blk(i,1:2)=[ni mi] => Delta_i full ni-by-mi complex block
%
% OUTPUT ARGUMENT
% - delta: matrix form of Delta.
%
% EXAMPLE
% blk=[-3 0;2 0;2 3];
% xi1=(1:15)';
% delta=xi2delta(xi1,blk)
% xi2=delta2xi(delta,blk)
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/xi2delta for more info!
