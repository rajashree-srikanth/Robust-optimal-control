%%
% DELTA2XI                           From matrix form to vector form
% ------------------------------------------------------------------
%
% This function converts a block-diagonal operator  from matrix form
% to vector form.
%
% CALL
% xi=delta2xi(delta,blk);
%
% INPUT ARGUMENTS
% - delta: matrix form of Delta=diag(Delta_1,...,Delta_nblk).
% - blk: matrix defining the structure of Delta. its first 2 columns
%        must be defined as follows for all i=1,...,nblk:
%        * blk(i,1:2)=[-ni 0] => Delta_i=di*eye(ni) with di real
%        * blk(i,1:2)=[ni 0]  => Delta_i=di*eye(ni) with di complex
%        * blk(i,1:2)=[ni mi] => Delta_i full ni-by-mi complex block
%
% OUTPUT ARGUMENT
% - xi: vector form of Delta.
%
% EXAMPLE
% blk=[-3 0;2 0;2 3];
% xi1=(1:15)';
% delta=xi2delta(xi1,blk)
% xi2=delta2xi(delta,blk)
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/delta2xi for more info!
