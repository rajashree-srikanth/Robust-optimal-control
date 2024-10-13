%%
% DBSAMPLE                  Generate random samples from gss objects
% ------------------------------------------------------------------
%
% This function generates  random samples of  the Delta  blocks of a
% gss object. It  returns the resulting gss (or ss)  objects and the
% sample values of each sampled block.
%
% CALL
% [sys,samples] = dbsample(sys1{,n,names})
%
% INPUT ARGUMENTS
% - sys1: gss object.
% - n: number of samples (default=1).
% - names: cell array of strings  with the names of the Delta blocks
%          to be sampled (by default all blocks of type PAR, POL and
%          LTI are sampled).
%
% OUTPUT ARGUMENTS
% - sys: 1xn cell array of gss objects (or of numerical arrays or ss
%        objects if  all blocks are sampled). if only  one sample is
%        requested (n=1), sys is directly  given as a gss object (or
%        a numerical array or ss object)
% - samples: 1xn structure array which contains the sample values of
%            each sampled block.
%
% WARNING
% Random sampling is only  possible for blocks of type  PAR, POL and
% LTI. The  field Distribution can be used for PAR blocks to specify
% the parameters distribution (uniform or normal, type  help gss for
% more details). It is ignored for POL and LTI blocks.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
