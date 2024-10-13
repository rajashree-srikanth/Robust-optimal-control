%%
% GSS/SIZE                                      Size of a gss object
% ------------------------------------------------------------------
%
% Overloaded SIZE function for gss objects.
%
% CALL
% size(sys) => display
% [no,ni,ndo,ndi,ns]=size(sys)
% [no,ni]=size(sys)
% [no,ni,ns]=size(sys)
% no=size(sys,1)
% ni=size(sys,2)
% ns=size(sys,'order')
% [ndo,ndi]=size(sys,'blk')
% [ndo,ndi]=size(sys,'name')
%
% INPUT ARGUMENTS
% The first argument is a gss object. A second argument (1,2,'order'
% 'blk',name) can be used to compute only no, ni, ns, ndo/ndi or the
% dimensions of the block 'name'.
%
% OUTPUT ARGUMENTS
% - no: number of external outputs.
% - ni: number of external inputs.
% - ndo: number of Delta-block outputs.
% - ndi: number of Delta-block inputs.
% - ns: number of states.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
