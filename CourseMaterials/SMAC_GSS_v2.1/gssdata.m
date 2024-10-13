%%
% GSSDATA                Get matrices and dimensions of a gss object
% ------------------------------------------------------------------
%
% This function computes the state-space matrices and the dimensions
% of the LTI part M of a gss object.
%
%            -------
%       +---| Delta |<--+                  .
%       |    -------    |               |  x =  a*x +  bw*w +  bu*u
%     w |               | z             |
%       |    -------    |        <=>    |  z = cz*x + dzw*w + dzu*u
%       +-->|       |---+               |
%           |   M   |                   |  y = cy*x + dyw*w + dyu*u
%   u ----->|       |-----> y          
%            -------
%
% CALL
% [a,bw,bu,cz,cy,dzw,dzu,dyw,dyu,st,no,ni,ndo,ndi,ns]=gssdata(sys)
%
% INPUT ARGUMENT
% - sys: gss object.
%
% OUTPUT ARGUMENTS
% - a,bw,bu,cz,cy,dzw,dzu,dyw,dyu: state-space matrices of sys.
% - st: sample time  in seconds (-1 if unspecified, 0 if M is a con-
%       tinuous-time model).
% - no: number of external outputs (size of y).
% - ni: number of external inputs (size of u).
% - ndo: number of Delta-block outputs (size of w).
% - ndi: number of Delta-block inputs (size of z).
% - ns: number of states of M.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
