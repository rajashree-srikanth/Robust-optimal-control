%%
% GEN_GRID                                 Generate a frequency grid
% ------------------------------------------------------------------
%
% This function generates  a logarithmically spaced  frequency grid,
% which is  automatically refined in  some frequency regions corres-
% ponding to weakly damped modes of a given LTI system.
%
% CALL
% grid=gen_grid(sys{,freq});
% grid=gen_grid(sys{,npts});
% grid=gen_grid(sys{,freq,npts});
%
% INPUT ARGUMENTS
% - sys: lti object or corresponding vector of eigenvalues.
% - freq: frequency interval in rad/s (the default value is computed
%         according to the system bandwidth).
% - npts: number of frequency points (def=50).
%
% OUTPUT ARGUMENT
% - grid: frequency grid.
%
% SMAC TOOLBOX - SMART LIBRARY
% See http://w3.onera.fr/smac/gen_grid for more info!
