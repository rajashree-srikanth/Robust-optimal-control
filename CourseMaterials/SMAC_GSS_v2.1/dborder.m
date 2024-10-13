%%
% DBORDER                       Reorder Delta blocks of a gss object
% ------------------------------------------------------------------
%
% This routine is used after an LFT realization  to merge parametric
% blocks of Delta with the same name  and properties, and to perform
% a lexicographical ordering of the blocks. To  enable compatibility
% with the  Simulink interface, non-parametric blocks all  appear in
% the upper-left part of Delta.
%
% CALL
% sys=dborder(sys1)
% [M,D]=dborder(M1,D1)
%
% INPUT ARGUMENTS
% - sys1: gss object.
% - M1,D1: fields of sys1.
%
% OUTPUT ARGUMENTS
% - sys: gss object with reordered Delta blocks.
% - M,D: fields of sys.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
