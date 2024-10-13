%%
% GSS/FEEDBACK                Feedback connection of two gss objects
% ------------------------------------------------------------------
%
% Overloaded FEEDBACK function for gss objects.
%
%                             --->O--->| sys1 |----+--->
% feedback(sys1,sys2)   <=>       ^                |
%                                 |                |
%                                 +----| sys2 |<---+
%
% CALL
% sys=feedback(sys1,sys2{,feedin,feedout,sign})
%
% INPUT ARGUMENTS
% - sys1,sys2: gss objects.
% - feedin,feedout: specify  which inputs/outputs  of sys1  are used
%                   for feedback. if omitted, all inputs/outputs are
%                   used.  otherwise, the  length of  feedin/feedout
%                   must be equal to the number of outputs/inputs of
%                   sys2 respectively.
% - sign: negative feedback  is used if sign=-1  or sign is omitted,
%         and positive feedback is used if sign=1.
%
% OUTPUT ARGUMENT
% - sys: gss object representing the feedback connection of sys1 and
%        sys2. sys has the same inputs and outputs as sys1, with the
%        same order.
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
