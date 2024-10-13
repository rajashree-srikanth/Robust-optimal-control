%%
% -----------------------------------------------------------------------
% Generalized State-Space (GSS) library for Matlab
% -----------------------------------------------------------------------
%
% Linear Fractional  Representations are  a straightforward  extension of
% state-space models with  uncertain elements, parametric variations, and
% isolated nonlinearities. They consist of a continuous- or discrete-time
% LTI model M(s) or M(z) in feedback loop with  a block diagonal operator
% Delta which collects uncertain, varying and nonlinear elements: 
% 
%              -------
%         +---| Delta |<--+     
%         |    -------    |
%       w |               | z       
%         |    -------    |             Delta = blockdiag(D1,...,DN)
%         +-->|       |---+
%             |   M   |  
%     u ----->|       |-----> y
%              -------
%
% Each Di block can be  a real or complex  uncertain or varying parameter
% (PAR),  a polytopic-type  uncertain or varying element  (POL), a linear
% time-invariant system (LTI),  a sector nonlinearity (SEC), a saturation
% (SAT), a deadzone (DZN) or any other nonlinear operator (NLB).
%
% There are different ways to model  such systems with Matlab. The 2 most
% classical ones rely on the uss object of the Robust Control Toolbox and
% the lfr object of  the Linear Fractional Representation  Toolbox. Buil-
% ding on many years of experience, an alternative is proposed here based
% on the new  Matlab class GSS  and the associated GSS object. It is con-
% ceived to be  more user-friendly than the  lfr class and  more powerful
% than the uss class.
%
% TYPE
% => 'help gss/description' to get a full description of the GSS object
% => 'help gss/creation' to learn how to create GSS objects
% => 'help gss/example' to display a comprehensive example
% => 'help gss/functions' to see the list of functions of the GSS library
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
