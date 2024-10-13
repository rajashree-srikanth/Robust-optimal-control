%%
% GSS/EVAL                                     Evaluate a gss object
% ------------------------------------------------------------------
%
% Overloaded EVAL function for gss objects.  The Delta blocks of the
% gss object sys1 can  be replaced with the values  available in the
% workspace, the values passed to the function or the nominal values
% defined in sys1.D. All these values can be numerical arrays or ss/
% tf/zpk/gss/lfr/uss  objects. It  is not  necessary  to define  all
% Delta blocks before using the function eval. Note that lfr and uss
% objects are converted to gss objects before evaluation.
%
% CALL
% sys=eval(sys1)                  all values available in workspace
% sys=eval(sys1,'nominal')        all available nominal values
% sys=eval(sys1,names,values)     selected values passed to function
% sys=eval(sys1,names,'nominal')  selected nominal values
%
% INPUT ARGUMENTS
% - sys1: gss object.
% - names: names of Delta blocks  to be evaluated  (1 x nblocks cell
%          array of strings).
% - values: corresponding values  for the npts  configurations to be
%           evaluated (npts x nblocks cell array of numerical arrays
%           or ss/tf/zpk/gss/lfr/uss objects).
% NOTE: If a single  Delta block  is considered, its name  and value
%       can be directly given witout being stored into a cell array.
%
% OUTPUT ARGUMENT
% - sys: numerical array or ss/gss object  if npts=1 ; 1 x npts cell
%        array of numerical arrays or ss/gss objects if npts>1.
%
% EXAMPLE
% 1. creation of a gss object
%   a=gss('alpha',2);           % nominal value is equal to 2
%   b=gss('beta',1);            % nominal value is equal to 1
%   c=gss('gamma','LTI',[1 1]);
%   d=gss('delta');
%   sys=c*ss(-3+a,b,1,a+b);
%   size(sys)
% 2. evaluation with all values available in the workspace
%   alpha=1;                    % alpha is set to 1
%   beta=d^2;                   % beta is replaced with delta^2
%   gamma=ss(-1,3,2,4);         % gamma is set to ss(-1,3,2,4)
%   sys1=eval(sys);
%   size(sys1)
% 3. evaluation with selected values passed to the function eval
%   sys2=eval(sys,{'alpha' 'beta' 'gamma'},{1 d^2 ss(-1,3,2,4)});
%   size(sys2)
% 4. direct evaluation without the function eval
%   sys3=ss(-1,3,2,4)*ss(-2,d^2,1,1+d^2);
%   size(sys3)
% 5. comparison
%   distgss(sys1,sys3)
%   distgss(sys2,sys3)
% 6. evaluation with all available or with selected nominal values
%   sys4=eval(sys,'nominal');
%   size(sys4)
%   sys5=eval(sys,{'alpha' 'beta'},'nominal');
%   size(sys5)
%   sys6=eval(sys,{'alpha' 'beta'},{'nominal' 1});
%   size(sys6)
% 7. direct evaluation without the function eval
%   sys7=c*ss(-1,1,1,3);
% 8. comparison
%   distgss(sys4,sys7)
%   distgss(sys5,sys7)
%   distgss(sys6,sys7)
% 9. evaluation for several configurations of Delta
%   sys8=eval(sys,{'alpha' 'beta'},{2 'nominal';'nominal' 1});
%   distgss(sys8{1},sys8{2})
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
