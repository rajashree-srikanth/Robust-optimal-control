%%
% DATA2GSS          Create low-order gss objects from tabulated data
% ------------------------------------------------------------------
%
% This routine uses Linear Least-Squares or Orthogonal Least-Squares
% to compute a multivariate  polynomial approximation of  a set of N
% real-valued n1 x n2 matrices depending on real parameters x1...xp.
% The degree of the polynomial approximation and/or the global rela-
% tive approximation  error are  set by the user.  The resulting ex-
% pression is then converted into a low-order static gss object with
% n1 outputs and  n2 inputs, and where the number  of repetitions n1
% ...np of x1...xp  in Delta = diag(x1*In1,...,xp*Inp) is as  low as
% possible.  An other gss object with  q additional  blocks covering
% the approximation errors is also computed. This routine is a user-
% friendly interface to  call the routines lapprox and  olsapprox of
% the APRICOT library of the SMAC toolbox.
%
% WARNING
% The APRICOT library  must be available.  It can be downloaded from
% the SMAC website: http://w3.onera.fr/smac/apricot.
%
% CALL
% [sys1,sys2,fdata,err]=data2gss(param,data{,opt})
%
% INPUT ARGUMENTS
% - param: structured variable describing the parameters x1...xp:
%          * names: parameters names (1 x p cell array)
%          * values: parameters values  at approximation points (pxN
%                    array)
% - data: set of  N real-valued n1 x n2 matrices to be  approximated
%         (n1 x n2 x N array). can also be a N x n2 array if n1=1, a
%         n1 x N array if n2=1, or a 1 x N or N x 1 array if n1=n2=1
% - opt: optional structured variable with fields:
%        * mtd: 'sls' (standard linear least-squares)  or 'ols' (or-
%               thogonal least-squares, default value).
%        * err: global relative error for each entry (default=0.05).
%        * deg: degree of the polynomials (not fixed by default).
%        * dpp: data pre-processing:  lines and columns  of data are
%               normalized,  and constant  lines/columns are removed
%               (default=1).
%
% REMARK
% The standard call consists  of specifying either the approximation
% degree (opt.deg) or  the desired accuracy (opt.err). If the degree
% is fixed, the APRICOT routine is called once and  the error is mi-
% nimized for the considered  degree. Alternatively, if the accuracy
% is specified, the degree is increased until the error becomes  lo-
% wer than the required value. Both options can also be specified in
% the same call. In this case, the degree is increased starting from
% the provided value until the error becomes lower than the required
% threshold.
%
% OUTPUT ARGUMENTS
% - sys1: static gss object  approximating data  with n1 outputs, n2
%         inputs and p real parametric blocks.
% - sys2: same as sys1  with q additional normalized real parametric
%         blocks covering the approximation errors (each n1 x n2 ma-
%         trix in data can  be recovered exactly for  some values of
%         these q blocks between -1 and 1).
% - fdata: values  of the approximating  polynomial function  at the
%          considered approximation points (same size as data).
% - err: structured array with the approximation errors:
%        * max: maximum global relative error over all entries.
%        * rel: global relative error for each entry (n1 x n2 array)
%        * abs: maximum local absolute error for each entry (n1 x n2
%               array).
%        * del: q x 2 array, where del(i,1) and del(i,2) are the re-
%               lative  and the absolute  errors corresponding to xi
%               respectively.
%
% SEE ALSO
%  lsapprox: linear least-squares approx (APRICOT library).
%  olsapprox: orthogonal least-squares approx (APRICOT library).
%
% SMAC TOOLBOX - GSS LIBRARY
% See http://w3.onera.fr/smac/gss for more info!
