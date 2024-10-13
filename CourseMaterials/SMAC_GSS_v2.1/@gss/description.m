            %% 
            % -----------------------------------------------------------------------
            % Matlab gss object description
            % -----------------------------------------------------------------------
            %
            % A Matlab gss object is a structure array with two fields:
            %  - M: ss/tf/zpk object representing the LTI model M.
            %  - D: 1xN structure array describing each Di block with fields:
            %     .Name           name of the block
            %                     => string with no trailing underscore
            %     .Type           type of the block (see above)
            %                     => string 'PAR','POL','LTI','SEC','SAT','DZN','NLB'
            %     .Size           size of the block
            %                     => number of rows and columns [nrow ncol]
            %     .NomValue       nominal value
            %                     => real PAR: real number pnom
            %                        cplx PAR: real or complex number pnom
            %                        POL: 0 or nrow x ncol real matrix
            %                        LTI: 0 or nrow x ncol matrix / ss/tf/zpk object
            %                        SEC: no predefined format but should define a NL
            %                             included into the sector defined by .Bounds
            %                        SAT: slope knom in the linear region
            %                        DZN: slope knom in the nonlinear regions
            %                        NLB: no predefined format
            %     .Bounds         bounds information
            %                     => real PAR: min and max values [pmin pmax]
            %                        cplx PAR: 1x3 vector [cr ci r] defining a circle
            %                                  of center cr+ci*i and of radius r>0
            %                        POL: nrow x ncol x nver real matrix defining the
            %                             nver vertices of a polytopic set
            %                        LTI: scalar b>0 such that ||Di||_inf < b or SISO
            %                             causal ss/tf/zpk  object B  defining a fre-
            %                             quency dependent bound  such that sigma_max
            %                             (Di(w)) < |B(w)| for all w
            %                        SEC: min and max slopes [kmin kmax]
            %                        SAT,DZN: lower and  upper limits  [zmin zmax] of
            %                                 the linear region
            %                        NLB: no predefined format
            %     .Distribution   probability distribution
            %                     => structure array with fields Type and Parameters
            %                        PAR: Type is a string & Parameters a row vector.
            %                             2 distributions are  fully supported. Type=
            %                             'uniform' and Parameters=[pmin pmax] or [cr
            %                             ci r] defines a uniform distribution on the
            %                             interval  [pmin pmax] (real PAR)  or on the
            %                             circle of center cr+ci*i and radius r (com-
            %                             plex  PAR).  Type='normal' and  Parameters=
            %                             [mu var] defines a normal distribution with
            %                             mean mu and  variance var. other  distribu-
            %                             tions  can be defined, but they will not be
            %                             recognized by dbnorm, dbunorm & dbsamples.
            %                        POL,LTI,SEC,SAT,DZN,NLB: no predefined format
            %     .RateBounds     bounds on the rate of variation
            %                     => PAR: min/max rates [rmin rmax]
            %                        POL,LTI,SEC,SAT,DZN,NLB: no predefined format
            %     .Normalization  stores bounds information in case of normalization
            %                     => PAR,LTI: same format as .Bounds (see dbnorm)
            %                        POL,SEC,SAT,DZN,NLB: no predefined format
            %     .Misc           can be used to store any additional data
            %                     => no predefined format
            %     The fields  Name, Type and Size  must be specified  for each block,
            %     whereas the other fields can be left empty.
            %
            % SMAC TOOLBOX - GSS LIBRARY
            % See http://w3.onera.fr/smac/gss for more info!

