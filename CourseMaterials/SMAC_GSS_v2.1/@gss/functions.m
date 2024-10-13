            %%
            % -----------------------------------------------------------------------
            % List of functions of the GSS Library v2.0
            % -----------------------------------------------------------------------
            %
            % - Gss objects generation
            %     gss              - core function for gss objects generation
            %     sym2gss          - create gss objects from symbolic expressions
            %     data2gss         - create gss objects from tabulated data
            %
            % - Overloaded functions
            %     gss/plus         - addition of two gss objects
            %     gss/minus        - subtraction of two gss objects
            %     gss/uminus       - change the sign of a gss object
            %     gss/mtimes       - multiplication of two gss objects
            %     gss/mrdivide     - right division for gss objects
            %     gss/mldivide     - left division for gss objects
            %     gss/inv          - inverse of a gss object
            %     gss/mpower       - repeated product of a gss object
            %     gss/horzcat      - horizontal concatenation of gss objects
            %     gss/vertcat      - vertical concatenation of gss objects
            %     gss/append       - append inputs and outputs of gss objects
            %     gss/transpose    - transpose of a gss object
            %     gss/ctranspose   - conjugate transpose of a gss object
            %     gss/ss           - construct a gss object from state-space matrices
            %     gss/tf           - construct a gss object from num and den
            %     gss/feedback     - feedback connection of two gss objects
            %     gss/eq           - check if two gss objects are equal
            %     gss/ne           - check if two gss objects are not equal
            %     gss/isempty      - check if a gss object is empty
            %     gss/eval         - evaluate a gss object
            %     gss/subsref      - subscripted reference for gss objects
            %     gss/end          - index of last I/O of a gss object
            %     gss/length       - length of a gss object
            %     gss/size         - size of a gss object
            %     gss/display      - display a gss object
            %     gss/get          - obtain some properties of a gss object
            %     gss/set          - change some properties of a gss object
            %
            %  - Conversions
            %     gss              - convert various objects to gss objects
            %     gssdata          - get matrices and dimensions of a gss object
            %     abcd2gss         - from static gss objects to dynamic gss objects
            %     gss2gss          - convert gss objects from v1.x to v2.x
            %     lfr2gss          - convert lfr objects into gss objects
            %     gss2lfr          - convert gss objects into lfr objects
            %     uss2gss          - convert uss objects into gss objects
            %     gss2uss          - convert gss objects into uss objects
            %
            % - Manipulation of the Delta block
            %     dbnorm           - normalize a gss object
            %     dbunorm          - unnormalize a gss object
            %     setnorm          - set preference for gss objects normalization
            %     dborder          - reorder the Delta blocks of a gss object
            %     dbsample         - generate random samples from a gss object
            %
            % - Order reduction
            %     mingss           - reduce the order of a gss object
            %     setred           - set preference for gss objects reduction
            %
            % - Simulink interface
            %     slk2gss          - from Simulink block diagrams to gss objects
            %     gsslib           - Simulink library for gss objects manipulation
            %
            % - Miscellaneous
            %     checkgss         - check whether a gss object is consistent
            %     distgss          - compute the distance between two gss objects
            %
            % Type 'help gss/fun' if fun is an overloaded function and 'help fun'
            % otherwise to get detailed information about the function fun.
            %
            % SMAC TOOLBOX - GSS LIBRARY
            % See http://w3.onera.fr/smac/gss for more info!

