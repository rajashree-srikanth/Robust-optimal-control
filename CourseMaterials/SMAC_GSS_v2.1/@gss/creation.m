            %% 
            % -----------------------------------------------------------------------
            % Matlab gss object creation
            % -----------------------------------------------------------------------
            %
            % There are different ways  to create gss objects, using  mainly the core
            % function gss, but also the functions sym2gss and data2gss.
            %
            % 1. Conversion of an existing object
            %        sys=gss(sys1)
            %    The input argument sys1 can be a gss object,  an lfr object obtained
            %    with the LFR Toolbox, a uss object  obtained with the Robust Control
            %    Toolbox, a ss/tf/zpk object or a standard matrix. Note that the call
            %    sys=gss can be used to create an empty gss object.
            %
            % 2. Creation from variables M and D
            %        sys=gss(M,D)
            %    M is a  ss/tf/zpk object or  a numeric array.  D is a  1xN structure
            %    array with  fields Name, Type, Size, NomValue, Bounds, Distribution,
            %    RateBounds, Normalization and Misc. Name must be defined for each Di
            %    block, whereas the other fields  can be empty/omitted. Type and Size
            %    are set to 'PAR' and [1 1] respectively if empty/omitted. A shortcut
            %    can be  used for Distribution, which can be reduced to a string with
            %    the distribution type or a 1x2 cell array with the distribution type
            %    and the  corresponding parameters. For example,  {'normal' [0 1]} is
            %    equivalent to struct('Type','normal','Parameters',[0 1]).
            %
            % 3. Creation of elementary gss objects with a single block
            %    3.1 Creation by properties and values
            %            sys=gss('Property1',Value1,'Property2',Value2,...)
            %        The allowed properties  are Name, Type,  Size, NomValue, Bounds,
            %        Distribution, RateBounds, Normalization and Misc.  Only Name and
            %        Type have  to be defined. If  undefined,  Size is set  to [1 1], 
            %        whereas the other properties are left empty.
            %        Ex: a=gss('Name','a','Type','PAR','Size',[1 1],'Bounds',[2 4])
            %    3.2 Creation by values only
            %            sys=gss(Name,Type,Size,NomValue,Bounds,Distribution,...
            %                    RateBounds,Normalization,Misc)
            %        The values must be given  in the right order. Only Name and Type
            %        have to be defined.  If empty or omitted,  Size is set to [1 1],
            %        whereas the other properties are left empty.
            %        Ex: a=gss('a','PAR',[1 1],[],[2 4])
            %    3.3 Use of a simplified call in the parametric case
            %            sys=gss(Name,NomValue,Bounds,Distribution,RateBounds)
            %        The values must  be given in  the right order.  Only Name has to
            %        be defined. NomValue=0 and  Bounds=[-1 1] if both are undefined,
            %        NomValue=mean(Bounds)  if only  Bounds is  defined,  and Bounds=
            %        [NomValue-1 NomValue+1]  if only NomValue is defined. If undefi-
            %        ned, Distribution.Type='uniform', Distribution.Parameters=Bounds
            %        and RateBounds=[0 0].  Finally, Type, Size, Normalization & Misc
            %        are always set to 'PAR', [1 1], [] and [] respectively.
            %    In all 3 cases, a shortcut can be used for Distribution.  The latter
            %    can indeed  be reduced  to a string with the  distribution type or a
            %    1x2 cell array with the distribution type and the corresponding para
            %    meters. For example, in the call a=gss('a',3,[2 4],Distribution), it
            %    is equivalent to set Distribution={'normal' [0 1]} or  Distribution=
            %    struct('Type','normal','Parameters',[0 1]).
            %    Starting from  R2014b, the  output argument  can be omitted  when an
            %    elementary gss object is created. In  other words, gss('a',[],[2 4])
            %    is a shortcut for a=gss('a',[],[2 4]).
            %
            % 4. Use of the reserved names Int and Delay
            %    Int and Delay can be used to create elementary gss objects such that
            %    M(s)=1/s and M(z)=1/z respectively, and Delta is empty.
            %    Ex: Int=gss('Int')
            %
            % 5. Interconnection of gss objects
            %    The overloaded functions plus, minus, uminus, mtimes, inv, mrdivide,
            %    mldivide, mpower,  horzcat, vertcat, append, transpose,  ctranspose,
            %    ss, tf, feedback can be applied to gss objects.  Type help gss/fname
            %    to get detailed information about the function fname.
            %
            % 6. Application of the structured tree decomposition algorithm to a sym-
            %    bolic polynomial  expression to obtain a low-order  gss object. Type
            %    'help sym2gss' to get detailed information.
            %
            % 7. Sparse polynomial  or rational approximation  to obtain  a low-order
            %    gss object from tabulated data. Type 'help data2gss' to get detailed
            %    information. See also the APRICOT library of the SMAC toolbox, which
            %    is available at http://w3.onera.fr/smac/apricot.
            %
            % Warning: A systematic order reduction  is performed by default with the
            %          function mingss each time a gss object is created or an elemen
            %          tary operation is applied to an existing gss object (addition,
            %          multiplication, division, concatenation...) . This setting can
            %          be changed using the function setred.
            %
            % Warning: No systematic normalization  is performed by  default, but PAR
            %          and LTI blocks  are normalized in case  of inversion or incon-
            %          sistency problem with the function dbnorm. This setting can be
            %          changed using the function setnorm.
            %
            % Warning: In case of an unexpected  error, use the function  checkgss to
            %          check whether the considered gss objects are consistent.
            %
            % SMAC TOOLBOX - GSS LIBRARY
            % See http://w3.onera.fr/smac/gss for more info!

