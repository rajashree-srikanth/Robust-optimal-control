            %%
            % -----------------------------------------------------------------------
            % Example of how to manipulate gss objects
            % -----------------------------------------------------------------------
            %
            % Two methods are proposed to build the series interconnection of:
            % - a 1st order actuator with position limits
            % - a 2nd order system with uncertain frequency and damping
            %
            % The actuator model with position limits is first computed:
            %        sat=gss('position','SAT',[1 1],1,[-2 3]);
            %        actuator=sat*tf(1,[10 1]);
            %        size(actuator)
            %
            % 1. The matrices of the 2nd order system are defined as gss objects:
            %        omega=gss('frequency',2,[1 3]);
            %        xi=gss('damping',0.6,[0.3 0.9]);
            %        A=[0 1;-omega^2 -2*xi*omega];
            %        B=[0;omega^2];
            %        C=[1 0];
            %        D=0;
            %    They are turned into a dynamic model with overloaded function ss:
            %        setred('no')
            %        sys1=ss(A,B,C,D);
            %        size(sys1)
            %    Frequency is repeated 5 times if no order reduction is performed.
            %        setred('default')
            %        sys1=ss(A,B,C,D);
            %        size(sys1)
            %    Frequency is repeated 3 times if order reduction is performed.
            %
            % 2. The matrices of the 2nd order system are defined as symbolic objects
            %        syms frequency damping
            %        A=[0 1;-frequency^2 -2*damping*frequency];
            %        B=[0;frequency^2];
            %        C=[1 0];
            %        D=0;
            %    They are turned into a dynamic model with sym2gss and abcd2gss:
            %        abcd=sym2gss([A B;C D]);
            %        sys2=abcd2gss(abcd,2);
            %        set(sys2,'damping','NomValue',0.6,'Bounds',[0.3 0.9]);
            %        set(sys2,'frequency','NomValue',2,'Bounds',[1 3]);
            %        size(sys2)
            %    Frequency is repeated  twice if order reduction  is performed, which
            %    corresponds to the minimal representation.
            %
            % The actuator and the 2nd order system are finally connected:
            %        sys1=sys1*actuator;
            %        size(sys1)
            %        sys2=sys2*actuator;
            %        size(sys2)
            %
            % It can be checked that the two gss objects are equivalent:
            %        distgss(sys1,sys2)
            %
            % The gss object sys2 is normalized (e.g. for analysis purposes):
            %        sysn=dbnorm(sys2);
            %        size(sysn)
            %        s2=eval(sys2,{'damping' 'frequency'},{0.3 3});
            %        sn=eval(sysn,{'damping' 'frequency'},{-1 1});
            %        distgss(s2,sn)
            %        sysu=dbunorm(sysn);
            %        size(sysu)
            %        distgss(sys2,sysu)
            %
            % SMAC TOOLBOX - GSS LIBRARY
            % See http://w3.onera.fr/smac/gss for more info!

