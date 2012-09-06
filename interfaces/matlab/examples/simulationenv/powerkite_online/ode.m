function [ dx ] = ode( t,x,u,p,w )

%% DIFFERENTIAL STATES :
   r = x(1);                   %  the length r of the cable
   phi = x(2);                 %  the angle phi
   theta = x(3);               %  the angle theta
   dr = x(4);                  %  first  derivative of r0    with respect to t
   dphi = x(5);                %  first  derivative of phi   with respect to t
   dtheta = x(6);              %  first  derivative of theta with respect to t
   n = x(7);                   %  winding number
   Psi = x(8);                 %  the roll angle Psi
   CL = x(9);                  %  the aerodynamic lift coefficient
   W = x(10);                  %  integral over the power at the generator
                               %  ( = ENERGY )
    

%% CONTROL :
   ddr0 = u(1);                %  second derivative of r0    with respect to t
   dPsi = u(2);                %  first  derivative of Psi   with respect to t
   dCL = u(3);                 %  first  derivative of CL    with respect to t
   

%% DISTURBANCE
   w_extra = w(1);
   

%% PARAMETERS :
                               %  PARAMETERS OF THE KITE :
            mk =  850.00;      %  mass of the kite               %  ( kg    )
             A =  500.00;      %  effective area                 %  ( m^2   )
             V =  720.00;      %  volume                         %  ( m^3   )
           cd0 =    0.04;      %  aerodynamic drag coefficient   %  (       )
                                     %  ( cd0: without downwash )
             K =    0.04;      %  induced drag constant          %  (       )


                               %  PHYSICAL CONSTANTS :
             g =    9.81;      %  gravitational constant         %  ( m /s^2)
           rho =    1.23;      %  density of the air             %  ( kg/m^3)

                               %  PARAMETERS OF THE CABLE :
          rhoc = 1450.00;      %  density of the cable           %  ( kg/m^3)
            cc =    1.00;      %  frictional constant            %  (       )
            dc = 0.05614;      %  diameter                       %  ( m     )


                               %  PARAMETERS OF THE WIND :
            w0 =   10.00;      %  wind velocity at altitude h0   %  ( m/s   )
            h0 =  100.00;      %  the altitude h0                %  ( m     )
            hr =    0.10;      %  roughness length               %  ( m     )
   
   

            
%% Equations            
            
% SPRING CONSTANT OF THE CABLE :
   AQ      =  pi*dc*dc/4.0                                       ;

% THE EFECTIVE MASS' :
   mc      =  rhoc*AQ*r        ;   % mass of the cable
   m       =  mk + mc     / 3.0;   % effective inertial mass
   m_      =  mk + mc     / 2.0;   % effective gravitational mass
   dm      =  (rhoc*AQ/ 3.0)*dr;   % time derivative of the mass


% WIND SHEAR MODEL :
   h       =  r*cos(theta)                                       ;
   w       =  log(h/hr) / log(h0/hr) *(w0+w_extra)               ;


% EFFECTIVE WIND IN THE KITE`S SYSTEM :
   we(1)   =  w*sin(theta)*cos(phi) -              dr    ;
   we(2)   = -w*sin(phi)            - r*sin(theta)*dphi  ;
   we(3)   = -w*cos(theta)*cos(phi) + r           *dtheta;


% CALCULATION OF THE KITE`S TRANSVERSAL AXIS :
   nwep    =  (we(2)*we(2) + we(3)*we(3))^( 0.5 );
   nwe     =  (we(1)*we(1) + we(2)*we(2) + we(3)*we(3))^(0.5);
   eta     =  asin( we(1)*tan(Psi)/ nwep )                       ;

 % ewep(1) =  0.0                                                ;
   ewep(2) =  we(2) / nwep                                       ;
   ewep(3) =  we(3) / nwep                                       ;

   et(1) =  sin(Psi)                                                  ;
   et(2) =  (-cos(Psi)*sin(eta))*ewep(2) - (cos(Psi)*cos(eta))*ewep(3);
   et(3) =  (-cos(Psi)*sin(eta))*ewep(3) + (cos(Psi)*cos(eta))*ewep(2);


% CONSTANTS FOR SEVERAL FORCES :
   Cg      =  (V*rho-m_)*g                                       ;
   Caer    =  (rho*A/2.0 )*nwe                                   ;
   Cf      =  (rho*dc/8.0)*r*nwe                                 ;


% THE DRAG-COEFFICIENT :
   CD      =  cd0 + K*CL*CL                                      ;



% SUM OF GRAVITATIONAL AND LIFTING FORCE :
   Fg(1) =  Cg  *  cos(theta)                                  ;
 % Fg(2) =  Cg  *  0.0                                         ;
   Fg(3) =  Cg  *  sin(theta)                                  ;


% SUM OF THE AERODYNAMIC FORCES :
   Faer(1) =  Caer*( CL*(we(2)*et(3)-we(3)*et(2)) + CD*we(1) )   ;
   Faer(2) =  Caer*( CL*(we(3)*et(1)-we(1)*et(3)) + CD*we(2) )   ;
   Faer(3) =  Caer*( CL*(we(1)*et(2)-we(2)*et(1)) + CD*we(3) )   ;


% THE FRICTION OF THE CABLE :
 % Ff(1) =  Cf  *  cc* we(1)                                   ;
   Ff(2) =  Cf  *  cc* we(2)                                   ;
   Ff(3) =  Cf  *  cc* we(3)                                   ;



% THE TOTAL FORCE :
   F(1) = Fg(1) + Faer(1)                                     ;
   F(2) =         Faer(2) + Ff(2)                             ;
   F(3) = Fg(3) + Faer(3) + Ff(3)                             ;



% THE PSEUDO ACCELERATION:
   a_pseudo(1) =  - ddr0 ...
                   + r*(                         dtheta*dtheta ...
                         + sin(theta)*sin(theta)*dphi  *dphi   ) ...
                   - dm/m*dr                                     ;

   a_pseudo(2) =  - 2.0*cos(theta)/sin(theta)*dphi*dtheta ...
                   - 2.0*dr/r*dphi ...
                   - dm/m*dphi                                   ;

   a_pseudo(3) =    cos(theta)*sin(theta)*dphi*dphi ...
                   - 2.0*dr/r*dtheta ...
                   - dm/m*dtheta                                 ;




% THE EQUATIONS OF MOTION:
   ddr          =  F(1)/m                + a_pseudo(1)           ;
   ddphi        =  F(2)/(m*r*sin(theta)) + a_pseudo(2)           ;
   ddtheta      = -F(3)/(m*r           ) + a_pseudo(3)           ;





% THE DERIVATIVE OF THE WINDING NUMBER :
   dn           =  (        dphi*ddtheta - dtheta*ddphi     ) / ...
                   (2.0*pi*(dphi*dphi    + dtheta*dtheta)   )      ;



% THE POWER AT THE GENERATOR :
   power        =   m*ddr*dr                                     ;



% REGULARISATION TERMS :
   regularisation =    5.0e2 * ddr0    * ddr0 ...
                     + 1.0e8 * dPsi    * dPsi ...
                     + 1.0e5 * dCL     * dCL ...
                     + 2.5e5 * dn      * dn ...
                     + 2.5e7 * ddphi   * ddphi ...
                     + 2.5e7 * ddtheta * ddtheta ...
                     + 2.5e6 * dtheta  * dtheta;



% THE "RIGHT-HAND-SIDE" OF THE ODE:

   dx(1)    =  dr                             ;
   dx(2)    =  dphi                           ;
   dx(3)    =  dtheta                         ;
   dx(4)    =  ddr0                           ;
   dx(5)    =  ddphi                          ;
   dx(6)    =  ddtheta                        ;
   dx(7)    =  dn                             ;
   dx(8)    =  dPsi                           ;
   dx(9)    =  dCL                            ;
   dx(10)   = (-power + regularisation)*1.0e-6;            

end
