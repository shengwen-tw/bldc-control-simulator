classdef bldc_dynamics
    properties
        R = 0.7;          %resistance [ohm]
        L = 2.72;         %self-inductance [mH]
        M = 1.5;          %mutual-inductance [mH]
        lambda_m = 0.105; %flux linkage of the motor [wb]
        P = 4;            %number of the motor poles
        Jm = 0.000284;    %inertia of the motor [kg*m/sec^2]
        Jl = 0;           %inertia of the load [kg*m/sec^2]
        B = 0.02;         %motor viscous friction constant [N*m/rad/sec]
        Kt = 1;           %torque-current constant
    end
    
    methods
        
        function retval = back_emf_fa(obj, theta_r)
            theta_r = mod(theta_r, 2*pi); %restrict theta_r in [0, 2*pi]
            
            if (theta_r >= 0) && (theta_r < pi/6)
                retval = theta_r * 6 / pi;
            elseif (theta_r >= pi/6) && (theta_r < 5*pi/6)
                retval = 1;
            elseif (theta_r >= 5*pi/6) && (theta_r < 7*pi/6)
                retval = (pi - theta_r) * 6 / pi;
            elseif (theta_r >= 7*pi/6) && (theta_r < 11*pi/6)
                retval = -1;
            elseif (theta_r >= 11*pi/6) && (theta_r <= 2*pi)
                retval = (theta_r - 2*pi) * 6 / pi;
            end
        end
        
        function retval = back_emf_fb(obj, theta_r)
            theta_r = theta_r - deg2rad(120);
            retval = back_emf_fa(obj, theta_r);
        end
        
        function retval = back_emf_fc(obj, theta_r)
            theta_r = theta_r - deg2rad(240);
            retval = back_emf_fa(obj, theta_r);
        end
    end
end