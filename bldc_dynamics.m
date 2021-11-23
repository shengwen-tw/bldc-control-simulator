classdef bldc_dynamics
    properties
        R = 0.7;          %resistance [ohm]
        L = 2.72;         %self-inductance [mH]
        M = 1.5;          %mutual-inductance [mH]
        lambda_m = 0.105; %flux linkage of the motor [wb]
        P = 4;            %number of the motor poles
        Jm = 0.000284;    %inertia of the motor [kg*m/sec^2]
        Jl = 0;           %inertia of the load [kg*m/sec^2]
        J;                %J = Jm + Jl
        B_vis = 0.02;     %motor viscous friction constant [N*m/rad/sec]
        Kt = 1;           %torque-current constant
        
        fa;               %back EMF of the phase A
        fb;               %back EMF of the phase B
        fc;               %back EMF of the phase C
        
        %matrices of the LTV (Linear Time Variant) system
        A = zeros(5, 5);  %state transition matrix
        B = zeros(5, 4);  %control matrix
        C = zeros(5, 3);  %back emf matrix
        
        x = zeros(5, 1);  %state vector = [i_a; i_b; i_c; omega_m; theta_r]
        u = zeros(4, 1);  %control vector = [v_a; v_b; v_c; T_l]
        e = zeros(3, 1);  %back emf vector = [e_a; e_b; e_c]        
    end
    
    methods
        function ret_obj = init(obj)            
            obj.J = obj.Jm + obj.Jl;
            
            obj.A(1, 1) = -obj.R/(obj.L-obj.M);
            obj.A(2, 2) = -obj.R/(obj.L-obj.M);
            obj.A(3, 3) = -obj.R/(obj.L-obj.M);
            obj.A(4, 4) = -obj.B_vis/obj.J;
            obj.A(5, 4) = obj.P/2;
            
            obj.B(1, 1) = 1/(obj.L-obj.M);
            obj.B(2, 2) = 1/(obj.L-obj.M);
            obj.B(3, 3) = 1/(obj.L-obj.M);
            obj.B(4, 4) = 1;
             
            obj.C(1, 1) = -1/(obj.L-obj.M);
            obj.C(2, 2) = -1/(obj.L-obj.M);
            obj.C(3, 3) = -1/(obj.L-obj.M);
            
            ret_obj = obj;
        end
        
        function x_dot = update_dynamics(obj)
            %update the back EMF according to the motor angle
            obj.fa = obj.back_emf_fa(obj.x(5));
            obj.fb = obj.back_emf_fb(obj.x(5));
            obj.fc = obj.back_emf_fc(obj.x(5));
            
            %A matrix is time variant, update the entry corresponding to
            %the back EMF
            lambda_div_J = obj.lambda_m/obj.J;
            A(4, 1) = lambda_div_J * obj.fa;
            A(4, 2) = lambda_div_J * obj.fb;
            A(4, 3) = lambda_div_J * obj.fc;
            
            %x_dot = Ax + Bu + Ce
            x_dot = obj.A*obj.x + obj.B*obj.u + obj.C*obj.e;
        end
        
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