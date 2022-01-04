classdef bldc_dynamics
    properties
        dt;               %time period of one iteration
        
        R = 0.7;          %resistance [ohm]
        L = 2.72;         %self-inductance [mH]
        M = 1.5;          %mutual-inductance [mH]
        lambda_m = 0.105; %flux linkage of the motor [wb]
        P = 4;            %number of the motor poles
        Jm = 0.000284;    %inertia of the motor [kg*m/s^2]
        Jl = 0;           %inertia of the load [kg*m/s^2]
        J = 0;            %J = Jm + Jl
        B_vis = 0.02;     %motor viscous friction constant [N*m/rad/s]
        Kt = 0.2;         %torque-current constant
        
        f_a = 0;          %back EMF of the phase A
        f_b = 0;          %back EMF of the phase B
        f_c = 0;          %back EMF of the phase C
        
        %matrices of the LTV (Linear Time Variant) system
        A = zeros(5, 5);  %state transition matrix
        B = zeros(5, 4);  %control matrix
        C = zeros(5, 3);  %back emf matrix
        
        %state vector = [i_a; i_b; i_c; omega_m; theta_r]
        x = [0;            %initial phase A current
             0;            %initial phase B current
             0;            %initial phase C current
             deg2rad(0);   %initial motor speed
             deg2rad(0)];  %initial motor position
        u = zeros(4, 1);   %control vector = [v_a; v_b; v_c; T_l]
        e = zeros(3, 1);   %back emf vector = [e_a; e_b; e_c]
        
        x_dot = zeros(5, 1);
        
        torque = 0;        %torque of the motor
                
        %BLDC control voltage
        v_bldc = 100;
    end
    
    methods
        function ret_obj = init(obj, dt)
            obj.dt = dt;
            
            %J
            obj.J = obj.Jm + obj.Jl;
            
            %A
            obj.A(1, 1) = -obj.R/(obj.L-obj.M);
            obj.A(2, 2) = -obj.R/(obj.L-obj.M);
            obj.A(3, 3) = -obj.R/(obj.L-obj.M);
            obj.A(4, 4) = -obj.B_vis/obj.J;
            obj.A(5, 4) = obj.P/2;
            
            %B
            obj.B(1, 1) = 1/(obj.L-obj.M);
            obj.B(2, 2) = 1/(obj.L-obj.M);
            obj.B(3, 3) = 1/(obj.L-obj.M);
            obj.B(4, 4) = 1;
             
            %C
            obj.C(1, 1) = -1/(obj.L-obj.M);
            obj.C(2, 2) = -1/(obj.L-obj.M);
            obj.C(3, 3) = -1/(obj.L-obj.M);
            
            ret_obj = obj;
        end
        
        function ret_obj = new_dynamics(obj)                        
            %calculate normalized back-emf voltage
            obj.f_a = obj.back_emf_fa(obj.x(5));
            obj.f_b = obj.back_emf_fb(obj.x(5));
            obj.f_c = obj.back_emf_fc(obj.x(5));
            
            %calculate back-emf voltage
            omega_times_lambda = obj.x(4) * obj.lambda_m;
            obj.e(1) = omega_times_lambda * obj.f_a;
            obj.e(2) = omega_times_lambda * obj.f_b;
            obj.e(3) = omega_times_lambda * obj.f_c;
            
            %update the time-variant entry of the matrix A
            lambda_div_J = obj.lambda_m / obj.J;
            obj.A(4, 1) = lambda_div_J * obj.f_a;
            obj.A(4, 2) = lambda_div_J * obj.f_b;
            obj.A(4, 3) = lambda_div_J * obj.f_c;
           
            %x_dot = Ax + Bu + Ce
            obj.x_dot = (obj.A*obj.x) + (obj.B*obj.u) + (obj.C*obj.e);
            
            ret_obj = obj;
        end
        
        function f_next = integrator(obj, f_now, f_dot, dt)
            %eulers method
            f_next = [f_now(1) + (f_dot(1) * dt);
                      f_now(2) + (f_dot(2) * dt);
                      f_now(3) + (f_dot(3) * dt);
                      f_now(4) + (f_dot(4) * dt);
                      f_now(5) + (f_dot(5) * dt)];
        end
        
        function ret_obj = update(obj)
            obj = obj.new_dynamics();
            obj.x = obj.integrator(obj.x, obj.x_dot, obj.dt);
                        
            %restrict rotor position in [0, 2*pi]
            if obj.x(5) >= 2*pi
                obj.x(5) = obj.x(5) - 2*pi;
            elseif obj.x(5) < 0
                obj.x(5) = obj.x(5) + 2*pi;
            end
            
            %calculate the motor torque
            sum_of_e_times_i = obj.e(1)*obj.x(1) +  obj.e(2)*obj.x(2) + obj.e(3)*obj.x(3);
            obj.torque = sum_of_e_times_i / obj.x(4);
            
            ret_obj = obj;
        end
        
        function retval = back_emf_fa(obj, theta_r)
            %restrict theta_r in [0, 2*pi]
            theta_r = mod(theta_r, 2*pi);
            
            %generate trapezoidal signal
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
            %phase shifting
            theta_r = theta_r - deg2rad(120);
            
            %restrict theta_r in [0, 2*pi]
            theta_r = mod(theta_r, 2*pi);
            
            retval = back_emf_fa(obj, theta_r);
        end
        
        function retval = back_emf_fc(obj, theta_r)
            %phase shifting
            theta_r = theta_r - deg2rad(240);
            
            %restrict theta_r in [0, 2*pi]
            theta_r = mod(theta_r, 2*pi);
            
            retval = back_emf_fa(obj, theta_r);
        end
    end
end