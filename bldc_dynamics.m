classdef bldc_dynamics
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