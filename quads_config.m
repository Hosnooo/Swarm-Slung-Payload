classdef quads_config
    properties
        payload_dims
        n
        rhos
    end
    
    methods(Static, Access = 'private')
        function rhos = distribute(obj)
            n = obj.n;
            width = obj.payload_dims.x; height = obj.payload_dims.y;

            % Calculate the original center of mass
            original_COM = [0; 0];
           
            % Case when only one quadrotor is added
            if n == 1
                rhos = original_COM;
            else
                % Calculate the position of each quadrotor relative to the center of mass
                % Assuming quadrotors are evenly spaced along the edges of the payload
                ang = (n - 2)*pi/n;
                theta = linspace(-ang/2, pi + ang/2, n); % Angle between each quadrotor
                if n==3
                    r = min(width, height) / 2 / cos(ang/2); % Distance of quadrotor from center
                    rhos = [original_COM(1) + ...
                        r * sin(theta); original_COM(2) + r * cos(theta)];
                else
                    r = min(width, height) / 2; % Distance of quadrotor from center
                    rhos = [original_COM(1) + ...
                        r * cos(theta); original_COM(2) + r * sin(theta)];
                end
                % 
                rhos = rhos + original_COM - mean(rhos,2);
            end
            
            % Add z component to rho
            rhos = [rhos; -obj.payload_dims.z/2*ones(1,n)];
        end

    end
    methods
        function obj = quads_config(payload, n)
            obj.payload_dims = payload.params.payload_dims;
            obj.n = n;
            obj.rhos = quads_config.distribute(obj);
        end
    end
end