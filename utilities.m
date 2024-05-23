classdef utilities
    % UTILITIES Utility class containing various mathematical operations

    methods(Static)
        %% Space Mappings Methods
        
        function v = V_operator(S)
            % V_OPERATOR computes the V operator from a skew-symmetric matrix.
            %
            %   Inputs:
            %       S: 3x3 skew-symmetric matrix
            %
            %   Output:
            %       v: Vector representation obtained by extracting components from S
            %
            %   Example:
            %       S = [0 z -y; -z 0 x; y -x 0];
            %       v = V_operator(S);
            %
            %       Result:
            %       v = [x; y; z];
        
            v = [S(2, 3); S(3, 1); S(1, 2)];
        end

        function S = Hat_operator(v)
            % HAT_OPERATOR computes the Hat operator from a vector.
            %
            %   Inputs:
            %       v: 3x1 vector
            %
            %   Output:
            %       S: 3x3 skew-symmetric matrix
            %
            %   Example:
            %       v = [vx; vy; vz];
            %       S = Hat_operator(v);
            %
            %       Result:
            %       S = [0 vz -vy; -vz 0 vx; vy -vx 0];
        
            S = [0 v(3) -v(2); -v(3) 0 v(1); v(2) -v(1) 0];
        end
        
        function R = Rotate(ang,axis)
            % Rotate generates a rotation matrix based on the given angle and axis.
            %
            % Inputs:
            %   ang: Angle of rotation (in radians)
            %   axis: Axis of rotation ('x', 'y', or 'z')
            %
            % Output:
            %   R: Rotation matrix
        
            if strcmp(axis,'x')
                R = utils.R_x(ang);
            elseif strcmp(axis, 'y')
                R = utils.R_y(ang);
            elseif strcmp(axis, 'z')
                R = utils.R_z(ang);
            else
                R = eye(3);
            end
        end
        
        function Rx = R_x(Theta_x)
            Rx = [1 0 0;
                  0 cos(Theta_x) sin(Theta_x);
                  0 -sin(Theta_x) cos(Theta_x)];
        end
        
        function Ry = R_y(Theta_y)
            Ry = [cos(Theta_y) 0 -sin(Theta_y);
                  0 1 0;
                  sin(Theta_y) 0 cos(Theta_y)];
        end
        
        function Rz = R_z(Theta_z)
            Rz = [cos(Theta_z) sin(Theta_z) 0;
                  -sin(Theta_z) cos(Theta_z) 0;
                  0 0 1];
        end
        
        %% Derivative Calculation Methods

        function [dy,x] = vec_derivative(x,y,x_star)
            % VEC_DERIVATIVE Computes the derivative of a vector y with respect to x using central differences.
            %
            %   Inputs:
            %       x: Vector of independent variable values
            %       y: Vector of dependent variable values
            %       x_star (optional): Value(s) of x where the derivative is desired
            %
            %   Outputs:
            %       dy: Vector containing the derivative of y with respect to x
            %       x: Vector of independent variable values
            %
            %   Example:
            %       [dy, x] = vec_derivative(x, y);
            %       dy_at_x_star = vec_derivative(x, y, x_star);
        
            % number of subintervals
            [a,b]=size(y);
            N = b-1;

            % preallocates vector to store derivative
            dy = zeros(a,b);

            % approximates derivative at lower bound using forward difference
            dy(:,1) = (y(:,2)-y(:,1))./(x(2)-x(1));

            % approximates derivative at upper bound using backward difference
            dy(:,N+1) = (y(:,N+1)-y(:,N))./(x(N+1)-x(N));

            % approximates derivatives at all other nodes using central differences
            for i = 2:N
                dy(:,i) = (y(:,i+1)-y(:,i-1))./(x(i+1)-x(i-1));
            end

            % approximates derivative at specified points via linear interpolation
            if nargin == 3
                dy = interp1(x,dy,x_star,'linear','extrap');
            end
        end

        function dy = point_derivative(step, yi, yi_minus)
            % POINT_DERIVATIVE Computes the derivative at a point using backward difference.
            %
            %   Inputs:
            %       step: Step size between two points
            %       yi: Value at the current point
            %       yi_minus: Value at the previous point
            %
            %   Output:
            %       dy: Approximated derivative at the current point
        
            dy = (yi - yi_minus) ./ step;
        end
        
        %% Output Extraction
        function states_vec = states_struct_to_vec(states_struct)
            % STATES_STRUCT_TO_VEC
            % Convert a struct of states to a vector form.
            % 
            % Input:
            %   states_struct: Struct containing states of the system
            %       * x0, v0: Payload position and velocity
            %       * R0: Payload attitude (rotation matrix)
            %       * omega0: Payload angular velocity
            %       * q: Link orientations (rotation matrices)
            %       * R: Quadrotor attitudes (rotation matrices)
            %       * omega: Quadrotor angular velocities
            %       * omega_link: Link angular velocities
            %
            % Output:
            %   states_vec: Vector form of the states

            states_vec = [states_struct.x0;
                          states_struct.v0;
                          reshape(states_struct.R0, [], 1);
                          states_struct.omega0;
                          reshape(states_struct.q, [], 1);
                          reshape(states_struct.R, [], 1);
                          reshape(states_struct.omega, [], 1);
                          reshape(states_struct.omega_link, [], 1)];
        end

        function states_struct = states_vec_to_struct(time_vec, states_vec,params)
            % STATES_VEC_TO_STRUCT
            % Convert a vector of states to a struct form.
            % 
            % Input:
            %   states_vec: Vector containing states of the system
            %   n: Number of Quadrotors
            %
            % Output:
            %   states_struct: Struct form of the states
            %       * x0, v0: Payload position and velocity
            %       * R0: Payload attitude (rotation matrix)
            %       * omega0: Payload angular velocity
            %       * q: Link orientations (rotation matrices)
            %       * R: Quadrotor attitudes (rotation matrices)
            %       * omega: Quadrotor angular velocities
            %       * x, v: Trajectories of payload position and velocity
            % 
            % Note: Additional calculations are performed to compute x and v.
            n = params.n;
            t_length = length(time_vec);
            if size(states_vec,1) ~= t_length
                states_vec = states_vec';
            end

            % Extract payload states
            states_struct.t = time_vec;
            states_struct.x0 = states_vec(:, 1:3);
            states_struct.v0 = states_vec(:, 4:6);
            R0 = reshape(states_vec(:, 7:15)', 3, 3, []);
            states_struct.R0 = permute(R0, [2, 1, 3]);

            states_struct.eulers0 = zeros(3, t_length);

            for i = 1:t_length
               states_struct.eulers0(:, i) = rotm2eul(states_struct.R0(:, :, i)');
            end

            states_struct.omega0 = states_vec(:, 16:18);
            
            % Extract Link states
            states_struct.q = reshape(states_vec(:, 19:19 + 3*n - 1)', 3, n, []);
            states_struct.q = permute(states_struct.q, [1, 2, 3]);
            states_struct.omega_link = reshape(states_vec(:, 19 + 15*n:19 + 18*n - 1)', 3, n, []);
            states_struct.omega_link = permute(states_struct.omega_link, [1, 2, 3]);

            % Extract quadrotor states
            R = reshape(states_vec(:, 19 + 3*n:19 + 12*n -1)', 3, 3, n, []);
            states_struct.R = permute(R, [2, 1, 3, 4]);
            
            states_struct.eulers = zeros(3,n, t_length);
            for i = 1:t_length
               for j = 1:n 
                states_struct.eulers(:,j,i) = rotm2eul(states_struct.R(:, :, j,i)');
               end
            end
            
            states_struct.omega = reshape(states_vec(:, 19 + 12*n:19 + 15*n - 1)', 3, n, []);
            states_struct.omega = permute(states_struct.omega, [1, 2, 3]);
            states_struct.x = zeros(3, n, t_length);
            states_struct.v = states_struct.x;
            for i = 1:t_length
                for j = 1:n
                    states_struct.x(:, j, i) =...
                    states_struct.x0(i, :)' + ...
                    states_struct.R0(:, :, i) * params.rho(:, j)...
                    - params.L(j) * states_struct.q(:, j, i);

                    states_struct.v(:, j, i) =...
                    states_struct.v0(i, :)' + ...
                    states_struct.R0(:, :, i) *...
                    utilities.Hat_operator(states_struct.omega0(i, :)) *...
                    params.rho(:, j) - params.L(j) * ...
                    utilities.Hat_operator(states_struct.omega_link(:, j, i)) *...
                    states_struct.q(:, j, i);
                end
            end
        end
        
        %% Plotting
        function newLimits = edit_limits(axis, limitIncreaseFactor)
            %% Plotting Enhancement: Modify Axis Limits with Expansion Factor
            % This function adjusts the limits of an axis to enhance the appearance
            % of plots. It increases each limit by a factor specified by 
            % 'limitIncreaseFactor'. The function handles both positive and negative
            % limits appropriately.
            %
            % Input:
            %   - axis: The current limits of the axis to be modified, in the format
            %           [xmin, xmax, ymin, ymax].
            %   - limitIncreaseFactor: The factor by which to increase the limits.
            %
            % Output:
            %   - newLimits: The modified limits of the axis after expansion.
        
            currentLimits = axis;
            evenIndices = 2:2:numel(currentLimits);
            oddIndices = 1:2:numel(currentLimits);
            increase_arr = repelem(currentLimits(evenIndices), 2);
            increase_arr(oddIndices) = -increase_arr(oddIndices);
            newLimits = currentLimits + limitIncreaseFactor * increase_arr;
        end
    end
end
