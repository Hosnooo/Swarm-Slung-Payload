classdef QuadPayLoadSystem < system_component
    % QUADPAYLOADSYSTEM Represents a quadrotor payload system

    properties
        initial_conditions
    end

    methods(Static, Access= 'private')
        function params = get_default_system(n)
            % GET_DEFAULT_SYSTEM returns a structure containing default parameters for the quadrotor payload system.
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       params: Structure containing default parameters for the system
            
            params.n = n;
            params.m0 = 1.5;
            
            payload_dims.x = 1;
            payload_dims.y = 0.8;
            payload_dims.z = 0.2;
            
            params.J0 = 1/12 * params.m0 * ...
                diag([payload_dims.y^2 + payload_dims.z^2,...
                payload_dims.x^2 + payload_dims.z^2,...
                payload_dims.x^2 + payload_dims.y^2]);
            
            params.m = 0.755 * ones(n, 1);
            params.J = repmat(diag([0.0820, 0.0845, 0.1377]), [1, 1, n]);
            params.L = ones(n, 1);
            params.rho = zeros(3,n);
                       
            params.P=zeros(6,3*params.n);
            for i=1:params.n
                params.P(:,1+3*(i-1):3*i)=[eye(3); utilities.Hat_operator(params.rho(:,i))];
            end
        end

        function init_vals = get_initials(obj)
            % GET_INITIALS returns initial values for a quadrotor system.
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       init_vals: Structure containing the following initial values:
            %           - x0: Initial position of the payload (m)
            %           - v0: Initial velocity of the payload (m/s)
            %           - R0: Initial attitude of the payload (3x3 matrix)
            %           - omega0: Initial angular velocity of the payload (rad/s)
            %           - q: Initial orientations of the quadrotors (3x1 vector)
            %           - R: Initial attitudes of the quadrotors (3x3xN array)
            %           - omega: Initial angular velocities of the quadrotors (3xN matrix)
            %           - omega_link: Initial angular velocities of the links (3xN matrix)
            
            % Number of quadrotors
            n = obj.params.n;
            
            init_vals = struct();
            
            init_vals.x0 = [1; 4.8; 0];
            init_vals.v0 = [0; 0; 0];
            init_vals.R0 = eye(3);
            init_vals.omega0 = [0; 0; 0];
            init_vals.q = repmat([0; 0; 1], 1, n);
            init_vals.R = repmat(eye(3), 1, 1, n);
            init_vals.omega = zeros(3, n);
            init_vals.omega_link = zeros(3, n);
            
        end
        
        % Dynamics
        function dydt = dynamic_model(obj,t, y, F, M)
            % DYNAMIC_MODEL calculates the dynamics of the quadrotor payload system.
            %
            %   Inputs:
            %       t: Time
            %       y: State vector
            %       F: Control forces
            %       M: Control moments
            %
            %   Outputs:
            %       dydt: Derivative of the state vector with respect to time
            
            % Function Handles
            Hat_operator = @ utilities.Hat_operator;
            % Parameters
            n = obj.params.n;
            m0 = obj.params.m0;
            J0 = obj.params.J0;
            mi = obj.params.m;
            Ji = obj.params.J;
            Li = obj.params.L;
            rhoi = obj.params.rho;

            % Model Constants
            g = 9.81;
            e3 = [0; 0; 1]; % Inertial Z unit vector
    
            % Unpack the state variables
            x0 = y(1:3);  % Payload position
            v0 = y(4:6);  % Payload velocity
            R0 = reshape(y(7:15), 3, 3);  % Payload attitude
            omega0 = y(16:18);  % Payload angular velocity
            q = reshape(y(19:19+3*n-1), 3, n);  % Link orientations
            R = reshape(y(19+3*n:19+12*n-1), 3, 3, n);  % Quadrotor attitudes
            omega = reshape(y(19+12*n:19+15*n-1), 3, n);  % Quadrotor angular velocities
            omega_link = reshape(y(19+15*n:19+18*n-1), 3, n);  % Link angular velocities
            
            % Preallocate derivatives
            dydt = zeros(size(y));
            
            % Compute derivatives
    
            % Packets
            s1_mats = {}; s1_sum = zeros(3,3); % m_q_qt
            s2_mats = {}; s2_sum = zeros(3,3); % m_q_qt_R0_rho
            s3_mats = {}; s3_sum = zeros(3,1); % u_m_q_qt_R0_omega02_rho
            s4_mats = {}; s4_sum = zeros(3,3); % m_rho_R0t_q_qt_R0_rho
            s5_mats = {}; s5_sum = zeros(3,3); % m_rho_R0t_q_qt
            s6_mats = {}; s6_sum = zeros(3,1); % rho_R0t_u_m_q_qt_R0_omega02_rho
            quad_inertia = {}; quad_inertia_sum = zeros(3,3); 
        
            for i=1:n
                % Control Input
                u_i = -F(i)*R(:,:,i)*e3;  % Total thrust for each quadrotor
                u_i_parr = q(:,i)*(q(:,i)')*u_i;  % Parallel component of control input
                u_i_perp = (eye(3) - q(:,i)*(q(:,i)'))*u_i;  % Perpendicular component of control input
                
                % Total inertia of the quadrotors
                quad_inertia{i} = mi(i)*Hat_operator(rhoi(:,i))*Hat_operator(rhoi(:,i));
                quad_inertia_sum = quad_inertia_sum + quad_inertia{i};
        
                % Summation Matrices
                s1_mats{i} = mi(i)*q(:,i)*(q(:,i)');  % Matrix involving quadrotor mass and orientation
                s1_sum = s1_sum + s1_mats{i};
                
                s2_mats{i} = mi(i)*q(:,i)*(q(:,i)')*R0*Hat_operator(rhoi(:,i));  % Matrix involving quadrotor mass, orientation, payload attitude, and link attachment point
                s2_sum = s2_sum + s2_mats{i};
        
                % Calculation of additional terms involved in the differential equations
                s3_mats{i} = u_i_parr - mi(i)*Li(i)*norm(omega_link(:,i))^2*q(:,i) ... 
                - mi(i)*q(:,i)*(q(:,i)')*R0*Hat_operator(omega0)*Hat_operator(omega0)*rhoi(:,i);
                s3_sum = s3_sum + s3_mats{i};
                
                s4_mats{i} = mi(i)*Hat_operator(rhoi(:,i))*R0'*q(:,i)*(q(:,i)')*R0*Hat_operator(rhoi(:,i));
                s4_sum = s4_sum + s4_mats{i};
        
                s5_mats{i} = mi(i)*Hat_operator(rhoi(:,i))*R0'*q(:,i)*(q(:,i)');
                s5_sum = s5_sum + s5_mats{i};
        
                s6_mats{i} = Hat_operator(rhoi(:,i))*R0'*...
                (u_i_parr - mi(i)*Li(i)*norm(omega_link(:,i))^2*q(:,i) ... 
                - mi(i)*q(:,i)*(q(:,i)')*R0*Hat_operator(omega0)*Hat_operator(omega0)*rhoi(:,i));
                s6_sum = s6_sum + s6_mats{i};
        
            end
            
            J_tot = J0 - quad_inertia_sum;  % Total inertia of the payload and quadrotors
        
            % Calculation of terms involved in the differential equations
            Mq = m0*eye(3) + s1_sum;
            omega0_dot_term = (J0 - s4_sum)\(s6_sum - Hat_operator(omega0)*J0*omega0);
            omega0_dot_coeff_v0_dot = - (J0 - s4_sum)\s5_sum;
            main_coeff_v0_dot = (Mq - s2_sum*omega0_dot_coeff_v0_dot);
            
            % dx_0/dt
            dydt(1:3) = v0;
            
            % d2x_0/dt2
            dydt(4:6) = main_coeff_v0_dot\(s3_sum + s2_sum*omega0_dot_term) + g*e3;
            x0_ddot = dydt(4:6);
        
            % dR_0/dt
            R0_dot = R0*Hat_operator(omega0);
            dydt(7:15) = reshape(R0_dot, [], 1);
            
            % domega_0/dt
            dydt(16:18) = (J0 - s4_sum)\...
                (s6_sum - Hat_operator(omega0)*J0*omega0 - s5_sum*(x0_ddot - g*e3));
            omega0_dot = dydt(16:18);
        
            for i = 1:n
                       
                % dq/dt
                q_dot = Hat_operator(omega_link(:,i))*q(:,i);
                dydt(19+3*(i-1):19+3*i-1) = q_dot;
                
                % dR/dt
                R_dot = R(:,:,i)*Hat_operator(omega(:,i));
                dydt(19+3*n+9*(i-1):19+3*n+9*i-1) = reshape(R_dot, [], 1);
                
                % domega/dt
                dydt(19+12*n+3*(i-1):19+12*n+3*i-1) = Ji(:,:,i)\...
                    (M(:,i) - cross(omega(:,i), Ji(:,:,i)*omega(:,i)));
                
                % domega_link/dt
                dydt(19+15*n+3*(i-1):19+15*n+3*i-1) = 1/Li(i)*Hat_operator(q(:,i))*...
                    (x0_ddot - g*e3 - R0*Hat_operator(rhoi(:,i))*omega0_dot +...
                    R0*Hat_operator(omega0)*Hat_operator(omega0)*rhoi(:,i))...
                    - 1/(mi(i)*Li(i))*Hat_operator(q(:,i))*u_i_perp;
        
                % % dx/dt
                % dydt(19+18*n+3*(i-1):19+18*n+3*i-1) = v0 + R0_dot*rhoi(:,i) - Li(i)*q_dot;
                % 
                % % dv/dt
                % dydt(19+21*n+3*(i-1):19+21*n+3*i-1) = x0_ddot +...
                %     R0_dot*Hat_operator(omega0)*rhoi(:,i) -...
                %     Li(i)*Hat_operator(omega_link(:,i))*q_dot;
        
            end
        end

    end

    methods
        % Constructor
        function obj = QuadPayLoadSystem(n, list_of_Quads, list_of_Links, payload)
            % QUADPAYLOADSYSTEM Constructor
            %   Inputs:
            %       n: Number of quadrotors
            %       list_of_Quads: Array of Quadrotor objects
            %       list_of_Links: Array of Link objects
            %       payload: Payload object
            %
            %   Outputs:
            %       None
            
            if nargin < 2
                obj.params = QuadPayLoadSystem.get_default_system(n);
            else
                if length(list_of_Quads) ~= n || length(list_of_Links) ~= n
                    error('N is not = Number of quadrotors, is not = Number of links')
                end
                obj.params.n = n;
                obj.params.m0 = payload.params.m0;
                obj.params.J0 = payload.params.J0;        
                
                obj.params.L = []; obj.params.rho = [];
                obj.params.m = []; obj.params.J = [];
                obj.params.P=zeros(6,3*n);
                for i = 1:n
                    obj.params.L = [obj.params.L, list_of_Links(i).params.L];
                    obj.params.rho = [obj.params.rho,...
                        reshape(list_of_Links(i).params.rho,3,1)];
                    obj.params.P(:,1+3*(i-1):3*i)=...
                        [eye(3); utilities.Hat_operator(list_of_Links(i).params.rho)];
                    obj.params.m = [obj.params.m, list_of_Quads(i).params.m];
                    obj.params.J = cat(3,obj.params.J, list_of_Quads(i).params.J);
                end
            end
            obj.initial_conditions = QuadPayLoadSystem.get_initials(obj);
            obj.params.g = 9.81;
        end
        
        % Simulation
        function states_out = simulate_dynamics(obj, t_vec, states_in, F, M)
            % SIMULATE
            % This method is used to simulate the system dynamics using a provided set of initial states,
            % control forces (F), and control moments (M).
            %
            % Inputs:
            %   t_vec: Time vector for simulation
            %   states_in: Initial state vector of the system
            %   F: Control forces applied to the system
            %   M: Control moments applied to the system
            %
            % Output:
            %   states_out: A struct representing the output of the simulation
            %
            % Example usage:
            %   states_out = simulate(time_vector, initial_states, control_forces, control_moments);
            %
            % This function integrates the system dynamics over time using MATLAB's ode45 solver.
            % The resulting states_out vector contains the state of the system at different time points.
            %
            % Inside this method, the dynamic_model method is used to compute the derivatives of the state
            % vector with respect to time, which represents the system's dynamic behavior.
            
            % Call the ode45 solver to integrate the system dynamics over time
            func = @(t,y) QuadPayLoadSystem.dynamic_model(obj, t, y, F, M);
            states_out = ode45(func, t_vec, states_in);
        end

        % Setter of Quad-hanging location
        function obj = set_rhos(obj,rhos)
            obj.params.rho = rhos;
        end

    end
end