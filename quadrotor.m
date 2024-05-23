classdef quadrotor < system_component
    % QUADROTOR Represents a quadrotor component of a system

    methods(Static, Access = 'private')
        function params = get_default_params()
            % GET_DEFAULT_PARAMS returns a structure containing parameters for a quadrotor with links.
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       params: Structure containing the following fields:
            %           * m: Mass of each quadrotor (kg)
            %           * J: Inertia matrix of each quadrotor (kg.m^2)

            params.m = 0.755;
            params.J = diag([0.0820, 0.0845, 0.1377]);

        end
    end

    methods
        function obj = quadrotor(m, J)
            % QUADROTOR Constructor
            %   Inputs:
            %      m: Mass of the quadrotor (kg)
            %      J: Inertia matrix of the quadrotor (kg.m^2)
            %
            %   Outputs:
            %       None
            if nargin < 1
                obj.params = quadrotor.get_default_params();
            else
                obj.params.m = m;
                obj.params.J = J;
            end
        end
    end

end
