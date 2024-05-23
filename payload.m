classdef payload < system_component
    % PAYLOAD Represents a payload component of a system

    methods(Static, Access = 'private')
        function params = get_default_params()
            % GET_DEFAULT_PARAMS returns a structure containing parameters for the
            % payload
            %
            %   Inputs:
            %       None
            %
            %   Outputs:
            %       params: Structure containing the following fields:
            %           - m0: Mass of the payload (kg)
            %           - payload_dims: Structure containing dimensions of the payload:
            %               * x: Length of payload (m)
            %               * y: Width of payload (m)
            %               * z: Height of payload (m)
            %           - J0: Inertia matrix of the payload (kg.m^2)

            params.m0 = 1.5;

            params.payload_dims.x = 1;
            params.payload_dims.y = 0.8;
            params.payload_dims.z = 0.2;

            params.J0 = 1/12*params.m0*...
                diag([params.payload_dims.y^2 + params.payload_dims.z^2,...
                params.payload_dims.x^2 + params.payload_dims.z^2,...
                params.payload_dims.x^2 + params.payload_dims.y^2]);

        end
    end

    methods
        function obj = payload(m0, payload_dims)
            % PAYLOAD Constructor
            %   Inputs:
            %      m0: Mass of the payload (kg)
            %      payload_dims: Structure containing dimensions of the payload:
            %               * x: Length of payload (m)
            %               * y: Width of payload (m)
            %               * z: Height of payload (m)
            %
            %   Outputs:
            %       None
            if nargin < 1
                obj.params = payload.get_default_params();
            else
                obj.params.m0 = m0;

                obj.params.payload_dims.x = payload_dims.x;
                obj.params.payload_dims.y = payload_dims.y;
                obj.params.payload_dims.z = payload_dims.z;

                obj.params.J0 = 1/12*obj.params.m0*...
                    diag([obj.params.payload_dims.y^2 + obj.params.payload_dims.z^2,...
                    obj.params.payload_dims.x^2 + obj.params.payload_dims.z^2,...
                    obj.params.payload_dims.x^2 + obj.params.payload_dims.y^2]);

            end
        end
    end

end
