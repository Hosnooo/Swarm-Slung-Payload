classdef link < system_component
    % LINK Represents a link component of a system

    methods(Static)
        % No static methods defined for this class
    end

    methods
        function obj = link(L, rho)
            % LINK Constructor
            %   Inputs:
            %       L: Length of the link (m)
            %       rho: Density of the link material (kg/m^3)
            %
            %   Outputs:
            %       None
            
            obj.params.L = L;
            obj.params.rho = rho;
        end
    end
end
