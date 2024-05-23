clc; clear; close all;
%% Manual Configuration of System Parameters
pyld = payload();
for n = 3:3
    
    %% Default Configuration of System Parameters 
    syst = QuadPayLoadSystem(n);
    
    % system parameters
    params = syst.params;
    
    % system initial conditions
    init_vals = syst.initial_conditions;
    
    % Inputs
    total_weight = (sum(params.m)+params.m0)*params.g;
    drone_w = params.m(1)*params.g;
    payload_w = params.m0*params.g;
    
    M = zeros(3, params.n); % Moments(N.m)
    
    %% Quads Configuration
    
    conf = quads_config(pyld,n);
    
    % Plotting
    f = figure;
    hold on;
    w = pyld.params.payload_dims.x;
    h = pyld.params.payload_dims.y;
    hRect = rectangle('Position',...
        [-w/2, -h/2, w, h], 'linewidth',3);
    hRectDummy = plot(nan, nan, 'k-', 'LineWidth', 2);

    hPoints = plot(conf.rhos(1,:),conf.rhos(2,:), ...
        'r*', 'MarkerSize',12, 'LineWidth',2);
    center = mean(conf.rhos,2);
    hCenter = plot(center(1),center(2), ...
        'bo', 'MarkerSize',12, 'LineWidth',2);
    new_limits = utilities.edit_limits(axis, 0.2);
    axis(new_limits)
    % Simulate constant input
    t = [0:0.01:10];
    states_in = utilities.states_struct_to_vec(init_vals);
    syst = syst.set_rhos(conf.rhos);
    F = total_weight/n*ones(1,n);
    output = syst.simulate_dynamics(t,states_in,F,M);
    text(-0.1,-0.1, ['$Z$-change: ' num2str(mean(output.y(3,:)))],...
        'Interpreter', 'latex', 'FontSize',15)
    xlabel('X-Dimension', 'Interpreter','latex', 'FontSize', 15)
    ylabel('Y-Dimension', 'Interpreter','latex', 'FontSize', 15)
    if n == 1
        legend([hRectDummy, hPoints, hCenter],...
            {'Payload Boudaries', 'Points of attachments', 'Center of the system'}, ...
        'location', 'best','Interpreter','latex', 'Fontsize', 12)
    end

    % saveas(f,['figure' num2str(n) '.png'])
    % close all;
end
states_struct = utilities.states_vec_to_struct(output.x, output.y,syst.params);
save('states.mat','states_struct')

