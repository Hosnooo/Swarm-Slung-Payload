function n= GetNQuads(thrust_drone,drone_w,payload_w,safetyfactor,drone_radius,pyld)
    % This function  purpose is to get the number of drones required 
    % in order to carry a specific payload
    % Inputs: thrust_quad-> thrust for each drone
    %         payload_w-> weight of payload
    %          drone_w->weight of one drone
    %         safetyfactor-> usually 1.2
    %         drone_radius-> maxium distance between center of mass of drone
    %         till propeller edge
    %         pyld-> is payload object
    % Outputs: number of drones needed

    intial_n=ceil(payload_w/(thrust_drone-drone_w));
    n_safety=ceil(intial_n*safetyfactor);
    min_sidelength=3*drone_radius;
    conf_safety = quads_config(pyld,n_safety);
    conf_= quads_config(pyld,intial_n);
    factor=0.01;
    
    if n_safety ==1
        n=n_safety;
    elseif  intial_n==1 
        rhos1=conf_safety.rhos(1:2,1:2);
        distance_safety=norm(rhos1(:,1)-rhos1(:,2));
            if distance_safety <min_sidelength
                n=intial_n;
                fprintf("The drones will be able to carry payload but without safety factor\n")
                newdrone_radius=distance_safety/(3+factor);
                fprintf("xonsider a drone with radius at most %f m ",newdrone_radius);
                
            else
                n=n_safety;
            end
    
    else    
        rhos1=conf_safety.rhos(1:2,1:2);
        rhos2=conf_.rhos(1:2,1:2);
        distance_safety=norm(rhos1(:,1)-rhos1(:,2));
        distance=norm(rhos2(:,1)-rhos2(:,2));
        if distance_safety <min_sidelength
            if distance<min_sidelength
               fprintf("The drones with this thrust capacity and radius will not be able to lift payload \n")
               n=0;
               RecommendN(distance_safety,factor,drone_w,payload_w,n_safety,safetyfactor,pyld,min_sidelength) 
            else
               fprintf("The drones will be able to carry payload but without safety factor\n")
               n=intial_n;
               RecommendN(distance_safety,factor,drone_w,payload_w,n_safety,safetyfactor,pyld,min_sidelength);
            end
        else
            n=n_safety;
        end
    end
end