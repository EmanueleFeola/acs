classdef CylinderLink < LinkBody   
    properties
        name string
        radius double
        length_ double
    end
    
    methods
        function this = CylinderLink(name, mass, radius, length_)
            this.name = name;
            this.radius = radius;
            this.length_ = length_;
            this.link_mass = mass;            
        end
    end
    
    methods
        function [] = computeInertiaTensor(this)
            % NB: the cylinder rotation axis is the x-axis
            
            % compute inertia tensor wrt to the link's center of mass
            % != compute inertia tensor wrt to frame i
            % != compute inertia tensor wrt to frame 0
            
            Ixx = (2*this.radius)^2;
            Iyy = 3*(2*this.radius)^2 + this.length_^2;
            Izz = Iyy;
            
            I = this.link_mass * [(1/2)*Ixx 0 0; 0 (1/2)*Iyy 0; 0 0 (1/2)*Izz;];
            %I = zeros(3); % debug without inertia
            
            this.I_num = I;
        end
    end
end