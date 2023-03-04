classdef CuboidLink < LinkBody
    properties
        a sym % cuboid dimension 1
        b sym % cuboid dimension 2
        c sym % cuboid dimension 3
    end
    
    methods
        function this = CuboidLink(mass, a, b, c)          
            this.a = a;
            this.b = b;
            this.c = c;
            this.link_mass = mass;            
        end
    end
    
    methods
        function [] = computeInertiaTensor(this)
            % compute inertia tensor wrt to link center of mass
            % != compute inertia tensor wrt to frame i
            % != compute inertia tensor wrt to frame 0
            
            Ixx = (this.b^2) + (this.c^2);
            Iyy = (this.a^2) + (this.c^2);
            Izz = (this.a^2) + (this.b^2);
            
            I = (1/12) * this.link_mass * [Ixx 0 0; 0 Iyy 0; 0 0 Izz;];
            I = simplify(I);
            
            this.I_sym = I;
        end
    end
end