classdef FixedLink < LinkBody   
    properties
        radius double
        length_ double
    end
    
    methods
        function this = FixedLink(mass, length_)                     
            this.length_ = length_;
            this.link_mass = mass;            
        end
    end
    
    methods
        function [] = computeInertiaTensor(this)
            Ixx = 0;
            Iyy = 0;
            Izz = 0;
            
            I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz;];
            
            this.I_sym = I;
        end
    end
end