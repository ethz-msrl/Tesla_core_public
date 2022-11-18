classdef MultiPoleObject
    %MULTIPOLEOBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Acoeff;
        Bcoeff;
        pos;
        dir;
        paramCov;
        numSources;
    end
    
    methods
        function obj = MultiPoleObject(numSources, Acoeff, Bcoeff, pos, dir, paramCov)
           obj.Acoeff = Acoeff;
           obj.Bcoeff = Bcoeff;
           obj.pos = pos;
           obj.dir = dir;
           obj.paramCov = paramCov;
           obj.numSources = numSources;
        end
        
        function [B,G, Covariance] = FieldGradient(this,pos)
           [ B, G, dBdA, dBdB, ~, ~, dGdA, dGdB ] = CalculateField(pos, this.Acoeff, this.Bcoeff, this.pos, this.dir);
           J = [dBdA dBdB; dGdA dGdB]; 
           Covariance = J*this.paramCov*J';
        end
    end
    
end

