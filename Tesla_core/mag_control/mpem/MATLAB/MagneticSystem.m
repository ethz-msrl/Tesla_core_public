classdef MagneticSystem < handle
    % add comments
    
    properties (SetAccess = public)
        CoilStructList;
        numCoils;
        workspaceDimensions;
        systemName;
    end
    
    methods (Static = true)
        function coilStruct = makeCoilStruct(Acoeff, Bcoeff, Plist, Zlist, percentError)
            coilStruct.A = Acoeff;
            coilStruct.B = Bcoeff;
            coilStruct.P = reshape(Plist,3,numel(Plist)/3);
            coilStruct.Z = reshape(Zlist,3,numel(Zlist)/3);
            coilStruct.numCoeff = max(length(Acoeff),length(Bcoeff));
            
            if ~exist('percentError','var')
                percentError = -1;
            end
            coilStruct.CalMeanPercentError = percentError;
        end
    end
    methods
        function this = MagneticSystem(CoilStructList, workspaceDimensions, systemName)
            this.CoilStructList =  CoilStructList;
            this.numCoils = length(CoilStructList);
            this.workspaceDimensions = workspaceDimensions;
            this.systemName = systemName;
        end
        
        function [BG, BGCurrentJacobian, BGPosJacobian] = FieldAndGradient(this, Position, CurrentVector)            
            BGCurrentJacobian = zeros(8,this.numCoils);
            BGPosJacobian = zeros(8,3);
            for coilNum = 1:this.numCoils
                for coeffNum = 1:this.CoilStructList(coilNum).numCoeff
                    [B,G, ~, ~, ~, ~, ~, ~, VecG, VecGGrad] = this.CalculateField(reshape(Position,3,1),this.CoilStructList(coilNum).A(:,coeffNum),this.CoilStructList(coilNum).B(:,coeffNum),this.CoilStructList(coilNum).P(:,coeffNum),this.CoilStructList(coilNum).Z(:,coeffNum));
                    BGPosJacobian = BGPosJacobian + [G;VecGGrad]*CurrentVector(coilNum);
                    BGCurrentJacobian(:,coilNum) = BGCurrentJacobian(:,coilNum) + [B;VecG];
                end
            end 
            
            BG = BGCurrentJacobian*reshape(CurrentVector,this.numCoils,1);
        end
        
        function A = FieldGradientActuationMatrix(this, Position)
            A = zeros(8,this.numCoils);
            for coilNum = 1:this.numCoils
                for coeffNum = 1:this.CoilStructList(coilNum).numCoeff
                    [B,~, ~, ~, ~, ~, ~, ~, VecG, ~] = this.CalculateField(reshape(Position,3,1),this.CoilStructList(coilNum).A(:,coeffNum),this.CoilStructList(coilNum).B(:,coeffNum),this.CoilStructList(coilNum).P(:,coeffNum),this.CoilStructList(coilNum).Z(:,coeffNum));
                    A(:,coilNum) = A(:,coilNum) + [B;VecG];
                end
            end
        end
        
        function [W, WrenchCurrentJacobian, WrenchPosJacobian] = WrenchOnDipole(this, ToolMoment, ToolPosition, CurrentVector)
            [BG, BGCurrentJacobian, BGPosJacobian] = FieldAndGradient(this, ToolPosition, CurrentVector);
            W = [ this.skew(ToolMoment) zeros(3,5); zeros(3,3) this.packM(ToolMoment) ]*BG;
            WrenchCurrentJacobian = [ this.skew(ToolMoment) zeros(3,5); zeros(3,3) this.packM(ToolMoment) ]*BGCurrentJacobian;
            WrenchPosJacobian = [ this.skew(ToolMoment) zeros(3,5); zeros(3,3) this.packM(ToolMoment) ]*BGPosJacobian;
            %WrenchCurrentJacobian = WrenchActuationMatrix(this, ToolMoment, ToolPosition);
            %W = WrenchCurrentJacobian*reshape(Currents,this.numCoils,1);
            
        end
        
        
        
        function A = WrenchActuationMatrix(this, ToolMoment, ToolPosition)
            Afg = FieldGradientActuationMatrix(this, ToolPosition);
            A = [ this.skew(ToolMoment) zeros(3,5); zeros(3,3) this.packM(ToolMoment) ] * Afg;
        end
        
        function inWorkspace = PointInWorkspace(this, Position)
            inWorkspace = all( this.workspaceDimensions(:,1) <= Position & Position <= this.workspaceDimensions(:,2));
        end
    end
    
    methods (Static = true) %% Helper Functions
        function S = skew(x)
            S = [0 -x(3) x(2);
                x(3), 0 , -x(1);
                -x(2) x(1) 0];
        end
        
        function M = packM(m)
            M = [m' 0 0;
                0    m(1)   0       m(2) m(3);
                -m(3) 0      m(1)   -m(3) m(2)];
        end
        function [ B, G, dBdA, dBdB, dBdRc, dBdZ, dGdA, dGdB, VecG, VecGGrad ] = CalculateField(point, Acoeff, Bcoeff, Rc, z)
            
            orderA = length(Acoeff);
            orderB = length(Bcoeff);
            Order = max(orderA,orderB);
            
            l = (1:Order);
            p = point-Rc;
            r = norm(p);
            nz = norm(z);
            zHat = z/nz;
            %z = [0;0;1];
            if r>0
                cTh = (p'*z)/(r*nz);
                PlCosTh = LegendrePolynomial(cTh, l, 0);
                dPlCosTh = LegendrePolynomial(cTh, l, 1);
                dPlCosTh2 = LegendrePolynomial(cTh, l, 2);
                dPlCosTh3 = LegendrePolynomial(cTh, l, 3);
                rHat = p/r;
                
            else
                cTh = 0;
                PlCosTh = LegendrePolynomial(0, l, 0);
                dPlCosTh = LegendrePolynomial(0, l, 1);
                dPlCosTh2 = LegendrePolynomial(0, l, 2);
                rHat = z/nz;
            end
            
            B = zeros(3,1);
            G = zeros(3,3);
            dBdA = zeros(3,orderA);
            dBdB = zeros(3,orderB);
            dGdA = zeros(5,orderA);
            dGdB = zeros(5,orderB);
            dBdRc = zeros(3,3);
            dBdZ = zeros(3,3);
            
            VecGGrad = zeros(5,3);
            x = [1;0;0];
            y = [0;1;0];
            
            for l = 1:length(Acoeff)
                n = l;
                %B = B + (Acoeff(l)*n*r^n-Bcoeff(l)*(n+1)/r^(n+1))*PlCosTh(l)/r^2*p...
                %      + (Acoeff(l)*r^n  +Bcoeff(l)/r^(n+1))      *dPlCosTh(l)/(r*nz)*(eye(3)-1/r^2*(p*p'))*z;
                
                dBdA(:,l) = -((n*r^n)*PlCosTh(l)/r^2*p...
                    + (r^n)*dPlCosTh(l)/(r*nz)*(eye(3)-1/r^2*(p*p'))*z);
                
                
                %     G = G - (Acoeff(l)*r^(n-2)*(PlCosTh(l)*n*((n-2)/r^2*(p*p')+eye(3))+...
                %         dPlCosTh(l)*((p'*z)/(r*nz)*((3-2*n)/r^2*(p*p')-eye(3))+(n-1)/(r*nz)*(z*p'+p*z'))+...
                %         dPlCosTh2(l)*( 1/nz^2*(z*z')-(p'*z)/(r^2*nz^2)*(p*z'+z*p')+(p'*z)^2/(r^4*nz^2)*(p*p'))));
                
                tmp = -(r^(n-2)*(PlCosTh(l)*n*((n-2)/r^2*(p*p')+eye(3))+...
                    dPlCosTh(l)*((p'*z)/(r*nz)*((3-2*n)/r^2*(p*p')-eye(3))+(n-1)/(r*nz)*(z*p'+p*z'))+...
                    dPlCosTh2(l)*( 1/nz^2*(z*z')-(p'*z)/(r^2*nz^2)*(p*z'+z*p')+(p'*z)^2/(r^4*nz^2)*(p*p'))));
                
                dGdA(:,l) = [tmp(:,1);tmp(2:3,2)];
                
                dBdZ = dBdZ - ((Acoeff(l)*n*r^n)*dPlCosTh(l)/(r*nz)*(1/r^2*(p*p')-(p'*z)/(r^2*nz^2)*(p*z'))+...
                    (Acoeff(l)*r^n)*dPlCosTh(l)/(r*nz)*(eye(3)-1/r^2*(p*p')-1/nz*(z*z')+(p'*z)/(r^2*nz^2)*(p*z'))+...
                    (Acoeff(l)*r^n)*dPlCosTh2(l)/(r*nz)*(1/(r*nz)*z*p'-(p'*z)/(r^3*nz)*(p*p')-(p'*z)/(r*nz^3)*(z*z')+(p'*z)^2/(r*nz)^3*(p*z')));
            end
            
            for l = 1:length(Bcoeff)
                n = l;
                %B = B + (Acoeff(l)*n*r^n-Bcoeff(l)*(n+1)/r^(n+1))*PlCosTh(l)/r^2*p...
                %      + (Acoeff(l)*r^n  +Bcoeff(l)/r^(n+1))      *dPlCosTh(l)/(r*nz)*(eye(3)-1/r^2*(p*p'))*z;
                
                
                dBdB(:,l) = -((-(n+1)/r^(n+1))*PlCosTh(l)/r^2*p...
                    + (1/r^(n+1))*dPlCosTh(l)/(r*nz)*(eye(3)-1/r^2*(p*p'))*z);
                
                tmp = -(1/r^(n+3)*(PlCosTh(l)*(n+1)*((n+3)/r^2*(p*p')-eye(3))+...
                    dPlCosTh(l)*((p'*z)/(r*nz)*((5+2*n)/r^2*(p*p')-eye(3))-(n+2)/(r*nz)*(z*p'+p*z'))+...
                    dPlCosTh2(l)*(1/nz^2*(z*z')-(p'*z)/(r^2*nz^2)*(p*z'+z*p')+(p'*z)^2/(r^4*nz^2)*(p*p'))));
                
                dGdB(:,l) = [tmp(:,1);tmp(2:3,2)];
                
                %     G = G + Bcoeff(l)*tmp;
                
                
                dBdZ = dBdZ + -((-Bcoeff(l)*(n+1)/r^(n+1))*dPlCosTh(l)/(r*nz)*(1/r^2*(p*p')-(p'*z)/(r^2*nz^2)*(p*z'))+...
                    (+Bcoeff(l)/r^(n+1))*dPlCosTh(l)/(r*nz)*(eye(3)-1/r^2*(p*p')-1/nz*(z*z')+(p'*z)/(r^2*nz^2)*(p*z'))+...
                    (+Bcoeff(l)/r^(n+1))*dPlCosTh2(l)/(r*nz)*(1/(r*nz)*z*p'-(p'*z)/(r^3*nz)*(p*p')-(p'*z)/(r*nz^3)*(z*z')+(p'*z)^2/(r*nz)^3*(p*z')));
                
                tmp1 = -Bcoeff(n)/r^(n+4)*(PlCosTh(n)*(n+1)*(n+3)*(rHat*x'+x*rHat'+(rHat'*x)*(eye(3)-(n+5)*(rHat*rHat')))...
                    -dPlCosTh(n)*3*(n^2+7*n+11)*cTh*(rHat'*x)*(rHat*rHat')...
                    +dPlCosTh(n)*(n+4)*(n+2)*((rHat'*x)*(zHat*rHat'+rHat*zHat')+(zHat'*x)*(rHat*rHat'))...
                    +dPlCosTh2(n)*(cTh*(2*n+7)*(rHat'*x)-(n+3)*(zHat'*x))*(rHat*zHat'+zHat*rHat')...
                    +dPlCosTh2(n)*(cTh*(2*n+7)*(zHat'*x)-cTh^2*(12+3*n)*(rHat'*x))*(rHat*rHat')...
                    -dPlCosTh2(n)*(n+3)*(rHat'*x)*(zHat*zHat')...
                    -(dPlCosTh(n)*(n+2)+dPlCosTh2(n)*cTh)*(x*zHat'+zHat*x'+zHat'*x*eye(3))...
                    +(dPlCosTh(n)*cTh*(5+2*n)+dPlCosTh2(n)*cTh^2)*(x*rHat'+rHat*x'+x'*rHat*eye(3))...
                    -dPlCosTh3(n)*(cTh*(rHat'*x)-(zHat'*x))*(zHat-cTh*rHat)*(zHat-cTh*rHat)'...
                );
            
                tmp2 = -Bcoeff(n)/r^(n+4)*(...
                     PlCosTh(n)*(n+1)*(n+3)*(rHat*y'+y*rHat'+(rHat'*y)*(eye(3)-(n+5)*(rHat*rHat')))...
                    -dPlCosTh(n)*3*(n^2+7*n+11)*cTh*(rHat'*y)*(rHat*rHat')...
                    +dPlCosTh(n)*(n+4)*(n+2)*((rHat'*y)*(zHat*rHat'+rHat*zHat')+(zHat'*y)*(rHat*rHat'))...
                    +dPlCosTh2(n)*(cTh*(2*n+7)*(rHat'*y)-(n+3)*(zHat'*y))*(rHat*zHat'+zHat*rHat')...
                    +dPlCosTh2(n)*(cTh*(2*n+7)*(zHat'*y)-cTh^2*(12+3*n)*(rHat'*y))*(rHat*rHat')...
                    -dPlCosTh2(n)*(n+3)*(rHat'*y)*(zHat*zHat')...
                    -(dPlCosTh(n)*(n+2)+dPlCosTh2(n)*cTh)*(y*zHat'+zHat*y'+zHat'*y*eye(3))...
                    +(dPlCosTh(n)*cTh*(5+2*n)+dPlCosTh2(n)*cTh^2)*(y*rHat'+rHat*y'+y'*rHat*eye(3))...
                    -dPlCosTh3(n)*(cTh*(rHat'*y)-(zHat'*y))*(zHat-cTh*rHat)*(zHat-cTh*rHat)'...
                );
                VecGGrad = VecGGrad + [tmp1;tmp2(2:3,:)];
            end
            B = zeros(3,1);
            G = zeros(5,1);
            if orderA
                B = B+ dBdA*Acoeff;
                G = G+ dGdA*Acoeff;
            end
            if orderB
                B = B + dBdB*Bcoeff;
                G = G + dGdB*Bcoeff;
            end
            VecG = G;
            G = [G(1:3) [G(2);G(4:5)] [G(3);G(5);-G(1)-G(4)]];
            dBdRc = -G;
            
        end
        
        function [ value ] = LegendrePolynomial(x, order, der )
            %LegendrePolynomial provides value of a legandre polynomial of order n at
            % position vector x and deritive order der. Only Reliable to the 18th order
            
            x = reshape(x,numel(x),1);
            order = reshape(order,1,numel(order));
            
            if ~exist('der','var')
                der = 0;
            end
            
            value = zeros(length(x),length(order));
            if ~isnumeric(x)
                value = sym(value);
            end
            for i = 1:length(order)
                n = order(i);
                
                for k=der:n
                    j=1:n;
                    value(:,i) = value(:,i)+x.^(k-der).*prod((k-der+1):k).*nchoosek(n,k).*prod((2*j + k - n - 1)./(j));
                end
                
                
                if ~isnumeric(x)
                    value(:,i) = simplify(value(:,i),500);
                end
            end
            
            
        end
        
    end
    
    
end