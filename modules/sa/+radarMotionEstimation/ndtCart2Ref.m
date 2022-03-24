function [Mean, Sigma] = ndtCart2Ref(MeanIn, SigmaIn, motionIn, dtIn)
% NDTCART2REF references the current point set according to 
% the motion vector.
% 
% [Mean, Sigma] = NDTCART2REF(MeanIn, SigmaIn, motionIn, dtIn)
% 
% Inputs:
%        MeanIn   in multidiminsion  2 x 1 x N (number of taregt) 
%        SigmaIn  in multidiminsion  2 x 2 x N (number of taregt)
%        motionIn in [vx, vy, omega] 1 x 3 
%        dtIn     in  scalar 
% 
% Output: 
%        Mean  in multidiminsion 2 x 1 x N (number of taregt) 
%        Sigma in multidiminsion 2 x 2 x N (number of taregt)
  
% @author Karim Haggag (TU Chemnitz, ET/IT, Prozessautomatisierung)
 
% This file is part of
% CREME - Credible Radar Ego-Motion Estimation
%
% Copyright (C) 2022 Chair of Automation Technology / TU Chemnitz
% For more information see https://mytuc.org/creme
%
% CREME is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% CREME is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this software.  If not, see <http://www.gnu.org/licenses/>.
%
% Contact Information: Sven Lange (sven.lange@etit.tu-chemnitz.de)
 
% using euler angle representation 
tx = motionIn(1)*dtIn; 
ty = motionIn(2)*dtIn;
theat_k = motionIn(3)*dtIn;
 
% Rotation matix (R)
R(1,1) = cos(theat_k) ; 
R(1,2) =-sin(theat_k) ;
R(2,1) = sin(theat_k) ;
R(2,2) = cos(theat_k) ;
 
% translation vector (t)
t(1,1) = tx ;
t(2,1) = ty ;
 
% output definition 
Mean  = zeros(2,1,size(MeanIn,3));
Sigma = zeros(2,2,size(MeanIn,3));
 
for i = 1 : size(MeanIn,3)    
    [Mean(:,:,i), Sigma(:,:,i)] = ndt_CartRef(MeanIn(:,:,i),...
                                  SigmaIn(:,:,i),R,t);
end 

end

function [Mean, Sigma] = ndt_CartRef(M,Sigma,R,t)
    Mean  = R*M + t ;
    Sigma = R*Sigma*R';
end 
