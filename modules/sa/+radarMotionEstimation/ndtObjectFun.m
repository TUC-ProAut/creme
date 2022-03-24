function cost = ndtObjectFun(M_i, M_j, Sigma_i,Sigma_j,varargin)
% NDTOBJECTFUN Implements the objective function based on the 
% likelihood approach, as well as the Mestimator concept.
% 
% Corresponding Publication: 
%   "A Credible and Robust approach to Ego-Motion Estimation using an
%    Automotive Radar" (<a href="https://mytuc.org/creme">details</a>)
% 
% cost = NDTOBJECTFUN(M_i, M_j, Sigma_i,Sigma_j)
% 
% Inputs:
% 
% Spatial component 
%      M_i       : the current mean  2 x 1 x N ( number of targets)
%      M_j       : the previous mean 2 x 1 x N ( number of targets)
%      Sigma_i   : the current covarriance 2 x 2 x N ( number of targets)
%      Sigma_j   : the previous covarriance  2 x 2 x N ( number of targets)
% 
% Doppler component 
%      v_k       : the measured velocity from radar N ( number of targets) x 1
%      ex_vk     : the calculated velocity  1 x 1 x N ( number of targets) 
%      stdDev_vk : the measured std_velocity from radar N ( number of targets) x 1
%      ex_Sigma  : the calculated varriance 1 x 1 x N ( number of targets)
% 
% 
% Optional Parameters : 
% 
% cost = NDTOBJECTFUN( M_i, M_j, Sigma_i, Sigma_j,...
%                     'Doppler', false,...
%                     'MEstimator', false,...
%                     'outlierRatio', 0.1,...
%                     'OutlierSigma', 50)
% 
%      Doppler      : append doppler into objective function
%      MEstimator   : apply robust cost function concept (logical: default false)
%      outlierRatio : outlier to inlier ratio
%      OutlierSigma : outlier standard deviation (numeric: default 50)
% 
% Output: 
%      cost : scalar value 1 x 1
  
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

% Input parse : 
p = inputParser;

% Spatial component
addRequired(p,'M_i',@isnumeric);
addRequired(p,'M_j',@isnumeric);
addRequired(p,'Sigma_i',@isnumeric);
addRequired(p,'Sigma_j',@isnumeric);

% Doppler component
addOptional(p,'v_k',[],@isnumeric);
addOptional(p,'ex_vk',[],@isnumeric);
addOptional(p,'stdDev_vkIn',[],@isnumeric);
addOptional(p,'ex_Sigma',[],@isnumeric);
% Doppler flag
addParameter(p,'Doppler',false,@islogical);
% Robust component
addParameter(p,'MEstimator',false,@islogical);
addParameter(p,'outlierRatio', 0,@isnumeric);
addParameter(p,'OutlierSigma',50,@isnumeric);

% input parse
parse(p,M_i,M_j,Sigma_i,Sigma_j,varargin{:});

% Required
M_i = p.Results.M_i;
M_j = p.Results.M_j;
Sigma_i = p.Results.Sigma_i;
Sigma_j = p.Results.Sigma_j;

% Optional
v_k = p.Results.v_k;
ex_vk = p.Results.ex_vk;
stdDev_vkIn = p.Results.stdDev_vkIn;
ex_Sigma = p.Results.ex_Sigma;

% Paramters 
doppler = p.Results.Doppler;
MEstimator = p.Results.MEstimator;
outlierRatio = p.Results.outlierRatio;
OutlierSigma = p.Results.OutlierSigma;

% Using M-Estimator 

% The Corrupted Gaussian : The outlier is added to each target in the previous pointset
% The default value is 50 comes from the varargin
OutlierCovMatrix = [OutlierSigma 0; 0 OutlierSigma];
% Outlier Mean is the target mean

% if the oulier ratio is same for all current points in the point set 
if length(outlierRatio) == 1
    if MEstimator
        outlierRatio = repmat(outlierRatio,1,size(M_i,3));
    else
        outlierRatio = zeros(1,size(M_i,3));
    end 
    
end 

% Liklehood
acc1 = 1;
for i =1 : size(M_i,3) % for the current point set 
    % GMM Summation 
    acc2 = 0;
    
    if doppler
        if size(stdDev_vkIn,1) == 1 
            velStdDev = stdDev_vkIn;
        else 
            velStdDev = stdDev_vkIn(i,:);
        end 
         dopplerRatio = mvnpdf(0,v_k(i,:)-ex_vk(:,:,i),velStdDev^2+ex_Sigma(:,:,i));
    else 
        dopplerRatio = 1 ;
    end 

    % All gaussian component are equally weighted in GMM 
    % TODO change the eqaully weighted 
    wi = 1/size(M_j,3);
    % GMM liklehood
    for j=1 : size(M_j,3)
        Mij = M_i(:,:,i)-M_j(:,:,j) ;
        Sigmaij = Sigma_i(:,:,i)+Sigma_j(:,:,j);
        
        if MEstimator
             outlier = mvnpdf(M_i(:,:,i),M_j(:,:,j),OutlierCovMatrix) ; % the outlier is added to the previous point sets as it represent the model 
        else
            outlier = 0;
        end 
         acc2 = acc2 + (1-outlierRatio(i))*(wi * mvnpdf([0; 0],Mij,Sigmaij) * dopplerRatio) + outlierRatio(i) * outlier ; % Just for checking if the outlier change if the point considered as inlier
    end
    
    % Liklehood option 1 
    acc1 = acc1 * ( acc2);
     
end
% negative log liklehood 
cost = -log(acc1);
end
