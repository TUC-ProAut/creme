%% Evaluate mixture implementation for Simulated Point Set Registration.
%
% Evaluation Example:
%   "Simulated Point Set Registration"
%
% Corresponding Publication: 
%   "A Credible and Robust approach to Ego-Motion Estimation using an
%    Automotive Radar" (<a href="https://mytuc.org/creme">details</a>)
% 
% Here we evaluate the 2D-point-set registration problem for a random set
% of landmarks with or without outliers. The number of landmarks as well 
% as the number of Monte-Carlo runs can be changed. For each landmark
% configuration a pair of measurements is selected based on the given
% uncertainty of each landmark, which is random too. 
% Likewise the transformation between the pair of measurements is random
% and the number of samples can be configured. These are applied to one
% landmark configuration and are repeated for the consecutive landmark
% configurations. 
% As choosing different initializations as starting point for the
% optimization is alike choosing different transformations between the
% measurement pair, the initialization is always zero.
% Mor information regarding the possible properties and its meaning can be
% found in the corresponding functions and classes.

% @author Sven Lange (TU Chemnitz, ET/IT, Prozessautomatisierung)

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

%% Define common properties for dataset generation

% Basic seed from which the other seeds are generated.
defaultSeed                   =                          123;

% Ranges for random transformation.
eTransformationProps = struct(...
    'xrange'                  ,                 [-0.25 0.25], ...
    'yrange'                  ,                 [-0.25 0.25], ...
    'thetarange'              ,              [-15 15]*pi/180, ...
    'seed'                    ,          defaultSeed + 24390);

eLmProps = struct(...
    'type'                    ,                     'circle', ...
    'size'                    ,                      [10 10], ...
    'amount'                  ,                           20, ...
    'seed'                    ,             defaultSeed + 90);

eProps = struct(...
    'transformation'          ,         eTransformationProps, ...
    'landmarks'               ,                     eLmProps, ...
    'numMonteCarloExperiments',                         1000, ... % Number of Monte Carlo experiments per landmark set.  E.g.: 1000
    'numLandmarkSets'         ,                          100, ... % Number of different landmark sets to generate.       E.g.: 100
    'OlPercentage'            ,                            0, ...
    'doNoisyMeasurementNoise' ,                        false, ...
    'doLandmarkClustering'    ,                           [], ...
    'initialization'          ,                       'Zero', ... % Initialization of the optimization problem. Default is 'Zero'. Use 'GroundTruth', if you are only interested in evaluating consistency.
    'workingFolder'           ,                           [], ... % Where to store the results? (Will be changed below.)
    'experimentName'          ,                           [], ... % Name of the experiment. (Will be changed below.)
    'useCached'               ,                        'ask');

for i=[1 2]

%% Generate the dataset (with and without clustered configuration)
if i==1        
experimentName                =         'CircPSR20NoCluster';
suffix                        =                           '';
eProps.workingFolder          = ['./e1Data/' experimentName];
eProps.experimentName         =               experimentName;
eProps.doLandmarkClustering   =                        false;
end
if i==2
experimentName                =           'CircPSR20Cluster';
suffix                        =                   '-Cluster';
eProps.workingFolder          = ['./e1Data/' experimentName];
eProps.experimentName         =               experimentName;
eProps.doLandmarkClustering   =                         true;
end

[experiment, experimentProps] = libmix4sam.registration.genPsr2DProblem(defaultSeed, eProps);

%% Run the algorithms on the generated dataset 

% Define common properties for all:
resultProps = struct(...
    'experimentProps'         ,              experimentProps, ...
    'doNumericalHessian'      ,                        false, ...
    'doOutlierModeling'       ,                        false, ...
    'useScaling'              ,                            0, ...
    'useCached'               ,                        'ask');

%% Likelihood implementation utilizing the Max-Sum-Mix approximation
resultProps.resultName        =         ['MaxSumMix' suffix];
resultProps.workingFolder     =         eProps.workingFolder;
resultProps.gmmImplementation =                  'MaxSumMix';

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.registration.runPsr2DExperiment(experiment, resultProps);

%% ICP implementation using PCL interface from motionEstimation package
% The implementation extends the PCL with a covariance calculation 
% based on:
% Censi, A. (2007) ‘An accurate closed-form estimate of ICP’s covariance’,
% in Proc. of Intl. Conf. on Robotics and Automation (ICRA), pp. 3167–3172.
% doi:10.1109/ROBOT.2007.363961.  
% and
% Prakhya, S.M. et al. (2015) ‘A closed-form estimate of 3D ICP
% covariance’, in Proc. of Intl. Conf. on Machine Vision Applications
% (MVA), pp. 526–529. doi:10.1109/MVA.2015.7153246.  

resultProps.resultName        =           ['ICP-Cov' suffix];
resultProps.workingFolder     =         eProps.workingFolder;
resultProps.gmmImplementation =                           [];

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.wrappers.icpCov.runPsr2DExperiment(experiment, resultProps);

%% Solve by using Sum-Approximation of the cost function
resultProps.resultName        =        ['Sum-Approx' suffix];
resultProps.workingFolder     =         eProps.workingFolder;
resultProps.gmmImplementation =                        'Sum';

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.wrappers.motionEstimation.runPsr2DExperiment(experiment, resultProps);


end

%% Clean workspace
clear i defaultSeed experimentName resultProps eTransformationProps eLmProps eProps

% -> Continue with e1Psr2DProblemPlot.m
