%% Evaluate mixture implementation for Simulated Radar Registration.
%
% Evaluation Example:
%   "Simulated Radar"
%
% Corresponding Publication: 
%   "A Credible and Robust approach to Ego-Motion Estimation using an
%    Automotive Radar" (<a href="https://mytuc.org/creme">details</a>)
%
% Here we evaluate the radar registration problem for a random set
% of landmarks with or without outliers. In contrast to the PSR 2D
% problem, the simulation is only in 2DoF (longitudinal translation
% and relative rotation). Additionally, a real sensor is simulated
% with a limited field of view and simulated Doppler velocity
% measurements.

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
    'thetarange'              ,              [-15 15]*pi/180, ...
    'seed'                    ,          defaultSeed + 24390);

eProps = struct(...
    'transformation'          ,         eTransformationProps, ...
    'numMonteCarloExperiments',                          500, ...  % Number of Monte Carlo experiments per landmark set.  E.g.: 500
    'numLandmarkSets'         ,                           50, ...  % Number of different landmark sets to generate.       E.g.: 50
    'initialization'          ,                       'Zero', ...  % Initialization of the optimization problem. Default is 'Zero'. Use 'GroundTruth', if you are only interested in evaluating consistency.
    'workingFolder'           ,                           [], ...  % Where to store the results? (Will be changed below.)
    'experimentName'          ,                           [], ...  % Name of the experiment. (Will be changed below.)
    'useCached'               ,                        'ask');


%% Generate the dataset without Clustering
experimentName                =    'Radar20_50x500NoCluster';
prefix                        =                  'SimRadar-';
eProps.workingFolder          = ['./e2Data/' experimentName];
eProps.experimentName         =               experimentName;
eProps.doLandmarkClustering   =                        false;

[experiment] = libmix4sam.registration.genRadarProblem(defaultSeed, eProps);

%% Run the algorithms on the generated dataset with the common Properties:

resultProps = struct(...
    'doNumericalHessian'      ,                        false, ...
    'useScaling'              ,                            6, ...
    'outlierWeight'           ,                    uint8(20), ...
    'useCached'               ,                        'ask');

%% Only as registration problem without Doppler information
resultProps.resultName        =           [prefix 'PSR-MSM'];
resultProps.workingFolder     =         eProps.workingFolder;
resultProps.useDoppler        =                        false;
resultProps.gmmImplementation =                  'MaxSumMix';

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.registration.runRadarExperiment(experiment, resultProps);

%% This time with Doppler information
resultProps.resultName        =       [prefix 'Doppler-MSM'];
resultProps.workingFolder     =         eProps.workingFolder;
resultProps.useDoppler        =                         true;
resultProps.gmmImplementation =                  'MaxSumMix';

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.registration.runRadarExperiment(experiment, resultProps);

%% Solve by using Sum-Approximation of the cost function
resultProps.resultName        =        [prefix 'PSR-Approx'];
resultProps.workingFolder     =         eProps.workingFolder;
resultProps.useDoppler        =                        false;
resultProps.gmmImplementation =                        'Sum';

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.wrappers.motionEstimation.runRadarExperiment(experiment, resultProps);

%% Solve by using Sum-Approximation of the cost function
resultProps.resultName        =    [prefix 'Doppler-Approx'];
resultProps.workingFolder     =         eProps.workingFolder;
resultProps.useDoppler        =                        true;
resultProps.gmmImplementation =                        'Sum';

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.wrappers.motionEstimation.runRadarExperiment(experiment, resultProps);








%% Generate the dataset with Clustering

experimentName                =      'Radar20_50x500Cluster';
prefix                        =           'SimRadarCluster-';
eProps.workingFolder          = ['./e2Data/' experimentName];
eProps.experimentName         =               experimentName;
eProps.doLandmarkClustering   =                         true;

[experiment] = libmix4sam.registration.genRadarProblem(defaultSeed, eProps);

%% Only as registration Problem without doppler information
resultProps.resultName        =           [prefix 'PSR-MSM'];
resultProps.workingFolder     =         eProps.workingFolder;
resultProps.useDoppler        =                        false;
resultProps.gmmImplementation =                  'MaxSumMix';

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.registration.runRadarExperiment(experiment, resultProps);

%% This time with doppler information
resultProps.resultName        =       [prefix 'Doppler-MSM'];
resultProps.workingFolder     =         eProps.workingFolder;
resultProps.useDoppler        =                         true;
resultProps.gmmImplementation =                  'MaxSumMix';

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.registration.runRadarExperiment(experiment, resultProps);

%% Solve by using Sum-Approximation of the cost function
resultProps.resultName        =        [prefix 'PSR-Approx'];
resultProps.workingFolder     =         eProps.workingFolder;
resultProps.useDoppler        =                        false;
resultProps.gmmImplementation =                        'Sum';

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.wrappers.motionEstimation.runRadarExperiment(experiment, resultProps);

%% Solve by using Sum-Approximation of the cost function
resultProps.resultName        =    [prefix 'Doppler-Approx'];
resultProps.workingFolder     =         eProps.workingFolder;
resultProps.useDoppler        =                        true;
resultProps.gmmImplementation =                        'Sum';

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.wrappers.motionEstimation.runRadarExperiment(experiment, resultProps);

%% Clean workspace
clear defaultSeed experimentName resultProps eTransformationProps eProps

% -> Continue with e2SimulatedRadarPlot.m
%e2SimulatedRadarPlot
