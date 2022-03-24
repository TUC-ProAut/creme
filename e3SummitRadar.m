%% Evaluate mixture implementation on Mobile Robot Dataset.
%
% Evaluation Example:
%   "Mobile Robot Dataset"
%
% Corresponding Publication: 
%   "A Credible and Robust approach to Ego-Motion Estimation using an
%    Automotive Radar" (<a href="https://mytuc.org/creme">details</a>)
% 
% Here we evaluate on a real world dataset collected with our mobile robot
% (an SummitXL from robotnik with customized sensor configuration). 

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

[sPath, sFilename] = fileparts(mfilename('fullpath'));

eProps = struct('experimentName','Turmbau1');
eProps.workingFolder = fullfile(sPath,'e3Data',eProps.experimentName);
load(fullfile(eProps.workingFolder, ['setup_' eProps.experimentName]),'experiment');

%% Run the algorithms on the generated dataset with the common properties:

resultProps = struct(...
    'gmmImplementation'       ,                  'MaxSumMix', ...
    'doNumericalHessian'      ,                        false, ...
    'useScaling'              ,                            2, ...
    'additionalDopplerScaling',                            2, ...
    'outlierWeight'           ,                    uint8(20), ...
    'useCached'               ,                        'ask', ...
    'workingFolder'           ,fullfile(eProps.workingFolder) ...
                     );
                 
prefix                        =                  'Summit-';

%% Only as registration Problem without Doppler information
resultProps.resultName        =           [prefix 'PSR-MSM'];
resultProps.useDoppler        =                        false;

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.registration.runRadarExperiment(experiment, resultProps);

%% This time with Doppler information
resultProps.resultName        =       [prefix 'Doppler-MSM'];
resultProps.useDoppler        =                         true;

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.registration.runRadarExperiment(experiment, resultProps);

%% Solve by using Sum-Approximation of the cost function
resultProps.resultName        =        [prefix 'PSR-Approx'];
resultProps.useDoppler        =                        false;
resultProps.gmmImplementation =                        'Sum';

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.wrappers.motionEstimation.runRadarExperiment(experiment, resultProps);

%% Solve by using Sum-Approximation of the cost function
resultProps.resultName        =    [prefix 'Doppler-Approx'];
resultProps.useDoppler        =                        true;
resultProps.gmmImplementation =                        'Sum';

fprintf('Doing %s\n', resultProps.resultName);
libmix4sam.wrappers.motionEstimation.runRadarExperiment(experiment, resultProps);
