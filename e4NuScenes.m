function e4NuScenes(sceneName, eProps)
%% Evaluate mixture implementation on nuScenes Dataset.
%
% Evaluation Example:
%   "nuScenes"
%
% Corresponding Publication: 
%   "A Credible and Robust approach to Ego-Motion Estimation using an
%    Automotive Radar" (<a href="https://mytuc.org/creme">details</a>)
% 
% Here we create the registration problem based on the nuScenes Mini
% dataset and do the motion estimation with different algorithms.
%
% The mini dataset consists of 10 different scenes. The scene to be
% evaluated can be chosen by giving the name as first parameter to this
% function. A json configuration file for this scene has to be present
% within the scripts working path. 
%
% If no scene is given as parameter, all possible scenes will be determined
% by searching for json-files within the scripts working path and the
% function will be called for each file consecutively.

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

%% Scene selection
% Find json-configuration files and execute this function for every file.
if nargin < 1
    sceneName = regexp(string(split(ls('e4NuScenes_*.json'),"  " | newline | char(9))),"e4NuScenes_(.*)\.json",'once','tokens');
    sceneName = [sceneName{:}]; % remove empty matches
    for i=1:numel(sceneName), e4NuScenes(sceneName(i)); end
    return;
end

%% Set environment for using NuScenes' python interface
% Here we try to add the given directory of the nuscenes-devkit to Matlab's
% python path configurations and import the nuScenes python module.
% If the devkit is used within a virtual environment, the path to the
% python binary of this environment has also to be given.
wrappers.nuScenesCheck(...
    'PyEnv', ...
    fullfile(getenv('HOME'),'.virtualenvs','nuscenes','bin','python'), ...
    'PackagePath', ...
    fullfile(getenv('HOME'),'workspace','nuscenes-devkit','python-sdk') );

%% Read properties from json
[sPath,sFilename] = fileparts(mfilename('fullpath'));
if nargin < 2 || isempty(eProps) % Read properties if not given
    eProps = libmix4sam.utils.readJson(...
        fullfile(sPath, strcat(sFilename,'_',sceneName,'.json')));
    eProps.workingFolder = fullfile(sPath,'e4Data',eProps.experimentName);
end

%% Generate experiment from dataset
experiment = creme.genNuScenesRadarProblem(eProps);

%% FLATTEN Experiment in case multiple scenes were selected
if iscell(experiment), experiment = cell2mat(experiment'); end

%% Run the algorithms on the generated dataset with the common Properties:

resultProps = struct(...
    'gmmImplementation'       ,                  'MaxSumMix', ...
    'doNumericalHessian'      ,                        false, ...
    'useSimplification'       ,                        false, ...
    'useScaling'              ,                            5, ...
    'additionalDopplerScaling',                            2, ...
    'outlierWeight'           ,                    uint8(50), ... % TODO Make Properties of outlier distribution available
    'outlierWeightDoppler'    ,                     uint8(5), ...
    'useCached'               ,                     'always', ...
    'workingFolder'           ,fullfile(eProps.workingFolder) ...
                     );

prefix                        =                  'nuScenes-';

%% Only as registration problem without Doppler information
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


end
