%% For the first run of this experiment.
%
% Evaluation Example:
%   "nuScenes"
%
% Corresponding Publication: 
%   "A Credible and Robust approach to Ego-Motion Estimation using an
%    Automotive Radar" (<a href="https://mytuc.org/creme">details</a>)
% 
% The mini dataset consists of 10 different scenes. For each scene, we can
% coose parameters for evaluation. These are stored as json file per scene.
% Here we create them with a standard initialization.

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

[sPath,sName] = fileparts(mfilename('fullpath'));
jsonInit = struct(...
    'datasetFolder',fullfile(getenv('HOME'),'datasets','nuscenes','v1.0-mini'), ...
    'datasetVersion','v1.0-mini', ...
    'datasetScene','TODO', ...
    'datasetUseKeyframesOnly',false, ...
    'estimateCorrespondences',true, ...
    'initialization','Zero', ...
    'experimentName','TODO', ...
    'useCached','always');
    
%% Look for available scenes

% Here we try to add the given directory of the nuscenes-devkit to Matlab's
% python path configurations and import the nuScenes python module.
% If the devkit is used within a virtual environment, the path to the
% python binary of this environment has also to be given.
wrappers.nuScenesCheck(...
    'PyEnv', ...
    fullfile(getenv('HOME'),'.virtualenvs','nuscenes','bin','python'), ...
    'PackagePath', ...
    fullfile(getenv('HOME'),'workspace','nuscenes-devkit','python-sdk') );
% Initialize the NuScenes class to read data from dataset
nusc = wrappers.NuScenes(jsonInit.datasetFolder,'Version',jsonInit.datasetVersion);

%% Read availabe scene names
sceneNames = arrayfun(@(x)string(x.name), nusc.Scenes);

%% Create json file for all scenes
for iScene = 1:numel(sceneNames)
    jsonInit.datasetScene = char(sceneNames(iScene));
    sceneNum = str2double(strrep(sceneNames(iScene),'scene-',''));
    jsonInit.experimentName = sprintf('NuScenes%04g', sceneNum);
    jsonOut = fullfile(sPath,sprintf('e4NuScenes_%04g.json',sceneNum));
    if exist(jsonOut, 'file')
        answer = questdlg('Json file already initialized. Overwrite?', ...
                'File exists', 'Yes', 'No', 'No');
        if strcmp(answer,'No'), continue; end
    end
    libmix4sam.utils.saveAsJson(jsonOut, jsonInit);
end
