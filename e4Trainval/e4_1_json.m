%% Create json config files automatically for the trainval dataset.
%
% Evaluation Example:
%   "nuScenes Trainval"
%
% Corresponding Publication: 
%   "A Credible and Robust approach to Ego-Motion Estimation using an
%    Automotive Radar" (<a href="https://mytuc.org/creme">details</a>)
% 

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
jsonInit = load(fullfile(sPath,[sName '.mat']));

%% Look for available scenes
wrappers.nuScenesCheck('PyEnv','/home/lasve/.virtualenvs/nuscenes/bin/python',...
    'PackagePath',fullfile(getenv('HOME'),'workspace','nuscenes-devkit','python-sdk') );
nusc = wrappers.NuScenes('/home/lasve/datasets/nuscenes/v1.0-trainval','Version','v1.0-trainval');

%% Read availabe scene names
sceneNames = arrayfun(@(x)string(x.name), nusc.Scenes);

%% Create json file for specific number of scenes
for i=1:500
    jsonInit.datasetScene = char(sceneNames(i));
    sceneNum = str2double(strrep(sceneNames(i),'scene-',''));
    jsonInit.experimentName = sprintf('NuScenes%04g', sceneNum);
    libmix4sam.utils.saveAsJson(fullfile(sPath,sprintf('e4NuScenes_%04g.json',sceneNum)),jsonInit);
end
