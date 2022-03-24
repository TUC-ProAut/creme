%% Do the experiments for all available json configs.
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

[sPath, sFilename] = fileparts(mfilename('fullpath'));

%% find all json files
sceneName = regexp(string(split(ls('e4NuScenes_*.json'),"  " | newline | char(9))),"e4NuScenes_(.*)\.json",'once','tokens');
sceneName = [sceneName{:}]; % remove empty matches

%%
for i=1:numel(sceneName)
    eProps = libmix4sam.utils.readJson(...
         fullfile(sPath, strcat('e4NuScenes_',sceneName(i),'.json')));
    eProps.workingFolder = fullfile(sPath,eProps.experimentName);
    e4NuScenes(sceneName(i),eProps);
end
