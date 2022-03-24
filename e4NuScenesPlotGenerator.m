%% Specific plots and evaluations for RAL-Radar Paper
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

function e4NuScenesPlotGenerator(workingFolder,dataFolder)
% Iterate through all available Scenes and generate the plots for them.

% Prepare some helper variables
if nargin < 1 || isempty(workingFolder)
    sPath = fileparts(mfilename('fullpath')); % Script-Path
else
    sPath = workingFolder;
end
if nargin < 2, dataFolder = 'e4Data'; end
    
% Find all json files matching a pattern in Script-Path
files = dir(fullfile(sPath,'e4NuScenes_*.json'));
tableFiles = string.empty;
tableScenes = string.empty;

for iFile = 1:length(files)
    eProps = libmix4sam.utils.readJson(...
      fullfile(files(iFile).folder, files(iFile).name));
    if isfield(eProps,'datasetScene') && ~isempty(eProps.datasetScene)
        disp(files(iFile).name)
        eFolder = fullfile(sPath, dataFolder, eProps.experimentName);
        if exist(eFolder,'dir')
            % Experiment wurde ausgeführt
            eNumber = string(regexp(eProps.datasetScene,'\w+-(\d+)','tokens','once'));
            assert(~isempty(eNumber),'Something went wrong! There should be a scene defined in datasetScene!');
            rFolder = [eFolder '_results']; % result folder
            if exist(rFolder, 'dir')
                warning('Result already there');
            else
                e4NuScenesPlot(char(eNumber), sPath, dataFolder);
                close all;
            end
            tableFiles(end +1) = fullfile(rFolder,'result.xls');
            tableScenes(end +1) = eNumber;
        else
            warning('json file for experiment exists, but no result directory (%s)',files(iFile).name);
        end
    end
    %sceneName = string(regexp(files(iFile).name,'\w+_(\d+)\.json','tokens','once')); % Select the scene number out of filename 
end

% Concatinate result tables
T = table.empty;
for iTable = 1:length(tableFiles)
    T = [T;concatTable(tableFiles(iTable),tableScenes(iTable))];
end

T = sortrows(T,{'Name','Scene'});
writetable(T, 'e4NuScenesPlotGenerator.xls', 'WriteRowNames', true);


end

function T = concatTable(filename,scene)
    assert(exist(filename,'file'),'Could not find the given file!');
    T1 = readtable(filename,'Sheet',1);
    T2 = readtable(filename,'Sheet',2);
    T3 = readtable(filename,'Sheet',3);

    T = outerjoin(T1,T2,'Keys','Name','LeftVariables',{'Name','PosRMSE','AngRMSE'},'RightVariables',{'ANEES'}); T.Properties.VariableNames{1} = 'Name';
    T = outerjoin(T,T3,'Keys','Name','RightVariables',{'Iterations','AverageTime'});
    T.Scene(:,1) = scene;
    T.AverageTime(:) = T.AverageTime(:)*1000;
end
