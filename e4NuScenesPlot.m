%% Specific plots and evaluations for RAL-Radar Paper
%
% Evaluation Example:
%   "nuScenes"
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

function [eval] = e4NuScenesPlot(eName,workingFolder,dataFolder)
%E4NUSCENESPLOT Generate the plots and evaluations for nuScenes.
%   If eName is given as a cell instead of a char, these scenes will be
%   concatinated.

% Get all available scenes by json
%files = dir('e4NuScenes_*.json'); files = string({files.name});
%a=arrayfun(@(x)length(regexp(x,'\w+_(\d+)\.json','tokens','once')),files,'UniformOutput',true)
%arrayfun(@(x)string(regexp(x,'\w+_(\d+)\.json','tokens','once')),files(logical(a)),'UniformOutput',true)

eNameDefault = {'0061', '0103', '0553', '0655', '0757', '0796', '0916', '1077', '1094', '1100'};

if nargin < 1, eName = eNameDefault; end % Default value
if nargin < 2 || isempty(workingFolder), workingFolder = fileparts(mfilename('fullpath')); end
if nargin < 3, dataFolder = 'e4Data'; end
eFolderTemplate = fullfile(workingFolder, dataFolder, 'NuScenes%ENAME%');

if iscell(eName)
    % We want to merge multiple scenes
    f = strrep(eFolderTemplate, '%ENAME%', eName{1});
    [e1, results1, e1_p] = libmix4sam.registration.loadRegistrationExperiment(f);
    e1 = {e1}; e1_p.datasetScene = [];
    for iScene = 2:length(eName) 
        f = strrep(eFolderTemplate, '%ENAME%', eName{iScene});
        [e1{iScene}, resultsNew, e_p] = libmix4sam.registration.loadRegistrationExperiment(f);
        results1 = mergeResults(results1,resultsNew);
        e1_p.name(iScene) = e_p.name;
        e1_p.size = [e1_p.size e_p.size];
    end
    eName = 'multiple'; %cell2mat(eName); % For result name later on.
else
    [e1, results1, e1_p] = libmix4sam.registration.loadRegistrationExperiment(...
        strrep(eFolderTemplate, '%ENAME%', eName));
end

%% Select specific scene-result out of multiple scenes.
% Use [] as scene name, if all should be selected!
%disp('available scenes:'); fprintf('%s\n',e1_p.name)

onlyScene = []; % e.g. [] or "scene-0553";
[e1, results1] = selectScene(onlyScene, e1, results1, e1_p);


%% Fill evaluation class with the results

% Define a working folder to save the output, leave it empty otherwise.
% e.g. [] or fullfile(workingFolder, dataFolder, ['NuScenes' char(eName) '_results'])
workingFolder = fullfile(workingFolder, dataFolder, ['NuScenes' char(eName) '_results']);

eval = libmix4sam.registration.EvalRadarExperiment();
eval.addExperimentData(e1);
eval.addResultData(results1);

if nargout > 0, return; end  % We only want the eval class instance.

eval.procAccuracy();      % Accuracy analysis
eval.procCredibility();   % Credibility / Confidence analysis

%% Create plots and save
hPlots = struct('Name',{},'Handle',{});

hPlots(end+1).Name        = 'ErrorOverTime';
[hPlots(end).Handle, ax]  = eval.plotErrorOverTime();
showSceneSegments(ax, onlyScene, e1_p)

hPlots(end+1).Name        = 'AccuracyErrorPlot';
[hPlots(end).Handle, ax]  = eval.Accuracy.plotError();
for i = findobj(ax.TransHist,'Type','Histogram'), set(i,'BinWidth',0.01), end

hPlots(end+1).Name        = 'CredibilityNEESComparison';
hPlots(end).Handle        = eval.Credibility.plotNEESComparison();

% Make the plot a bit nicer
neesplot = libmix4sam.utils.PlotNice(hPlots(end).Handle);
neesplot.setCommonBinWidth(0.1);
neesplot.setCommonLimits([0 10],[0 1]);
neesplot.subVertCommonX();

if numel(e1_p.size) == 1
    hPlots(end+1).Name        = 'Trajectory';
    hPlots(end).Handle        = eval.plotTrajectory();
end

hPlots(end+1).Name        = 'OutlierAnalysis';
hPlots(end).Handle        = eval.plotOutlierAnalysis();

if ~isempty( workingFolder )
    if ~exist( workingFolder, 'dir'), mkdir(workingFolder); end
    for iPlot = 1:numel(hPlots)
        savefig( hPlots(iPlot).Handle,...
            fullfile(workingFolder, [hPlots(iPlot).Name '.fig']),'compact');
    end
end
    
%% Create tables and save
TAccuracy    = eval.Accuracy.showComparisonTable()
TCredibility = eval.Credibility.showComparisonTable()
TAdditional  = eval.tableAdditional()

if ~isempty( workingFolder )
    fname = fullfile(workingFolder,'result.xls');
    writetable(TAccuracy,    fname, 'Sheet', 'Accuracy'   , 'WriteRowNames', true);
    writetable(TCredibility, fname, 'Sheet', 'Credibility', 'WriteRowNames', true);
    writetable(TAdditional,  fname, 'Sheet', 'Additional' , 'WriteRowNames', true);
end


end

%% DEBUGGING
%eval.plotOneProblem(1,110,1); % LMset, ExperimentNr, ResultSet 

function [e1, results1] = selectScene(sceneName, e1, results1, e1_p)
    if isempty(sceneName) || ~iscell(e1)
        % FLATTEN Experiment in case multiple scenes were selected
        if iscell(e1), e1 = cell2mat(e1'); end
        return; 
    end
    idx = find(ismember(e1_p.name,sceneName),1);
    e1 = e1{idx};
    slices = [0 cumsum(e1_p.size)];
    m = false(slices(end),1);
    m((slices(idx)+1):slices(idx+1)) = true;
    for i=1:length(results1)
        results1(i).data(~m) = [];
    end
end

function showSceneSegments(ax,onlyScene,e1_p)
    if numel(ax) > 1 
        for i=1:numel(ax), showSceneSegments(ax(i),onlyScene,e1_p); end
        return
    end
    if ~isempty(onlyScene) % Show title of scene only
        %TODO
        return
    end
    slices = [0 cumsum(e1_p.size)]+1;
    for iSlice = 1:numel(e1_p.size)
        a = xline(ax, slices(iSlice),'k--',num2cell(e1_p.name(iSlice).split('-')),'LabelOrientation','horizontal');
        set(get(get(a,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end
end

function results = mergeResults(results, result2)
    for iRes = 1:length(result2)
        % Search for the corresponding result
        idx = find(contains({results.name},result2(iRes).name));
        assert(~isempty(idx),'The structure of the result to add does not match the other one!');
        results(idx).data = [results(idx).data; result2(iRes).data];
        results(idx).timing = results(idx).timing + result2(iRes).timing;
    end
end
