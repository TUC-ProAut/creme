%% Visualize evaluation results of mixture implementation for Mobile Robot Dataset Registration.
%
% Evaluation Example:
%   "Mobile Robot Dataset"
%
% Corresponding Publication: 
%   "A Credible and Robust approach to Ego-Motion Estimation using an
%    Automotive Radar" (<a href="https://mytuc.org/creme">details</a>)
%
% This file can be seen as extension to e3SummitRadar. The results will be
% loaded and different benchmarks for evaluation are executed.

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

%% Load the results

% Dataset / experiment name. E.g.: 'Turmbau1', 'Turmbau2', 'Sportplatz2',
% 'Weinholdbau3', 'Weinholdbau4'
eName = 'Turmbau1';

sPath = fileparts(mfilename('fullpath'));
[e1, results1] = libmix4sam.registration.loadRegistrationExperiment(...
    fullfile(sPath, 'e3Data', eName));

%% Fill evaluation class with the results

% Define a working folder to save the output, leave it empty otherwise.
% e.g. [] or fullfile(sPath, 'e3Data', [eName '_results_'])
workingFolder = fullfile(sPath, 'e3Data', [eName '_results_']);

eval = libmix4sam.registration.EvalRadarExperiment();
eval.addExperimentData(e1);
eval.addResultData(results1);
eval.procAccuracy();      % Accuracy analysis
eval.procCredibility();   % Credibility / Confidence analysis

%% Create plots and save
hPlots = struct('Name',{},'Handle',{});

hPlots(end+1).Name        = 'ErrorOverTime';
hPlots(end).Handle        = eval.plotErrorOverTime();

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

hPlots(end+1).Name        = 'Trajectory';
hPlots(end).Handle        = eval.plotTrajectory();

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

%% Save for https://github.com/uzh-rpg/rpg_trajectory_evaluation
rpg = eval.toXYZqxqyqzqw();
rpgConfig = struct('Datasets',struct(),'Algorithms',struct());
% Create directory structure and data
if ~isempty( workingFolder )
    subdirPlatfrom = 'Robo';
    dirPlatform = fullfile(workingFolder, subdirPlatfrom);
    subdirDataset = eName;
    if ~exist( dirPlatform, 'dir'), mkdir(dirPlatform); end
    for iRpg = 1:numel(rpg)
        subdirAlgorithm = strrep(rpg(iRpg).Name,'-','_');
        if ~exist( fullfile(dirPlatform, subdirAlgorithm), 'dir'), mkdir( fullfile(dirPlatform, subdirAlgorithm) ); end
        dirDataset = fullfile(dirPlatform, subdirAlgorithm, [subdirPlatfrom '_' subdirAlgorithm '_' subdirDataset]);
        if ~exist( dirDataset, 'dir'), mkdir(dirDataset); end
        writematrix(rpg(iRpg).Gt, fullfile(dirDataset, 'stamped_groundtruth.txt'), 'Delimiter', ' ');
        writematrix(rpg(iRpg).Result, fullfile(dirDataset, 'stamped_traj_estimate.txt'), 'Delimiter', ' ');
        fid = fopen(fullfile(dirDataset, 'eval_cfg.yaml'),'w');
        fprintf(fid,'align_num_frames: 800\n');
        fprintf(fid,'align_type: posyaw\n');
        fclose(fid);
        rpgConfig.Datasets.(subdirDataset) = struct('label', subdirDataset);
        rpgConfig.Algorithms.(subdirAlgorithm) = struct('fn','traj_est','label',subdirAlgorithm);
    end
end
rpgConfig.RelDistances = [1 2.5 5 10 15];
%rpgConfig.RelDistancePercentages = [];
yaml.WriteYaml(fullfile(dirPlatform, 'e3_summit.template.yaml'), rpgConfig);


