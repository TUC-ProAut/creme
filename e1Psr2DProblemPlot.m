%% Visualize evaluation results of mixture implementation for Simulated Point Set Registration.
%
% Evaluation Example:
%   "Simulated Point Set Registration"
%
% Corresponding Publication: 
%   "A Credible and Robust approach to Ego-Motion Estimation using an
%    Automotive Radar" (<a href="https://mytuc.org/creme">details</a>)
%
% This file can be seen as extension to e1Psr2DProblem. The results will be
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
[eCluster, resultsCluster] = libmix4sam.registration.loadRegistrationExperiment(...
    './e1Data/CircPSR20Cluster');

[eNoCluster, resultsNoCluster] = libmix4sam.registration.loadRegistrationExperiment(...
    './e1Data/CircPSR20NoCluster');

%% Fill evaluation class with the results

% Define a working folder to save the output, leave it empty otherwise.
workingFolder = './e1Data/results';   % e.g. [] or './e1Data/results'

eval = libmix4sam.registration.EvalPsr2DExperiment();
eval.addExperimentData(eNoCluster);
eval.addResultData(resultsCluster);
eval.addResultData(resultsNoCluster);
eval.procAccuracy();      % Accuracy analysis
eval.procCredibility();   % Credibility / Confidence analysis

%% Create plots and save
hPlots = struct('Name',{},'Handle',{});

hPlots(end+1).Name        = 'AccuracyErrorPlot';
[hPlots(end).Handle, ax]  = eval.Accuracy.plotError();
for i = findobj(ax.TransHist,'Type','Histogram'), set(i,'BinWidth',0.01), end

hPlots(end+1).Name        = 'CredibilityNEESComparison';
hPlots(end).Handle        = eval.Credibility.plotNEESComparison();

if ~isempty( workingFolder )
    if ~exist( workingFolder, 'dir'), mkdir(workingFolder); end
    for iPlot = 1:numel(hPlots)
        savefig( hPlots(iPlot).Handle,...
            fullfile(workingFolder, [hPlots(iPlot).Name '.fig']),'compact');
    end
end
    
% Make the plot a bit nicer
neesplot = libmix4sam.utils.PlotNice(hPlots(end).Handle);
neesplot.subVertCommonX();

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

