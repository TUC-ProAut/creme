function [experiment, p] = genNuScenesRadarProblem(varargin)
%GENNUSCENESRADARPROBLEM Generate a radar registration problem in 2D based on real 
%   word data from NuScenes dataset.

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

%% Evaluate input arguments 
p = inputParser;
p.KeepUnmatched = true;

% Directory with the data
validFct = @(x)exist(x,'Dir');
addParameter(p,'datasetFolder', [], validFct);

validFct = @(x)validateattributes(x,{'char','string'},{});
addParameter(p,'datasetVersion', [], validFct);

validFct = @(x)validateattributes(x,{'numeric','char','string'},{});
addParameter(p,'datasetScene', [], validFct);

addParameter(p,'datasetUseKeyframesOnly', false, @islogical);

% Calculate the doppler velocity based on the ego motion ground truth
addParameter(p,'predictDoppler', false, @islogical);

% Use calibration information to correct measurements into body frame
addParameter(p,'useCalibration', true, @islogical);

validFct = @(x)assert(ismember(x,{'GroundTruth','Zero'}),...
    'Unknown initialization method given!');
addParameter(p,'initialization', 'Zero', validFct);

% Should we try to estimate the coorespondences for evaluation?
% (only possible, if ground truth of trajectory is given)
addParameter(p,'estimateCorrespondences', true, @islogical);

% Save Experiment: if not empty, a folder with the name will be created and
% the result ist stored. 
addParameter(p,'workingFolder',[], @ischar);

% Name of the Experiment
addParameter(p,'experimentName',[], @ischar);

% Should we use results from previous runs, if parameters are the same?
addParameter(p,'useCached','ask',@(x)ismember(x,{'always','ask','never'}));

% If we already loaded a NuScenes Dataset, we can give it to save
% processing time.
addParameter(p,'useNusc',[],@(x)isa(x,'wrappers.NuScenes'));


parse(p, varargin{:});

p = p.Results;


%% Check, if the experiment is already done

% We don't want to save the following parameters as they may be changed
% without having influence on the generated data
useCached = p.useCached; 
workingFolder = p.workingFolder;
experimentName = p.experimentName;
nusc = p.useNusc;
p = rmfield(p,{'useCached','workingFolder','experimentName','useNusc'}); 

if ~isempty(workingFolder) && ~isempty(experimentName)
    fname = fullfile(workingFolder, ['setup_' experimentName]);
    if exist([fname '.mat'], 'file')
        if strcmp(useCached, 'ask')
            answer = questdlg('Problem generation already done. Do you want to redo it?', ...
                'Redo problem?', ...
                'Redo', 'Skip','Skip');
        end
        if strcmp(useCached,'always') || strcmp(answer,'Skip')
            warning('Problem generation already done. Loading old data.');
            data = load(fname);
            experiment = data.experiment;
            assert(isequaln(p, rmfield(data.p,{'name','size'})),... % Name and size are not known before and they are only convenience variables!
              'The previously generated experiment''s parameters don''t match the current ones!');
            p = data.p;
            return; 
        end
    end
end

%%
tStart = tic;

%% Read Data

if isempty(nusc)
    nusc = wrappers.NuScenes(p.datasetFolder, 'Version', p.datasetVersion);
end
if ischar(p.datasetScene), p.datasetScene = string(p.datasetScene); end 
nuscRadar = nusc.getRadarSensor('RADAR_FRONT', p.datasetScene, ...
    'useKeyframesOnly', p.datasetUseKeyframesOnly,...
    'useCalibration', p.useCalibration, ...
    'predictDoppler',p.predictDoppler,...
    'usePolar',true);

if iscell(nuscRadar)
    r = cellfun(@(x)libmix4sam.sensors.Radar.ConvertNuScenesData(x.data),nuscRadar,'UniformOutput',false);
    gt = cellfun(@(x)x.ego,nuscRadar);
    name = cellfun(@(x)x.name,nuscRadar);
    %s = 
else
    if p.predictDoppler, [nuscRadar.data.doppler] = nuscRadar.data.dopplerGt; end % We use the predicted doppler based on GT instead of the measured one
    r = libmix4sam.sensors.Radar.ConvertNuScenesData(nuscRadar.data);
    gt = nuscRadar.ego;
    name = nuscRadar.name;
end
p.name = name; % Store the scene name for convenience

%% Extract and restructure measurements to build optimization problem
experiment = collectExperiment(r, gt, p);

%%
fprintf('Dataset generation took %0.2d seconds.\n', toc(tStart));

%% Store number of experiments for convenience
if iscell(experiment)
    p.size = cellfun(@(x)length(x),experiment);
else
    p.size = length(experiment);
end

%% Save the result!
if ~isempty(workingFolder) && ~isempty(experimentName)
    if ~exist(workingFolder,'dir'), mkdir(workingFolder); end
    save([fname '.mat'], 'experiment','p');
    % Have the properties human readable too.
    libmix4sam.utils.saveAsJson([fname '.json'],p);
end

end

function experiment = collectExperiment(r,gt,p)

if iscell(r)
    experiment = cell(1,length(r));
    for i=1:length(r), experiment{i} = collectExperiment(r{i},gt(i),p); end
    return;
end

numExperiments = length(r)-1;

for iExperiment = numExperiments:-1:1
    experiment(iExperiment,1).previous = r(iExperiment).copy();
    experiment(iExperiment,1).current = r(iExperiment+1).copy();
    experiment(iExperiment,1).init = [0;0];
end

gt = gt.getAsSE2();            % Make 2D
gt = diff(gt);                 % Make relative
gt = num2cell(gt.toNumeric',1); % Prepare assignment to experiment structure
[experiment.gt] = gt{:};

if strcmp(p.initialization,'GroundTruth')
    init = cellfun(@(x)[x(1);x(3)],gt,'UniformOutput',false);
    [experiment.init] = init{:}; 
end

% 
if p.estimateCorrespondences
    eval = libmix4sam.registration.EvalRadarExperiment();
    eval.addExperimentData(experiment);
    eval.estimateCorrespondences();
    experiment = eval.Experiment;
end


end
