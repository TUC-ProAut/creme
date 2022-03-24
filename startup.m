function [  ] = startup(  )
%STARTUP Configure path on startup.
%   Creates a standard configuration file for needed dependencies
%   (startub.json), which can be changed to match the machine's path
%   configuration.

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

if ~exist('startup.json','File')
    % Default configuration for startup (autogenerated, because we don't
    % want to have personal configurations within source control)
    pathConfig = struct();
    % Usually, all devel-sources live in one path called 'workspace'
    pathConfig.workspace = fullfile(getenv('HOME'),'workspace');
    % CREME (by determing the scripts working path)
    pathConfig.creme = fileparts(mfilename('fullpath'));
    % libmix4sam folder (within, the matlab folder should be located)
    pathConfig.libmix4sam = fullfile(pathConfig.creme,'..','..','libmix4sam');
    % Write default properties to json file
    fid = fopen('startup.json', 'w');
    % Newer matlab version can use 'PrettyPrint', true for readability
    % As alternative, open with vscode and use F1+"format document".
    fprintf(fid, jsonencode(pathConfig)); 
    fclose(fid);
    fprintf('CREME startup was run for the first time.\n');
    fprintf('Wrote a default configuration to startup.json,\n');
    fprintf('please CHECK first all path configurations within\n');
    fprintf('this file AND RUN startup.m AGAIN AFTERWARDS.\n');
    fprintf('\nStopping execution of startup now.\n');
    return;
else
    pathConfig = jsondecode(fileread('startup.json'));
end

% We add the current folder to the path
path(path, pathConfig.creme);

% Adaptive Robust Numerical Differentiation Toolbox (optional, needed for 
% some configuration options)
path(path,fullfile(pathConfig.libmix4sam,'matlab','External','DERIVESTsuite'));

% Simple handling YAML-Files (optional, needed for interface to
% rpg_trajectory_evaluation)
path(path,fullfile(pathConfig.libmix4sam,'matlab','External','yamlmatlab'));

% Sum-Approach
path(path,fullfile(pathConfig.creme,'modules','sa'));

% ICP with covariance
path(path,fullfile(pathConfig.creme,'modules','icp_cov','matlab'));
path(path,fullfile(pathConfig.creme,'modules','icp_cov','install')); % Binaries

% Module to load data from nuScenes dataset
path(path,fullfile(pathConfig.creme,'modules','dataset-tool'));

% Include statistical tests and more from libmix4sam
path(path,fullfile(pathConfig.libmix4sam,'matlab','statistics'));


% Include gtsam & libmix4sam
path(path,fullfile(pathConfig.libmix4sam, 'matlab'));
libmix4sam.configurePath(pathConfig.libmix4sam, 'useUnstable', false);

% For manual checking of all linked libraries and their availability,
% print the ldd command.
fprintf('===== SHOWING LDD FOR gtsam_wrapper =====\n');
if system(['ldd ' which('gtsam_wrapper')]), error('gtsam_wrapper not found!'); end
fprintf('===== SHOWING LDD FOR libmix4sam_wrapper =====\n');
if system(['ldd ' which('libmix4sam_wrapper')]), error('libmix4sam_wrapper not found!'); end
