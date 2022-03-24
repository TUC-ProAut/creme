%% Compile ICP implementation including covariance estimation.
%
% We compile everything from within Matlab. Therefore, depending on the
% system's OS version and Matlab version, some modifications may be needed.
% This script includes examples for different scenarios.
%
% ISSUES:
%   Newer versions of PCL use the c++14 standard, which some Matlab
%   versions did not support yet. In this case, edit
%   MATLAB/R2020b/bin/glnxa64/mexopts/g++_glnxa64.xml
%   to use c++14 instead of C++11.


% @author Sven Lange    (TU Chemnitz, ET/IT, Prozessautomatisierung)
% @author Karim Haggag  (TU Chemnitz, ET/IT, Prozessautomatisierung)

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


%% Configuration properties

% We need the io-component only for the c++ examples, so it is optional.
% When only using matlab binding, it can be deactivated.
pclUseIO = {};  % activate: {'-DUSEPCLIO','-lpcl_io'} deactivate: {}

% Als PCL-source, the system wide installation or a local folder can be
% choosen.
pclSource = 'system_20.04'; % 'system_18.04', 'system_20.04' or 'local'

% Script's working path
scriptpath = fileparts(mfilename('fullpath'));

switch pclSource
    case 'local'   % Local installation example
        % (PCL can be easiely compiled from source with only the packages
        % needed!)
        pclVersion = '1.11';
        pclInstallDir = fullfile(getenv('HOME'),'workspace','libpcl','install');
        pclIncludeDir = fullfile(pclInstallDir,'include',['pcl-' pclVersion]);
        pclLibraryDir = fullfile(pclInstallDir,'lib');
        flag_PclRequired = {...
            '-lboost_system',...
            '-lpcl_common',...
            '-lpcl_registration',...
            '-lpcl_kdtree',...
            '-lpcl_search',...
            ['-L' pclLibraryDir],...
            ['-I' pclIncludeDir],...
            ['LDFLAGS="\$LDFLAGS -Wl,-rpath=' pclLibraryDir '"'],... % To add search path for libraries like LD_LIBRARY_PATH does '-Wl',['-R' pclLibraryDir],...  
            '-I/usr/include/eigen3/'};
    case 'system_18.04'  % System wide installation example, Ubuntu 18.04
        % (tested in combination with Matlab R2021a)
        % (g++_glnxa64.xml does not need to be modified)
        flag_PclRequired = {...
            '-lboost_system',...
            '-lpcl_common',...
            '-lpcl_registration',...
            '-lpcl_kdtree',...
            '-lpcl_search',...
            '-I/usr/include/pcl-1.8/',...
            '-I/usr/include/eigen3/',...
            '-I/usr/include/vtk-6.3/'};
        
    case 'system_20.04' % System wide installation example, Ubuntu 20.04
        % (tested in combination with Matlab R2021a, modified g++_glnxa64.xml, see top)
        % flag_PclRequired = {...
        %     '-lboost_system',...
        %     '-lpcl_common',...
        %     '-lpcl_registration',...
        %     '-lpcl_kdtree',...
        %     '-lpcl_search',...
        %     '-I/usr/include/pcl-1.10/',...
        %     '-I/usr/include/eigen3/',...
        %     '-I/usr/include/vtk-7.1/',...
        %     'CXXFLAGS=''$CXXFLAGS -std=c++14''',...
        %     '-f g++_glnxa64.xml'};%'CXXFLAGS=''$CXXFLAGS -std=c++14'''
        
        % (tested in combination with Matlab R2021b, no xml modification needed)
        flag_PclRequired = {...
            '-lboost_system',...
            '-lpcl_common',...
            '-lpcl_registration',...
            '-lpcl_kdtree',...
            '-lpcl_search',...
            '-I/usr/include/pcl-1.10/',...
            '-I/usr/include/eigen3/',...
            '-I/usr/include/vtk-7.1/',...
            'CXXFLAGS="\$CXXFLAGS -std=c++14"'};

    otherwise
        error('Wrong pclSource!');
end

%% Make ICP-Wrapper
mex ( 'icp2dTr.cpp', 'pcl_mex_conversions.cpp',... 
     '../src/cov_func_point_to_point_2d.cpp','../src/reg_2d.cpp',...
     '-I../src',...
     flag_PclRequired{:}, pclUseIO{:})
  
%% Make Covariance-Wrapper
mex ( 'icp2dCov.cpp','pcl_mex_conversions.cpp',... 
     '../src/cov_func_point_to_point_2d.cpp','../src/reg_2d.cpp',...
     '-I../src',...
     flag_PclRequired{:}, pclUseIO{:})  

%% Collect the needed files into an install directory

if ~exist(fullfile(scriptpath,'icp2dTr.mexa64'),'file')
    error('Compilation not completed!')
end
if ~exist(fullfile(scriptpath,'icp2dCov.mexa64'),'file')
    error('Compilation not completed!')
end

disp('Creating install Directory ...');
if ~exist(fullfile(scriptpath,'..','install'),'dir')
    mkdir(fullfile(scriptpath,'..','install'));
end
if ~exist(fullfile(scriptpath,'..','install','+icpWithCov'),'dir')
    mkdir(fullfile(scriptpath,'..','install','+icpWithCov'));
end

movefile(fullfile(scriptpath,'icp2dTr.mexa64'), fullfile(scriptpath,'..','install','+icpWithCov','icp2dTr.mexa64'));
movefile(fullfile(scriptpath,'icp2dCov.mexa64'), fullfile(scriptpath,'..','install','+icpWithCov','icp2dCov.mexa64'));

disp('... done!');

