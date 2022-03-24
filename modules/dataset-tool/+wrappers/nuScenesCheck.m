function nuScenesCheck(varargin)
%NUSCENESCHECK Check, if NuScenes is working!
%   Check if everything is ready to use.
%
%   See also NUSCENES.

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

    p = inputParser;
    addParameter(p, 'PackagePath', [], @ischar); 
    addParameter(p, 'PyEnv', [], @ischar);
    parse(p, varargin{:}); p = p.Results;

    if ~isempty(p.PyEnv), changePythonEnvironment(p.PyEnv); end
    
    if ~isempty(p.PackagePath) 
        eval('import wrappers.NuScenes.AddPythonPath');
        eval(['AddPythonPath(''' p.PackagePath ''')']); 
    end

    % Check, if NuScenes module can be loaded now.
    try
        % We need to use eval, otherwise python will be loaded as soon as
        % entering this function!
        eval('py.importlib.import_module(''nuscenes'');');
    catch
        warning('%s\n%s','Can not find nuscenes module within python path!', ...
                 'Try to add the path to the nuscenes-devkit or install nuscenes via pip for your current python environment!');
    end
end

function changePythonEnvironment(pyEnv)
% The NuScenes package may be installed within a separated
% virtual environment. To use it, we need to tell matlab where
% to find the python binary within this environment.
    if strcmp(pyenv().Executable, pyEnv)
        disp('The current pyenv is already set to the given one.');
    else
        assert(pyenv().Status == matlab.pyclient.Status.NotLoaded,...
            sprintf('%s\n%s','Can not set the python environment, if it is already loaded!',...
             'You need to restart Matlab and call this function again without calling some python functionality within matlab.'));
        disp('Trying to set new python environment. Matlab needs to be restarted afterwards!');
        pyenv('Version', pyEnv) % Load python env 
    end
end

