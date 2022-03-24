classdef NuScenes < handle
%NuScenes Wrapper class for NuScenes dataset.
%   This class implements an interface to the NuScenes Python package
%   to use it through Matlab's External Language Interface.
%
%   See also NUSCENESCHECK.

% @author Sven Lange (TU Chemnitz, ET/IT, Prozessautomatisierung)
% @author Karim Haggag (TU Chemnitz, ET/IT, Prozessautomatisierung)

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

    properties 
        Scenes             % all scenes encoded within the given dataset
        Filters = struct('Radar',... % Select only a filtered set:
                         struct('InvalidState',[0 4 11],...
                                'DynProp',[1 3 5]));
    end

    properties (SetAccess = private)
        PyEnv                     % Load python from virtual environment.
        NuscenesDataPath = [];    % where data set is stored, just keep in mind
                                  % you need to give the complete path to the folderincludes (maps, samples, sweeps)
        NuscenesDataVersion;
        Dataset = [];             % Instance to the dataset class (Python)
    end
    
    methods 
        
        function self = NuScenes(varargin)
        %NUSCENES Constructor.
            p = inputParser;
            addOptional( p, 'dataPath', [],         @ischar);
            addParameter(p, 'Version', 'v1.0-mini', @ischar); % Which dataset version to load?
            addParameter(p, 'PyEnv',   [],          @ischar);
            parse(p, varargin{:});

            self.PyEnv = pyenv().Executable;
            if ~isempty(p.Results.PyEnv)
                self.CheckPythonEnvironment(p.Results.PyEnv);
            end
            if ~isempty(p.Results.dataPath)
                self.setNuScenesDataPath(p.Results.dataPath); 
            end
            self.NuscenesDataVersion = p.Results.Version;
            if ~isempty(self.NuscenesDataPath), self.loadDataset(); end
        end
        
        function setNuScenesDataPath(self, dataPath)
        %SETNUSCENESDATAPATH Set the path to the NuScenes dataset.
            if nargin < 2, error('You need to specify a path!'); end
            assert(exist(dataPath,'dir'),'Given directory does not exist!');
            errmsg = 'Given directory does not qualify as full nuscenes dataset!';
            assert(exist(fullfile(dataPath,'samples'),'dir'),errmsg);
            assert(exist(fullfile(dataPath,'sweeps'),'dir'),errmsg);
            assert(exist(fullfile(dataPath,'maps'),'dir'),errmsg);
            self.NuscenesDataPath = dataPath;
        end
        
        function ret = get.Dataset(self)
        % Initialize the NuScenes python class for the given dataset path.
            
            % Try to load dataset, if not present.
            if isempty(self.Dataset), self.loadDataset(); end
            ret = self.Dataset;
        end
        
        function ret = get.Scenes(self)
        % Returns the available scenes within the dataset as a structure.
            if isempty(self.Scenes)
                self.Scenes = cellfun(@(x)struct(x),cell(self.Dataset.scene));
            end            
            ret = self.Scenes;
        end
        
        function scene = getSceneNo(self, scene)
        %GETSCENENO Convert to Scene index, if scene is given as string.
            assert(nargin > 1, 'You need to give a scene name as first argument!');
            if ischar(scene) || isstring(scene)
                sceneInd = find(strcmp(string({self.Scenes.name}), scene), 1);
                assert(~isempty(sceneInd),sprintf('Scene %s could not be found!',scene));
                scene = sceneInd;
            end
            validateattributes(scene,'numeric',{'integer','nonnegative','nonzero','<=',length(self.Scenes)});
        end
        
        function ret = sensorNames(self, scene)
        %SENSORNAMES Get sensor names from within scene.
        %   Todo: Don't look only in the first Data-Entry, but check over
        %   all data within the samples.
            scene = self.getSceneNo(scene);
            sample = struct(self.Dataset.get('sample', self.Scenes(scene).first_sample_token));
            ret =  string(fieldnames(struct(sample.data)));
        end
        
        function ret = sensorCalibration(self, scene, sensorName)
        %SENSORCALIBRATION Get sensor calibratoin from within scene for
        %   specific sensor.
        %   Todo: Don't look only in the first Data-Entry, but check over
        %   all data within the samples.
            scene = self.getSceneNo(scene);
            sample = struct(self.Dataset.get('sample', self.Scenes(scene).first_sample_token));
            token = struct(self.Dataset.get('sample_data', struct(sample.data).(sensorName))).calibrated_sensor_token;
            ret =  struct(self.Dataset.get('calibrated_sensor',token));
            if exist('SE3Type','class')
                rot = quat2rotm(cell2mat(cell(ret.rotation)));
                tran = cell2mat(cell(ret.translation));
                ret.transformation = SE3Type( ...
                    'Translation', tran, ...
                    'Rotation', permute(rot,[3 1 2]), ...
                    'Name', ['CALIBRATION_' sensorName]);
            end
        end
        
        function sensorData = getRadarSensor(self, varargin)
        %GETRADARSENSOR Collects the data for a radar sensor from the
        %   dataset into a matlab structure. 
        %   This specific method collects and calculates additional data
        %   only applicable to radar sensors. 
            p = inputParser;
            validFct = @(x)assert((ischar(x) || isstring(x)) && contains(x,'RADAR','IgnoreCase',true),...
                'Given sensor name must be a radar sensor''s name!');
            addRequired(p,'sensorName', validFct);
            addOptional(p,'scene', [], @(x)isnumeric(x) || isstring(x) || iscell(x)); % Only string or numeric. ischar conflicts with following name-value pairs!
            addParameter(p,'useKeyframesOnly', false, @islogical);
            % Calculate the rotation component, which is included within the
            % given velocity measurements.
            addParameter(p,'readYawRate',false, @islogical);
            % Use the extrinsic calibration information between Sensor and
            % Body frame
            addParameter(p,'useCalibration',true, @islogical);
            % Calculate the doppler based on ground truth ego motion
            addParameter(p,'predictDoppler',false, @islogical);
            % Calculate the Polar coordinates of the targets including std
            addParameter(p,'usePolar', false, @islogical);
            parse(p, varargin{:}); 
            
            pGetSensor = rmfield(p.Results, unique(... % Delete parameters not needed for getSensor
                [p.UsingDefaults {'readYawRate','predictDoppler','usePolar'}]));
            sensorData = self.getSensor(p.Results.sensorName, pGetSensor);
            
            if iscell(sensorData)
                for i=1:length(sensorData)
                    sensorData{i} = self.appendDoppler(sensorData{i},p.Results);
                    if p.Results.usePolar
                        sensorData{i} = self.appendPolar(sensorData{i});
                    end
                end
            else
                sensorData = self.appendDoppler(sensorData,p.Results);
                if p.Results.usePolar
                    sensorData = self.appendPolar(sensorData);
                end
            end
            
        end
        
        function sensorData = getSensor(self, varargin)
        %GETSENSOR Collects the data for one sensor from the dataset into a
        %   matlab structure.
            p = inputParser;
            addRequired(p,'sensorName', @(x)ischar(x) || isstring(x));
            addOptional(p,'scene', [], @(x)isnumeric(x) || isstring(x) || iscell(x)); % Only string or numeric. ischar conflicts with following name-value pairs!
            addParameter(p,'useCalibration', true, @islogical);
            addParameter(p,'useKeyframesOnly', false, @islogical);
            parse(p, varargin{:}); p = p.Results;
            
            assert(nargin > 1, 'You need to give a sensor name as first argument!');
            if nargin < 3 || isempty(p.scene) || iscell(p.scene) || length(p.scene) > 1
                if ~iscell(p.scene)
                    if isvector(p.scene)
                        p.scene = num2cell(p.scene);
                    else
                        p.scene = num2cell(1:length(self.Scenes)); % Select all available scenes
                    end
                end
                sensorData = cell(1,length(p.scene));
                for iScene = 1:length(p.scene)
                    sensorData{1, iScene} = self.getSensor(p.sensorName,p.scene{iScene},'useCalibration',p.useCalibration);
                end
                return;
            end
            scene = self.getSceneNo(p.scene);
            if p.useKeyframesOnly
                head = self.getSensorKeyframesHeadInfo(p.sensorName, scene);
            else
                head = self.getSensorHeadInfo(p.sensorName, scene);
            end
            
            sensorData = struct();
            sensorData.name = string(self.Scenes(scene).name);
            sensorData.description = string(self.Scenes(scene).description);
            sensorData.timestamp = self.Header2timestamp(head);
            if exist('SE3Type','class')
                sensorData.ego = self.header2egoSE3(head);
            else
                sensorData.ego = self.header2ego(head);
            end
            
            sensorData.data = self.filterRadar(self.header2pointCloud(head));

            if p.useCalibration
                data = arrayfun(@(x)struct(self.Dataset.get('calibrated_sensor',x.calibrated_sensor_token)), head);
                data = arrayfun(@(x)cell2struct(...
                horzcat(cell(x.translation),cell(x.rotation)),...
                {'x','y','z','qw','qx','qy','qz'},2), data);
                data = num2cell(data);
                [sensorData.data.T_BS] = data{:};
            end
        end

    end
    
    % Helper Functions
    
    methods(Hidden)

        function [nusc] = loadDataset(self)
        %LOADDATASET Initialize the NuScenes dataset class and provide
        %   header information to the dataset.
            try
                py.importlib.import_module('nuscenes')
            catch
                warning('Could not load the NuScene python package. Please use nuScenesCheck and try loadDataset again!')
                return
            end
            nusc = py.nuscenes.NuScenes( pyargs( ...
                'version',self.NuscenesDataVersion, ... 
                'dataroot', self.NuscenesDataPath, ...
                'verbose','True'));
            self.Dataset = nusc;
        end
        
        function sensorData = getSensorKeyframesHeadInfo(self, sensorName, scene)
        %GETSENSORKEYFRAMESHEADINFO Same as getSensorHeadInfo, but here we
        %   extract the header information for the keyframes only.
            
            assert(nargin > 1, 'You need to give a sensor name as first argument!');
            % check if no selected scene is provided, then take first scene 
            if nargin < 3 || isempty(scene)
                sensorData = cell(1,length(self.Scenes));
                for iScene = 1:length(self.Scenes)
                    sensorData{1, iScene} = self.getSensorKeyframesHeadInfo(sensorName,iScene);
                end
                return;
            end
            nusc = self.Dataset;
            scene = self.getSceneNo(scene);
            
            % To iterate only through keyframes, we choose sample.next
            dataPtr = self.Scenes(scene).first_sample_token;
            dataCnt = 1;

            while ~isempty(char(dataPtr))
                samples(dataCnt) = struct(nusc.get('sample', dataPtr));
                dataPtr = samples(dataCnt).next;
                dataCnt = dataCnt + 1;
            end
            sensorData = arrayfun(...
                @(x) struct(nusc.get('sample_data', struct(x.data).(sensorName))), samples);

        end
        
        function sensorData = getSensorHeadInfo(self, sensorName, scene)
        %GETSENSORHEADINFO Get header information for a specific sensor
        %   from scene or scenes. 
        %   This information can be used to extract its data from the
        %   dataset. 
            
            assert(nargin > 1, 'You need to give a sensor name as first argument!');
            % check if no selected scene is provided, then take first scene 
            if nargin < 3 || isempty(scene) || iscell(scene)
                if ~iscell(scene)
                    scene = num2cell(1:length(self.Scenes));
                end
                sensorData = cell(1,length(scene));
                for iScene = 1:length(scene)
                    sensorData{1, iScene} = self.getSensorHeadInfo(sensorName,scene{iScene});
                end
                return;
            end
            nusc = self.Dataset;
            scene = self.getSceneNo(scene);
            
            % Get first frame from scene (should be a keyframe)
            sample = struct(nusc.get('sample', self.Scenes(scene).first_sample_token));

            % For this frame, we look into the table 'sample_data', to select the token
            % for the sensor we want
            sensorData = struct(nusc.get('sample_data', struct(sample.data).(sensorName)));
            % To iterate through all the data, we choose
            while ~isempty(char(sensorData(end).next))
                sensorData(end + 1,1) = struct(nusc.get('sample_data', sensorData(end).next));
            end
        end
        
        function data = header2ego(self, sensorHeadInfo)
        %HEADER2EGO Extract ego pose from data and return as struct.
        %   Can be used, in case SE3Type is not available.
        %
        % INPUT:
        %   sensorHeadInfo ... struct with sensor's header information
            data = arrayfun(@(x)struct(self.Dataset.get('ego_pose',x.ego_pose_token)), sensorHeadInfo);
            data = arrayfun(@(x)cell2struct(...
                horzcat(cell(x.translation),cell(x.rotation)),...
                {'x','y','z','qw','qx','qy','qz'},2), data);
        end
        
        function data = header2egoSE3(self, sensorHeadInfo)
        %HEADER2EGOSE3 Extract ego pose from data and return as SE3Type.
        %
        % INPUT:
        %   sensorHeadInfo ... struct with sensor's header information
            data = arrayfun(@(x)struct(self.Dataset.get('ego_pose',x.ego_pose_token)), sensorHeadInfo);
            rot = arrayfun(@(x)cell2struct( cell(x.rotation), ...
                {'qw','qx','qy','qz'},2), data);
            rot = permute( quat2rotm( ...
                    [[rot.qw];[rot.qx];[rot.qy];[rot.qz]]' ), [3 1 2]);
            tran = arrayfun(@(x)cell2struct( cell(x.translation), ...
                {'X','Y','Z'},2), data);
            sceneToken = struct(self.Dataset.get('sample', sensorHeadInfo(1).sample_token)).scene_token;
            sceneName = char(struct(self.Dataset.get('scene', sceneToken)).name);
            data = SE3Type( ...
                    double(self.Header2timestamp(sensorHeadInfo))/1e6, ...
                    'Translation', [[tran.X];[tran.Y];[tran.Z]]', ...
                    'Rotation', rot, ...
                    'Name', sceneName);
        end
        
        function pcdOut = filterRadar(self, pcdIn)
        %FILTERRADAR Thin out the measurement by using the radar's cluster
        %   list information.
        %   See the ARS408 technical documentation.
            fNames = fieldnames(pcdIn(1));
            fNames(1) = []; % ignore timestamp field
            pcdOut = arrayfun(@helperFct, pcdIn);
            function in = helperFct(in)
                f = ismember(in.InvalidState, self.Filters.Radar.InvalidState);
                %f = f & in.AmbigState == 3;
                % filter dynamic ones
                f = f & ismember(in.DynProp, self.Filters.Radar.DynProp);
                for iField=1:length(fNames)
                    in.(fNames{iField}) = in.(fNames{iField})(f);
                end
            end
        end
        
        function sensorData = appendDoppler(self, sensorData, p)
        %APPENDDOPPLER Helper, see getSensorRadar()
            
            % Simple version ignoring standard deviation in phi
            %J_dop1 = @(x,y)[x/(x^2 + y^2)^(1/2), y/(x^2 + y^2)^(1/2)];
            % Including standard Deviation in phi
            J_dop2 = @(x,y,vx,vy)[(vx*y^2 - vy*x*y)/(x^2 + y^2)^(3/2), (x*(vy*x - vx*y))/(x^2 + y^2)^(3/2), x/(x^2 + y^2)^(1/2), y/(x^2 + y^2)^(1/2)];
            
            % Calculate doppler by projecting velocity onto vector pointing
            % from target to sensor.
            for iData = 1:length(sensorData.data)
                X = sensorData.data(iData).X;    Y = sensorData.data(iData).Y;
                Vx = sensorData.data(iData).Vx;  Vy = sensorData.data(iData).Vy;
                % Calculate using the Doppler prediction model
                % (Barjenbruch)
                % vx*cos(phi)+vy*sin(phi)
                % syms vx vy x y real
                % vdopp = simplify(vx*cos(atan2(y,x))+vy*sin(atan2(y,x)))
                sensorData.data(iData).doppler = (Vx.*X + Vy.*Y)./sqrt(X.^2 + Y.^2);
                % jacobian(vdopp,[vx vy])
                % simplify(jacobian(vdopp,[x y vx vy]))
                X_rms  = sensorData.data(iData).X_rms;    Y_rms = sensorData.data(iData).Y_rms;
                Vx_rms = sensorData.data(iData).Vx_rms;  Vy_rms = sensorData.data(iData).Vy_rms;
                % Simple version ignoring standard deviation in phi
                %sensorData.data(iData).doppler_rms1 = arrayfun(...
                %    @(x,y,sx,sy)sqrt(J_dop1(x,y)*diag([sx.^2 sy.^2])*J_dop1(x,y)'),...
                %    X,Y,Vx_rms,Vy_rms);
                % Including standard Deviation in phi
                sensorData.data(iData).doppler_rms = arrayfun(...
                    @(x,y,vx,vy,sx,sy,svx,svy)sqrt(J_dop2(x,y,vx,vy)*diag([sx.^2 sy.^2 svx.^2 svy.^2])*J_dop2(x,y,vx,vy)'),...
                    X,Y,Vx,Vy,X_rms,Y_rms,Vx_rms,Vy_rms);
            end
            
            % Calculate ground truth Doppler velocity 
            % (Model equation from Barjenbruch "Joint Spatial- and
            % Doppler-based Ego-Motion Estimation for Automotive Radars")
            if (p.predictDoppler)
                vEgo = sensorData.ego.calculateVelocity;
                vEgo = [0 0; vEgo(:,[1 2])];
                for iData = 1:length(sensorData.data)
                    phi = cart2pol(sensorData.data(iData).X, sensorData.data(iData).Y);
                    sensorData.data(iData).dopplerGt = -vEgo(iData,1).*cos(phi)-vEgo(iData,2).*sin(phi);
                end
            end
        end
                
        function data = header2pointCloud(self, sensorHeadInfo)
        %HEADER2POINTCLOUD This function gets sensor data, based on the
        %   provided sensorHeadInfo.
        %
        %   More information on the nuScenes Point-Cloud-Data format:
        %   https://github.com/nutonomy/nuscenes-devkit/blob/master/python-sdk/nuscenes/utils/data_classes.py#L313
        %
        % EXAMPLE:
        %   radar = nusc.header2pointCloud(nusc.getSensorHeadInfo('RADAR_FRONT', 1));            
            
            % define a radar struct 
            data(length(sensorHeadInfo)) = struct(...
                'TimeStamp', [], 'X',[],'Y',[],'Z',[], 'DynProp',[], 'ID',[],'RCS',[] , ...
                'Vx',[],'Vy',[],'Vx_comp',[],'Vy_comp',[],...
                'IsQualityValid',[], 'AmbigState',[], 'X_rms',[], 'Y_rms',[],...
                'InvalidState',[], 'PHD0', [], 'Vx_rms', [], 'Vy_rms',[]);
            isBinaryWarning = true;
            
            for i = 1 : length(sensorHeadInfo)
                   
                % - check if the pcd file is ascii or no 
                fid = fopen(fullfile(self.NuscenesDataPath, string(sensorHeadInfo(i).filename))) ;
                pcd_type  = textscan(fid,'%s %s', 1,'Delimiter',' ', 'headerlines', 10); 
                fclose(fid);
                
                % if pcd file in not ascii, call 
                if cell2mat(pcd_type{2}) == "binary"
                    if isBinaryWarning
                        msg = "To read all the information from the radar's pcd file, we need to convert\n";
                        msg = msg.append("it first from binary to ascii format. This will be done now for the complete Scene.\n");
                        msg = msg.append("For every file, we create a backup of the original, having the extension .back.");
                        warning('NUSCENES:CONVERSION',msg);
                        isBinaryWarning = false;
                    end
                    self.convertPointcloud2Ascii(fullfile(self.NuscenesDataPath,...
                                                          string(sensorHeadInfo(i).filename)));
                end
                
                fileID = fopen(fullfile(self.NuscenesDataPath, string(sensorHeadInfo(i).filename)));
                % FIELDS time x y z dyn_prop id rcs vx vy vx_comp vy_comp is_quality_valid ambig_state x_rms y_rms invalid_state pdh0 vx_rms vy_rms
                radar = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f', 'HeaderLines', 11);
                fclose(fileID);

                % read raw data 
                data(i).TimeStamp = double(sensorHeadInfo(i).timestamp);
                data(i).X = radar{1,1};
                data(i).Y = radar{1,2};
                data(i).Z = radar{1,3};
                data(i).DynProp = radar{1,4};
                data(i).ID = radar{1,5};
                data(i).RCS = radar{1,6};
                data(i).Vx = radar{1,7};
                data(i).Vy = radar{1,8};
                data(i).Vx_comp = radar{1,9};
                data(i).Vy_comp = radar{1,10};
                data(i).IsQualityValid = radar{1,11};
                data(i).AmbigState = radar{1,12};
                data(i).X_rms = self.rms2metric(double(radar{1, 13}));
                data(i).Y_rms = self.rms2metric(double(radar{1, 14})); 
                data(i).InvalidState = radar{1,15};
                data(i).PHD0 = radar{1,16};
                data(i).Vx_rms = self.rms2metric(double(radar{1, 17})); 
                data(i).Vy_rms = self.rms2metric(double(radar{1, 18})); 
            end
        end
    end
    
    methods(Static, Hidden)
        
        function data = Header2timestamp(sensorHeadInfo)
        %HEADER2TIMESTAMP Convert nuScenes header information into an
        %   integer timestamp.
        % INPUT:
        %   sensorHeadInfo ... struct with sensor's header information
        %
            data = arrayfun(@(x)int64(x.timestamp), sensorHeadInfo);
        end
        
        function convertPointcloud2Ascii(pcdFile)
        %CONVERTPOINTCLOUD2ASCEE Convert binary pcd file to ascii format. 
        %   Matlab needs the pcd file in ascii format, so we utilize the
        %   PCL libaray for conversion. The original file will be renamed
        %   to pcdFile.back.pcd. 
            
            % 1 - first make a copy with '.back'
            [b, c] = fileparts(pcdFile);
            fprintf('Converting pcd binary to ascii format for %s\n',c);
            assert(copyfile( pcdFile, fullfile( b, c.append('.back') )),...
                'Could not make a backup of original binary pcd file! Stopping now!');
            
            % 2 - convert pcdFileto ascii 
            cmd_ = ['pcl_convert_pcd_ascii_binary' ' "' char(pcdFile) '" "' char(pcdFile) '" ' '0'];
            [state, cmdout] = system(cmd_);
            assert( state == 0, sprintf('%s\n%s',...
                'Could not use pcl_convert_pcd_ascii_binary successfully to convert pcd to ascii!',...
                cmdout));
        end
        
        function ret = rms2metric(rmsValue)
        %RMS2METRIC Following the technical documentation of the used radar
        %   sensor, we can convert the rms values to meters or meters per
        %   second using a look-up-table. Alternative is to use the
        %   underliing exponential function as done here.
            
            %x=5;y=0.005*exp(0.2534*x)
            ret = 0.005.*exp(0.2534.*rmsValue);
        end

        function sensorData = appendPolar(sensorData)
        %APPENDPOLAR Convert the Cartesian target information into polar
        %   space and add it as separate data fields.
            
            for iData = 1:length(sensorData.data)
                X = sensorData.data(iData).X;          Y = sensorData.data(iData).Y;
                X_rms = sensorData.data(iData).X_rms;  Y_rms = sensorData.data(iData).Y_rms;
                [ rho, phi, rho_rms, phi_rms] = arrayfun(...
                    @wrappers.NuScenes.converCart2Polar, X, Y, X_rms, Y_rms);
                sensorData.data(iData).rho = rho;
                sensorData.data(iData).phi = phi;
                sensorData.data(iData).rho_rms = rho_rms;
                sensorData.data(iData).phi_rms = phi_rms;
            end
        end
        
    end
    
    methods(Static)
        
        function CheckPythonEnvironment(pyEnv)
        % The NuScenes package may be installed within a separated
        %    virtual environment. To use it, we need to tell matlab where
        %    to find the python binary within this environment.
            assert(nargin > 0, 'You need to specify the python binary of your environment!');
            
            assert(strcmp(pyenv().Executable, pyEnv),...
                sprintf('%s\n%s',...
                'The given python environment and the current one do not match!',...
                'Restart matlab and change the envrononment by using nuScenesCheck function.'));
        end
        
        function AddPythonPath(varargin)
        %ADDPYHTONPATH Adds an additional path to matlab's pythonpath
        %   environment varable to find the nuscenes package. 
        %   E.g. if you don't install the package, but downloaded the
        %   nuscenes-devkit into some directory. 
            p = inputParser;
            default = fullfile(getenv('HOME'),'workspace','nuscenes-devkit','python-sdk');
            validFct = @(x)assert(exist(x,'dir'),'Given directory does not exist!');
            addOptional(p,'packagePath', default, validFct);
            parse(p, varargin{:});
            validFct(p.Results.packagePath);
            if ~any(ismember(cellfun(@(x)string(x),cell(py.sys.path)),p.Results.packagePath))
                insert(py.sys.path, int32(0), p.Results.packagePath);
            end
        end
        
        function [rho, phi, std_rho, std_phi] = converCart2Polar(x, y, std_x, std_y)
        %CONVERTCART2POLAR Return the polar coordinates for each target,
        %   given in Cartesian cooridinates.
            
            f1        = @(x,y)[sqrt(x.^2+y.^2);atan2(y,x)];
            p1        = f1(x, y);
            % Convert given noise into polar space by error prediction.  
            % jacobian = [ jacbian(f1,x) jacbian(f1,y) ]
            jx = @(x,y)[x.*1.0./sqrt(x.^2+y.^2);...
                        -(imag(x)+real(y))./((imag(x)+real(y)).^2+(imag(y)-real(x)).^2)]; % d(f)/dx
            jy = @(x,y)[y.*1.0./sqrt(x.^2+y.^2);...
                        -(imag(y)-real(x))./((imag(x)+real(y)).^2+(imag(y)-real(x)).^2)]; % d(f)/dy
            jcob_     = [ jx(x, y) jy(x, y)];
            cov_cart  = [std_x.^2 0 ; 0 std_y.^2];
            cov_polar = jcob_*cov_cart*jcob_';
            rho       =  p1(1);
            phi       =  p1(2);
            std_rho   = sqrt(cov_polar(1,1));
            std_phi   = sqrt(cov_polar(2,2)); 
        end
    end
    
end
