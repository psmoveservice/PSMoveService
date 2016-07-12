incl_path = fullfile(pwd, '..', 'src', 'psmoveclient');
addpath(incl_path);

lib_path = fullfile(pwd, '..', 'build', 'src', 'psmoveclient', 'Debug');
lib_ext = 'dylib';
%if strcmpi(computer(), 'WIN64')
%lib_path = 
%lib_ext = 'dll'
%end
addpath(lib_path);

% s_psmcontrollerlist = struct('controller_id', -1, 'controller_type', -1, 'count', 0);
%%
loadlibrary(fullfile(lib_path, ['libPSMoveClient_CAPI.', lib_ext]), fullfile(incl_path, 'PSMoveClient_CAPI.h'),...
    'addheader', fullfile(incl_path, 'PSMoveClient_export.h'),...
    'alias', 'psm',...
    'mfilename', 'psm_proto');
% libfunctionsview('psm')
calllib('psm','PSM_Initialize', 'localhost', '9512');

test = calllib('psm', 'PSM_GetController', 0);
setdatatype(test, 's_PSMControllerPtr', 1)

s_cl = libpointer('s_PSMControllerList');
calllib('psm','PSM_GetControllerList', s_cl);

% controllers = (PSMController **)malloc(sizeof(PSMController *)*PSMOVESERVICE_MAX_CONTROLLER_COUNT);
% int controller_count = PSM_GetControllerList(controllers);
% unsigned int data_stream_flags = PSMControllerDataStreamFlags::includePositionData |
%             PSMControllerDataStreamFlags::includePhysicsData | PSMControllerDataStreamFlags::includeRawSensorData |
%             PSMControllerDataStreamFlags::includeRawTrackerData;
% result = PSM_RegisterAsControllerListener(controllers[ctrl_ix]);
% result = PSM_StartControllerDataStream(controllers[ctrl_ix], data_stream_flags);

% PSMResult result = PSM_UpdateController(controllers[0]);

%PSM_StopControllerDataStream(controllers[ctrl_ix]);

calllib('psm','PSM_Shutdown');
clear s_cl
unloadlibrary('psm');