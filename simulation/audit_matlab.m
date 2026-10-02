% Run in the Windows MATLAB installation to verify runtime product availability.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'results','matlab'); if ~exist(out,'dir'), mkdir(out); end
diary(fullfile(out,'toolbox_runtime_audit.txt'));
disp(version); disp(matlabroot); ver;
features={'SIMULINK','Simscape','SimHydraulics','SimMechanics','Aerospace_Blockset','Aerospace_Toolbox','Computer_Vision_Toolbox','Sensor_Fusion_and_Tracking_Toolbox'};
for k=1:numel(features)
    fprintf('%s license test: %d\n',features{k},license('test',features{k}));
end
% license(test) alone does not prove a seat can currently be checked out.
disp('Installed products, feature tests, and successful model execution are separate evidence.');
diary off;
