%% ROS 2 Sensor Node with IMU, Magnetometer, GPS
clear workspace;

%% Initialization
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');
setenv('ROS_DOMAIN_ID', '0');
node = ros2node("/sensor_node");

%% Subscribers
subIMU1 = ros2subscriber(node, "/imu1", "sensor_msgs/Imu", @(msg) imuCallback(msg, 'IMU1'));
subIMU2 = ros2subscriber(node, "/imu2", "sensor_msgs/Imu", @(msg) imuCallback(msg, 'IMU2'));
subMAG = ros2subscriber(node, "/imu2/mag", "sensor_msgs/MagneticField", @(msg) magCallback(msg, 'MAG'));
subGPS = ros2subscriber(node, "/gps/fix", "sensor_msgs/NavSatFix", @(msg) gpsCallback(msg, 'GPS'));

%% create bt to stop
subs = {subIMU1, subIMU2, subMAG, subGPS};

%fig = uifigure('Name', 'Sensor Controller', 'Position', [500, 500, 350, 200]);
%stopButton = uibutton(fig, 'push', ...
%    'Text', 'Stop & Shutdown', ...
%	'Position', [100, 50, 150, 50], ...
%	'ButtonPushedFcn', @(src,event) delete(fig));
fig = uifigure( ...
    'Name', 'Sensor Controller', ...
    'Position', [500, 500, 350, 200], ...
    'CloseRequestFcn', @(src,evt) shutdownROS(src, subs, node));

uibutton(fig, 'push', ...
    'Text', 'Stop & Shutdown', ...
    'Position', [100, 50, 150, 50], ...
    'ButtonPushedFcn', @(src,event) shutdownROS(fig, subs, node));

disp("Node active. Monitoring IMU1, IMU2, MAG, GPS...");

%% Create Database
%db_conn = sqlite('sensor_logs.db', 'create');
db_file = 'sensor_logs.db';
if exist(db_file, 'file')
    db_conn = sqlite(db_file, 'connect'); % if already created
else
    db_conn = sqlite(db_file, 'create');  % if not already created
end

% create tables
execute(db_conn, ['CREATE TABLE IF NOT EXISTS imu_logs (' ...
	'id INTEGER PRIMARY KEY AUTOINCREMENT, ' ...
	'timestamp REAL, ' ...
	'sensor TEXT, ' ...
	'ax REAL, ay REAL, az REAL, ' ...
	'gx REAL, gy REAL, gz REAL)']);
execute(db_conn, ['CREATE TABLE IF NOT EXISTS mag_logs (' ...
	'id INTEGER PRIMARY KEY AUTOINCREMENT, ' ...
	'timestamp REAL, ' ...
	'bx REAL, by REAL, bz REAL)']);
%execute(db_conn, ['CREATE TABLE IF NOT EXISTS gps_logs (' ...
%    'id INTEGER PRIMARY KEY AUTOINCREMENT, ' ...
%    'timestamp REAL, ' ...
%    'bx REAL, by REAL, bz REAL)']);

%% main loop
while isvalid(fig)
	pause(0.1);
	drawnow;
end

%% kill ros and cleanup

%delete(subIMU1); delete(subIMU2); delete(subMAG); delete(subGPS);
%clear subIMU1 subIMU2 subMAG subGPS node
function shutdownROS(fig, subs, node)
    disp("Shutting down ROS node...");

    for i = 1:numel(subs)
        if isvalid(subs{i})
            delete(subs{i});
        end
    end

    if isvalid(node)
        delete(node);
    end

    delete(fig);
    clear workspace;
    disp("cleanup complete");
end

%% --- Callback Functions ---

% IMU Callback (6-DOF)
function imuCallback(msg, sensorName)
	accel_threshold = 0.1;
	gyro_threshold = 0.05;

	ax = msg.linear_acceleration.x; ay = msg.linear_acceleration.y; az = msg.linear_acceleration.z;
	gx = msg.angular_velocity.x; gy = msg.angular_velocity.y; gz = msg.angular_velocity.z;

	if any(abs([ax, ay, az]) > accel_threshold) || any(abs([gx, gy, gz]) > gyro_threshold)
        fprintf('--- %s Motion Detected ---\n', sensorName);
        fprintf('Accel: [%.2f, %.2f, %.2f] | Gyro: [%.2f, %.2f, %.2f]\n', ax, ay, az, gx, gy, gz);
	end
end

% Magnetometer Callback
function magCallback(msg, sensorName)
	B = msg.magnetic_field;
	fprintf('--- %s Magnetic Field ---\n', sensorName);
	fprintf('Bx: %.2f | By: %.2f | Bz: %.2f\n', B.x, B.y, B.z);
end

% GPS Callback
function gpsCallback(msg, sensorName)
	fprintf('--- %s GPS Fix ---\n', sensorName);
	fprintf('Lat: %.6f | Lon: %.6f | Alt: %.2f\n', msg.latitude, msg.longitude, msg.altitude);
end
