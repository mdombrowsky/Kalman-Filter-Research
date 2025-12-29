%% ROS 2 GPS Subscriber Node
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'); 
setenv('ROS_DOMAIN_ID', '0');

%% Initialize
node = ros2node("/gps_listener_node");

%% Subscriber for GPS fix
subGPS = ros2subscriber(node, "/gps/fix", "sensor_msgs/NavSatFix", @gpsCallback);

%% Create UI Window to stop
fig = uifigure('Name', 'GPS Listener', 'Position', [500, 500, 350, 200]);
stopButton = uibutton(fig, 'push', ...
    'Text', 'Stop & Shutdown', ...
    'Position', [100, 50, 150, 50], ...
    'ButtonPushedFcn', @(src,event) delete(fig));

disp("Node active. Listening to GPS messages...");

%% main loop
while isvalid(fig)
    pause(0.1); 
    drawnow;    
end

%% Cleanup
disp("Shutting down node...");
delete(subGPS);
clear subGPS node
disp("Cleanup complete.");

%% --- Callback Function ---

function gpsCallback(msg)
    fprintf('--- GPS Fix ---\n');
    fprintf('Lat: %.6f | Lon: %.6f | Alt: %.2f\n', msg.latitude, msg.longitude, msg.altitude);
end
