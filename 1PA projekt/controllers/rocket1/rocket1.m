% MATLAB controller for Webots
% File:          ping_pong.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;

front_motor1=wb_robot_get_device('front_motor1');
rear_motor1=wb_robot_get_device('rear_motor1');
wb_motor_set_position(front_motor1,inf);
wb_motor_set_position(rear_motor1,inf);
wb_motor_set_velocity(front_motor1,-1);
wb_motor_set_velocity(rear_motor1,-1);
wb_motor_set_acceleration(front_motor1, 50);
wb_motor_set_acceleration(rear_motor1, 50);

ds1=wb_robot_get_device('ds1');
wb_distance_sensor_enable(ds1,TIME_STEP);
ds12 =wb_robot_get_device('ds12');
wb_distance_sensor_enable(ds12,TIME_STEP);

lidar1=wb_robot_get_device('lidar1');
wb_lidar_enable(lidar1,TIME_STEP);
wb_lidar_enable_point_cloud(lidar1);

offset = 0;
% get and enable devices, e.g.:
%  camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
%  motor = wb_robot_get_device('motor');

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
while wb_robot_step(TIME_STEP) ~= -1

  wall_proximity11=wb_distance_sensor_get_value(ds1);
  wall_proximity12=wb_distance_sensor_get_value(ds12);
  
  %if wall_proximity11<50 | wall_proximity12<50
  %  wb_motor_set_velocity(front_motor1,0);
  %  wb_motor_set_velocity(rear_motor1,0);
  %end
  
  
  if wall_proximity11 > 128
    offset = 0
  elseif wall_proximity11 < 128 & wall_proximity11 > 91
    offset =  -4
  elseif wall_proximity11 < 92 & wall_proximity11 > 54
    offset =  -5
  elseif wall_proximity11 < 55
    offset =  -6
  end
  
  if wall_proximity12 > 128
    offset = 0
  elseif wall_proximity12 < 128 & wall_proximity12 > 91
    offset =  4
  elseif wall_proximity12 < 92 & wall_proximity12 > 54
    offset =  5
  elseif wall_proximity12 < 55
    offset =  6
  end
  
  
  image = wb_lidar_get_range_image(lidar1)
  i = 67 + offset;
  c = 1;
  buff(c) = image(i-1);
  c = 2;
  while i < (90 + offset)
    buff(c) = image(i);
    delta_buff(c) = abs(buff(c) - buff(c - 1));
    c=c+1;
    i=i+1;
  end
  
  buff
  %min_value = min(buff)
  max_delta = max(delta_buff)
  %ball_pos = find(abs(buff-min_value) < 0.001)
  ball_pos = find(abs(delta_buff-max_delta) < 0.001)
  
  if ball_pos < 12 & wall_proximity12>40
    wb_motor_set_velocity(front_motor1,-4);
    wb_motor_set_velocity(rear_motor1,-4);
  elseif ball_pos > 12 & wall_proximity11>40
    wb_motor_set_velocity(front_motor1,4);
    wb_motor_set_velocity(rear_motor1,4);
  elseif ball_pos == 0 | 12 
    wb_motor_set_velocity(front_motor1,0);
    wb_motor_set_velocity(rear_motor1,0);
  end
  
  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
