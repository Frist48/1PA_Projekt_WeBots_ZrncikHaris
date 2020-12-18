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
wb_motor_set_acceleration(front_motor1, 50);
wb_motor_set_acceleration(rear_motor1, 50);

ds1=wb_robot_get_device('ds1');
wb_distance_sensor_enable(ds1,TIME_STEP);
ds12 =wb_robot_get_device('ds12');
wb_distance_sensor_enable(ds12,TIME_STEP);

lidar1=wb_robot_get_device('lidar1');
wb_lidar_enable(lidar1,TIME_STEP);
wb_lidar_enable_point_cloud(lidar1);

while wb_robot_step(TIME_STEP) ~= -1

  wall_proximity11=wb_distance_sensor_get_value(ds1);
  wall_proximity12=wb_distance_sensor_get_value(ds12);
  
  wb_motor_set_velocity(front_motor1,0);
  wb_motor_set_velocity(rear_motor1,0);

  image = wb_lidar_get_range_image(lidar1)
  i = 60;
  c = 1;
  buff(c) = image(i-1);
  c = 2;
  
  while i < (98)
    buff(c) = image(i);
    delta_buff(c) = abs(buff(c) - buff(c-1));
    c = c+1;
    i = i+1;
  end
  
  max_delta = max(delta_buff);
  ball_pos = find(abs(delta_buff-max_delta) < 0.001);
  
  if ball_pos < 19 & wall_proximity12 > 30
    wb_motor_set_velocity(front_motor1,-10);
    wb_motor_set_velocity(rear_motor1,-10);
  elseif ball_pos > 19 & wall_proximity11 > 40
    wb_motor_set_velocity(front_motor1,10);
    wb_motor_set_velocity(rear_motor1,10);
  elseif ball_pos == 19
    wb_motor_set_velocity(front_motor1,0);
    wb_motor_set_velocity(rear_motor1,0);
  end
  
  drawnow;

end

