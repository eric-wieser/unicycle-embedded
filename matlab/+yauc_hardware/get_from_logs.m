function [z, u] = get_from_logs(msg, ctrl, plant)
  % mapping between protobuf LogEntry fields and z frame
  z_name_map.droll          = 'droll';
  z_name_map.dyaw           = 'dyaw';
  z_name_map.dAngleW        = 'dwheel';
  z_name_map.dpitch         = 'dpitch';
  z_name_map.dAngleTT       = 'dflywheel';
  z_name_map.xOrigin        = 'xc';
  z_name_map.yOrigin        = 'yc';
  z_name_map.roll           = 'roll';
  z_name_map.yaw            = 'yaw';
  z_name_map.pitch          = 'pitch';
  z_name_map.x              = 'x';
  z_name_map.y              = 'y';
  z_name_map.AngleW         = 'wheel';
  z_name_map.AngleTT        = 'flywheel';

  u_name_map.TurntableInput = 'ct';
  u_name_map.WheelInput     = 'cw';

  u = zeros(plant.in_frame.ndim, length(msg));
  z = zeros(plant.out_frame.ndim, length(msg));

  for z_name = fieldnames(z_name_map)', z_name = z_name{1};
    z_frame_name = z_name_map.(z_name);
    z_index = plant.out_frame.i.(z_frame_name);
    z(z_index,:) = [msg.(z_name)];
  end
  for u_name = fieldnames(u_name_map)', u_name = u_name{1};
    u_frame_name = u_name_map.(u_name);
    u_index = plant.in_frame.i.(u_frame_name);
    u(u_index,:) = [msg.(u_name)];
  end
  u = u(:,1:end-1);  % truncate the last unused action

  % scale from [-1 1], which we get from the robot, back to torques
  % This is basically reversing the gsat part of the controller, which we
  % don't actually im
  u = u .* ctrl.policy.maxU(:);
end
