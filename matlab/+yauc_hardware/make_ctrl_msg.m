function msg = make_ctrl_msg(ctrl)
  %OUTPUT_CTRL_CONFIG  Convert the policy to a struct matching the protobuf
  %                    message

  % mapping between policy frame names and protobuf LinearPolicy fields
  p_name_map = struct();
  p_name_map.droll     = 'k_droll';
  p_name_map.dyaw      = 'k_dyaw';
  p_name_map.dwheel    = 'k_dAngleW';
  p_name_map.dpitch    = 'k_dpitch';
  p_name_map.dflywheel = 'k_dAngleTT';
  p_name_map.xc        = 'k_xOrigin';
  p_name_map.yc        = 'k_yOrigin';
  p_name_map.roll      = 'k_roll';
  p_name_map.yaw       = 'k_yaw';
  p_name_map.pitch     = 'k_pitch';

  % mapping between u frame names and protobug SetController fields
  u_name_map = struct();
  u_name_map.ct = 'turntable';
  u_name_map.cw = 'wheel';

  % check that all the names are correct, and we didn't miss any
  p_frame = ctrl.in_frame;
  u_frame = ctrl.out_frame;
  assert(isequal(sort(fieldnames(p_name_map)), sort(p_frame.names)));
  assert(isequal(sort(fieldnames(u_name_map)), sort(u_frame.names)));

  pol = yauc_hardware.get_linear_policy(ctrl);

  for u_name = fieldnames(u_name_map)', u_name = u_name{1};
    u_index = u_frame.i.(u_name);
    u_proto_name = u_name_map.(u_name);

    % copy across bias
    msg.(u_proto_name).k_bias = pol.b(u_index);

    % copy across weights
    for p_name = fieldnames(p_name_map)', p_name = p_name{1};
      p_index = p_frame.i.(p_name);
      p_proto_name = p_name_map.(p_name);
      msg.(u_proto_name).(p_proto_name) = pol.w(u_index, p_index);
    end
  end
end
