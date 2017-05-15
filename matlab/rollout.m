function traj = rollout(start, ctrl, H, plant, cost, verb)
  this_file = mfilename('fullpath');
  root_dir = fileparts(fileparts(this_file));
  logs_dir = fullfile(root_dir, 'logs');

  % save policy params to mat file to load onto robot
  ctrl_msg = make_ctrl_msg(ctrl);
  ctrl_file = fullfile(root_dir, 'ctrl.mat');
  % ctrl_file = uiputfullfile('*.mat', ...
  %   'Save controller message', fullfile(logs_dir, 'ctrl.mat'));
  save_msg(ctrl_file, ctrl_msg);

  % load rollout data from robot
  log_file = uigetfullfile('*.mat', 'Load rollout logs', logs_dir);
  log_msg = load_msg(log_file);
  [z, u] = get_from_logs(log_msg, ctrl, plant);

  % evaluate the loss function, exactly as we would in normal rollout
  D = ctrl.D;
  for i = 1:size(z,2)
     L(i) = cost.fcn(struct('m',z(1:D,i))).m;
  end

  % set these in the same order as rollout, for matching display
  traj.observed = z;
  traj.latent = z;  % best we can do
  traj.action = u;
  traj.loss = L;
  traj.dt = plant.dt;
end

function f = uigetfullfile(varargin)
  [f, path] = uigetfile(varargin{:});
  f = fullfile(path, f);
end

function f = uiputfullfile(varargin)
  [f, path] = uiputfile(varargin{:});
  f = fullfile(path, f);
end

function msg = load_msg(fname)
  s = load(fname, 'msg');
  msg = s.msg;
end

function save_msg(fname, msg)
  [~] = msg;  % squash warning about unused var
  builtin('save', fname, 'msg');
end

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

function pol = get_linear_policy(ctrl)
  %% Produce a linear policy from the given controller
  % We assume that there is a @gSat term that means the policy aims to produce
  % control outputs in [-1, 1]
  z_frame = ctrl.in_frame;
  u_frame = ctrl.out_frame;
  if isa(ctrl, 'CtrlRandom')
    % make up some reasonable policy, unless we deside that random inputs are
    % desirable, and implement them on the hardware
    zE = z_frame.units;
    uE = u_frame.units;

    pol.b = u_frame.zeros;
    pol.w = zE.pitch' .* uE.cw *  1 / deg2rad(45) ...
          + zE.dyaw'  .* uE.ct * -1 / deg2rad(90);
  else
    pol = ctrl.policy.p;
    if ~isfield(pol, 'b') || ~isfield(pol, 'w')
      error('Expected a linear policy type')
    end
  end
end

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

  pol = get_linear_policy(ctrl);

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
