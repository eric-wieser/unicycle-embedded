function traj = rollout(start, ctrl, H, plant, cost, verb)
  import yauc_hardware.*;
  import yauc_hardware.ui.*;

  if nargin < 6; verb = 0; end
  if nargin < 5; cost = []; end

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
  log_file = getfullfilemulti('*.mat', 'Load rollout logs', logs_dir);
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

  % TODO: make this optional - full results are useful
  traj_c = apply_constraints(traj, plant, verb);

  if size(traj_c.observed, 2) ~= 0
    traj = traj_c;
    return
  end

  warning('Trajectory was empty');
  keyboard;

  % try again
  traj = rollout(start, ctrl, H, plant, cost, verb);
end

function traj = apply_constraints(traj, plant, verb)
  % truncate the trajectory to the portion that does not violate the constraints
  % this ought to match the pilco rollout logic
  if ~isfield(plant,'constraint')
    return;
  end

  H = size(traj.latent, 2) - 1;
  for i = 1:(H+1)
    if plant.constraint(traj.latent(:,i))
      if verb; disp('state constraints violated...'); end;
      H = i - 1;
      break;
    end
  end

  traj.observed = traj.observed(:,1:H);
  traj.latent   = traj.latent(:,1:H);
  traj.action   = traj.action(:,1:H-1);  % actions happen between states, so are one shorter
  traj.loss     = traj.loss(:,1:H);
end

