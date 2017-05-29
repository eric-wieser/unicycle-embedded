function pol = get_affine_policy(ctrl)
  %% Produce an affine policy from the given controller
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
    pol.w = zE.pitch' .* uE.cw *  1 / deg2rad(15) ...
          + zE.dyaw'  .* uE.ct *  1 / deg2rad(90);
  else
    pol = ctrl.policy.p;
    if ~isfield(pol, 'b') || ~isfield(pol, 'w')
      error('Expected a linear policy type')
    end
  end
end
