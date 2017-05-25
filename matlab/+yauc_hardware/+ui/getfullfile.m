function f = getfullfile(varargin)
  [f, path] = uigetfile(varargin{:});
  if isequal(f, 0)
    error('embedded:rollout:nofile', 'No file specified');
  end
  f = fullfile(path, f);
end
