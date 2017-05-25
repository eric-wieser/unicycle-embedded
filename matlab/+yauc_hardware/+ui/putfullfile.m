function f = putfullfile(varargin)
  [f, path] = uiputfile(varargin{:});
  if f == 0
    error('embedded:rollout:nofile', 'No file specified');
  end
  f = fullfile(path, f);
end