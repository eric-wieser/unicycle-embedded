function f = getfullfilemulti(varargin)
  % Select multiple files, queuing up extra files for later invocations
  persistent last
  if isempty(last)
    last = yauc_hardware.ui.getfullfile(varargin{:}, 'MultiSelect', 'on');
    if ~iscell(last)
      % 10/10 for api design, matlab - of course the user selecting a single
      % variable should return a completely different data type...
      last = {last};
    end
    fprintf('Queued %d log files for opening', length(last));
  end
  f = last{1};
  last = last(2:end);
end