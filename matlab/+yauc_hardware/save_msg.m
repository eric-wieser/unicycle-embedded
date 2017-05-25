function save_msg(fname, msg)
  [~] = msg;  % squash warning about unused var
  builtin('save', fname, 'msg');
end
