function msg = load_msg(fname)
  s = load(fname, 'msg');
  msg = s.msg;
end
