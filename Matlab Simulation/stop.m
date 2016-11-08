function stop(btn, EventData)
  global s;
  fclose(s);
  delete(s);
  clear s;
  close all;
  clear all;
end