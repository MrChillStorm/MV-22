###############################################################################
##
###############################################################################

setlistener("/controls/probe/position-extended", func(v) {
  if (v.getValue()) {
    interpolate("/controls/probe/position-extended-pos", 1, 0.3);
  } else {
    interpolate("/controls/probe/position-extended-pos", 0, 0.3);
  }
});
