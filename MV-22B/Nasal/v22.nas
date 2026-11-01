# MV-22B Osprey with the JSBSim flight model (fdm/genv22.py): the start and stop sequences, the nacelle command, the
# wing stow, the 3-D model's nacelles and the sounds' helpers. The flight model mixes the controls, governs the rotor
# speed, tilts the nacelles and augments the stability itself; no Nasal runs it.
# After the original v22.nas by Maik Justus, partly based on bo105.nas by Melchior FRANZ.

if (!contains(globals, "cprint")) {
  globals.cprint  = func {};
}

var optarg        = aircraft.optarg;
var makeNode      = aircraft.makeNode;

var sin           = func(a) { math.sin(a * math.pi / 180.0) }
var cos           = func(a) { math.cos(a * math.pi / 180.0) }
var clamp         = func(v, min = 0, max = 1) { v < min ? min : v > max ? max : v }
var max           = func(a, b) { a > b ? a : b }
var min           = func(a, b) { a < b ? a : b }
var normatan      = func(x) { math.atan2(x, 1) * 2 / math.pi }

var FDM = "/fdm/jsbsim/";

# nacelles ==========================================================
# The pilot's nacelle command, the V-22's convention: 90 deg with the rotors up (helicopter), 0 in airplane mode, up to
# 97.5 aft. The flight model moves the nacelles towards it at the conversion rate, within the conversion protection's
# corridor. set_tilt() keeps the original aircraft's interface (its "tilt": 0 up, 90 forward): the mixture keys (m, M)
# step the command 10 deg, alt-m sets the nacelles vertical.
var nacelle_cmd    = props.globals.getNode("/controls/flight/nacelle-cmd-deg", 1);
var nacelle_pos    = props.globals.getNode("/sim/model/v22/nacelle-deg", 1);
var input_tilt     = props.globals.getNode("sim/model/v22/inputtilt", 1);
var animation_tilt = props.globals.getNode("sim/model/v22/animation_tilt", 1);
if (nacelle_cmd.getValue() == nil) {
  nacelle_cmd.setDoubleValue(90);
}

# AUTO NAC's schedule, the nacelle angle for the calibrated airspeed (Systems/mv22b-displays.xml, from fdm/genv22.py,
# which displays.nas reads too). Starting in the air the flight model sets the nacelles at it from its first step and
# trims itself; the 3-D model draws the nacelles at it before that
var DATA = "/instrumentation/mv22b/displays/data/";
io.read_properties(getprop("/sim/aircraft-dir") ~ "/Systems/mv22b-displays.xml", DATA);
var interp = func(x, xs, ys) {
  var n = size(xs);
  if (n == 0) return nil;
  if (x <= xs[0]) return ys[0];
  for (var i = 1; i < n; i += 1)
    if (x < xs[i]) return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
  return ys[n - 1];
}
var column = func(path, tag) {
  var out = [];
  var node = props.globals.getNode(DATA ~ path);
  if (node != nil) foreach (var c; node.getChildren(tag)) append(out, c.getValue());
  return out;
}
var autonac_kt = column("autonac", "kt");
var autonac_nac = column("autonac", "nac");
var autonac = func(kt) {
  var nac = interp(kt, autonac_kt, autonac_nac);
  return nac == nil ? 90 : nac;
}
var aloft = !getprop("/sim/presets/onground");
var start_kt = getprop("/sim/presets/airspeed-kt") or 0;
var start_nac = aloft ? autonac(start_kt) : 90;


var set_tilt = func (delta = 0, target = nil) {
  if (props.globals.getNode("sim/crashed", 1).getBoolValue()) {
    return;
  }
  if (wing_state.getValue() != 0) {
    return;
  }
  var tilt = (target == nil ? 90 - nacelle_cmd.getValue() : target) + delta;
  tilt = clamp(tilt, -7.5, 90);
  input_tilt.setValue(tilt);
  nacelle_cmd.setDoubleValue(90 - tilt);
}

controls.adjMixture = func(v) set_tilt(v > 0 ? 10 : -10);
controls.adjPropeller = func(v) wing_fold(v > 0);

# timers ============================================================
aircraft.timer.new("/sim/time/hobbs/helicopter", nil).start();

# engines, rotor, wing ==============================================
var state          = props.globals.getNode("sim/model/v22/state", 1);
var wing_state     = props.globals.getNode("sim/model/v22/wing_state", 1);
var wing_rotation  = props.globals.getNode("sim/model/v22/wing_rotation", 1);
var engine1        = props.globals.getNode("sim/model/v22/engine_right", 1);
var engine2        = props.globals.getNode("sim/model/v22/engine_left", 1);
var rotor_rpm      = props.globals.getNode("/rotors/main/rpm", 1);
var collective     = props.globals.getNode("/controls/engines/engine[0]/throttle", 1);
var cutoff0        = props.globals.getNode("/controls/engines/engine[0]/cutoff", 1);
var cutoff1        = props.globals.getNode("/controls/engines/engine[1]/cutoff", 1);
var rotor_brake    = props.globals.getNode("/controls/rotor/brake", 1);
var blade_folding  = props.globals.getNode("sim/model/v22/blade_folding", 1);
var fold_incidence = props.globals.getNode("sim/model/v22/wingfoldincidence", 1);
var torque_pct0    = props.globals.getNode("/engines/engine[0]/torque-pct", 1);
var torque_pct1    = props.globals.getNode("/engines/engine[1]/torque-pct", 1);
var stall_filtered = props.globals.getNode("/rotors/main/stall-filtered", 1);
var torque_sound_filtered = props.globals.getNode("/rotors/gear/torque-sound-filtered", 1);
var flaperon_override = props.globals.getNode(FDM ~ "ctl/flaperon-override", 1);
var flaperon_set   = props.globals.getNode(FDM ~ "ctl/flaperon-set-deg", 1);
var out_wing_flap  = props.globals.getNode("sim/model/v22/wing/flap", 1);

# the flaperons follow sim/model/v22/wing/flap (0..1 of their hover setting) while the wing stows or unstows
var hold_flaperons = func(on) {
  flaperon_override.setIntValue(on);
  if (on) {
    out_wing_flap.setDoubleValue(getprop(FDM ~ "ctl/flaperon-deg") / 67);
  }
}

# wing_state
#  0 fly-position
#  1 moving the nacelles up
#  2 folding the blades
#  3 rotating the wing and moving the nacelles down
# 10 fully folded
# 11 rotating the wing and moving the nacelles up
# 12 unfolding the blades
# 13 -> 0
var update_wing_state = func {
  var ws        = wing_state.getValue();
  var new_state = arg[0];
  if (new_state == (ws + 1)) {
    wing_state.setValue(new_state);
    if (new_state == 1) {
      var delta = abs(animation_tilt.getValue() - 0);
      settimer(func { update_wing_state(2) }, max(1.2, delta / 9 + 0.5));
      interpolate(animation_tilt, 0, delta / 9);
      hold_flaperons(1);
      interpolate(out_wing_flap, 0, 5.5);
      interpolate(fold_incidence, 1, 1);
    }
    if (new_state == 2) {
      settimer(func { update_wing_state(3) }, 4);
      interpolate(blade_folding, 1, 3.5);
    }
    if (new_state == 3) {
      settimer(func { update_wing_state(4) }, 12);
      interpolate(animation_tilt, 90, 9);
      interpolate(wing_rotation, 90, 11.5);
    }
    if (new_state == 4) {
      wing_state.setValue(10);
    }
    if (new_state == 11) {
      settimer(func { update_wing_state(12) }, 12);
      interpolate(wing_rotation, 0, 11.5);
      settimer(func { interpolate(animation_tilt, 0, 8.5); }, 2.5);
      settimer(func { interpolate(out_wing_flap, 1, 5.5); }, 7);
    }
    if (new_state == 12) {
      settimer(func { update_wing_state(13) }, 4);
      interpolate(blade_folding, 0, 3.5);
    }
    if (new_state == 13) {
      nacelle_cmd.setDoubleValue(90);
      interpolate(fold_incidence, 0, 1);
      hold_flaperons(0);
      wing_state.setValue(0);
    }
  }
}

var wing_fold = func {
  if (props.globals.getNode("sim/crashed", 1).getBoolValue()) {
    return;
  }
  if (state.getValue() or rotor_rpm.getValue() > 1) {
    return;              # engines running or rotors turning
  }
  var ws = wing_state.getValue();
  if (arg[0] == 1) {
    if (ws == 10) {
      update_wing_state(11);
    }
  } else {
    if (ws == 0) {
      update_wing_state(1);
    }
  }
}

# state (engines, for the sounds)
# 0 off
# 1 engine 1 starting
# 2 engine 2 starting
# 3 both at idle
# 4 rotor accelerating
# 5 flying
var start_timer = nil;

var engines = func {
  if (props.globals.getNode("sim/crashed", 1).getBoolValue()) {
    return;
  }
  if (arg[0] == 1) {
    if (state.getValue() != 0) {
      return;
    }
    var ws = wing_state.getValue();
    if (ws == 10) {
      update_wing_state(11);       # unstow first: the start waits for the wing
    }
    setprop("/controls/electric/battery-switch", 1);
    state.setValue(1);
    var go = func {
      if (wing_state.getValue() != 0) {
        settimer(go, 1);
        return;
      }
      rotor_brake.setDoubleValue(0);
      cutoff0.setIntValue(0);
      settimer(func {
        if (state.getValue() >= 1) {
          state.setValue(2);
          cutoff1.setIntValue(0);
        }
      }, 12);
    }
    go();
  } else {
    cutoff0.setIntValue(1);
    cutoff1.setIntValue(1);
    state.setValue(0);
  }
}

# the states the sounds follow, from the engines' spool and the rotor speed; the rotor brake parks the stopped rotors
var update_state = func {
  var s  = state.getValue();
  var sp0 = getprop(FDM ~ "eng[0]/spool") or 0;
  var sp1 = getprop(FDM ~ "eng[1]/spool") or 0;
  var nr = getprop(FDM ~ "drive/nr-pct") or 0;
  if (s >= 2 and sp0 > 0.99 and sp1 > 0.99) {
    state.setValue(nr > 90 ? 5 : nr > 50 ? 4 : 3);
  }
  if (s == 0 and cutoff0.getValue() and cutoff1.getValue() and rotor_rpm.getValue() < 30) {
    rotor_brake.setDoubleValue(1);
  }
}

# sound =============================================================
var last_wing_rotation   = 0;
var wing_rotation_speed  = props.globals.getNode("sim/model/v22/wing_rotation_speed", 1);
var last_blade_folding   = 0;
var blade_folding_speed  = props.globals.getNode("sim/model/v22/blade_folding_speed", 1);
var last_flap            = 0;
var flap_speed           = props.globals.getNode("sim/model/v22/flap_speed", 1);
var flap_pos             = props.globals.getNode("/surface-positions/flap-pos-norm", 1);
var last_animation_tilt  = 0;
var animation_tilt_speed = props.globals.getNode("sim/model/v22/animation_tilt_speed", 1);

var rate = func(node, last, value, dt) {
  var r = dt > 0.00001 ? abs(value - last) / dt : abs(value - last) * 120;
  var f = dt / (0.05 + dt);
  node.setValue(r * f + (node.getValue() or 0) * (1 - f));
}

var update_sound = func(dt) {
  var wr = wing_rotation.getValue() or 0;
  rate(wing_rotation_speed, last_wing_rotation, wr, dt); last_wing_rotation = wr;
  var bf = blade_folding.getValue() or 0;
  rate(blade_folding_speed, last_blade_folding, bf, dt); last_blade_folding = bf;
  var fp = flap_pos.getValue() or 0;
  rate(flap_speed, last_flap, fp, dt); last_flap = fp;
  var at = animation_tilt.getValue() or 0;
  rate(animation_tilt_speed, last_animation_tilt, at, dt); last_animation_tilt = at;
  # the engines' load for their sounds (the original's torque scale): half at idle, full at the rating, whatever
  # the rotor speed (the engines spool up first); the blade slap from the rotor speed and the torque
  var t = clamp(((torque_pct0.getValue() or 0) + (torque_pct1.getValue() or 0)) / 200);
  torque_sound_filtered.setDoubleValue(t * 0.5 + 0.5);
  stall_filtered.setDoubleValue(clamp(rotor_rpm.getValue() * 0.004 - 0.2) * min(1, 0.5 + 0.6 * t));
}

# skid slide sound
var Skid = {
  new : func(n) {
    var m = { parents : [Skid] };
    var soundN = props.globals.getNode("sim/sound", 1).getChild("slide", n, 1);
    var gearN  = props.globals.getNode("gear", 1).getChild("gear", n, 1);
    m.compressionN = gearN.getNode("compression-norm", 1);
    m.rollspeedN   = gearN.getNode("rollspeed-ms", 1);
    m.frictionN    = gearN.getNode("ground-friction-factor", 1);
    m.wowN         = gearN.getNode("wow", 1);
    m.volumeN      = soundN.getNode("volume", 1);
    m.pitchN       = soundN.getNode("pitch", 1);
    m.volumeN.setDoubleValue(0);
    m.pitchN.setDoubleValue(0);
    m.self = n;
    return m;
  },
  update : func {
    me.wowN.getBoolValue() or return;
    var rollspeed = abs(me.rollspeedN.getValue() or 0);
    me.pitchN.setDoubleValue(rollspeed * 0.6);
    var s = normatan(20 * rollspeed);
    var fr = me.frictionN.getValue();
    var f = clamp(((fr == nil ? 1 : fr) - 0.5) * 2);
    var c = clamp((me.compressionN.getValue() or 0) * 2);
    me.volumeN.setDoubleValue(s * f * c * 2);
  },
};

# crash handler =====================================================
var crash = func {
  if (arg[0]) {
    cutoff0.setIntValue(1);
    cutoff1.setIntValue(1);
    setprop("/controls/lighting/strobe", 0);
    setprop("/controls/lighting/beacon", 0);
    setprop("/controls/lighting/nav-lights", 0);
    stall_filtered.setValue(0);
    state.setValue(0);
  }
}

# view management ===================================================
var elapsedN = props.globals.getNode("/sim/time/elapsed-sec", 1);
var flap_mode = 0;
var down_time = 0;
controls.flapsDown = func(v) {
  if (!flap_mode) {
    if (v < 0) {
      down_time = elapsedN.getValue();
      flap_mode = 1;
      dynamic_view.lookat(5, -20, 0, 0.2, 0.6, 0.85, 0.2, 55);
    } elsif (v > 0) {
      flap_mode = 2;
      var p = "/sim/view/dynamic/enabled";
      setprop(p, !getprop(p));
    }
  } else {
    if (flap_mode == 1) {
      if (elapsedN.getValue() < down_time + 0.2) {
        return;
      }
      dynamic_view.resume();
    }
    flap_mode = 0;
  }
}

dynamic_view.register(func {
  var lowspeed      = 1 - normatan(me.speedN.getValue() / 50);
  var r             = sin(me.roll) * cos(me.pitch);
  me.heading_offset = (me.roll < 0 ? -50 : -30) * r * abs(r);
  me.pitch_offset   = -50 * sin(me.pitch) * lowspeed + 15 * sin(me.roll) * sin(me.roll);
  me.roll_offset    = -15 * r * lowspeed;
});

# main() ============================================================
var delta_time = props.globals.getNode("/sim/time/delta-realtime-sec", 1);

var main_loop = func {
  var dt = delta_time.getValue();
  # the 3-D model's nacelles (its "tilt": 0 up, 90 forward) follow the flight model's, but while the wing stows
  # (nil before the flight model's first output: the start's, vertical on the ground; "or" would take airplane mode's
  # 0 deg for nil too)
  if (wing_state.getValue() == 0) {
    var nac = nacelle_pos.getValue();
    animation_tilt.setDoubleValue(90 - (nac == nil ? start_nac : nac));
  }
  if (flaperon_override.getValue()) {
    flaperon_set.setDoubleValue(67 * (out_wing_flap.getValue() or 0));
  }
  update_state();
  update_sound(dt);
  settimer(main_loop, 0);
}

var crashed = 0;
var variant = nil;
var doors = nil;
var config_dialog = nil;

var is_initialized = 0;
setlistener("/sim/signals/fdm-initialized", func {
  # airborne: running, the nacelles where the flight model put them for the airspeed (AUTO NAC's angle; the lever is
  # its level-flight trim's); on the ground: cold and dark
  var kt = getprop("/velocities/airspeed-kt") or 0;
  if (getprop("/position/altitude-agl-ft") > 50) {
    var nac = getprop("/fdm/jsbsim/ctl/nacelle-deg");
    if (nac == nil) nac = autonac(kt);
    nacelle_cmd.setDoubleValue(nac);
    input_tilt.setValue(90 - nac);
    state.setValue(5);
  } else {
    collective.setDoubleValue(0);
    state.setValue(0);
  }

  if (is_initialized) {
    return;
  }
  is_initialized = 1;

  setlistener("/sim/signals/reinit", func(n) {
    n.getBoolValue() and return;
    cprint("/32;1", "reinit");
    if (variant != nil) variant.scan();
    crashed = 0;
  });

  setlistener("sim/crashed", func(n) {
    cprint("/31;1", "crashed ", n.getValue());
    if (n.getBoolValue()) {
      crash(crashed = 1);
    }
  });

  main_loop();
});
