var rotor_wash_loop = func {

  ##### water and grass shader effect
  var vpos = geo.viewer_position();
  var apos = geo.aircraft_position();

  var lat_to_m = 110952.0;
  var lon_to_m = math.cos(apos.lat()*math.pi/180.0) * lat_to_m;

  var alt = getprop("/position/altitude-agl-ft");

  var delta_x = (apos.lat() - vpos.lat()) * lat_to_m;
  var delta_y = -(apos.lon() - vpos.lon()) * lon_to_m;

  setprop("/environment/aircraft-effects/wash-x", delta_x);
  setprop("/environment/aircraft-effects/wash-y", delta_y);

  var rpm_factor = getprop("rotors/main/rpm")/400.0;

  var strength = 20.0/alt;
  if (strength > 1.0) {
    strength = 1.0;
  }
  strength = strength * rpm_factor;

  setprop("/environment/aircraft-effects/wash-strength", strength);

  # particle effect colors
  var land = getprop("/gear/gear/ground-is-solid");

  var red_diffuse = getprop("/rendering/scene/diffuse/red") or 0.8;      # (HDR has no scene diffuse colour)

  if (land) {
    setprop("/sim/model/v22/effects/particles/rotorwash/redcombinedstart",   red_diffuse * 0.40);
    setprop("/sim/model/v22/effects/particles/rotorwash/greencombinedstart", red_diffuse * 0.40);
    setprop("/sim/model/v22/effects/particles/rotorwash/bluecombinedstart",  red_diffuse * 0.40);
    setprop("/sim/model/v22/effects/particles/rotorwash/redcombinedend",     red_diffuse * 0.70);
    setprop("/sim/model/v22/effects/particles/rotorwash/greencombinedend",   red_diffuse * 0.70);
    setprop("/sim/model/v22/effects/particles/rotorwash/bluecombinedend",    red_diffuse * 0.70);
  } else {
    setprop("/sim/model/v22/effects/particles/rotorwash/redcombinedstart",   red_diffuse * 0.65);
    setprop("/sim/model/v22/effects/particles/rotorwash/greencombinedstart", red_diffuse * 0.80);
    setprop("/sim/model/v22/effects/particles/rotorwash/bluecombinedstart",  red_diffuse * 0.75);
    setprop("/sim/model/v22/effects/particles/rotorwash/redcombinedend",     red_diffuse * 0.85);
    setprop("/sim/model/v22/effects/particles/rotorwash/greencombinedend",   red_diffuse * 1.00);
    setprop("/sim/model/v22/effects/particles/rotorwash/bluecombinedend",    red_diffuse * 0.90);
  }
}

var rotor_washTimer = maketimer(0.01,rotor_wash_loop);
rotor_washTimer.start();
