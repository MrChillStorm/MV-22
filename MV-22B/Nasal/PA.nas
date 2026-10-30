#################################################################
## PA managment                     BARANGER Emmanuel 04/2026  ##
#################################################################
var PAOn = func {

  ##### Altitude and heading
  var AircraftAltitude = getprop("position/altitude-ft");
  var AircraftHeading  = getprop("orientation/heading-deg");

  setprop("autopilot/settings/target-altitude-ft",AircraftAltitude);
  setprop("autopilot/settings/true-heading-deg",AircraftHeading);

  setprop("autopilot/locks/altitude","altitude-hold");
  setprop("autopilot/locks/heading","true-heading-hold");

}

var PAOff = func {

  setprop("autopilot/locks/altitude","");
  setprop("autopilot/locks/heading","");
  setprop("autopilot/locks/speed", "");

}

var PAToggle = func {
  var alt_lock = getprop("autopilot/locks/altitude") or "";
  var hdg_lock = getprop("autopilot/locks/heading") or "";

  var active = ( alt_lock == "altitude-hold" ) and ( hdg_lock == "true-heading-hold" );

  screen.log.write("AP Status " ~ ( active ? "OFF":"ON" ), 1, 0, 0);

  if (active) {
    PAOff();
  } else {
    PAOn();
  }
}
