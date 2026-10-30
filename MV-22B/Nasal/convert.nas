# ************************************
# ****  Helijah          02/2026  ****
# **** Based on 5H1N0B1 work 2013 ****
# ************************************

var convert = func {
  var rpm0  = getprop("/engines/engine[0]/rpm");
  var cht0  = getprop("/engines/engine[0]/cht-degc");
  var egt0  = getprop("/engines/engine[0]/egt-degc");
  var mp0   = getprop("/engines/engine[0]/mp-osi");
  var run0  = getprop("/engines/engine[0]/running");
  var flow0 = getprop("/engines/engine[0]/fuel-flow-gph");
  var otf0  = getprop("/engines/engine[0]/oil-temperature-degf");
  var trq0  = getprop("/engines/engine[0]/torque-ftlb");
  var cylt0 = 0.0;

  var rpm1  = getprop("/engines/engine[1]/rpm");
  var cht1  = getprop("/engines/engine[1]/cht-degc");
  var egt1  = getprop("/engines/engine[1]/egt-degc");
  var mp1   = getprop("/engines/engine[1]/mp-osi");
  var run1  = getprop("/engines/engine[1]/running");
  var flow1 = getprop("/engines/engine[1]/fuel-flow-gph");
  var otf1  = getprop("/engines/engine[1]/oil-temperature-degf");
  var trq1  = getprop("/engines/engine[1]/torque-ftlb");
  var cylt1 = 0.0;

  var rpm2  = getprop("/engines/engine[2]/rpm");
  var cht2  = getprop("/engines/engine[2]/cht-degc");
  var egt2  = getprop("/engines/engine[2]/egt-degc");
  var mp2   = getprop("/engines/engine[2]/mp-osi");
  var run2  = getprop("/engines/engine[2]/running");
  var flow2 = getprop("/engines/engine[2]/fuel-flow-gph");
  var otf2  = getprop("/engines/engine[2]/oil-temperature-degf");
  var trq2  = getprop("/engines/engine[2]/torque-ftlb");
  var cylt2 = 0.0;

  var rpm3  = getprop("/engines/engine[3]/rpm");
  var cht3  = getprop("/engines/engine[3]/cht-degc");
  var egt3  = getprop("/engines/engine[3]/egt-degc");
  var mp3   = getprop("/engines/engine[3]/mp-osi");
  var run3  = getprop("/engines/engine[3]/running");
  var flow3 = getprop("/engines/engine[3]/fuel-flow-gph");
  var otf3  = getprop("/engines/engine[3]/oil-temperature-degf");
  var trq3  = getprop("/engines/engine[3]/torque-ftlb");
  var cylt3 = 0.0;

  var oat   = getprop("/environment/temperature-degc");
  var ias   = getprop("/instrumentation/airspeed-indicator/indicated-speed-kt");

  # started in the air the engines run from the first call, before the temperatures have a value: from the air's
  # (a nil here stopped this loop for the rest of the flight)
  if (cht0 == nil) cht0 = oat;
  if (egt0 == nil) egt0 = oat;
  if (cht1 == nil) cht1 = oat;
  if (egt1 == nil) egt1 = oat;
  if (cht2 == nil) cht2 = oat;
  if (egt2 == nil) egt2 = oat;
  if (cht3 == nil) cht3 = oat;
  if (egt3 == nil) egt3 = oat;

  # ---------------------------------------- Engine 0 ----------------------------------------
  if ( !mp0 ) {
    mp0  = 10;
  }
  if ( mp0 < 10 ) {
    mp0  = 10;
  }
  if ( !rpm0 ) {
    rpm0 = 0;
  }
  if (rpm0 > 100.0) {
    var fuel_pres0 = rpm0 / 125;
    var oil_pres0  = rpm0 / 25;
    var cart0      = cht0 / 20;
  } else {
    var fuel_pres0 = 0.0;
    var oil_pres0  = 0.0;
    var cart0      = 0.0;
  }
  if (run0) {
    cht0           = cht0 + (mp0 * 8 + oat - ias/3 - cht0) / 250;
    egt0           = egt0 + ((mp0 * 30 + cht0 * 2) * mp0 / (flow0 * 2 + 1) - egt0) / 100;
    cylt0          = egt0 / 1.5;
  } else {
    if ( !cht0  ) {
      cht0         = oat;
    }
    if ( !egt0  ) {
      egt0         = oat;
    }
    cht0           = cht0 + (oat - cht0)/100;
    egt0           = egt0 + (oat - egt0)/100;
    cylt0          = egt0 / 1.5;
  }
  if ( !trq0 ) {
    trq0  = 0;
  }
  if ( trq0 < 0 ) {
    trq0  = 0;
  }
  setprop("/engines/engine[0]/oil-temperature-degc", convertTemp(otf0));
  setprop("/engines/engine[0]/cht-degc", cht0);
  setprop("/engines/engine[0]/egt-degc", egt0);
  setprop("/engines/engine[0]/egt-degf-calc", egt0 * 9/5 + 32);
  setprop("/engines/engine[0]/oil-pressure-psi", oil_pres0);
  setprop("/engines/engine[0]/fuel-pressure-psi", fuel_pres0);
  setprop("/engines/engine[0]/cyl-temp", cylt0);
  setprop("/engines/engine[0]/carb-temp-degc", cart0);
  setprop("/engines/engine[0]/itt-norm", egt0 / 100);
  setprop("/engines/engine[0]/torque", trq0 / 42);
  setprop("/engines/engine[0]/cht-degf", convertTempF(getprop("/engines/engine[0]/cht-degc")));
  # ---------------------------------------- Engine 1 ----------------------------------------
  if ( !mp1 ) {
    mp1  = 10;
  }
  if (mp1 < 10) {
    mp1  = 10;
  }
  if ( !rpm1 ) {
    rpm1 = 0;
  }
  if (rpm1 > 100.0) {
    var fuel_pres1 = rpm1 / 125;
    var oil_pres1  = rpm1 / 25;
    var cart1      = cht1 / 20;
  } else {
    var fuel_pres1 = 0.0;
    var oil_pres1  = 0.0;
    var cart1      = 0.0;
  }
  if (run1) {
    cht1           = cht1 + (mp1 * 8 + oat - ias/3 - cht1) / 250;
    egt1           = egt1 + ((mp1 * 30 + cht1 * 2) * mp1 / (flow1 * 2 + 1) - egt1) / 100;
    cylt1          = egt1 / 1.5;
  } else {
    if ( !cht1  ) {
      cht1         = oat;
    }
    if ( !egt1  ) {
      egt1         = oat;
    }
    cht1           = cht1 + (oat - cht1)/100;
    egt1           = egt1 + (oat - egt1)/100;
    cylt1          = egt1 / 1.5;
  }
  if ( !trq1 ) {
    trq1  = 0;
  }
  if ( trq1 < 0 ) {
    trq1  = 0;
  }
  setprop("/engines/engine[1]/oil-temperature-degc", convertTemp(otf1));
  setprop("/engines/engine[1]/cht-degc", cht1);
  setprop("/engines/engine[1]/egt-degc", egt1);
  setprop("/engines/engine[1]/egt-degf-calc", egt1 * 9/5 + 32);
  setprop("/engines/engine[1]/oil-pressure-psi", oil_pres1);
  setprop("/engines/engine[1]/fuel-pressure-psi", fuel_pres1);
  setprop("/engines/engine[1]/cyl-temp", cylt1);
  setprop("/engines/engine[1]/carb-temp-degc", cart1);
  setprop("/engines/engine[1]/itt-norm", egt1 / 100);
  setprop("/engines/engine[1]/torque", trq1 / 42);
  setprop("/engines/engine[1]/cht-degf", convertTempF(getprop("/engines/engine[1]/cht-degc")));
  # ---------------------------------------- Engine 2 ----------------------------------------
  if ( !mp2 ) {
    mp2  = 10;
  }
  if ( mp2 < 10 ) {
    mp2  = 10;
  }
  if ( !rpm2 ) {
    rpm2 = 0;
  }
  if (rpm2 > 100.0) {
    var fuel_pres2 = rpm2 / 125;
    var oil_pres2  = rpm2 / 25;
    var cart2      = cht2 / 20;
  } else {
    var fuel_pres2 = 0.0;
    var oil_pres2  = 0.0;
    var cart2      = 0.0;
  }
  if (run2) {
    cht2           = cht2 + (mp2 * 8 + oat - ias/3 - cht2) / 250;
    egt2           = egt2 + ((mp2 * 30 + cht2 * 2) * mp2 / (flow2 * 2 + 1) - egt2) / 100;
    cylt2          = egt2 / 1.5;
  } else {
    if ( !cht2  ) {
      cht2         = oat;
    }
    if ( !egt2  ) {
      egt2         = oat;
    }
    cht2           = cht2 + (oat - cht2)/100;
    egt2           = egt2 + (oat - egt2)/100;
    cylt2          = egt2 / 1.5;
  }
  if ( !trq2 ) {
    trq2  = 0;
  }
  if ( trq2 < 0 ) {
    trq2  = 0;
  }
  setprop("/engines/engine[2]/oil-temperature-degc", convertTemp(otf2));
  setprop("/engines/engine[2]/cht-degc", cht2);
  setprop("/engines/engine[2]/egt-degc", egt2);
  setprop("/engines/engine[2]/egt-degf-calc", egt2 * 9/5 + 32);
  setprop("/engines/engine[2]/oil-pressure-psi", oil_pres2);
  setprop("/engines/engine[2]/fuel-pressure-psi", fuel_pres2);
  setprop("/engines/engine[2]/cyl-temp", cylt2);
  setprop("/engines/engine[2]/carb-temp-degc", cart2);
  setprop("/engines/engine[2]/itt-norm", egt2 / 100);
  setprop("/engines/engine[2]/torque", trq2 / 42);
  setprop("/engines/engine[2]/cht-degf", convertTempF(getprop("/engines/engine[2]/cht-degc")));
  # ---------------------------------------- Engine 3 ----------------------------------------
  if ( !mp3 ) {
    mp3  = 10;
  }
  if ( mp3 < 10 ) {
    mp3  = 10;
  }
  if ( !rpm3 ) {
    rpm3 = 0;
  }
  if (rpm3 > 100.0) {
    var fuel_pres3 = rpm3 / 125;
    var oil_pres3  = rpm3 / 25;
    var cart3      = cht3 / 20;
  } else {
    var fuel_pres3 = 0.0;
    var oil_pres3  = 0.0;
    var cart3      = 0.0;
  }
  if (run3) {
    cht3           = cht3 + (mp3 * 8 + oat - ias/3 - cht3) / 250;
    egt3           = egt3 + ((mp3 * 30 + cht3 * 2) * mp3 / (flow3 * 2 + 1) - egt3) / 100;
    cylt3          = egt3 / 1.5;
  } else {
    if ( !cht3  ) {
      cht3         = oat;
    }
    if ( !egt3  ) {
      egt3         = oat;
    }
    cht3           = cht3 + (oat - cht3)/100;
    egt3           = egt3 + (oat - egt3)/100;
    cylt3          = egt3 / 1.5;
  }
  if ( !trq3 ) {
    trq3  = 0;
  }
  if ( trq3 < 0 ) {
    trq3  = 0;
  }
  setprop("/engines/engine[3]/oil-temperature-degc", convertTemp(otf3));
  setprop("/engines/engine[3]/cht-degc", cht3);
  setprop("/engines/engine[3]/egt-degc", egt3);
  setprop("/engines/engine[3]/egt-degf-calc", egt3 * 9/5 + 32);
  setprop("/engines/engine[3]/oil-pressure-psi", oil_pres3);
  setprop("/engines/engine[3]/fuel-pressure-psi", fuel_pres3);
  setprop("/engines/engine[3]/cyl-temp", cylt3);
  setprop("/engines/engine[3]/carb-temp-degc", cart3);
  setprop("/engines/engine[3]/itt-norm", egt3 / 100);
  setprop("/engines/engine[3]/torque", trq3 / 42);
  setprop("/engines/engine[3]/cht-degf", convertTempF(getprop("/engines/engine[3]/cht-degc")));
  # ------------------------------------------------------------------------------------------
  setprop("/systems/electrical/amp", (rpm0 + rpm1 + rpm2 + rpm3) / 100 );
}

var convertTemp = func(degF) {
  var degC = 0;
  if ( degF != nil ) {
    degC = (degF - 32) * 5/9;
  }
  return degC;
}

var convertTempF = func(degC) {
  var degF = 0;
  if ( degC != nil ) {
    degF = (degC * (9/5)) + 32;
  }
  return degF;
}

###  Main loop ###
var update_convert = func {
  convert();
  settimer(update_convert, 0.01);
}
setlistener("/sim/signals/fdm-initialized", update_convert);
