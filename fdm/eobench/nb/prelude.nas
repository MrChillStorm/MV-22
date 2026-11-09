# FlightGear's globals the VM alone hasn't, for the bench
globals.LOG_INFO = 3; globals.LOG_WARN = 4; globals.LOG_ALERT = 5;
globals.maketimer = func(t, f, g = nil) { return {start: func nil, stop: func nil, restart: func(t) nil, isRunning: 0}; };
globals.settimer = func(f, t, r = 0) nil;
globals.setlistener = func(a, b, c = 0, d = 1) 0;
globals._setlistener = func(a, b, c = 0, d = 1) 0;
globals.removelistener = func(i) nil;
globals.fgcommand = func(a, b = nil) 0;
globals.isa = func(a, b) 0;
globals.ghosttype = func(g) nil;
