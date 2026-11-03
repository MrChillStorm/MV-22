"""The crew alerts (the EICAS's): one table for the flight model's alert channel (genv22.py: alert/<id>, 1 while the
alert stands), the cockpit's alert manager (Nasal/alerts.nas, from Systems/mv22b-alerts.xml, which genv22.py --fg
writes) and the flight manual.

The levels after MIL-STD-411F (aircrew station alerting): a warning red, with the master warning and its tone, and a
voice (the standard's verbal warnings only with red ones); a caution yellow (drawn amber), with the master caution and
its chime; an advisory green, neither. An autopilot mode the aircraft let go of (not the pilot) is listed amber with its
own tone till the pilot acknowledges it or engages the mode again (14 CFR 25.1329(j): a distinct aural alert; AC
25.1322-1: repeated, and cancelled by the crew); those come from the flight model's flags (hh/dropped and the
autopilot's), which the manager reads and clears.

Each condition holds for on_s before its alert stands, and is gone for off_s before it clears: one state rising from 0
to 1 at 1/on_s per second while the condition holds and falling at 1/off_s, the alert set at 1 and cleared at 0, so a
value on its threshold does not chime twice. On the ground with the rotors' thrust under half the weight (alert/ground)
the alerts are listed without the masters or a sound: starting, taxiing, shutting down.

when: the manual's words; src: where the threshold comes from, 'none' where nothing does yet. critical: a time-critical warning (AC 25.1322-1 6.b):
its voice its own aural, said again while it stands whether acknowledged or not, its text on the flight format, with its
recovery (cue) where the model shows one.
"""
import math
from xmlfun import v, p, tag, S, D, M, Q, SQ, MAX, MIN, ABS, GT, LT, IFTHEN, T1, T2
import vrs as VRS

WARNING, CAUTION, ADVISORY, AUTOPILOT = 'warning', 'caution', 'advisory', 'autopilot'
AF = '/instrumentation/mv22b/afcs/'      # the autopilot's modes (ENG OUT: genv22.eng_out)
NAC_TOL = 2.0           # deg past the conversion corridor's upper boundary before OVERSPEED: the nacelles' lag behind it
# SINK RATE: the vortex ring state ahead. Johnson's model (vrs.py) has its onset at the stability boundary z_N(x), where
# a rotor's total flow stops rising with its descent rate: -0.45 vh in vertical descent (the V-22's flight tests
# [TP-2005-213477]), lower with the edgewise speed, the region closed at x = XM. The warning at K_VRS of it, each rotor
# in its own vh: K_VRS puts it at the V-22's own warning in vertical descent, 800 ft/min [GL] (v22data.VRS_SINK), at
# the design weight at sea level, where the model's vh is 75.2 ft/s (thrustmax.py) and its onset 2030 ft/min (the V-22's
# warning was set at about half the descent rate vulnerable to the state after the Marana accident: secondary sources).
# Not projected ahead on the rates: in the model's entries (vrswarn.py) it leads the onset by 6.5-8.4 s with the lever
# drawn back at 0.02 per s (12-15 s at 0.01, 3-4 s at 0.05), and any projection either warns in the report's own flights
# (the autoland's landing, the hover hold's stop) or adds under 2 s
K_VRS = 0.394
# FLAPPING: the gimbals' tilt at the stop (a warning; v22data.FLAP_STOP, the XV-15's [TX]) and past the wing clearance
# Bell designed for (a caution, 10.5 deg [RC])
FLAP_CLEAR_DEG = 10.5
# HOVER PWR: slowing (under HOVER_KT) with the nacelles past HOVER_NAC, the power to hover at this height over what the
# running engines have now (each its own limit or its share of the drive's): the model's own power to hover out of
# ground effect at 104 % rotor speed against the weight and the density (data/hover_power_v22.json, its trims:
# thrustmax.py table), with the ground effect on the induced power, HOVER_FM of it (estimate; the flight display's)
HOVER_KT, HOVER_NAC, HOVER_FM = 60.0, 60.0, 0.75
HOVER_ERR = 0.0
# ROTOR RPM: the nacelles up, the rotor speed under the one at which the rotors' greatest thrust no longer
# holds the hover at this weight and density: their greatest thrust (the blades' stall) T_MAX100 at 100 % and sea level,
# with the rotor speed squared and the density, against the hover's thrust over the weight K_DL (the wing's download);
# both the model's (thrustmax.py, the reference loading at 1000 ft: 92 % there)
T_MAX100, K_DL = 65301.0, 1.129
# NR: the rotor speed off its reference by more than NR_BAND (%) for 1 s; ROTOR RPM only below it so, the rotor
# drooping. Unsourced, but checked (alertcheck.py): in the report's flights the governor's excursions reach 2.5 % (the
# hover hold's stop), 3.4 % briefly (TF's response) and 5.5 % for under 1 s (the approach's conversion), none of them
# an alert
NR_BAND = 3.0
# WHEELS (MIL-STD-411F 5.1.3.7, a VTOL's: set up for a landing, the gear not down, after a delay): the nacelles past
# WHEELS_NAC, under WHEELS_FT above the ground, descending; held WHEELS_S
WHEELS_NAC, WHEELS_FT, WHEELS_S = 60.0, 200.0, 3.0
# PULL UP: in conversion or airplane mode (the nacelles under WHEELS_NAC: no landing there, the proprotors' clearance),
# descending to the ground in less than a response and the pull-out: the height above the ground under the descent
# rate times PULLUP_S (detecting and acting on a directive warning, the decision given: AM-24/23 table 4.2-2) and
# the height lost arresting it at PULLUP_G (the terrain following's fly-up, genv22 ap/tf-flyup-g)
PULLUP_S, PULLUP_G = 5.0, 0.5
# read by vrswarn.py and the displays: kept out of genv22's inlining
KEEP = {'alert/vrs/sink-fpm', 'alert/sink-fps', 'alert/flap-deg', 'alert/hover-power-ratio', 'alert/limit-pct',
        'alert/hover-oge-hp', 'alert/hover-ige-hp', 'alert/hover-hp'}


def on(x):
    return GT(p(x), v(0.5))


def NOT(a):
    return D(v(1), a)


def table(U):
    """the alerts in their order of priority within each level"""
    (lv, ln), (uv, un) = U.CPS
    fuel = S(*[p('propulsion/tank[%d]/contents-lbs' % i) for i in range(len(U.TANKS))])
    nr = p('drive/nr-pct')
    out = [M(p('alert/eng%d-ran' % e), NOT(on('eng[%d]/running' % e))) for e in (0, 1)]
    # ENG OUT flaring, settling or down: the rotor speed its own (the nacelles coming up move the governor's reference from
    # airplane mode's, the windmilling rotor lagging it): neither NR's nor ROTOR RPM's lever down is the crew's to act on
    # there (in FlightGear NR came in the flare, ROTOR RPM's voice a second before touchdown)
    eo_settle = M(on(AF + 'eng-out'), GT(p('ap/eo-phase'), v(2.5)))
    a = [
        # its cue: the recovery the model's own entries show quickest (vrswarn.py recovery: from the warning, 46-64 ft
        # lost; from the onset 139-175 ft, against 221-232 with the lever alone; the nose alone does not recover)
        dict(id='sink-rate', level=WARNING, text='SINK RATE', voice='sink rate', critical=True, cue='NAC FWD  POWER',
             cond=M(p('alert/vrs/cond'), NOT(on('gear/wow'))), on_s=0.5, off_s=2.0,
             when='either rotor descending past %.2f of the vortex ring state\'s onset for its edgewise speed (Johnson\'s '
                  'model, in its own hover induced velocity): 800 ft/min in vertical descent at the reference loading at '
                  'sea level' % K_VRS,
             src='Johnson\'s VRS model [TP-2005-213477]; the V-22\'s warning in vertical descent [GL]'),
        dict(id='pull-up', level=WARNING, text='PULL UP', voice='pull up', critical=True,
             cond=M(LT(p('ctl/nacelle-deg'), v(WHEELS_NAC)), NOT(on('gear/wow')), GT(p('alert/sink-fps'), v(0)),
                    NOT(on(AF + 'eng-out')),
                    LT(p('position/h-agl-ft'), S(M(p('alert/sink-fps'), v(PULLUP_S)),
                                                 Q(tag('pow', p('alert/sink-fps'), v(2)), v(2 * 32.174 * PULLUP_G))))),
             on_s=0.5, off_s=1.0,
             when='nacelles under %.0f deg, descending to the ground in less than %.0f s and a %.1f g pull-out; not '
                  'with ENG OUT on (no power to climb away: the descent to the ground is the mode\'s, its flare closing '
                  'on the ground quicker than %.0f s)' % (WHEELS_NAC, PULLUP_S, PULLUP_G, PULLUP_S),
             src='response time: AM-24/23 table 4.2-2; the pull-out: the terrain following\'s fly-up'),
        dict(id='rotor-rpm', level=WARNING, text='ROTOR RPM', voice='rotor r p m', critical=True,
             cond=M(NOT(on('gear/wow')), GT(p('ctl/nacelle-deg'), v(HOVER_NAC)), p('alert/rotor-flying'), NOT(eo_settle),
                    LT(p('drive/nr-pct'), D(p('gov/nr-ref-pct'), v(NR_BAND))),
                    LT(p('drive/nr-pct'), M(v(100 * math.sqrt(K_DL * 0.0023769 / T_MAX100)),
                                            SQ(Q(p('inertia/weight-lbs'), p('atmosphere/rho-slugs_ft3')))))),
             on_s=0.5, off_s=1.0,
             when='nacelles past %.0f deg: the rotor speed drooping, too low for the rotors\' greatest thrust to hold '
                  'the hover at this weight and density; not from ENG OUT\'s flare on (its lever spends the rotor speed)' % HOVER_NAC,
             src='the model\'s greatest thrust and download in the hover (thrustmax.py)'),
        # an engine that ran and stopped (not one not started yet); both stopped on the ground: shut down, cleared.
        # Both out: DUAL ENG OUT, collecting the two (AC 25.1322-1's collector: the alerts it collects inhibited; the
        # warnings' voice is the highest new one's, so with both listed only engine one's was said)
        dict(id='dual-eng-out', level=WARNING, text='DUAL ENG OUT', voice='dual engine out',
             cond=M(out[0], out[1]), on_s=0, off_s=0,
             when='both engines that ran stopped (ENG 1 OUT and ENG 2 OUT collected into it)',
             src='the model\'s running flags; the collector [AC 25.1322-1]'),
        dict(id='eng1-out', level=WARNING, text='ENG 1 OUT', voice='engine one out',
             cond=M(out[0], NOT(out[1])), on_s=0, off_s=0,
             when='an engine that ran stops, the other running (both: DUAL ENG OUT; both shut down on the ground, the '
                  'alerts clear)', src='the model\'s running flag'),
        dict(id='eng2-out', level=WARNING, text='ENG 2 OUT', voice='engine two out',
             cond=M(out[1], NOT(out[0])), on_s=0, off_s=0,
             when='an engine that ran stops, the other running (both: DUAL ENG OUT; both shut down on the ground, the '
                  'alerts clear)', src='the model\'s running flag'),
        # the speed tape's red: past the conversion corridor's upper boundary for the nacelles' angle; in airplane mode
        # 280 KCAS or Mach 0.48, 220 KCAS above 94 % rotor speed
        dict(id='overspeed', level=WARNING, text='OVERSPEED', voice='overspeed',
             cond=IFTHEN(GT(p('ctl/nacelle-deg'), v(0.5)),
                         GT(p('ctl/nacelle-deg'), S(T1('velocities/ve-kts', uv, un), v(NAC_TOL))),
                         MAX(GT(p('velocities/vc-kts'), IFTHEN(GT(nr, v(94)), v(U.VMO_APLN_100), v(U.VMO_APLN))),
                             GT(p('velocities/mach'), v(U.MMO_APLN)))),
             on_s=1.0, off_s=2.0,
             when='past the conversion corridor\'s upper speed for the nacelles\' angle; in airplane mode 280 KCAS or '
                  'Mach 0.48, 220 KCAS above 94 % rotor speed',
             src='the conversion corridor [PD fig. 5], VMO and MMO [GL]'),
        dict(id='flapping', level=WARNING, text='FLAPPING', voice='flapping',
             cond=GT(p('alert/flap-deg'), v(U.FLAP_STOP * 180 / math.pi - 0.05)), on_s=0.2, off_s=1.0,
             when='a gimbal at its flapping stop (%.0f deg)' % (U.FLAP_STOP * 180 / math.pi),
             src='v22data.FLAP_STOP: the XV-15\'s [TX]'),
        dict(id='wheels', level=WARNING, text='WHEELS', voice='landing gear',
             cond=M(NOT(on('gear/wow')), LT(p('gear/gear-pos-norm'), v(0.99)), GT(p('ctl/nacelle-deg'), v(WHEELS_NAC)),
                    LT(p('position/h-agl-ft'), v(WHEELS_FT)), LT(p('velocities/h-dot-fps'), v(-1))),
             on_s=WHEELS_S, off_s=1.0,
             when='nacelles past %.0f deg under %.0f ft, descending, the gear not down' % (WHEELS_NAC, WHEELS_FT),
             src='MIL-STD-411F 5.1.3.7; the angle, height and delay ours'),
        dict(id='fuel-low', level=CAUTION, text='FUEL LOW', cond=LT(fuel, v(1000)), on_s=1.0, off_s=5.0,
             when='under 1000 lb of fuel', src='none'),
        dict(id='nr', level=CAUTION, text='NR',
             cond=M(GT(nr, v(5)), GT(ABS(D(nr, p('gov/nr-ref-pct'))), v(NR_BAND)), NOT(eo_settle), NOT(M(out[0], out[1], on('gear/wow')))),
             on_s=1.0, off_s=2.0,
             when='the rotor speed more than %g %% off its reference; not from ENG OUT\'s flare on (the nacelles coming up '
                  'move its reference), nor on the ground with both engines out' % NR_BAND,
             src='none; the report\'s flights\' excursions checked against it (alertcheck.py)'),
        dict(id='hover-pwr', level=CAUTION, text='HOVER PWR',
             cond=M(NOT(on('gear/wow')), GT(p('ctl/nacelle-deg'), v(HOVER_NAC)), LT(p('velocities/ve-kts'), v(HOVER_KT)),
                    p('alert/rotor-flying'), GT(p('alert/hover-power-ratio'), v(1 + HOVER_ERR))),
             on_s=2.0, off_s=3.0,
             when='slower than %.0f KEAS, nacelles past %.0f deg: the power to hover at this height more than the '
                  'engines running have' % (HOVER_KT, HOVER_NAC),
             src='the model\'s own power to hover (its hover ceiling\'s trims)'),
        dict(id='flap-clear', level=CAUTION, text='FLAPPING HIGH',
             cond=GT(p('alert/flap-deg'), v(FLAP_CLEAR_DEG)), on_s=0.5, off_s=1.0,
             when='a gimbal past %.1f deg of flapping, the wing clearance\'s design value' % FLAP_CLEAR_DEG,
             src='[RC]'),
        dict(id='airspeed-low', level=CAUTION, text='AIRSPEED LOW',
             cond=M(NOT(on('gear/wow')), LT(p('ctl/nacelle-deg'), D(T1('velocities/ve-kts', lv, ln), v(NAC_TOL)))),
             on_s=1.0, off_s=2.0,
             when='slower than the conversion corridor\'s least speed for the nacelles\' angle',
             src='the conversion corridor [PD fig. 5]'),
        dict(id='conv-prot', level=CAUTION, text='CONV PROT',
             cond=GT(ABS(D(p('/controls/flight/nacelle-cmd-deg'), p('ctl/nacelle-cmd-limited-deg'))), v(1)), on_s=1.0, off_s=1.0,
             when="the conversion protection holding the nacelle command off the pilot's",
             src='the conversion corridor [PD fig. 5]'),
        dict(id='gear-down', level=CAUTION, text='GEAR DOWN',
             cond=M(GT(p('position/h-agl-ft'), v(50)), GT(p('gear/gear-pos-norm'), v(0.99)), GT(p('velocities/vc-kts'), v(U.VLE))),
             on_s=1.0, off_s=2.0, when='the gear down above %.0f KCAS (an assumed limit) and 50 ft' % U.VLE,
             src='assumed: no public V-22 figure'),
        dict(id='rotor-brake', level=ADVISORY, text='ROTOR BRAKE', cond=GT(p('/controls/rotor/brake'), v(0.05)), on_s=0, off_s=0,
             when='the rotor brake on',
             src='-'),
        dict(id='battery-off', level=ADVISORY, text='BATTERY OFF', cond=LT(p('/controls/electric/battery-switch'), v(0.5)),
             on_s=0, off_s=0, when='the battery switch off',
             src='-'),
    ]
    return a


# the autopilot's let-go modes: the flag the flight model sets (its value), the text, the level, the mode's own flag,
# when (the manual's words)
EVENTS = [('hh/dropped', 1, 'HVR CPLD OFF', AUTOPILOT, 'hh/on', 'the hover coupler let go'),
          ('hh/dropped', 2, 'HVR CPLD UNAVAIL', ADVISORY, 'hh/on', 'the hover coupler refused (listed for 10 s)'),
          ('ap/dropped', 1, 'CPLD OFF', AUTOPILOT, 'ap/on', 'the coupling let go, by the stick or on the ground'),
          ('ap/nac-dropped', 1, 'AUTO NAC OFF', AUTOPILOT, 'ap/auto-nac-on', 'AUTO NAC let go')]


def vrs_terms(o):
    """SINK RATE's condition (alert/vrs/cond), and the descent rate it stands at now along the more critical rotor's
    shaft (alert/vrs/sink-fpm, for the flight display's vertical speed scale)"""
    xs = [VRS.XM * i / 24 for i in range(25)]
    zns = [VRS.vrs_points(x)[1] for x in xs]
    conds, sinks = [], []
    for s in 'rl':
        b = 'aero/rotor/%s/vrs/' % s
        zn = T1(b + 'x', xs, zns)
        conds.append(M(LT(p(b + 'x'), v(VRS.XM)), LT(p(b + 'z'), M(v(K_VRS), zn))))
        sinks.append(IFTHEN(LT(p(b + 'x'), v(VRS.XM)), M(v(-60 * K_VRS), zn, p(b + 'vh-fps')), v(1e5)))
    o.fn('alert/vrs/cond', MAX(*conds), desc='either rotor past K_VRS of the vortex ring state\'s onset')
    o.fn('alert/vrs/sink-fpm', MIN(*sinks), desc='the axial descent rate SINK RATE stands at, ft/min (1e5: none)')


def NEG_(a):
    return M(v(-1), a)


def hover_terms(o, U, ge):
    """the power to hover here (alert/hover-hp), out of ground effect (alert/hover-oge-hp) and with the wheels 10 ft up
    (alert/hover-ige-hp, the flight display's marks), and over what the running engines have (alert/hover-power-ratio)"""
    import os, json
    d = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'hover_power_v22.json')))
    sig = d['sigma'][::-1]
    data = [row[::-1] for row in zip(*d['hp'])]                       # [weight][sigma ascending]
    oge = o.fn('alert/hover-oge-hp', T2('inertia/weight-lbs', 'atmosphere/sigma', d['gw'], sig, data),
               desc='the power to hover out of ground effect at 104 % rotor speed, hp')
    zh = (U.WL_PIV + U.DZ_HUB - U.MAIN_GEAR[2]) / 12

    def ground(z):
        kge = tag('pow', S(v(1), M(v(ge[0] * U.R ** 2), Q(v(1), tag('pow', z, v(2))))), v(ge[1]))
        return S(v(1 - HOVER_FM), M(v(HOVER_FM), kge))
    o.fn('alert/hover-ige-hp', M(oge, ground(v(zh + 10))), desc='... with the wheels 10 ft over the ground')
    wheels = MAX(v(0), D(p('position/h-agl-ft'), Q(D(p('inertia/cg-z-in'), v(U.MAIN_GEAR[2])), v(12))))
    need = o.fn('alert/hover-hp', M(oge, ground(S(v(zh), wheels))), desc='... at this height')
    nrun = MAX(v(1), S(p('eng[0]/running'), p('eng[1]/running')))
    have = M(nrun, MIN(p('eng/power-available-hp'), p('eng/xmsn-limit-hp')))
    o.fn('alert/hover-power-ratio', Q(need, MAX(v(1), have)), desc='the power to hover here over the engines\' (HOVER PWR)')
    # each engine's limit, its own or its share of the drive's, in the torque gauge's percent (of its rating): the
    # gauges' line (the FADEC holds the engine there, so the gauge's 100 % is passed only where the engine limits)
    o.fn('alert/limit-pct', M(v(100 / U.P_MRP), MIN(p('eng/power-available-hp'), p('eng/xmsn-limit-hp'))),
         desc='each engine\'s power limit, % of its rating')


def channel(o, U, execrate, ge):
    """the alert channel, on the flight model's outputs (genv22.Model.outputs); ge: its ground effect's (k, exponent)"""
    o.declare('/controls/electric/battery-switch', 1)          # FlightGear's (Systems/controls.xml); on in the tests
    o.channel('alerts', execrate=execrate)
    dt = p('simulation/channel-dt')
    init = GT(p('ctl/init-fast'), v(0.5))
    for e in (0, 1):
        r = o.declare('alert/eng%d-ran' % e, 0)
        both_off = M(NOT(on('eng[0]/running')), NOT(on('eng[1]/running')), on('gear/wow'))
        o.fn('alert/eng%d-ran-next' % e, IFTHEN(on('eng[%d]/running' % e), v(1), IFTHEN(GT(both_off, v(0.5)), v(0), r)),
             out='alert/eng%d-ran' % e)
    o.fn('alert/ground', M(on('gear/wow'), LT(S(p('aero/rotor/r/thrust-lbs'), p('aero/rotor/l/thrust-lbs')),
                                           M(v(0.5), p('inertia/weight-lbs')))),
         desc='on the ground, the rotors carrying under half the weight: alerts listed, no masters or sounds')
    vrs_terms(o)
    o.fn('alert/sink-fps', MAX(v(0), NEG_(p('velocities/h-dot-fps'))), desc='the descent rate, ft/s')
    # the rotors turned up at least once to flight speed (ROTOR RPM and HOVER PWR none before: a start on the ground)
    rf = o.declare('alert/rotor-flying', 0)
    o.fn('alert/rotor-flying-next', IFTHEN(GT(p('drive/nr-pct'), v(80)), v(1), IFTHEN(on('gear/wow'), M(rf, GT(p('drive/nr-pct'), v(50))), rf)),
         out='alert/rotor-flying')
    o.fn('alert/flap-deg', M(v(180 / math.pi), MAX(p('rotor/r/tilt-rad'), p('rotor/l/tilt-rad'))), desc='the greater gimbal tilt')
    hover_terms(o, U, ge)
    for a in table(U):
        n = 'alert/' + a['id']
        if not a['on_s'] and not a['off_s']:
            o.fn(n, GT(a['cond'], v(0.5)), desc=a['text'])
            continue
        c = o.fn(n + '/c', GT(a['cond'], v(0.5)), desc=a['text'] + '\'s condition')
        s = o.declare(n + '/s', 0)
        st = o.declare(n, 0)
        up = S(s, M(dt, v(1 / a['on_s']))) if a['on_s'] else v(1)
        down = D(s, M(dt, v(1 / a['off_s']))) if a['off_s'] else v(0)
        o.fn(n + '/s-next', IFTHEN(init, c, MAX(v(0), MIN(v(1), IFTHEN(c, up, down)))), out=n + '/s')
        o.fn(n + '-next', IFTHEN(GT(p(n + '/s'), v(0.999)), v(1), IFTHEN(LT(p(n + '/s'), v(0.001)), v(0), st)), out=n,
             desc=a['text'])


def data(U):
    """Systems/mv22b-alerts.xml: the table for Nasal/alerts.nas"""
    rows = []
    for a in table(U):
        rows.append('  <alert><id>%s</id><level>%s</level><text>%s</text>%s%s</alert>'
                    % (a['id'], a['level'], a['text'], '<voice>%s</voice>' % a['voice'] if a.get('voice') else '',
                       ('<critical>1</critical>' if a.get('critical') else '') + ('<cue>%s</cue>' % a['cue'] if a.get('cue') else '')))
    for flag, val, text, level, mode, _ in EVENTS:
        rows.append('  <event><flag>%s</flag><value>%d</value><text>%s</text><level>%s</level><mode>%s</mode></event>'
                    % (flag, val, text, level, mode))
    return ('<!-- generated by fdm/genv22.py from fdm/alerts.py: the crew alerts, in their order of priority within each\n'
            '     level; each alert/<id> is the flight model\'s, each event a flag it sets for Nasal/alerts.nas -->\n'
            '<PropertyList>\n%s\n</PropertyList>\n' % '\n'.join(rows))
