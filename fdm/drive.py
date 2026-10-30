"""The tiltrotor's engines and drive: two turboshafts, one in each nacelle, the proprotor gearboxes and the interconnect
shaft that gears the two rotors together (one rotor speed), as a JSBSim system (FCS components, no Nasal).

Sources: the XV-15's two Lycoming LTC1K-4K, 1550 shp take-off rating each [CR B-6]; the V-22's two Rolls-Royce T406 /
AE 1107C, 6150 shp each [RC, PD] (v22data.py, xv15data.py). The engines' fuel controls are not modelled in detail:
the thrust control lever sets each engine's power demand, from flight idle to the power available, which falls with
altitude as delta / sqrt(theta) (a generic turboshaft's at its temperature limit); the engine follows it with a lag
(ENGINE_TAU) and a topping governor takes power off when the rotor overspeeds by more than 2 % of its reference. The
FADECs share the drive system's torque limit (P_XMSN_100 at 100 % rotor speed) between the running engines. The
rotor speed itself is governed through the collective (genv22.governor), as the XV-15's and the V-22's are.
"""
import math
from xmlfun import *

U = None                  # set by genv22.configure()
IDLE = 0.17               # flight idle, fraction of the rating: the rotors' power at flat pitch at 104 % with the losses
                          # (about 1850 hp for the V-22), so that they reach their governed speed with the lever down
SFC = 0.42                # lb/shp/h at the rating, rising 50 % towards no load, at least 10 % of the rating's flow (generic)


STARTUP_S = 30.0          # s, an engine from light-off to flight idle (estimate)
SPOOLDOWN_S = 8.0         # s, its run-down after cut-off
BRAKE_FTLB = 60000.0      # the rotor brake's torque at full application, at the rotors (estimate)


def build(execrate):
    """The engines and the drive train. eng/init-running (1: running) picks the state a run starts in on the ground;
    above 50 ft it always starts running. A started engine spools up over STARTUP_S to its lever's power; the rotors
    turn with the drive train from rest."""
    f = FCS('%s drive' % U.NAME)
    W0 = U.OMEGA_100
    J = 2 * U.J_ROTOR + U.J_DRIVE
    for n in range(2):
        e = '/controls/engines/engine[%d]' % n
        f.declare(e + '/cutoff', 0)
        f.declare('eng[%d]/power-hp' % n, 0.5 * U.P_MRP)
        f.declare('eng[%d]/spool' % n, 1.0)
    f.declare('eng/init-running', 1)
    f.declare('/controls/rotor/brake', 0.0)
    f.declare('drive/hold-rpm', 0)
    f.declare('drive/omega-dot', 0.0)
    init = GT(p('ctl/init-fast'), v(0.5))
    W = p('rotor/omega-rad_sec')
    f.channel('start state')
    run0 = f.fn('eng/running-at-start', MAX(p('eng/init-running'), GT(p('position/h-agl-ft'), v(50))))
    for n in range(2):
        e = '/controls/engines/engine[%d]/cutoff' % n
        f.fn('eng[%d]/cutoff-init' % n, IFTHEN(init, D(v(1), run0), p(e)), out=e)
    f.fn('drive/brake-init', IFTHEN(init, D(v(1), run0), p('/controls/rotor/brake')), out='/controls/rotor/brake')
    f.channel('engines', execrate=execrate)
    th = Q(p('atmosphere/T-R'), v(518.67))
    dl = Q(p('atmosphere/P-psf'), v(2116.22))
    avail = f.fn('eng/power-available-hp', M(v(U.P_MRP), MIN(v(1.0), Q(dl, SQ(th)))), desc='each engine, at its rating')
    nr = f.fn('drive/nr-pct', M(v(100 / W0), W))
    over = MAX(v(0), D(nr, S(p('gov/nr-ref-pct'), v(2.0))))
    top = f.fn('eng/topping', MAX(v(0), MIN(v(1), D(v(1), M(v(0.4), over)))), desc='topping governor: power cut for overspeed')
    tcl = p('ctl/tcl-norm')
    cdt = p('simulation/channel-dt')
    tanks = getattr(U, 'TANKS', None)
    for n in range(2):
        run = LT(p('/controls/engines/engine[%d]/cutoff' % n), v(0.5))
        # (with tanks, the V-22's: and fuel in them; the XV-15 test builds run on)
        fuel = S(*[p('propulsion/tank[%d]/contents-lbs' % i) for i in range(len(tanks))]) if tanks else None
        f.fn('eng[%d]/running' % n, M(run, GT(fuel, v(0.5))) if tanks else run)
    nrun = MAX(v(1), S(p('eng[0]/running'), p('eng[1]/running')))
    # (a torque limit: the power at the rotor speed, from the same 30 % floor as the engines' torque below)
    xmsn = f.fn('eng/xmsn-limit-hp', Q(M(v(U.P_XMSN_100), Q(MAX(W, v(0.3 * W0)), v(W0))), nrun),
                desc='each running engine\'s share of the drive\'s torque limit')
    for n in range(2):
        g = 'eng[%d]' % n
        run = p(g + '/running')
        sp = p(g + '/spool')
        f.fn(g + '/spool-next', IFTHEN(init, run0, MAX(v(0), MIN(v(1), S(sp, M(cdt, IFTHEN(GT(run, v(0.5)), v(1 / STARTUP_S), v(-1 / SPOOLDOWN_S))))))),
             out=g + '/spool', desc='the gas generator from rest (0) to flight idle and above (1)')
        dem = f.fn(g + '/demand-hp', M(run, top, SQR(sp), MIN(avail, xmsn), S(v(IDLE), M(v(1 - IDLE), tcl))))
        P = p(g + '/power-hp')
        f.fn(g + '/power-next', IFTHEN(init, dem, S(P, M(Q(cdt, v(U.ENGINE_TAU)), D(dem, P)))), out=g + '/power-hp')
        f.fn(g + '/torque-pct', M(v(100 / U.P_MRP), P))
        f.fn(g + '/n1-pct', M(sp, S(v(60), M(v(45), SQ(MAX(v(0), Q(P, v(U.P_MRP))))))), desc='gas generator speed, for the gauges')
        f.fn(g + '/fuel-flow-pph', M(run, sp, v(SFC), MAX(v(0.1 * U.P_MRP), M(P, D(v(1.5), M(v(0.5 / U.P_MRP), P))))))
    if tanks:
        # the fuel the engines burn, drawn as the transfer sequence drains the tanks [GS]: the aft sponson first, then
        # the forward sponsons, the feed tanks last (each pair in proportion to its contents); none in FlightGear's fuel
        # freeze (JSBSim's own belongs to its engines)
        f.declare('/sim/freeze/fuel', 0)
        burn = f.fn('fuel/burn-lbs', M(S(p('eng[0]/fuel-flow-pph'), p('eng[1]/fuel-flow-pph')), Q(cdt, v(3600)),
                                       D(v(1), p('/sim/freeze/fuel'))), desc='this step\'s')
        c = lambda i: p('propulsion/tank[%d]/contents-lbs' % i)
        groups = [[4], [2, 3], [0, 1]]          # TANKS' order: feed L, R, forward sponsons L, R, aft sponson
        left = burn
        for g in groups:
            have = S(*[c(i) for i in g]) if len(g) > 1 else c(g[0])
            take = f.fn('fuel/take-%s' % ''.join(map(str, g)), MIN(have, left))
            for i in g:
                share = Q(c(i), MAX(have, v(1e-6))) if len(g) > 1 else v(1)
                f.fn('fuel/tank%d-next' % i, MAX(v(0), D(c(i), M(take, share))), out='propulsion/tank[%d]/contents-lbs' % i)
            left = D(left, take)
    f.channel('drive train')
    Qr, Ql = p('aero/rotor/r/torque-ftlb'), p('aero/rotor/l/torque-ftlb')
    Wn = Q(W, v(W0))
    # accessories (2 % of the rating at 100 % speed) and gearbox losses (2 % of the rotors' torque) (estimates); the rotor
    # brake, smooth through standstill
    loss = f.fn('drive/loss-ftlb', S(M(v(0.02 * 2 * U.P_MRP * 550 / W0), Wn), M(v(0.02), S(ABS(Qr), ABS(Ql))),
                                     M(p('/controls/rotor/brake'), v(BRAKE_FTLB), Q(W, S(ABS(W), v(0.3))))))
    qeng = f.fn('drive/engine-torque-ftlb', Q(M(v(550), S(p('eng[0]/power-hp'), p('eng[1]/power-hp'))), MAX(W, v(0.3 * W0))))
    held = GT(S(init, p('drive/hold-rpm')), v(0.5))
    wd = f.fn('drive/omega-dot-calc', IFTHEN(held, v(0), Q(D(D(D(qeng, Qr), Ql), loss), v(J))), out='drive/omega-dot')
    f.fn('drive/omega-next', IFTHEN(init, M(run0, v(W0 / 100), p('gov/nr-ref-pct')), MAX(v(0), S(W, M(wd, p('simulation/dt'))))),
         out='rotor/omega-rad_sec')
    f.fn('drive/power-required-hp', Q(M(S(Qr, Ql, loss), W), v(550)))
    # each rotor's shaft torque: its aerodynamic torque and its own acceleration; its reaction acts on the nacelle
    for side, Qa in (('r', Qr), ('l', Ql)):
        f.fn('drive/%s/shaft-torque-ftlb' % side, S(Qa, M(v(U.J_ROTOR), wd)))
    return f
