#!/usr/bin/env python3
"""The crew alerts' tones (Nasal/alerts.nas): made here to the human-factors numbers of the FAA's flight deck displays
handbook (DOT/FAA/AM-24/23, ch. 4, from AC 25.1322-1, MIL-STD-1472H and CAA Paper 82017), written to
MV-22B/Sounds/alerts/ and given their entries in Sounds/v22-sound.xml (between BEGIN and END), which also loses the
original overspeed loop (above 315 kt; the OVERSPEED warning, fdm/alerts.py, has the limits).

- the master warning: five urgent pulses, 120 ms each with 25 ms ramps, 60 ms apart (under 150 ms: urgent), alternating
  600 and 750 Hz, each with five harmonics (more than four, harmonically related), 1 s with its silence; played once
  per cycle (alerts.nas: tone, then the voice, every 4 s for 10 s), its gain eased in over its first 0.25 s there
- the master caution: one chime, 880 Hz with two harmonics, 20 ms rise, decaying over a second; played once
- a let-go autopilot mode: a warble between 600 and 900 Hz at 2 Hz (500-1000 Hz, 1-3 Hz), four harmonics, a 1 s loop
  with whole cycles (seamless), played while it stands

All within 200-4500 Hz, peaks at -1 dBFS; heard in the cockpit only (FlightGear's avionics sounds: tied to the listener,
their own volume, sim/sound/avionics/volume). LEVEL: the alerts' factor against the cockpit's rotor sound, set in
FlightGear (the handbook asks 20-30 dB over the ambient at the onset; FlightGear's levels are relative).

    python3 alertsound.py
"""
import os, re
import numpy as np
from scipy.io import wavfile as wav

HERE = os.path.dirname(os.path.abspath(__file__))
SOUNDS = os.path.join(HERE, '..', 'MV-22B', 'Sounds')
SUBDIR = 'alerts'
FS = 44100
PEAK_DBFS = -1.0
LEVEL = 1.0
OUT = '/instrumentation/mv22b/alerts/'
BEGIN = '    <!-- the crew alerts\' tones: written by fdm/alertsound.py (Nasal/alerts.nas) -->'
END = '    <!-- end of fdm/alertsound.py\'s tones -->'


def complex_tone(f, t, n, rolloff=1.0):
    """f (Hz, an array for a sweep) with n harmonics at 1/k^rolloff"""
    ph = 2 * np.pi * np.cumsum(np.broadcast_to(f, t.shape)) / FS
    return sum(np.sin(k * ph) / k ** rolloff for k in range(1, n + 1))


def ramps(n, r):
    """an envelope of n samples with raised-cosine ramps of r samples"""
    e = np.ones(n)
    w = 0.5 - 0.5 * np.cos(np.pi * np.arange(r) / r)
    e[:r] = w; e[-r:] = w[::-1]
    return e


def warning():
    y = np.zeros(FS)
    pulse, gap, ramp = int(0.120 * FS), int(0.060 * FS), int(0.025 * FS)
    for i in range(5):
        t = np.arange(pulse) / FS
        y[i * (pulse + gap):i * (pulse + gap) + pulse] = complex_tone(600.0 if i % 2 == 0 else 750.0, t, 5) * ramps(pulse, ramp)
    return y


def caution():
    t = np.arange(int(1.2 * FS)) / FS
    env = np.minimum(1, t / 0.020) * np.exp(-t / 0.30)
    env[-int(0.05 * FS):] *= np.linspace(1, 0, int(0.05 * FS))
    return complex_tone(880.0, t, 3, rolloff=1.5) * env


def autopilot():
    t = np.arange(FS) / FS
    return complex_tone(750.0 + 150.0 * np.sin(2 * np.pi * 2.0 * t), t, 4)


def entry(tag, fn, mode, prop, volume):
    return ('    <%s>\n      <name>alert-%s</name>\n      <type>avionics</type>\n      <mode>%s</mode>\n'
            '      <path>Sounds/%s/%s</path>\n      <condition>\n        <property>%s</property>\n      </condition>\n'
            '      <volume>\n%s      </volume>\n    </%s>\n' % (tag, tag, mode, SUBDIR, fn, OUT + prop, volume, tag))


def install():
    out = os.path.join(SOUNDS, SUBDIR)
    os.makedirs(out, exist_ok=True)
    peak = 10 ** (PEAK_DBFS / 20)
    for name, y in (('warning', warning()), ('caution', caution()), ('autopilot', autopilot())):
        y = y / np.abs(y).max() * peak
        wav.write(os.path.join(out, name + '.wav'), FS, np.round(y * 32767).astype(np.int16))
        print('%-9s %.2f s, rms %.1f dBFS' % (name, len(y) / FS, 20 * np.log10(np.sqrt(np.mean(y ** 2)))))
    fixed = '        <factor>%g</factor>\n' % LEVEL
    body = (entry('warning', 'warning.wav', 'once', 'aural/warning',
                  '        <property>%saural/warning-gain</property>\n%s' % (OUT, fixed)) +
            entry('caution', 'caution.wav', 'once', 'aural/caution', fixed) +
            entry('autopilot', 'autopilot.wav', 'looped', 'aural/autopilot', fixed))
    p = os.path.join(SOUNDS, 'v22-sound.xml')
    t = open(p, encoding='latin-1').read()
    block = BEGIN + '\n' + body + END + '\n'
    if BEGIN in t:
        t = t[:t.index(BEGIN)] + block + t[t.index(END) + len(END) + 1:]
    else:
        at = t.rindex('  </fx>')
        t = t[:at] + '\n' + block + '\n' + t[at:]
    t = re.sub(r'\n    <overspeed>\n      <name>overspeed</name>.*?</overspeed>\n', '\n', t, flags=re.S)
    open(p, 'w', encoding='latin-1').write(t)


if __name__ == '__main__':
    install()
