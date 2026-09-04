# -*- coding: utf-8 -*-
"""
Ranger 2400 - RL Thermal Soaring
=================================
Reddy et al. (2018) Nature 562, 236-239

MIMARI  (gercek Pixhawk kurulumuyla ayni)
------------------------------------------
    RL POLICY  (1.5 s'de bir)
        az, omega -> ayrik bank seviyesi  [-30 -15 0 +15 +30]
              |
              v
    ATTITUDE KONTROLCU  (Pixhawk GUIDED benzeri, her adim)
        roll: aci -> hedef roll rate -> aileron   [kademeli]
        pitch: SABIT TRIM  (ucak CG sayesinde kendi durur)
        rudder: koordinasyon
              |
              v
    JSBSim kontrol yuzeyleri

Pitch kontrolcusu YOK. Model dogrulama testinde (test_ranger.py):
  - elevator=0 sabit, 60 s -> V=15.7 m/s, theta=-1.4 deg  KARARLI
  - 20 deg bank, 17 s      -> roll 19.8 sabit, V 16.6 sabit
CG (%22 MAC) aerodinamik merkezin (%25 MAC) onunde oldugu icin
ucak longitudinal olarak kendi kendine kararli.

KULLANIM
--------
FlightGear (once bunu ac, ayri terminal):
  fgfs --aircraft=ask21 --fdm=null \
       --native-fdm=socket,in,60,localhost,5550,udp \
       --timeofday=noon --disable-real-weather-fetch --fog-disable \
       --disable-ai-traffic --disable-sound --geometry=1024x768 \
       --lat=39.9483187 --lon=32.6899477 --altitude=4500 --heading=90

Klavye (Soaring Control penceresine tikla):
  W/S=elevator  A/D=aileron  Q/E=rudder
  1=MANUAL   2=RL_SOARING   ESC=cikis
"""

import os
import io
import math
import json
import time
import random
import socket
import struct
from collections import deque

import numpy as np
import jsbsim

try:
    import pygame
    PYGAME_OK = True
except ImportError:
    PYGAME_OK = False
    print("[WARN] pygame yok -> pip install pygame")

PROJ_ROOT  = os.path.dirname(os.path.abspath(__file__))
STATE_PATH = os.path.join(PROJ_ROOT, "soaring_state.json")


# ================================================================== #
#  CONFIG                                                             #
# ================================================================== #
AIRCRAFT     = "ranger2400"
ENABLE_FG    = True
FG_HOST      = "127.0.0.1"
FG_PORT      = 5550

START_LAT    = 39.9483187
START_LON    = 32.6899477
START_ALT_FT = 4500.0
START_KTS    = 32.0            # ~16.5 m/s (dogal trim hizi)

MODE_MANUAL = "MANUAL"
MODE_RL     = "RL_SOARING"

OMEGA_MODE = 'oracle'  # 'oracle' | 'eq8'


# ================================================================== #
#  INPUT                                                              #
# ================================================================== #
class InputDevice:
    STEP = 0.05

    def __init__(self):
        self.joy = None
        self.has_joy = False
        self._a = self._e = self._r = 0.0
        if not PYGAME_OK:
            return
        pygame.init(); pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joy = pygame.joystick.Joystick(0); self.joy.init()
            self.has_joy = True
            print(f"[INPUT] Joystick: {self.joy.get_name()}")
        else:
            print("[INPUT] Klavye | W/S=elev A/D=ail Q/E=rud | 1=MANUAL 2=RL ESC=cikis")

    def poll(self):
        if not PYGAME_OK:
            return dict(aileron=0., elevator=0., rudder=0.,
                        m_manual=False, m_rl=False, quit=False)
        pygame.event.pump()
        k = pygame.key.get_pressed()

        if self.has_joy:
            def ax(i):
                v = self.joy.get_axis(i)
                return v if abs(v) > 0.08 else 0.0
            return dict(aileron=ax(2), elevator=-ax(3), rudder=ax(0),
                        m_manual=bool(self.joy.get_button(0)),
                        m_rl=bool(self.joy.get_button(1)),
                        quit=k[pygame.K_ESCAPE])

        if   k[pygame.K_d]: self._a = min( 1., self._a + self.STEP)
        elif k[pygame.K_a]: self._a = max(-1., self._a - self.STEP)
        else:               self._a *= 0.85
        if   k[pygame.K_w]: self._e = min( 1., self._e + self.STEP)
        elif k[pygame.K_s]: self._e = max(-1., self._e - self.STEP)
        else:               self._e *= 0.85
        if   k[pygame.K_e]: self._r = min( 1., self._r + self.STEP)
        elif k[pygame.K_q]: self._r = max(-1., self._r - self.STEP)
        else:               self._r *= 0.85

        return dict(aileron=self._a, elevator=self._e, rudder=self._r,
                    m_manual=k[pygame.K_1], m_rl=k[pygame.K_2],
                    quit=k[pygame.K_ESCAPE])


# ================================================================== #
#  FLIGHTGEAR BRIDGE                                                  #
# ================================================================== #
class FlightGearBridge:
    VERSION = 24

    def __init__(self, host=FG_HOST, port=FG_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (host, port)
        self.ok = True
        print(f"[FG] UDP -> {host}:{port}")

    def send(self, fdm):
        if not self.ok:
            return
        try:
            raw = (self.VERSION, 0,
                   math.radians(fdm['position/long-gc-deg']),
                   math.radians(fdm['position/lat-geod-deg']),
                   fdm['position/h-sl-meters'],
                   fdm['position/h-agl-ft'] * 0.3048,
                   fdm['attitude/phi-rad'], fdm['attitude/theta-rad'],
                   fdm['attitude/psi-rad'], 0.0, 0.0,
                   fdm['velocities/p-rad_sec'], fdm['velocities/q-rad_sec'],
                   fdm['velocities/r-rad_sec'],
                   fdm['velocities/vt-fps']      * 0.3048,
                   fdm['velocities/h-dot-fps']   * 0.3048,
                   fdm['velocities/v-north-fps'] * 0.3048,
                   fdm['velocities/v-east-fps']  * 0.3048,
                   fdm['velocities/v-down-fps']  * 0.3048)
            vals = [0.0 if (isinstance(v, float) and (v != v or abs(v) > 1e30))
                    else v for v in raw]
            h = struct.pack("!IIdddffffffffffffff", *vals)
            self.sock.sendto((h + b'\x00'*(408-len(h)))[:408], self.addr)
        except Exception as ex:
            print(f"[FG] kapandi: {ex}")
            self.ok = False


# ================================================================== #
#  TERMAL ALANI                                                       #
# ================================================================== #
class Thermal:
    PEAK, FADE = 0.30, 0.80

    def __init__(self, lat, lon, strength, radius, height, birth, life):
        self.lat, self.lon = lat, lon
        self.strength, self.radius, self.height = strength, radius, height
        self.birth, self.life = birth, life

    def age(self, t):
        f = (t - self.birth) / max(1.0, self.life)
        if f < 0 or f > 1:      return 0.0
        if f < self.PEAK:       return f / self.PEAK
        if f < self.FADE:       return 1.0
        return (1.0 - f) / (1.0 - self.FADE)

    def alive(self, t):
        return 0.0 <= (t - self.birth) <= self.life

    def w(self, lat, lon, alt, t):
        if alt > self.height:
            return 0.0
        dx = (lon - self.lon) * 111320.0 * math.cos(math.radians(lat))
        dy = (lat - self.lat) * 110540.0
        v  = self.strength * math.exp(-(dx*dx + dy*dy) / (2*self.radius**2)) # Eq. 11
        return v * max(0.0, 1.0 - alt/self.height) * self.age(t)


class ThermalField:
    """
    Ucak etrafinda rastgele dogup solan termaller.
    Donus yaricapi (16.6 m/s, 30 deg bank) ~ 49 m, termal yaricapi 50-120 m.
    """
    def __init__(self):
        self.spawn_r  = 260.0
        self.min_r    = 40.0
        self.max_n    = 8
        self.interval = 12.0
        self.life     = (200, 480)
        self.strength = (2.5, 5.5)
        self.radius   = (90, 170)
        self.height   = 3200.0
        self.items    = []
        self.last_t   = -1e9

    def _spawn(self, lat, lon, t):
        for _ in range(30):
            a = random.uniform(0, 2*math.pi)
            d = random.uniform(self.min_r, self.spawn_r)
            la = lat + d*math.cos(a)/110540.0
            lo = lon + d*math.sin(a)/(111320.0*max(.01, math.cos(math.radians(lat))))
            if any(math.hypot((lo-i.lon)*111320.0*math.cos(math.radians(la)),
                              (la-i.lat)*110540.0) < 150.0 for i in self.items):
                continue
            th = Thermal(la, lo, random.uniform(*self.strength),
                         random.uniform(*self.radius), self.height,
                         t, random.uniform(*self.life))
            self.items.append(th)
            print(f"  [TERMAL] +  guc={th.strength:.1f}m/s r={th.radius:.0f}m "
                  f"omur={th.life:.0f}s  (n={len(self.items)})")
            return

    def update(self, lat, lon, t):
        n = len(self.items)
        self.items = [i for i in self.items if i.alive(t)]
        if len(self.items) < n:
            print(f"  [TERMAL] -  soldu  (n={len(self.items)})")
        if t - self.last_t >= self.interval and len(self.items) < self.max_n:
            self._spawn(lat, lon, t)
            self.last_t = t

    def sum_at(self, lat, lon, alt, t):
        return sum(i.w(lat, lon, alt, t) for i in self.items)

    def gradient(self, lat, lon, alt, psi, t, half_span=1.2):
        """Kanat uclari arasi fark. POZITIF = SOL kanatta daha fazla lift."""
        e_r, n_r = math.cos(psi), -math.sin(psi)
        cl = max(1e-6, math.cos(math.radians(lat)))
        def off(s):
            return (lat + s*n_r*half_span/110540.0,
                    lon + s*e_r*half_span/(111320.0*cl))
        laR, loR = off(+1.0)
        laL, loL = off(-1.0)
        return self.sum_at(laL, loL, alt, t) - self.sum_at(laR, loR, alt, t)

    def to_list(self, t):
        return [dict(lat=i.lat, lon=i.lon,
                     strength_ms=i.strength*i.age(t),
                     radius_m=i.radius, height_m=i.height) for i in self.items]


# ================================================================== #
#  SENSOR TAHMINI  (Paper Eq. 4-6)                                    #
# ================================================================== #
class WzEstimator:
    """az = d(wz)/dt, pitch salinimi duzeltmesi + 2 kademe smoothing."""
    A0_AI  = math.radians(14.0)
    SIG1, SIG2 = 4.0, 1.0

    def __init__(self, dt):
        self.dt = dt
        self.a1 = dt/(self.SIG1+dt)
        self.a2 = dt/(self.SIG2+dt)
        self.prev_wz = None
        self.s1 = self.s2 = 0.0
        self.buf = deque(maxlen=800)
        self.theta_trim = 0.0

    def reset(self):
        self.prev_wz = None
        self.s1 = self.s2 = 0.0

    def update(self, climb, V, pitch, roll):
        if abs(math.degrees(roll)) < 6.0:
            self.buf.append(pitch)
            if self.buf:
                self.theta_trim = float(np.mean(self.buf))
        c = math.cos(roll)
        d_alpha = self.A0_AI*(1.0/c - 1.0) if abs(c) > 0.15 else 0.0
        d_vz    = -V * (d_alpha - (pitch - self.theta_trim))
        wz      = climb - d_vz
        az      = (wz - self.prev_wz)/self.dt if self.prev_wz is not None else 0.0
        self.prev_wz = wz
        self.s1 = self.a1*az      + (1-self.a1)*self.s1
        self.s2 = self.a2*self.s1 + (1-self.a2)*self.s2 #filtre kısmı
        return self.s2


class OmegaEstimator:
    """Yanal gradyani 2 kademe smooth et (Table 1: ta, ta/4)."""
    SIG1, SIG2 = 1.5, 0.375

    def __init__(self, dt):
        self.a1 = dt/(self.SIG1+dt)
        self.a2 = dt/(self.SIG2+dt)
        self.s1 = self.s2 = 0.0

    def reset(self):
        self.s1 = self.s2 = 0.0

    def update(self, raw):
        self.s1 = self.a1*raw     + (1-self.a1)*self.s1
        self.s2 = self.a2*self.s1 + (1-self.a2)*self.s2
        return self.s2
    
    def update_honest(self, p_rad_sec, roll_rad, target_roll_rad):
        """Eq. 8 — termal modelini gormez, sadece ucak dinamigi."""
        TAU        = 0.45
        COEFF_AERO = -0.02
        
        err = target_roll_rad - roll_rad
        
        # Gecis anlarinda model bozuluyor -> sadece oturmusken guncelle
        if abs(math.degrees(err)) > 5.0:
            return self.s2          # eski degeri koru
        
        expected  = err / TAU
        omega_aero = COEFF_AERO * roll_rad
        raw        = p_rad_sec - expected - omega_aero
        
        self.s1 = self.a1*raw     + (1-self.a1)*self.s1
        self.s2 = self.a2*self.s1 + (1-self.a2)*self.s2
        return self.s2


# ================================================================== #
#  ATTITUDE KONTROLCU  (Pixhawk GUIDED benzeri)                       #
# ================================================================== #
class AttitudeController:
    """
    Tek gorevi: verilen bank acisini tut.
    Pitch'e KARISMAZ - elevator sabit trim (ucak CG ile kendi durur).
    Gercek uctaki Pixhawk katmaninin karsiligi.
    """
    K_ANGLE   = 2.0      # aci hatasi (rad) -> hedef roll rate (rad/s)
    P_MAX     = 0.55     # max roll rate (rad/s) = 31 deg/s
    K_RATE    = 1.20     # rate hatasi -> aileron
    K_RATE_I  = 0.30
    I_LIMIT   = 0.25
    AIL_LIMIT = 0.60

    ELEV_TRIM = 0.00     # test_ranger.py: V=15.7 m/s, L/D=16.5
    TURN_COMP = 0.30     # bankta hafif burun yukari: K*(1/cos-1)
    RUDDER_K  = 0.25     # koordinasyon

    V_STALL   = 9.5      # sadece guvenlik esigi
    V_SAFE    = 12.0

    def __init__(self, dt):
        self.dt = dt
        self.reset()

    def reset(self):
        self.rate_i    = 0.0
        self.recovering = False

    def step(self, roll_cmd_deg, roll_rad, p_rad_sec, V):
        # --- guvenlik: stall ---
        if self.recovering:
            if V > self.V_SAFE:
                self.recovering = False
                print(f"  [STALL] toparlandi V={V:.1f}")
        elif V < self.V_STALL:
            self.recovering = True
            print(f"  [STALL] uyari V={V:.1f} -> kanatlar duz, burun asagi")

        if self.recovering:
            self.rate_i = 0.0
            p_des = max(-self.P_MAX, min(self.P_MAX, 2.0*(0.0 - roll_rad)))
            ail   = max(-0.5, min(0.5, self.K_RATE*(p_des - p_rad_sec)))
            return ail, -0.10, 0.0        # burun asagi = negatif elevator

        # --- ROLL: kademeli kontrol ---
        err   = math.radians(roll_cmd_deg) - roll_rad
        p_des = max(-self.P_MAX, min(self.P_MAX, self.K_ANGLE*err))
        r_err = p_des - p_rad_sec
        self.rate_i = max(-self.I_LIMIT,
                           min(self.I_LIMIT, self.rate_i + r_err*self.dt))
        ail = self.K_RATE*r_err + self.K_RATE_I*self.rate_i
        ail = max(-self.AIL_LIMIT, min(self.AIL_LIMIT, ail))

        # --- PITCH: sabit trim + donus kompanzasyonu ---
        c    = max(0.4, math.cos(roll_rad))
        elev = self.ELEV_TRIM + self.TURN_COMP*(1.0/c - 1.0)
        elev = max(-0.25, min(0.25, elev))

        # --- RUDDER: koordinasyon ---
        rud = self.RUDDER_K * roll_rad

        return ail, elev, rud


# ================================================================== #
#  RL SOARING POLICY  (Reddy et al. 2018)                             #
# ================================================================== #
class SoaringPolicy:
    """
    Tek gorevi: az ve omega'ya bakip HEDEF BANK SEVIYESI dondurmek.
    Kontrol yuzeyleriyle isi yok.
    """
    T_A       = 1.5
    LEVELS    = [-30, -15, 0, 15, 30]
    K_FACTOR  = 0.8                 # esik = 0.8 * rolling std (paper)
    STD_WIN   = 900
    MIN_AZ    = 0.003
    MIN_OM    = 0.0002

    TABLE = {
        (+1, +1, -30): +15, (+1, +1, -15): -15, (+1, +1,   0): -15,
        (+1, +1, +15): -15, (+1, +1, +30): -15,
        ( 0, +1, -30): -15, ( 0, +1, -15): -15, ( 0, +1,   0): -15,
        ( 0, +1, +15):   0, ( 0, +1, +30): -15,
        (-1, +1, -30): -15, (-1, +1, -15): -15, (-1, +1,   0): -15,
        (-1, +1, +15): +15, (-1, +1, +30): -15,
        (+1,  0, -30):   0, (+1,  0, -15): +15, (+1,  0,   0): None,
        (+1,  0, +15): -15, (+1,  0, +30):   0,
        ( 0,  0, -30):   0, ( 0,  0, -15):   0, ( 0,  0,   0): None,
        ( 0,  0, +15):   0, ( 0,  0, +30):   0,
        (-1,  0, -30):   0, (-1,  0, -15): -15, (-1,  0,   0): None,
        (-1,  0, +15): +15, (-1,  0, +30):   0,
        (+1, -1, -30): +15, (+1, -1, -15): +15, (+1, -1,   0): +15,
        (+1, -1, +15): +15, (+1, -1, +30): -15,
        ( 0, -1, -30): +15, ( 0, -1, -15):   0, ( 0, -1,   0): +15,
        ( 0, -1, +15): +15, ( 0, -1, +30):   0,
        (-1, -1, -30): +15, (-1, -1, -15): -15, (-1, -1,   0): +15,
        (-1, -1, +15): +15, (-1, -1, +30):   0,
    }

    def __init__(self):
        self.az_buf = deque(maxlen=self.STD_WIN)
        self.om_buf = deque(maxlen=self.STD_WIN)
        self.reset()

    def reset(self):
        self.idx       = 2          # -> 0 derece
        self.target    = 0.0        # nihai hedef (mu_f)
        self.ramp_from = 0.0        # rampa baslangici (mu_i)
        self.cmd       = 0.0        # anlik komut (mu_d) -> attitude kontrolcuye
        self.last_t    = -self.T_A
        self.last_dec  = None

    @staticmethod
    def _disc(v, k):
        return +1 if v > k else (-1 if v < -k else 0)

    @staticmethod
    def _std(buf, floor):
        return floor if len(buf) < 20 else max(floor, float(np.std(np.asarray(buf))))

    def update(self, az_s, om_s, t):
        self.az_buf.append(az_s)
        self.om_buf.append(om_s)

        # --- T_A'da bir karar ---
        if t - self.last_t >= self.T_A:
            k_az = self.K_FACTOR * self._std(self.az_buf, self.MIN_AZ)
            k_om = self.K_FACTOR * self._std(self.om_buf, self.MIN_OM)
            az_d = self._disc(az_s, k_az)
            om_d = self._disc(om_s, k_om)
            mu   = self.LEVELS[self.idx]

            act = self.TABLE.get((az_d, om_d, mu))
            if act is None:
                act = random.choice([-15, +15])       # kesif

            if   act > 0: self.idx = min(4, self.idx + 1)
            elif act < 0: self.idx = max(0, self.idx - 1)

            self.ramp_from = self.cmd
            self.target    = float(self.LEVELS[self.idx])
            self.last_t    = t

            print(f"  [RL] az={az_d:+d}(K={k_az:.3f})  w={om_d:+d}(K={k_om:.4f})"
                  f"  mu={mu:+3d}  D={act:+3d}  ->  {self.target:+.0f} deg")

            self.last_dec = dict(az_d=az_d, om_d=om_d, mu=mu, action=act,
                                 target=self.target, level=self.idx,
                                 K_az=round(k_az, 4), K_om=round(k_om, 5))

        # --- Paper Eq.3: T_A boyunca lineer rampa ---
        f = min(1.0, max(0.0, (t - self.last_t)/self.T_A))
        self.cmd = self.ramp_from + (self.target - self.ramp_from)*f
        return self.cmd


# ================================================================== #
#  JSBSim                                                             #
# ================================================================== #
def build_fdm():
    f = jsbsim.FGFDMExec(PROJ_ROOT)
    f.set_debug_level(0)
    if not f.load_model(AIRCRAFT):
        raise SystemExit(f"{AIRCRAFT} yuklenemedi "
                         f"(aircraft/{AIRCRAFT}/{AIRCRAFT}.xml var mi?)")
    f['ic/lat-geod-deg'] = START_LAT
    f['ic/long-gc-deg']  = START_LON
    f['ic/h-sl-ft']      = START_ALT_FT
    f['ic/vt-kts']       = START_KTS
    f['ic/psi-true-deg'] = 90.0
    f['ic/phi-deg']      = 0.0
    f['ic/theta-deg']    = 0.0
    f.run_ic()
    for _ in range(int(3.0 / f.get_delta_t())):     # trim otursun
        f['fcs/aileron-cmd-norm']  = 0.0
        f['fcs/elevator-cmd-norm'] = 0.0
        f['fcs/rudder-cmd-norm']   = 0.0
        f.run()
    return f


FIELD = ThermalField()


def seed_first_thermal(lat, lon):
    """Baslangicta ucagin 120 m onune guclu bir termal koy."""
    d   = 120.0
    la  = lat
    lo  = lon + d / (111320.0 * math.cos(math.radians(lat)))
    FIELD.items.append(Thermal(la, lo, 4.5, 150.0,
                               FIELD.height, 0.0, 600.0))
    print(f"  [TERMAL] baslangic termali: 120 m doguda, "
          f"guc=4.5 m/s, r=150 m")



# ================================================================== #
#  MAIN                                                               #
# ================================================================== #
def run():
    fdm = build_fdm()
    dt  = fdm.get_delta_t()
    seed_first_thermal(START_LAT, START_LON)

    fg   = FlightGearBridge() if ENABLE_FG else None
    inp  = InputDevice()
    wz   = WzEstimator(dt)
    om   = OmegaEstimator(dt)
    att  = AttitudeController(dt)
    pol  = SoaringPolicy()

    mode  = MODE_MANUAL
    p_int = max(1, int(2.0/dt))

    if PYGAME_OK:
        win  = pygame.display.set_mode((470, 130))
        pygame.display.set_caption("Soaring Control - bu pencereye tikla!")
        font = pygame.font.SysFont("monospace", 13)
    else:
        win = font = None

    print()
    print("=" * 76)
    print("  Ranger 2400 - RL Thermal Soaring    Reddy et al. 2018, Nature")
    print(f"  Model: {AIRCRAFT}    2.4 m / 4 kg    Bank: {SoaringPolicy.LEVELS}")
    print("  Mimari: RL policy -> bank komutu -> attitude kontrolcu (Pixhawk gibi)")
    print("=" * 76)
    print(f"  {'Step':>6} {'Alt(m)':>7} {'V':>6} {'Climb':>7} {'Roll':>6}"
          f" {'Cmd':>5} {'Updr':>6} {'az_s':>7} {'om_s':>8}  {'MOD':>10}")
    print("-" * 76)

    step, t0 = 0, time.time()
    alt0 = None

    while True:
        t_sim = step * dt
        cmd   = inp.poll()

        if cmd["quit"]:
            break
        if cmd["m_manual"] and mode != MODE_MANUAL:
            mode = MODE_MANUAL
            print("\n  [MOD] MANUAL")
        elif cmd["m_rl"] and mode != MODE_RL:
            mode = MODE_RL
            pol.reset(); att.reset()
            pol.cmd = pol.ramp_from = math.degrees(fdm['attitude/roll-rad'])
            print("\n  [MOD] RL_SOARING")

        # ---- state ----
        lat   = fdm['position/lat-geod-deg']
        lon   = fdm['position/long-gc-deg']
        alt   = fdm['position/h-sl-meters']
        V     = fdm['velocities/vt-fps']    * 0.3048
        roll  = fdm['attitude/roll-rad']
        pitch = fdm['attitude/pitch-rad']
        psi   = fdm['attitude/psi-rad']
        p     = fdm['velocities/p-rad_sec']
        climb = fdm['velocities/h-dot-fps'] * 0.3048
        if alt0 is None:
            alt0 = alt

        # ---- guvenlik ----
        if (alt != alt or V != V or abs(math.degrees(roll)) > 70.0
                or V < 4.0 or V > 45.0):
            print(f"\n  [RESET] alt={alt:.0f} V={V:.1f} "
                  f"roll={math.degrees(roll):.0f}")
            fdm = build_fdm()
            pol.reset(); att.reset(); wz.reset(); om.reset()
            pol.az_buf.clear(); pol.om_buf.clear()
            continue

        # ---- termal ----
        FIELD.update(lat, lon, t_sim)
        updraft = FIELD.sum_at(lat, lon, alt, t_sim)
        fdm['atmosphere/wind-down-fps'] = -updraft * 3.28084
        grad = FIELD.gradient(lat, lon, alt, psi, t_sim)

        # ---- sensor ----
        az_s = wz.update(climb, V, pitch, roll)
        om_s = (om.update_honest(p, roll, math.radians(pol.cmd))
                if OMEGA_MODE == 'eq8' else om.update(grad))

        # ---- kontrol ----
        if mode == MODE_MANUAL:
            ail, elev, rud = cmd["aileron"], cmd["elevator"], cmd["rudder"]
            roll_cmd = 0.0
        else:
            roll_cmd = pol.update(az_s, om_s, t_sim)     # RL -> bank komutu
            ail, elev, rud = att.step(roll_cmd, roll, p, V)   # attitude katmani
            if abs(cmd["elevator"]) > 0.15:
                elev = cmd["elevator"]

        fdm['fcs/aileron-cmd-norm']  = ail
        fdm['fcs/elevator-cmd-norm'] = elev
        fdm['fcs/rudder-cmd-norm']   = rud
        fdm['fcs/throttle-cmd-norm'] = 0.0
        fdm.run()
        step += 1

        # ---- gercek zaman ----
        sl = step*dt - (time.time() - t0)
        if sl > 0:
            time.sleep(sl)
        if fg:
            fg.send(fdm)

        # ---- terminal ----
        if step % p_int == 0:
            print(f"  {step:>6} {alt:>7.1f} {V:>6.1f} {climb:>7.2f}"
                  f" {math.degrees(roll):>6.1f} {roll_cmd:>5.0f}"
                  f" {updraft:>6.2f} {az_s:>7.3f} {om_s:>8.4f}  {mode:>10}")

        # ---- pencere ----
        if win:
            win.fill((15, 15, 25))
            mc = (0, 255, 120) if mode == MODE_RL else (100, 180, 255)
            gain = alt - alt0
            lines = [
                (f"MOD:{mode}  Alt:{alt:.0f}m  V:{V:.1f}m/s", mc),
                (f"Climb:{climb:+.2f}  Roll:{math.degrees(roll):+.1f}"
                 f"  Cmd:{roll_cmd:+.0f}  Updr:{updraft:.2f}", (220,220,220)),
                (f"az:{az_s:+.3f}  w:{om_s:+.4f}  lvl:{pol.idx}"
                 f"  kazanc:{gain:+.0f}m", (170,170,190)),
                ("W/S=elev A/D=ail Q/E=rud   1=MANUAL  2=RL   ESC=cikis",
                 (70,70,110)),
            ]
            for i, (t_, c_) in enumerate(lines):
                win.blit(font.render(t_, True, c_), (7, 8 + i*30))
            pygame.display.flip()

        # ---- viz state ----
        if step % 10 == 0:
            try:
                with io.open(STATE_PATH, "w", encoding="utf-8") as fh:
                    json.dump(dict(
                        lat=lat, lon=lon, alt_m=alt, speed_ms=V,
                        climb_ms=climb, roll_deg=math.degrees(roll),
                        pitch_deg=math.degrees(pitch),
                        heading_deg=math.degrees(psi),
                        target_roll=roll_cmd, updraft_ms=updraft,
                        az_smooth=round(az_s, 4), omega_smooth=round(om_s, 5),
                        mode=mode, stall_guard=att.recovering,
                        last_rl_decision=pol.last_dec,
                        thermals=FIELD.to_list(t_sim)), fh)
            except Exception:
                pass

        # ---- bitis ----
        if alt < 5.0 and step > 500:
            print("\n  [YERE INDI]")
            break

    print(f"\n  Toplam irtifa degisimi: {alt - alt0:+.0f} m")
    print("  [QUIT]")


if __name__ == "__main__":
    run()
