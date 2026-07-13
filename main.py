"""
SGS Glider - JSBSim + FlightGear Thermal Soaring Simulation
-----------------------------------------------------------
Reddy et al. (2018) "Glider soaring via reinforcement learning in the field"
Nature 562, 236-239.

Klavye kontrolleri (Soaring Control penceresine tikla):
    W / S          elevator
    A / D          aileron
    Q / E          rudder
    Shift / Ctrl   gaz artir / azalt
    1              MANUAL mod
    2              RL_SOARING mod
    3              TAKEOFF mod
    ESC            cikis

FlightGear:
fgfs --aircraft=ask21 --fdm=null \
     --native-fdm=socket,in,60,localhost,5550,udp \
     --timeofday=noon --season=summer \
     --disable-real-weather-fetch --fog-disable \
     --disable-ai-traffic --disable-sound \
     --geometry=1024x768 \
     --lat=39.9483187 --lon=32.6899477 \
     --altitude=4500 --heading=90
"""

import jsbsim
import time
import math
import numpy as np
import random
import struct
import socket
import os
import json
from collections import deque

STATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "soaring_state.json")

try:
    import pygame
    PYGAME_OK = True
except ImportError:
    PYGAME_OK = False
    print("[WARN] pygame not found — run: pip install pygame")


# ------------------------------------------------------------------ #
#  CONFIG                                                              #
# ------------------------------------------------------------------ #
ENABLE_FLIGHTGEAR = True
FG_HOST           = "127.0.0.1"
FG_PORT           = 5550

START_LAT    =  39.9483187
START_LON    =  32.6899477
START_ALT_FT = 4500.0

MODE_MANUAL  = "MANUAL"
MODE_RL      = "RL_SOARING"
MODE_TAKEOFF = "TAKEOFF"


# ------------------------------------------------------------------ #
#  PS5 / KLAVYE CONTROLLER                                             #
# ------------------------------------------------------------------ #
class PS5Controller:
    DEADZONE      = 0.08
    KEY_STEP      = 0.05
    THROTTLE_STEP = 0.02

    def __init__(self):
        self.available = False
        self.joy       = None
        self._aileron  = 0.0
        self._elevator = 0.0
        self._rudder   = 0.0
        self._throttle = 0.3

        if not PYGAME_OK:
            print("[INPUT] pygame yok")
            return

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joy = pygame.joystick.Joystick(0)
            self.joy.init()
            print(f"[PS5] Baglandi: {self.joy.get_name()}")
            self.available = True
        else:
            print("[INPUT] Klavye modu | W/S=elev A/D=ail Q/E=rud Shift/Ctrl=gaz")
            print("[INPUT] 1=MANUAL  2=RL  3=TAKEOFF  ESC=cikis")

    def _ax(self, i):
        try:
            v = self.joy.get_axis(i)
            return v if abs(v) > self.DEADZONE else 0.0
        except Exception:
            return 0.0

    def _btn(self, i):
        try:
            return bool(self.joy.get_button(i))
        except Exception:
            return False

    def get_inputs(self):
        if not PYGAME_OK:
            return dict(throttle=0.0, rudder=0.0, elevator=0.0, aileron=0.0,
                        btn_manual=False, btn_rl=False, btn_reset=False,
                        btn_takeoff=False, quit=False)
        pygame.event.pump()
        keys = pygame.key.get_pressed()

        if self.available:
            thr = (-self._ax(1) + 1.0) / 2.0
            rud =  self._ax(0)
            elv = -self._ax(3)
            ail =  self._ax(2)
            bm  = self._btn(0)
            br  = self._btn(1)
            brs = self._btn(2)
            bt  = self._btn(3)
        else:
            if   keys[pygame.K_d]: self._aileron  = min( 1.0, self._aileron  + self.KEY_STEP)
            elif keys[pygame.K_a]: self._aileron  = max(-1.0, self._aileron  - self.KEY_STEP)
            else:                  self._aileron  *= 0.85

            if   keys[pygame.K_w]: self._elevator = min( 1.0, self._elevator + self.KEY_STEP)
            elif keys[pygame.K_s]: self._elevator = max(-1.0, self._elevator - self.KEY_STEP)
            else:                  self._elevator *= 0.85

            if   keys[pygame.K_e]: self._rudder   = min( 1.0, self._rudder   + self.KEY_STEP)
            elif keys[pygame.K_q]: self._rudder   = max(-1.0, self._rudder   - self.KEY_STEP)
            else:                  self._rudder   *= 0.85

            if   keys[pygame.K_LSHIFT]: self._throttle = min(1.0, self._throttle + self.THROTTLE_STEP)
            elif keys[pygame.K_LCTRL]:  self._throttle = max(0.0, self._throttle - self.THROTTLE_STEP)

            thr = self._throttle
            ail = self._aileron
            elv = self._elevator
            rud = self._rudder
            bm  = keys[pygame.K_1]
            br  = keys[pygame.K_2]
            brs = keys[pygame.K_3]
            bt  = keys[pygame.K_4]

        return dict(
            throttle   = float(np.clip(thr,  0.0,  1.0)),
            rudder     = float(np.clip(rud, -1.0,  1.0)),
            elevator   = float(np.clip(elv, -1.0,  1.0)),
            aileron    = float(np.clip(ail, -1.0,  1.0)),
            btn_manual  = bool(bm),
            btn_rl      = bool(br),
            btn_reset   = bool(brs),
            btn_takeoff = bool(bt),
            quit        = bool(keys[pygame.K_ESCAPE]),
        )


# ------------------------------------------------------------------ #
#  FLIGHTGEAR UDP BRIDGE                                               #
# ------------------------------------------------------------------ #
class FlightGearBridge:
    FG_NET_FDM_VERSION = 24

    def __init__(self, host=FG_HOST, port=FG_PORT):
        self.sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr    = (host, port)
        self.enabled = True
        print(f"[FG] UDP bridge {host}:{port}")

    def send(self, fdm):
        try:
            vals = (
                self.FG_NET_FDM_VERSION, 0,
                math.radians(fdm['position/long-gc-deg']),
                math.radians(fdm['position/lat-geod-deg']),
                fdm['position/h-sl-meters'],
                float(fdm['position/h-agl-ft'] * 0.3048),
                float(fdm['attitude/phi-rad']),
                float(fdm['attitude/theta-rad']),
                float(fdm['attitude/psi-rad']),
                0.0, 0.0,
                float(fdm['velocities/p-rad_sec']),
                float(fdm['velocities/q-rad_sec']),
                float(fdm['velocities/r-rad_sec']),
                float(fdm['velocities/vt-fps'] * 0.3048),
                float(-fdm['velocities/v-down-fps'] * 0.3048),
                float(fdm['velocities/v-north-fps'] * 0.3048),
                float(fdm['velocities/v-east-fps']  * 0.3048),
                float(fdm['velocities/v-down-fps']  * 0.3048),
            )
            hdr = struct.pack("!IIdddffffffffffffff", *vals)
            self.sock.sendto((hdr + b'\x00'*(408-len(hdr)))[:408], self.addr)
        except Exception as e:
            if self.enabled:
                print(f"[FG] {e}")
                self.enabled = False


# ------------------------------------------------------------------ #
#  THERMAL MODEL                                                       #
# ------------------------------------------------------------------ #
class Thermal:
    def __init__(self, lat, lon, strength_ms=3.0, radius_m=80.0, height_m=3000.0):
        self.lat      = lat
        self.lon      = lon
        self.strength = strength_ms
        self.radius   = radius_m
        self.height   = height_m

    def updraft_at(self, ac_lat, ac_lon, ac_alt_m):
        if ac_alt_m > self.height:
            return 0.0
        dx   = (ac_lon - self.lon) * 111320.0 * math.cos(math.radians(ac_lat))
        dy   = (ac_lat - self.lat) * 110540.0
        dist = math.sqrt(dx*dx + dy*dy)
        w    = self.strength * math.exp(-dist*dist / (2.0*self.radius*self.radius))
        return w * max(0.0, 1.0 - ac_alt_m / self.height)

THERMALS = [
    Thermal(START_LAT,          START_LON,          4.5, 120.0, 3200.0),
    Thermal(START_LAT + 0.0010, START_LON + 0.0015, 3.0,  95.0, 3000.0),
    Thermal(START_LAT - 0.0008, START_LON - 0.0012, 3.5, 100.0, 3100.0),
    Thermal(START_LAT + 0.0055, START_LON + 0.0020, 2.5,  90.0, 3000.0),
    Thermal(START_LAT + 0.0030, START_LON - 0.0005, 2.0,  90.0, 3000.0),
]

def updraft_sum(lat, lon, alt_m):
    return sum(t.updraft_at(lat, lon, alt_m) for t in THERMALS)


# ------------------------------------------------------------------ #
#  WZ ESTIMATOR  —  Reddy et al. Eq. 4-6                              #
# ------------------------------------------------------------------ #
class WzEstimator:
    """
    Dikey ruzgar hizini (wz) ve ivmesini (az) tahmin eder.

    Reddy et al. Eq. 4:
        az = d(wz)/dt = d(uz - vz)/dt

    uz  : GPS/baro hizinin dikey bileseni = -climb_rate (asagi pozitif)
          Biz: uz = -climb_ms  (yukari pozitif = biz, asagi pozitif = paper)
    vz  : hava hizinin dikey bileseni; pitch salinimlari vz'yi bozar.

    Eq. 5-6 (basitlestirilmis pitch bias duzeltme):
        Delta_vz = -V * (Delta_alpha - Delta_phi)
        Delta_alpha ~ (alpha0 - alpha_i) * (1/cos(mu) - 1)
        Delta_phi   = pitch_rad - phi_trim

    Bu formul: bank donuslerindeki pitch-alpha degisimleri vz'yi bozar,
    bunu cikararak wz ve az tahminini temizleriz.
    """

    ALPHA0_MINUS_ALPHAI = math.radians(14.0)  # paper Table 1: 14 deg
    SIGMA_A  = 0.5   # az smoothing zaman sabiti (s) — paper: sigma_a = 8ta/3 ~ 4s
                     # ama simulasyonda pitch noise az, daha kisa aliyoruz
    SIGMA_A2 = 0.2   # ikinci kademe

    def __init__(self, dt):
        self.dt       = dt
        self.alpha_1  = dt / (self.SIGMA_A  + dt)
        self.alpha_2  = dt / (self.SIGMA_A2 + dt)

        self.prev_wz  = None   # onceki wz tahmini
        self.az_s1    = 0.0    # birinci kademe smooth az
        self.az_s2    = 0.0    # ikinci kademe smooth az (policy icin)
        self.phi_trim = 0.0    # duz ucustaki pitch (ilk N adimdan ogrenilir)
        self._trim_buf = deque(maxlen=500)

    def update(self, climb_ms, airspeed_ms, pitch_rad, roll_rad):
        """
        Guncelleme: her sim adiminda cagrilir.
        Donus: (wz_est, az_smooth)
        """
        # Pitch trim ogrenme (kucuk roll acisinda ortalama pitch)
        if abs(math.degrees(roll_rad)) < 5.0:
            self._trim_buf.append(pitch_rad)
            if len(self._trim_buf) > 0:
                self.phi_trim = float(np.mean(self._trim_buf))

        # uz: GPS dikey hizi (yukari pozitif)
        uz = climb_ms

        # Delta_phi: trim'den sapma
        delta_phi = pitch_rad - self.phi_trim

        # Delta_alpha: bank acisina bagli hucum acisi degisimi (Eq. 6)
        mu = roll_rad
        cos_mu = math.cos(mu)
        if abs(cos_mu) > 0.1:
            delta_alpha = self.ALPHA0_MINUS_ALPHAI * (1.0/cos_mu - 1.0)
        else:
            delta_alpha = 0.0

        # Delta_vz (Eq. 5): pitch salinimindan kaynaklanan sahte vz
        delta_vz = -airspeed_ms * (delta_alpha - delta_phi)

        # wz tahmini: Eq. wz = uz - vz_trim_corrected
        # vz_trim ~ 0 duz ucusta; sadece degisimi cikarmak yeterli
        wz_est = uz - delta_vz

        # az = dwz/dt
        if self.prev_wz is not None:
            az_raw = (wz_est - self.prev_wz) / self.dt
        else:
            az_raw = 0.0
        self.prev_wz = wz_est

        # Cift kademeli smoothing
        self.az_s1 = self.alpha_1 * az_raw  + (1 - self.alpha_1) * self.az_s1
        self.az_s2 = self.alpha_2 * self.az_s1 + (1 - self.alpha_2) * self.az_s2

        return wz_est, self.az_s2


# ------------------------------------------------------------------ #
#  OMEGA ESTIMATOR  —  Reddy et al. Eq. 7-8                           #
# ------------------------------------------------------------------ #
class OmegaEstimator:
    """
    Roll-wise tork (omega) tahmini.

    Eq. 7: tau * dmu/dt = mu_d - mu + omega(t) + omega_aero(t)
    Eq. 8: omega = dmu/dt - (mu_d - mu)/tau - omega_aero

    omega_aero dort etki:
      1. Dihedral  : stabilize, T_dih ~ 20s  → coeff = -1/20
      2. Overbanking: destabilize, T_ob ~ -20s → coeff = +1/20
      3. Trim bias  : sabit offset (0 aliyoruz)
      4. Aileron loss at low speed (ihmal)

    Net omega_aero ~ (-1/20 + 1/20)*mu = 0  ama asimetri var:
    dihedral dogrusal, overbanking kucuk yaricapta guclendigindan
    net katsayi hafif negatif: -0.02 * mu (paper'daki kalibrasyon)

    Smoothing: sigma_w = T_A, sigma_w' = T_A/4
    """
    TAU          = 0.45   # paper Table 1: feedback control timescale (s)
    COEFF_AERO   = -0.02  # net omega_aero / mu (rad/s per rad)
    SIGMA_W1     = 1.5    # = T_A
    SIGMA_W2     = 0.375  # = T_A/4

    def __init__(self, dt):
        self.dt      = dt
        self.alpha_1 = dt / (self.SIGMA_W1 + dt)
        self.alpha_2 = dt / (self.SIGMA_W2 + dt)
        self.om_s1   = 0.0
        self.om_s2   = 0.0

    def update(self, p_rad_sec, roll_rad, target_roll_rad):
        """Donus: omega_smooth (policy icin)"""
        actual_rate   = p_rad_sec
        expected_rate = (target_roll_rad - roll_rad) / self.TAU
        omega_aero    = self.COEFF_AERO * roll_rad
        omega_raw     = actual_rate - expected_rate - omega_aero

        self.om_s1 = self.alpha_1 * omega_raw + (1 - self.alpha_1) * self.om_s1
        self.om_s2 = self.alpha_2 * self.om_s1 + (1 - self.alpha_2) * self.om_s2
        return self.om_s2


# ------------------------------------------------------------------ #
#  RL SOARING CONTROLLER  (Reddy et al. 2018)                         #
# ------------------------------------------------------------------ #
class SoaringRLController:
    """
    Dis dongu: policy — her T_A saniyede bank acisini degistirir.
    Ic dongu: PID roll tracker + hiz tutan pitch PD + elevator rate limiter.
    """

    # Policy
    DEAD_BAND     = 5.0
    T_A           = 1.5
    MU_LEVELS     = [-30, -15, 0, 15, 30]
    K_FACTOR      = 0.8
    STD_WINDOW    = 600    # ~5s rolling buffer
    MIN_STD_AZ    = 0.05
    MIN_STD_OMEGA = 0.02

    # Roll PID
    KP_ROLL      = 3.0
    KI_ROLL      = 0.8
    KD_ROLL      = 0.3
    ROLL_I_LIMIT = 0.5

    # Pitch / hiz tutma
    V_TARGET      = 26.0
    KP_V          = 0.05
    KD_V          = 0.8
    ELEV_SIGN     = +1.0
    ELEV_RATE_LIM = 0.25   # max deflection/s — termal spike'i onler

    # Stall korumasi
    V_STALL_GUARD = 20.0
    V_RECOVER     = 23.0

    RUDDER_COORD  = 0.0    # koordinasyon rudder (0 = kapalı)

    POLICY = {
        (+1, +1, -30): +15, (+1, +1, -15): -15, (+1, +1,   0): -15,
        (+1, +1, +15): -15, (+1, +1, +30): -15,
        ( 0, +1, -30): -15, ( 0, +1, -15): -15, ( 0, +1,   0): -15,
        ( 0, +1, +15):   0, ( 0, +1, +30): -15,
        (-1, +1, -30): -15, (-1, +1, -15): -15, (-1, +1,   0): +15,
        (-1, +1, +15): -15, (-1, +1, +30): -15,
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
        (-1, -1, -30): +15, (-1, -1, -15): +15, (-1, -1,   0): +15,
        (-1, -1, +15): +15, (-1, -1, +30):   0,
    }

    def __init__(self, dt):
        self.dt = dt

        self.az_buffer    = deque(maxlen=self.STD_WINDOW)
        self.omega_buffer = deque(maxlen=self.STD_WINDOW)

        self.mu_deg        = 0.0
        self.target_roll   = 0.0
        self.last_action_t = 0.0

        # Roll PID
        self.roll_integral = 0.0
        self.prev_roll_rad = 0.0

        # Pitch
        self.prev_pitch    = 0.0
        self.prev_elevator = 0.0

        # Stall
        self.recovering = False

        self._last_decision = None

    def _snap_mu(self, mu):
        return min(self.MU_LEVELS, key=lambda x: abs(x - mu))

    def _discretize(self, val, thr):
        if val >  thr: return +1
        if val < -thr: return -1
        return 0

    def _std(self, buf, mn):
        if len(buf) < 10: return mn
        return max(mn, float(np.std(np.array(buf))))

    def step(self, roll_rad, pitch_rad, airspeed_ms,
             az_smooth, omega_smooth, p_rad_sec, t_now):
        """
        Parametreler:
          az_smooth    : WzEstimator'dan gelen temizlenmis az
          omega_smooth : OmegaEstimator'dan gelen temizlenmis omega
          p_rad_sec    : JSBSim roll rate (PID D terimi icin)
          t_now        : sim zamani (s)
        """
        self.az_buffer.append(az_smooth)
        self.omega_buffer.append(omega_smooth)
        self.mu_deg = math.degrees(roll_rad)

        # ── Policy: T_A'da bir karar ─────────────────────────────────
        if t_now - self.last_action_t >= self.T_A:
            std_az    = self._std(self.az_buffer,    self.MIN_STD_AZ)
            std_omega = self._std(self.omega_buffer, self.MIN_STD_OMEGA)
            K_az      = self.K_FACTOR * std_az
            K_omega   = self.K_FACTOR * std_omega

            az_d       = self._discretize(az_smooth,    K_az)
            omega_d    = self._discretize(omega_smooth, K_omega)
            mu_snapped = self._snap_mu(self.mu_deg)

            action = self.POLICY.get((az_d, omega_d, mu_snapped))
            if action is None:
                action = random.choice([-15, +15])
            if action != 0:
                if abs(self.mu_deg - self._snap_mu(self.mu_deg+action)) < self.DEAD_BAND:
                    action = 0

            self.target_roll   = float(np.clip(self.mu_deg + action, -30, 30))
            self.last_action_t = t_now

            print(f"  [RL] az={az_d:+d}(K={K_az:.3f})  w={omega_d:+d}(K={K_omega:.3f})"
                  f"  mu={mu_snapped:+d}  D={action:+d}  hedef={self.target_roll:.1f}")

            self._last_decision = dict(
                az_d=az_d, om_d=omega_d, mu=mu_snapped,
                action=action, target=self.target_roll,
                K_az=round(K_az,3), K_om=round(K_omega,3),
            )

        # ── Roll PID ─────────────────────────────────────────────────
        roll_err = math.radians(self.target_roll) - roll_rad
        self.roll_integral = max(-self.ROLL_I_LIMIT,
                                  min(self.ROLL_I_LIMIT,
                                      self.roll_integral + roll_err*self.dt))
        aileron = (self.KP_ROLL * roll_err
                   + self.KI_ROLL * self.roll_integral
                   - self.KD_ROLL * p_rad_sec)
        aileron = max(-1.0, min(1.0, aileron))

        # ── Pitch: hiz tutma PD + rate limiter ───────────────────────
        speed_err  = airspeed_ms - self.V_TARGET
        pitch_rate = (pitch_rad - self.prev_pitch) / self.dt
        self.prev_pitch = pitch_rad

        elev_raw = self.ELEV_SIGN * (self.KP_V*speed_err - self.KD_V*pitch_rate)
        elev_raw = max(-0.5, min(0.5, elev_raw))

        max_step     = self.ELEV_RATE_LIM * self.dt
        elevator     = self.prev_elevator + max(-max_step,
                                                 min(max_step,
                                                     elev_raw - self.prev_elevator))
        self.prev_elevator = elevator

        rudder = self.RUDDER_COORD * math.radians(self.mu_deg)

        # ── Stall korumasi ────────────────────────────────────────────
        if self.recovering:
            if airspeed_ms > self.V_RECOVER:
                self.recovering = False
                print(f"  [STALL] Kurtarildi Spd={airspeed_ms:.1f}")
        elif airspeed_ms < self.V_STALL_GUARD:
            self.recovering = True
            print(f"  [STALL] Uyari! Spd={airspeed_ms:.1f}")

        if self.recovering:
            self.roll_integral = 0.0
            aileron            = max(-0.6, min(0.6, 0.5*(0.0-roll_rad)))
            elevator           = self.ELEV_SIGN * (-0.25)
            rudder             = 0.0
            self.target_roll   = 0.0
            self.prev_elevator = elevator

        self.prev_roll_rad = roll_rad
        return aileron, elevator, rudder, 0.0


# ------------------------------------------------------------------ #
#  JSBSim SETUP                                                        #
# ------------------------------------------------------------------ #
def build_fdm():
    fdm = jsbsim.FGFDMExec(jsbsim.get_default_root_dir())
    fdm.set_debug_level(0)
    fdm.load_model("SGS")
    fdm['ic/lat-geod-deg'] = START_LAT
    fdm['ic/long-gc-deg']  = START_LON
    fdm['ic/h-sl-ft']      = START_ALT_FT
    fdm['ic/vt-kts']       = 35.0
    fdm['ic/psi-true-deg'] = 90.0
    fdm['ic/phi-deg']      = 0.0
    fdm['ic/theta-deg']    = 1.5
    fdm.run_ic()
    print(f"[JSBSim] lat={START_LAT} lon={START_LON} alt={START_ALT_FT}ft")
    return fdm


# ------------------------------------------------------------------ #
#  MAIN LOOP                                                           #
# ------------------------------------------------------------------ #
def run():
    fdm  = build_fdm()
    fg   = FlightGearBridge() if ENABLE_FLIGHTGEAR else None
    ps5  = PS5Controller()
    dt   = fdm.get_delta_t()

    wz_est    = WzEstimator(dt)
    om_est    = OmegaEstimator(dt)
    rl        = SoaringRLController(dt)

    print_int = max(1, int(2.0 / dt))
    mode      = MODE_MANUAL

    if PYGAME_OK:
        ctrl_win  = pygame.display.set_mode((460, 130))
        pygame.display.set_caption("Soaring Control — bu pencereye tikla!")
        ctrl_font = pygame.font.SysFont("monospace", 13)
    else:
        ctrl_win = ctrl_font = None

    print()
    print("=" * 72)
    print("  SGS Glider — RL Thermal Soaring  (Reddy et al. 2018)")
    print(f"  Konum: {START_LAT:.5f}N {START_LON:.5f}E  Alt: {START_ALT_FT}ft")
    print(f"  Termal: {len(THERMALS)}  |  State: {STATE_PATH}")
    print("=" * 72)
    print(f"  {'Step':>6}  {'Alt(m)':>7}  {'Spd':>6}  {'Climb':>7}"
          f"  {'Roll':>6}  {'Updr':>6}  {'az_s':>6}  {'om_s':>6}  {'MOD':>10}")
    print("-" * 72)

    step    = 0
    t_start = time.time()

    while True:
        t_sim = step * dt

        # ── input ──────────────────────────────────────────────────
        inp = ps5.get_inputs()
        if inp.get("quit"):
            print("\n  [QUIT]"); break

        if inp["btn_manual"]:
            if mode != MODE_MANUAL: print("\n  [MOD] MANUAL")
            mode = MODE_MANUAL
        elif inp["btn_rl"]:
            if mode != MODE_RL:
                print("\n  [MOD] RL_SOARING")
                rl.roll_integral  = 0.0
                rl.prev_elevator  = 0.0
            mode = MODE_RL
        elif inp["btn_takeoff"]:
            if mode != MODE_TAKEOFF: print("\n  [MOD] TAKEOFF")
            mode = MODE_TAKEOFF

        # ── JSBSim state ───────────────────────────────────────────
        lat       = fdm['position/lat-geod-deg']
        lon       = fdm['position/long-gc-deg']
        alt_m     = fdm['position/h-sl-meters']
        speed_ms  = fdm['velocities/vt-fps']    * 0.3048
        roll_rad  = fdm['attitude/roll-rad']
        pitch_rad = fdm['attitude/pitch-rad']
        psi_rad   = fdm['attitude/psi-rad']
        p_rad_sec = fdm['velocities/p-rad_sec']
        climb_ms  = fdm['velocities/h-dot-fps'] * 0.3048  # pozitif = yukari

        # ── Termal → atmosfer ───────────────────────────────────────
        total_updraft = updraft_sum(lat, lon, alt_m)
        fdm['atmosphere/wind-d-fps'] = -total_updraft * 3.28084

        # ── Sensor cue tahmini (paper Eq. 4-8) ──────────────────────
        wz_est_val, az_smooth = wz_est.update(climb_ms, speed_ms,
                                               pitch_rad, roll_rad)
        omega_smooth = om_est.update(p_rad_sec, roll_rad,
                                     math.radians(rl.target_roll))

        # ── Kontrol ─────────────────────────────────────────────────
        if mode == MODE_MANUAL:
            aileron  = inp["aileron"]
            elevator = inp["elevator"]
            rudder   = inp["rudder"]
            throttle = inp["throttle"]

        elif mode == MODE_TAKEOFF:
            aileron  = inp["aileron"]
            elevator = 0.3
            rudder   = inp["rudder"]
            throttle = 1.0
            if alt_m > START_ALT_FT * 0.3048 + 50:
                mode = MODE_RL
                print("\n  [MOD] Takeoff → RL_SOARING")

        else:  # RL
            aileron, elevator, rudder, throttle = rl.step(
                roll_rad, pitch_rad, speed_ms,
                az_smooth, omega_smooth, p_rad_sec, t_sim)
            if abs(inp["elevator"]) > 0.1:
                elevator = inp["elevator"]

        # ── JSBSim'e yaz ───────────────────────────────────────────
        fdm['fcs/aileron-cmd-norm']  = aileron
        fdm['fcs/elevator-cmd-norm'] = elevator
        fdm['fcs/rudder-cmd-norm']   = rudder
        fdm['fcs/throttle-cmd-norm'] = throttle
        fdm.run()
        step += 1

        # ── Real-time sync ─────────────────────────────────────────
        sleep_t = step*dt - (time.time()-t_start)
        if sleep_t > 0:
            time.sleep(sleep_t)

        if fg: fg.send(fdm)

        # ── Print ──────────────────────────────────────────────────
        if step % print_int == 0:
            print(f"  {step:>6}  {alt_m:>7.1f}  {speed_ms:>6.1f}"
                  f"  {climb_ms:>7.2f}  {math.degrees(roll_rad):>6.1f}"
                  f"  {total_updraft:>6.2f}  {az_smooth:>6.3f}"
                  f"  {omega_smooth:>6.3f}  {mode:>10}")

        # ── Pygame pencere ─────────────────────────────────────────
        if PYGAME_OK and ctrl_win:
            ctrl_win.fill((15,15,25))
            mc = {"MANUAL":(100,180,255),"RL_SOARING":(0,255,120),
                  "TAKEOFF":(255,200,0)}.get(mode,(220,220,220))
            lines = [
                (f"MOD:{mode}  Alt:{alt_m:.0f}m  Spd:{speed_ms:.1f}m/s", mc),
                (f"Climb:{climb_ms:+.2f}  Roll:{math.degrees(roll_rad):+.1f}"
                 f"  Updr:{total_updraft:.2f}", (220,220,220)),
                (f"az:{az_smooth:+.3f}  w:{omega_smooth:+.3f}"
                 f"  Stall:{'YES' if rl.recovering else 'no'}", (180,180,180)),
                ("W/S=elev A/D=ail Q/E=rud Sh/Ct=gaz 1=MAN 2=RL 3=TO ESC=quit",
                 (70,70,110)),
            ]
            for i,(txt,col) in enumerate(lines):
                ctrl_win.blit(ctrl_font.render(txt,True,col),(6,6+i*27))
            pygame.display.flip()

        # ── Viz state ──────────────────────────────────────────────
        if step % 10 == 0:
            try:
                with open(STATE_PATH,"w") as f:
                    json.dump({
                        "lat": lat, "lon": lon, "alt_m": alt_m,
                        "speed_ms": speed_ms, "climb_ms": climb_ms,
                        "roll_deg": math.degrees(roll_rad),
                        "pitch_deg": math.degrees(pitch_rad),
                        "heading_deg": math.degrees(psi_rad),
                        "updraft_ms": total_updraft,
                        "az_smooth": round(az_smooth,4),
                        "omega_smooth": round(omega_smooth,4),
                        "mode": mode,
                        "stall_guard": rl.recovering,
                        "last_rl_decision": rl._last_decision,
                        "thermals": [
                            {"lat":t.lat,"lon":t.lon,
                             "strength_ms":t.strength,
                             "radius_m":t.radius,
                             "height_m":t.height}
                            for t in THERMALS],
                    }, f)
            except Exception:
                pass

        # ── Bitis ──────────────────────────────────────────────────
        if alt_m < 1.0 and step > 500:
            print("\n  [LANDED]"); break
        if alt_m > 4800.0:
            print("\n  [MAX ALT]"); break
        if step > 600000:
            break


if __name__ == "__main__":
    run()