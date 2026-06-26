"""
Volantex Ranger 2400 - JSBSim + FlightGear Thermal Soaring Simulation
----------------------------------------------------------------------
PS5 Controller:
    Left stick  up/down   → throttle
    Left stick  left/right→ rudder
    Right stick up/down   → elevator
    Right stick left/right→ aileron
    Cross  (X)            → MANUAL mode  
    Circle (O)            → RL mode      (autonomous soaring)
    Triangle              → TAKEOFF mode (full throttle + climb)
    Square                → reset position

FlightGear:
fgfs --aircraft=ask21 --fdm=null \
     --native-fdm=socket,in,60,localhost,5550,udp \
     --timeofday=noon --season=summer \
     --disable-real-weather-fetch --fog-disable \
     --disable-ai-traffic --disable-sound \
     --geometry=1024x768 \
     --lat=39.9483187 --lon=32.6899477 \
     --altitude=9000 --heading=90
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

try:
    import pygame
    PYGAME_OK = True
except ImportError:
    PYGAME_OK = False
    print("[WARN] pygame not found — run: pip install pygame")


# ------------------------------------------------------------------ #
#  CONFIG                                                              #
# ------------------------------------------------------------------ #
USE_CUSTOM_AIRCRAFT = False
ENABLE_FLIGHTGEAR   = True
FG_HOST             = "127.0.0.1"
FG_PORT             = 5550

# Ankara koordinatı
START_LAT =  39.9483187
START_LON =  32.6899477
START_ALT_FT = 4500.0    # 300m AGL başla

# Mod tanımları
MODE_MANUAL  = "MANUAL"
MODE_RL      = "RL_SOARING"
MODE_TAKEOFF = "TAKEOFF"


# ------------------------------------------------------------------ #
#  PS5 CONTROLLER                                                      #
# ------------------------------------------------------------------ #
class PS5Controller:
    """
    Klavye + PS5 joystick (varsa) input.

    Klavye kontrolleri:
        W / S          → elevator (yukarı / aşağı)
        A / D          → aileron  (sol / sağ)
        Q / E          → rudder   (sol / sağ)
        Sol Shift      → throttle artır
        Sol Ctrl       → throttle azalt
        1              → MANUAL mod
        2              → RL_SOARING mod
        3              → TAKEOFF mod
        ESC            → çıkış
    """
    DEADZONE     = 0.08
    KEY_STEP     = 0.05   # her frame input artışı
    THROTTLE_STEP = 0.02

    def __init__(self):
        self.available = False
        self.joy = None

        # Klavye state
        self._aileron  = 0.0
        self._elevator = 0.0
        self._rudder   = 0.0
        self._throttle = 0.3   # başlangıç gaz

        if not PYGAME_OK:
            print("[INPUT] pygame yok — tüm inputlar sıfır")
            return

        pygame.init()
        pygame.joystick.init()
        count = pygame.joystick.get_count()

        if count > 0:
            self.joy = pygame.joystick.Joystick(0)
            self.joy.init()
            print(f"[PS5] Bağlandı: {self.joy.get_name()}")
            self.available = True
        else:
            print("[INPUT] Joystick yok — klavye modu aktif")
            print("[INPUT] W/S=elevator  A/D=aileron  Q/E=rudder")
            print("[INPUT] Shift=gaz+  Ctrl=gaz-")
            print("[INPUT] 1=MANUAL  2=RL  3=TAKEOFF")

    def _joy_axis(self, idx):
        try:
            v = self.joy.get_axis(idx)
            return v if abs(v) > self.DEADZONE else 0.0
        except Exception:
            return 0.0

    def _joy_btn(self, idx):
        try:
            return bool(self.joy.get_button(idx))
        except Exception:
            return False

    def get_inputs(self):
        if not PYGAME_OK:
            return {"throttle":0.0,"rudder":0.0,"elevator":0.0,
                    "aileron":0.0,"btn_manual":False,"btn_rl":False,
                    "btn_reset":False,"btn_takeoff":False,"quit":False}

        pygame.event.pump()
        keys = pygame.key.get_pressed()

        quit_pressed = keys[pygame.K_ESCAPE]

        if self.available:
            # --- PS5 joystick ---
            throttle = (-self._joy_axis(1) + 1.0) / 2.0
            rudder   =  self._joy_axis(0)
            elevator = -self._joy_axis(3)
            aileron  =  self._joy_axis(2)
            btn_manual   = self._joy_btn(0)
            btn_rl       = self._joy_btn(1)
            btn_reset    = self._joy_btn(2)
            btn_takeoff  = self._joy_btn(3)
        else:
            # --- Klavye ---
            # Aileron: A / D (decay toward 0 when not pressed)
            if keys[pygame.K_d]:
                self._aileron = min(1.0,  self._aileron + self.KEY_STEP)
            elif keys[pygame.K_a]:
                self._aileron = max(-1.0, self._aileron - self.KEY_STEP)
            else:
                self._aileron *= 0.85   # spring back

            # Elevator: W (up) / S (down)
            if keys[pygame.K_w]:
                self._elevator = min(1.0,  self._elevator + self.KEY_STEP)
            elif keys[pygame.K_s]:
                self._elevator = max(-1.0, self._elevator - self.KEY_STEP)
            else:
                self._elevator *= 0.85

            # Rudder: Q / E
            if keys[pygame.K_e]:
                self._rudder = min(1.0,  self._rudder + self.KEY_STEP)
            elif keys[pygame.K_q]:
                self._rudder = max(-1.0, self._rudder - self.KEY_STEP)
            else:
                self._rudder *= 0.85

            # Throttle: Shift / Ctrl
            if keys[pygame.K_LSHIFT]:
                self._throttle = min(1.0, self._throttle + self.THROTTLE_STEP)
            elif keys[pygame.K_LCTRL]:
                self._throttle = max(0.0, self._throttle - self.THROTTLE_STEP)

            throttle    = self._throttle
            aileron     = self._aileron
            elevator    = self._elevator
            rudder      = self._rudder
            btn_manual  = keys[pygame.K_1]
            btn_rl      = keys[pygame.K_2]
            btn_reset   = keys[pygame.K_3]
            btn_takeoff = keys[pygame.K_4]

        return {
            "throttle"   : float(np.clip(throttle,  0.0,  1.0)),
            "rudder"     : float(np.clip(rudder,   -1.0,  1.0)),
            "elevator"   : float(np.clip(elevator, -1.0,  1.0)),
            "aileron"    : float(np.clip(aileron,  -1.0,  1.0)),
            "btn_manual"  : bool(btn_manual),
            "btn_rl"      : bool(btn_rl),
            "btn_reset"   : bool(btn_reset),
            "btn_takeoff" : bool(btn_takeoff),
            "quit"        : bool(quit_pressed),
        }


# ------------------------------------------------------------------ #
#  FLIGHTGEAR UDP BRIDGE                                               #
# ------------------------------------------------------------------ #
class FlightGearBridge:
    FG_NET_FDM_VERSION = 24

    def __init__(self, host=FG_HOST, port=FG_PORT):
        self.sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr    = (host, port)
        self.enabled = True
        print(f"[FG] UDP bridge → {host}:{port}")

    def send(self, fdm):
        try:
            lon_rad   = math.radians(fdm['position/long-gc-deg'])
            lat_rad   = math.radians(fdm['position/lat-geod-deg'])
            alt_m     = fdm['position/h-sl-meters']
            agl_m     = fdm['position/h-agl-ft'] * 0.3048
            phi_rad   = fdm['attitude/phi-rad']
            theta_rad = fdm['attitude/theta-rad']
            psi_rad   = fdm['attitude/psi-rad']
            vn  = fdm['velocities/v-north-fps'] * 0.3048
            ve  = fdm['velocities/v-east-fps']  * 0.3048
            vd  = fdm['velocities/v-down-fps']  * 0.3048
            p   = fdm['velocities/p-rad_sec']
            q   = fdm['velocities/q-rad_sec']
            r   = fdm['velocities/r-rad_sec']
            spd = float(math.sqrt(vn**2 + ve**2 + vd**2))
            vals = (
                self.FG_NET_FDM_VERSION, 0,
                lon_rad, lat_rad, alt_m, float(agl_m),
                float(phi_rad), float(theta_rad), float(psi_rad),
                0.0, 0.0,
                float(p), float(q), float(r),
                spd, float(-vd),
                float(vn), float(ve), float(vd),
            )
            header = struct.pack("!IIdddffffffffffffff", *vals)
            pkt = header + b'\x00' * (408 - len(header))
            self.sock.sendto(pkt[:408], self.addr)
        except Exception as e:
            if self.enabled:
                print(f"[FG] Send error: {e}")
                self.enabled = False


# ------------------------------------------------------------------ #
#  THERMAL MODEL                                                       #
# ------------------------------------------------------------------ #
class Thermal:
    def __init__(self, lat_deg, lon_deg,
                 strength_ms=3.0, radius_m=80.0, height_m=600.0):
        self.lat      = lat_deg
        self.lon      = lon_deg
        self.strength = strength_ms
        self.radius   = radius_m
        self.height   = height_m

    def updraft_at(self, ac_lat, ac_lon, ac_alt_m):
        if ac_alt_m > self.height:
            return 0.0
        dx   = (ac_lon - self.lon) * 111320.0 * math.cos(math.radians(ac_lat))
        dy   = (ac_lat - self.lat) * 110540.0
        dist = math.sqrt(dx**2 + dy**2)
        w    = self.strength * math.exp(-(dist**2) / (2.0 * self.radius**2))
        w   *= max(0.0, 1.0 - ac_alt_m / self.height)
        return w


# ------------------------------------------------------------------ #
# ------------------------------------------------------------------ #
class SoaringRLController:

    DEAD_BAND   = 5.0
    T_A         = 1.5
    DT          = 0.0083
    PHI_D       = -2.0
    MU_LEVELS   = [-30, -15, 0, 15, 30]
    K_FACTOR    = 0.8
    STD_WINDOW  = 400
    MIN_STD_AZ    = 0.05
    MIN_STD_OMEGA = 0.01

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

    def __init__(self):
        sigma_az_1 = (8 * self.T_A) / 3
        sigma_az_2 = (2 * self.T_A) / 3
        self.alpha_az_1 = self.DT / (sigma_az_1 + self.DT)
        self.alpha_az_2 = self.DT / (sigma_az_2 + self.DT)

        sigma_om_1 = self.T_A
        sigma_om_2 = self.T_A / 4
        self.alpha_om_1 = self.DT / (sigma_om_1 + self.DT)
        self.alpha_om_2 = self.DT / (sigma_om_2 + self.DT)

        self.az_s1 = 0.0
        self.az_s2 = 0.0
        self.om_s1 = 0.0
        self.om_s2 = 0.0

        self.prev_climb_ms = None
        self.prev_t         = None

        self.az_buffer    = deque(maxlen=self.STD_WINDOW)
        self.omega_buffer = deque(maxlen=self.STD_WINDOW)

        self.mu_deg         = 0.0
        self.target_roll    = 0.0
        self.last_action_t  = 0.0

        self.kp_pitch = 0.25
        self.kd_pitch = 0.08
        self.prev_pitch = 0.0

        self.last_K_az    = 0.0
        self.last_K_omega = 0.0

    def _snap_mu(self, mu):
        return min(self.MU_LEVELS, key=lambda x: abs(x - mu))

    def _discretize(self, val, threshold):
        if val >  threshold: return +1
        if val < -threshold: return -1
        return 0

    def _rolling_std(self, buf, min_std):
        if len(buf) < 10:
            return min_std
        arr = np.array(buf)
        return max(min_std, float(np.std(arr)))

    def step(self, roll_rad, pitch_rad, airspeed_ms, climb_rate_ms, fdm=None):
        now = time.time()

        if self.prev_climb_ms is not None and self.prev_t is not None:
            dt_real = max(1e-3, now - self.prev_t)
            az_raw = (climb_rate_ms - self.prev_climb_ms) / dt_real
        else:
            az_raw = 0.0
        self.prev_climb_ms = climb_rate_ms
        self.prev_t        = now

        omega_raw = fdm['velocities/p-rad_sec'] if fdm else 0.0

        self.az_s1 = self.alpha_az_1 * az_raw + (1 - self.alpha_az_1) * self.az_s1
        self.az_s2 = self.alpha_az_2 * self.az_s1 + (1 - self.alpha_az_2) * self.az_s2

        self.om_s1 = self.alpha_om_1 * omega_raw + (1 - self.alpha_om_1) * self.om_s1
        self.om_s2 = self.alpha_om_2 * self.om_s1 + (1 - self.alpha_om_2) * self.om_s2

        self.az_buffer.append(self.az_s2)
        self.omega_buffer.append(self.om_s2)

        self.mu_deg = math.degrees(roll_rad)

        if now - self.last_action_t >= self.T_A:
            std_az    = self._rolling_std(self.az_buffer,    self.MIN_STD_AZ)
            std_omega = self._rolling_std(self.omega_buffer, self.MIN_STD_OMEGA)
            K_az      = self.K_FACTOR * std_az
            K_omega   = self.K_FACTOR * std_omega
            self.last_K_az, self.last_K_omega = K_az, K_omega

            az_d       = self._discretize(self.az_s2, K_az)
            omega_d    = self._discretize(self.om_s2, K_omega)
            mu_snapped = self._snap_mu(self.mu_deg)

            action = self.POLICY.get((az_d, omega_d, mu_snapped))
            if action is None:
                action = random.choice([-15, +15])
            if action != 0:
                target = self._snap_mu(self.mu_deg + action)
                if abs(self.mu_deg - target) < self.DEAD_BAND:
                    action = 0

            self.target_roll   = float(np.clip(self.mu_deg + action, -30, 30))
            self.last_action_t = now
            print(f"  [RL] az={az_d:+d}(K={K_az:.3f})  ω={omega_d:+d}(K={K_omega:.3f})  "
                  f"μ={mu_snapped:+d}°  →  Δμ={action:+d}°  target={self.target_roll:.1f}°")

        roll_err = math.radians(self.target_roll) - roll_rad
        aileron  = max(-0.6, min(0.6, 0.4 * roll_err))

        pitch_err = pitch_rad - math.radians(self.PHI_D)
        dpitch    = pitch_rad - self.prev_pitch
        elevator  = max(-0.4, min(0.4, -(self.kp_pitch * pitch_err + self.kd_pitch * dpitch)))
        self.prev_pitch = pitch_rad

        return aileron, elevator, -0.05 * math.radians(self.mu_deg), 0.0



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
    fdm['ic/psi-true-deg'] = 90.0   # heading east
    fdm['ic/phi-deg']      = 0.0
    fdm['ic/theta-deg']    = 1.5
    fdm.run_ic()

    print(f"[JSBSim] Start: lat={START_LAT} lon={START_LON} alt={START_ALT_FT}ft")
    return fdm


# ------------------------------------------------------------------ #
#  THERMALS  (Ankara civarı)                                           #
# ------------------------------------------------------------------ #
THERMALS = [
    # Merkez — güçlü
    Thermal(START_LAT + 0.0000, START_LON + 0.0000,
            strength_ms=4.5, radius_m=100.0, height_m=1200.0),
    # Kuzeydoğu
    Thermal(START_LAT + 0.00100, START_LON + 0.0015,
            strength_ms=3.0, radius_m=75.0,  height_m=1150.0),
    # Güneybatı
    Thermal(START_LAT - 0.0008, START_LON - 0.0012,
            strength_ms=3.5, radius_m=85.0,  height_m=1200.0),
    # Doğu
    Thermal(START_LAT + 0.0055, START_LON + 0.0020,
            strength_ms=2.5, radius_m=60.0,  height_m=1200.0),
    # Kuzey
    Thermal(START_LAT + 0.0030, START_LON - 0.0005,
            strength_ms=2.0, radius_m=55.0,  height_m=1200.0),
]


# ------------------------------------------------------------------ #
#  MAIN LOOP                                                           #
# ------------------------------------------------------------------ #
def run():
    fdm  = build_fdm()
    fg   = FlightGearBridge() if ENABLE_FLIGHTGEAR else None
    ps5  = PS5Controller()
    rl   = SoaringRLController()

    dt             = fdm.get_delta_t()
    print_interval = max(1, int(2.0 / dt))
    mode           = MODE_MANUAL   # başlangıç modu

    # Küçük pygame kontrol penceresi aç (klavye input için gerekli)
    if PYGAME_OK:
        pygame.init()
        ctrl_win = pygame.display.set_mode((400, 120))
        pygame.display.set_caption("Soaring Control — bu pencereye tıkla!")
        ctrl_font = pygame.font.SysFont("monospace", 13)
    else:
        ctrl_win = None
        ctrl_font = None

    print()
    print("=" * 70)
    print("  Ranger 2400 — PS5 + RL Thermal Soaring")
    print(f"  Konum : {START_LAT:.6f}N  {START_LON:.6f}E")
    print(f"  Termal: {len(THERMALS)} adet")
    print()
    print("  PS5:  X=MANUAL  O=RL_SOARING  △=TAKEOFF  □=reset")
    print("=" * 70)
    print(f"  {'Step':>6}  {'Alt(m)':>7}  {'Spd':>6}  {'Climb':>7}  "
          f"{'Roll°':>6}  {'Updraft':>8}  {'MOD':>10}")
    print("-" * 70)

    step    = 0
    t_start = time.time()

    while True:
        # ── PS5 input ──────────────────────────────────────────────
        inp = ps5.get_inputs()

        # mod değiştirme butonları
        if inp.get("quit"):
            print("\n  [QUIT]")
            break

        if inp["btn_manual"]:
            if mode != MODE_MANUAL:
                print(f"\n  [MOD] → MANUAL")
            mode = MODE_MANUAL
        elif inp["btn_rl"]:
            if mode != MODE_RL:
                print(f"\n  [MOD] → RL_SOARING")
            mode = MODE_RL
        elif inp["btn_takeoff"]:
            if mode != MODE_TAKEOFF:
                print(f"\n  [MOD] → TAKEOFF")
            mode = MODE_TAKEOFF

        # ── state ──────────────────────────────────────────────────
        lat       = fdm['position/lat-geod-deg']
        lon       = fdm['position/long-gc-deg']
        alt_m     = fdm['position/h-sl-meters']
        speed_ms  = fdm['velocities/vt-fps'] * 0.3048
        roll_rad  = fdm['attitude/roll-rad']
        pitch_rad = fdm['attitude/pitch-rad']
        climb_ms  = -fdm['velocities/h-dot-fps'] * 0.3048

        # ── thermals ───────────────────────────────────────────────
        total_updraft = sum(t.updraft_at(lat, lon, alt_m) for t in THERMALS)
        fdm['atmosphere/wind-d-fps'] = -total_updraft * 3.28084

        # ── kontrol ────────────────────────────────────────────────
        if mode == MODE_MANUAL:
            aileron  = inp["aileron"]
            elevator = inp["elevator"]
            rudder   = inp["rudder"]
            throttle = inp["throttle"]

        elif mode == MODE_TAKEOFF:
            # tam gaz + burnu yukarı tut
            aileron  = inp["aileron"]     # manuel roll
            elevator = 0.3               # burnu kaldır
            rudder   = inp["rudder"]
            throttle = 1.0               # tam gaz
            # 200m'nin üstüne çıkınca otomatik RL'e geç
            if alt_m > START_ALT_FT * 0.3048 + 50:
                mode = MODE_RL
                print(f"\n  [MOD] Takeoff tamamlandı → RL_SOARING")

        else:  # MODE_RL
            aileron, elevator, rudder, throttle = rl.step(
                roll_rad, pitch_rad, speed_ms, climb_ms, fdm=fdm)
            # PS5 elevator override (ince ayar)
            if abs(inp["elevator"]) > 0.1:
                elevator = inp["elevator"]

        # ── JSBSim'e yaz ───────────────────────────────────────────
        fdm['fcs/aileron-cmd-norm']  = aileron
        fdm['fcs/elevator-cmd-norm'] = elevator
        fdm['fcs/rudder-cmd-norm']   = rudder
        fdm['fcs/throttle-cmd-norm'] = throttle

        fdm.run()
        step += 1

        # ── real-time sync ─────────────────────────────────────────
        elapsed_sim  = step * dt
        elapsed_wall = time.time() - t_start
        sleep_t = elapsed_sim - elapsed_wall
        if sleep_t > 0:
            time.sleep(sleep_t)

        # ── FlightGear ─────────────────────────────────────────────
        if fg:
            fg.send(fdm)

        # ── print ──────────────────────────────────────────────────
        if step % print_interval == 0:
            print(f"  {step:>6}  {alt_m:>7.1f}  {speed_ms:>6.1f}  "
                  f"{climb_ms:>7.2f}  {math.degrees(roll_rad):>6.1f}  "
                  f"{total_updraft:>8.2f}  {mode:>10}")

        # ── pygame kontrol penceresi ─────────────────────────────────
        if PYGAME_OK and ctrl_win:
            ctrl_win.fill((15, 15, 25))
            mode_color = {"MANUAL":(100,180,255),"RL_SOARING":(0,255,120),"TAKEOFF":(255,200,0)}.get(mode,(220,220,220))
            lines = [
                (f"MOD: {mode}   Alt: {alt_m:.0f}m   Spd: {speed_ms:.1f}m/s", mode_color),
                (f"Climb: {climb_ms:+.2f}m/s   Roll: {math.degrees(roll_rad):+.1f}deg   Updraft: {total_updraft:.2f}m/s", (220,220,220)),
                (f"W/S=elev  A/D=ail  Q/E=rud  Shift/Ctrl=gaz", (100,100,140)),
                (f"1=MANUAL  2=RL_SOARING  3=TAKEOFF  ESC=cikis", (100,100,140)),
            ]
            for i,(txt,col) in enumerate(lines):
                surf = ctrl_font.render(txt, True, col)
                ctrl_win.blit(surf, (8, 8 + i*26))
            pygame.display.flip()

        # ── viz state yaz (her 10 stepte bir) ──────────────────────
        if step % 10 == 0:
            try:
                state = {
                    "lat"       : lat,
                    "lon"       : lon,
                    "alt_m"     : alt_m,
                    "speed_ms"  : speed_ms,
                    "climb_ms"  : climb_ms,
                    "roll_deg"  : math.degrees(roll_rad),
                    "heading_deg": math.degrees(fdm["attitude/psi-rad"]),
                    "updraft_ms": total_updraft,
                    "mode"      : mode,
                    "thermals"  : [
                        {"lat": t.lat, "lon": t.lon,
                         "strength_ms": t.strength,
                         "radius_m": t.radius,
                         "height_m": t.height}
                        for t in THERMALS
                    ],
                }
                with open("/tmp/soaring_state.json", "w") as f:
                    json.dump(state, f)
            except Exception:
                pass

        # ── bitiş koşulları ────────────────────────────────────────
        if alt_m < 1.0 and step > 500:
            print("\n  [LANDED / CRASHED]")
            break
        if alt_m > 4000.0:
            print("\n  [MAX ALTITUDE]")
            break
        if step > 500000:
            break


if __name__ == "__main__":
    run()