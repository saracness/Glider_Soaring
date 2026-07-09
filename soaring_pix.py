"""
Pixhawk (ArduPlane) uzerinde Reddy et al. (2018) termal soaring policy'sini
GUIDED modda uygulayan test scripti.

Davranis: GUIDED moda gecildigi an policy roll (bank) hedefi uretmeye ve
surekli SET_ATTITUDE_TARGET gondermeye baslar. Baska bir mod secilirse
(RTL, MANUAL, FBWA, ...) hicbir komut gondermez.
"""

from pymavlink import mavutil
import numpy as np
import random
import time

# ------------------------------------------------------------------ #
#  BAGLANTI                                                            #
# ------------------------------------------------------------------ #
SERIAL_PORT = "COM3"
BAUD        = 115200

LOOP_HZ = 10.0                 # attitude komutu + policy pompasi
DT      = 1.0 / LOOP_HZ

# ------------------------------------------------------------------ #
#  POLICY PARAMETRELERI (Reddy et al. 2018)                           #
# ------------------------------------------------------------------ #
K_AZ        = 0.5      # m/s^2 - test sonrasi rolling-std ile guncellenecek
K_OMEGA     = 0.1      # rad/s - test sonrasi rolling-std ile guncellenecek
DEAD_BAND   = 5.0       # derece
T_A         = 1.5       # saniye, policy karar araligi
PHI_D       = -2.0      # istenen pitch (derece)
MU_LEVELS   = [-30, -15, 0, 15, 30]

sigma_az    = (2 * T_A) / 3
sigma_omega = T_A / 4
ALPHA_AZ    = DT / (sigma_az    + DT)
ALPHA_OMEGA = DT / (sigma_omega + DT)

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


def snap_mu(mu_deg):
    return min(MU_LEVELS, key=lambda x: abs(x - mu_deg))


def discretize(value, threshold):
    if value > threshold:
        return +1
    if value < -threshold:
        return -1
    return 0


def get_action(az_d, omega_d, mu_snapped):
    action = POLICY.get((az_d, omega_d, mu_snapped))
    if action is None:
        action = random.choice([-15, +15])
    return action


def apply_dead_band(mu_deg, action):
    if action == 0:
        return 0
    target = snap_mu(mu_deg + action)
    if abs(mu_deg - target) < DEAD_BAND:
        return 0
    return action


# ------------------------------------------------------------------ #
#  MAVLINK YARDIMCILARI                                                #
# ------------------------------------------------------------------ #
ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE  = 1
ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE = 2
ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE   = 4
ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE        = 64   # thrust alanini yoksay

# rate'leri yoksay (quaternion kullan), throttle'i yoksay (ArduPilot yonetsin)
ATTITUDE_TYPE_MASK = (
    ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
    | ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    | ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
    | ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE
)


def euler_to_quaternion(roll, pitch, yaw):
    cr, sr = np.cos(roll  / 2), np.sin(roll  / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw   / 2), np.sin(yaw   / 2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return [w, x, y, z]


def request_message_interval(master, mavlink_msg_id, hz):
    interval_us = int(1e6 / hz)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavlink_msg_id, interval_us, 0, 0, 0, 0, 0)


def send_attitude(master, roll_deg, pitch_deg, yaw_rad):
    q = euler_to_quaternion(
        np.radians(roll_deg), np.radians(pitch_deg), yaw_rad)
    try:
        master.mav.set_attitude_target_send(
            0, master.target_system, master.target_component,
            ATTITUDE_TYPE_MASK, q, 0, 0, 0, 0.0)
    except Exception as e:
        print(f"[WARN] attitude gonderilemedi: {e}")


# ------------------------------------------------------------------ #
#  UCUS STATE - tek bir non-blocking pompa ile guncellenir              #
# ------------------------------------------------------------------ #
class FlightState:
    def __init__(self):
        self.mode        = None
        self.armed       = False
        self.roll_deg    = 0.0
        self.yaw_rad     = 0.0
        self.rollspeed   = 0.0     # rad/s
        self.climb_ms    = 0.0     # az icin turevi alinacak
        self._prev_climb = None
        self._prev_climb_t = None
        self.az_ms2      = 0.0     # dus/climb turevinden tahmini dikey ivme
        self.last_att_t  = 0.0
        self.last_vfr_t  = 0.0

    def pump(self, master):
        """Bufferdaki tum mesajlari (blocking olmadan) tuket, state'i guncelle."""
        while True:
            msg = master.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()

            if t == "HEARTBEAT":
                self.mode  = mavutil.mode_string_v10(msg)
                self.armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            elif t == "ATTITUDE":
                self.roll_deg  = np.degrees(msg.roll)
                self.yaw_rad   = msg.yaw
                self.rollspeed = msg.rollspeed
                self.last_att_t = time.time()

            elif t == "VFR_HUD":
                now = time.time()
                if self._prev_climb is not None and self._prev_climb_t is not None:
                    dt_real = max(1e-3, now - self._prev_climb_t)
                    self.az_ms2 = (msg.climb - self._prev_climb) / dt_real
                self._prev_climb   = msg.climb
                self._prev_climb_t = now
                self.climb_ms   = msg.climb
                self.last_vfr_t = now

    def sensors_fresh(self, max_age=1.0):
        now = time.time()
        return (now - self.last_att_t < max_age) and (now - self.last_vfr_t < max_age)


# ------------------------------------------------------------------ #
#  MAIN                                                                #
# ------------------------------------------------------------------ #
def main():
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD)
    master.wait_heartbeat()
    print(f"Connected: sys={master.target_system} comp={master.target_component}")

    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, LOOP_HZ)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, LOOP_HZ)

    st = FlightState()

    az_s1        = 0.0
    az_s2        = 0.0
    om_s1        = 0.0
    om_s2        = 0.0
    target_roll  = 0.0
    last_action_t = time.time()
    last_mode     = None

    print("Bekleniyor... (GUIDED moda gecince policy devreye girer)")

    try:
        while True:
            loop_start = time.time()
            st.pump(master)

            if st.mode != last_mode:
                print(f"[MOD] {last_mode} -> {st.mode}")
                last_mode = st.mode
                # mod degisince bank hedefini mevcut rolle sifirla
                target_roll  = st.roll_deg
                last_action_t = time.time()

            active = (st.mode == "GUIDED") and st.armed and st.sensors_fresh()

            if active:
                # -- smoothing (T_A periyodunda) --
                az_s1 = ALPHA_AZ * st.az_ms2 + (1 - ALPHA_AZ) * az_s1
                az_s2 = ALPHA_AZ * az_s1     + (1 - ALPHA_AZ) * az_s2
                om_s1 = ALPHA_OMEGA * st.rollspeed + (1 - ALPHA_OMEGA) * om_s1
                om_s2 = ALPHA_OMEGA * om_s1        + (1 - ALPHA_OMEGA) * om_s2

                if time.time() - last_action_t >= T_A:
                    az_d       = discretize(az_s2, K_AZ)
                    omega_d    = discretize(om_s2, K_OMEGA)
                    mu_snapped = snap_mu(st.roll_deg)

                    action = get_action(az_d, omega_d, mu_snapped)
                    action = apply_dead_band(st.roll_deg, action)

                    target_roll = float(np.clip(mu_snapped + action, -30, 30))
                    last_action_t = time.time()
                    print(f"  az={az_d:+d}  w={omega_d:+d}  mu={mu_snapped:+d} deg "
                          f"-> dmu={action:+d} deg  target={target_roll:.1f} deg")

                send_attitude(master, target_roll, PHI_D, st.yaw_rad)

            elif st.mode == "GUIDED" and not st.sensors_fresh():
                print("[WARN] ATTITUDE/VFR_HUD verisi bayat - komut gonderilmiyor")

            sleep_left = DT - (time.time() - loop_start)
            if sleep_left > 0:
                time.sleep(sleep_left)

    except KeyboardInterrupt:
        print("\n[CIKIS] Kullanici durdurdu.")


if __name__ == "__main__":
    main()
