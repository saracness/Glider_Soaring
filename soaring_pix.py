"""
Pixhawk (ArduPlane) uzerinde Reddy et al. (2018, Nature 562:236-239)
termal soaring policy'sini GUIDED modda uygulayan test scripti.

Davranis: GUIDED moda gecildigi an policy roll (bank) hedefi uretmeye ve
surekli SET_ATTITUDE_TARGET gondermeye baslar. Baska bir mod secilirse
hicbir komut gondermez.

Bu surum, makalenin "Estimation of the vertical wind acceleration"
bolumundeki (4)-(6) denklemlerini birebir uyguluyor - amac, termal
sinyalini (az) ucagin KENDI pitch salinimindan ve banking sirasindaki
hucum-acisi (AoA) degisiminden ayristirmak:

    az == dwz/dt = d/dt(uz - vz)                                  (4)
    Delta_vz = -V * (Delta_alpha - Delta_phi)                     (5)
    Delta_alpha ~= (alpha0 - alpha_i) * (1/cos(mu) - 1)            (6)

  uz  : yer-referansli tirmanma hizi (VFR_HUD.climb, GPS+baro EKF)
  vz  : hava kutlesine gore dikey hiz - DOGRUDAN olculmuyor; ucagin
        pitch sapmasi (Delta_phi) ve bank'a bagli AoA sapmasindan
        (Delta_alpha) tahmin ediliyor. Sabit terim (vz0) turevde
        zaten kayboldugu icin sadece Delta_vz(t) hesaplaniyor.
  (alpha0 - alpha_i): makalede "deneylerden cikarilir" deniyor, sayisal
        deger PAYLASILMIYOR (kanat geometrisine ozgu). CALIB_C_ALPHA_DEG
        sizin Ranger 2400'unuz icin SAHADA olculmeli - asagida.

ω (roll-wise torque) icin ayni mantik: makale "olculen bank acisinin,
BEKLENEN (komutlanan) bank acisindan sapmasi" diyor ama tam formulu
Supplementary Information'da (bu dosyada yok). Burada eq.(3)'teki
lineer bank rampasindan (mu_i -> mu_f, ta suresinde) turetilen
"komutlanan roll rate"i olculen rollspeed'den cikararak makul bir
yorum uyguladim - bu tam makale formulu degil, ayni prensibin
(kendi kontrolunun payini cikar) makul bir uzantisidir.

KALIBRASYON GEREKLI (asagidaki iki sabit None oldugu surece policy
calismaz, script bilerek reddeder):
  CALIB_PHI_LEVEL_DEG : sabit hizda, duz-seviye ucuşta trim pitch (deg)
  CALIB_C_ALPHA_DEG    : (alpha0 - alpha_i), derece cinsinden

  Basit saha kalibrasyonu:
   1) Sakin havada, sabit airspeed'de duz-seviye ucus yapin, ATTITUDE.pitch
      ortalamasini kaydedin -> CALIB_PHI_LEVEL_DEG.
   2) Ayni sakin havada, +-15 ve +-30 derece bank'ta birkac saniye sabit
      donus yapin; her bank'ta airspeed sabit kalacak sekilde elevator
      trim/pitch degisimini kaydedin. Delta_phi'yi (o banktaki pitch -
      CALIB_PHI_LEVEL_DEG) ölçüp (1/cos(mu)-1)'e karsi dogrusal fit edin;
      egim (alpha0-alpha_i) = CALIB_C_ALPHA_DEG'i verir.
  Bu adimlar atlanirsa/kabaca girilirse az sinyali yanlis olur - policy
  yine "kendi manevrasini" termal sanabilir, tam da onlemek istediginiz
  sorun.

Onceki soaring_pix.py surumunde duzeltilenler (throttle-ignore biti,
non-blocking mesaj pompasi, surekli attitude gonderimi, stream-rate
istegi) burada da geçerli, tekrar yazilmadi - sadece az/omega tahmini
ve bank komutlama makaleye gore güncellendi.
"""

from pymavlink import mavutil
import numpy as np
import random
import time
from collections import deque

# ------------------------------------------------------------------ #
#  BAGLANTI                                                            #
# ------------------------------------------------------------------ #
SERIAL_PORT = "COM3"
BAUD        = 115200

LOOP_HZ = 10.0                 # attitude komutu + policy pompasi
DT      = 1.0 / LOOP_HZ

# ------------------------------------------------------------------ #
#  SAHA KALIBRASYONU - None oldukca policy CALISMAZ                    #
# ------------------------------------------------------------------ #
CALIB_PHI_LEVEL_DEG = None     # ör: 1.5  (duz-seviye ucusta trim pitch)
CALIB_C_ALPHA_DEG   = None     # ör: 3.0  ((alpha0-alpha_i), saha testiyle)

# ------------------------------------------------------------------ #
#  POLICY PARAMETRELERI (Reddy et al. 2018)                           #
# ------------------------------------------------------------------ #
DEAD_BAND   = 5.0       # derece
T_A         = 1.5       # saniye, policy karar araligi (eq. 3'teki ta)
PHI_D       = -2.0      # istenen pitch (derece)
MU_LEVELS   = [-30, -15, 0, 15, 30]

# Iki kademeli exponential smoothing zaman sabitleri (main.py ile ayni,
# Extended Data Fig. 4'teki "exponential smoothing kernel of timescale
# sigma_a" tarifinin repo icinde onceden kullanilan somutlastirmasi)
sigma_az_1  = (8 * T_A) / 3
sigma_az_2  = (2 * T_A) / 3
sigma_om_1  = T_A
sigma_om_2  = T_A / 4
ALPHA_AZ_1  = DT / (sigma_az_1 + DT)
ALPHA_AZ_2  = DT / (sigma_az_2 + DT)
ALPHA_OM_1  = DT / (sigma_om_1 + DT)
ALPHA_OM_2  = DT / (sigma_om_2 + DT)

# Adaptif esikler: Ka, Komega = 0.8 * o gunku/o pencerdeki std (makale +
# main.py'deki K_FACTOR/STD_WINDOW ile ayni)
K_FACTOR      = 0.8
STD_WINDOW    = 400        # ornek sayisi (~40s @ 10Hz)
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


def snap_mu(mu_deg):
    return min(MU_LEVELS, key=lambda x: abs(x - mu_deg))


def discretize(value, threshold):
    if value > threshold:
        return +1
    if value < -threshold:
        return -1
    return 0


def rolling_std(buf, min_std):
    if len(buf) < 10:
        return min_std
    return max(min_std, float(np.std(np.array(buf))))


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
#  VERTICAL WIND ACCELERATION - eq. (4)-(6)                           #
# ------------------------------------------------------------------ #
def estimate_delta_vz(airspeed_ms, pitch_deg, mu_deg):
    """Delta_vz = -V*(Delta_alpha - Delta_phi), eq. (5)-(6)."""
    delta_phi_rad = np.radians(pitch_deg - CALIB_PHI_LEVEL_DEG)
    cos_mu = max(1e-3, np.cos(np.radians(mu_deg)))
    delta_alpha_rad = np.radians(CALIB_C_ALPHA_DEG) * (1.0 / cos_mu - 1.0)
    return airspeed_ms * (delta_phi_rad - delta_alpha_rad)


# ------------------------------------------------------------------ #
#  MAVLINK YARDIMCILARI                                                #
# ------------------------------------------------------------------ #
ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE  = 1
ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE = 2
ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE   = 4
ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE        = 64   # thrust alanini yoksay

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
        self.mode         = None
        self.armed        = False
        self.roll_deg     = 0.0
        self.pitch_deg    = 0.0
        self.yaw_rad      = 0.0
        self.rollspeed    = 0.0     # rad/s, ham gyro
        self.airspeed_ms  = 0.0
        self.climb_ms     = 0.0     # uz (yer-referansli)
        self._prev_uzc    = None
        self._prev_uzc_t  = None
        self.az_ms2       = 0.0     # duzeltilmis az (eq. 4)
        self.last_att_t   = 0.0
        self.last_vfr_t   = 0.0

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
                self.roll_deg   = np.degrees(msg.roll)
                self.pitch_deg  = np.degrees(msg.pitch)
                self.yaw_rad    = msg.yaw
                self.rollspeed  = msg.rollspeed
                self.last_att_t = time.time()

            elif t == "VFR_HUD":
                now = time.time()
                self.airspeed_ms = msg.airspeed
                self.climb_ms    = msg.climb

                # eq. (5)-(6): ucagin kendi pitch/AoA katkisini uz'den cikar
                delta_vz = estimate_delta_vz(
                    self.airspeed_ms, self.pitch_deg, self.roll_deg)
                uz_corrected = self.climb_ms - delta_vz

                if self._prev_uzc is not None and self._prev_uzc_t is not None:
                    dt_real = max(1e-3, now - self._prev_uzc_t)
                    self.az_ms2 = (uz_corrected - self._prev_uzc) / dt_real
                self._prev_uzc   = uz_corrected
                self._prev_uzc_t = now
                self.last_vfr_t  = now

    def sensors_fresh(self, max_age=1.0):
        now = time.time()
        return (now - self.last_att_t < max_age) and (now - self.last_vfr_t < max_age)


# ------------------------------------------------------------------ #
#  BANK RAMPASI - eq. (3): mu_d(t) = mu_i + (mu_f-mu_i)*t/ta            #
# ------------------------------------------------------------------ #
class BankRamp:
    def __init__(self, mu_deg):
        self.mu_i = mu_deg
        self.mu_f = mu_deg
        self.t0   = time.time()

    def set_target(self, mu_i_deg, mu_f_deg):
        self.mu_i = mu_i_deg
        self.mu_f = mu_f_deg
        self.t0   = time.time()

    def sample(self, now):
        """(komutlanan_roll_deg, komutlanan_roll_rate_rad_s) dondurur."""
        if now - self.t0 >= T_A or T_A <= 0:
            return self.mu_f, 0.0
        frac = (now - self.t0) / T_A
        commanded_roll = self.mu_i + (self.mu_f - self.mu_i) * frac
        commanded_rate = np.radians(self.mu_f - self.mu_i) / T_A
        return commanded_roll, commanded_rate


# ------------------------------------------------------------------ #
#  MAIN                                                                #
# ------------------------------------------------------------------ #
def main():
    if CALIB_PHI_LEVEL_DEG is None or CALIB_C_ALPHA_DEG is None:
        raise SystemExit(
            "CALIB_PHI_LEVEL_DEG / CALIB_C_ALPHA_DEG kalibre edilmemis.\n"
            "Bu sabitler olculmeden policy'yi calistirmak, tam onlemeye "
            "calistigimiz hatayi (kendi manevrayi termal sanmak) geri "
            "getirir. Dosyanin ustundeki kalibrasyon notlarina bakin.")

    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD)
    master.wait_heartbeat()
    print(f"Connected: sys={master.target_system} comp={master.target_component}")

    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, LOOP_HZ)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, LOOP_HZ)

    st = FlightState()

    az_s1, az_s2 = 0.0, 0.0
    om_s1, om_s2 = 0.0, 0.0
    az_buf    = deque(maxlen=STD_WINDOW)
    omega_buf = deque(maxlen=STD_WINDOW)

    ramp          = BankRamp(0.0)
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
                ramp = BankRamp(st.roll_deg)     # rampayi mevcut rolle sifirla
                last_action_t = time.time()

            active = (st.mode == "GUIDED") and st.armed and st.sensors_fresh()

            if active:
                now = time.time()
                commanded_roll, commanded_rate = ramp.sample(now)

                # az: iki kademeli smoothing (Extended Data Fig. 4)
                az_s1 = ALPHA_AZ_1 * st.az_ms2 + (1 - ALPHA_AZ_1) * az_s1
                az_s2 = ALPHA_AZ_2 * az_s1     + (1 - ALPHA_AZ_2) * az_s2

                # omega: olculen roll rate - komutlanan (rampanin) roll rate
                omega_raw = st.rollspeed - commanded_rate
                om_s1 = ALPHA_OM_1 * omega_raw + (1 - ALPHA_OM_1) * om_s1
                om_s2 = ALPHA_OM_2 * om_s1     + (1 - ALPHA_OM_2) * om_s2

                az_buf.append(az_s2)
                omega_buf.append(om_s2)

                if now - last_action_t >= T_A:
                    k_az    = K_FACTOR * rolling_std(az_buf,    MIN_STD_AZ)
                    k_omega = K_FACTOR * rolling_std(omega_buf, MIN_STD_OMEGA)

                    az_d       = discretize(az_s2, k_az)
                    omega_d    = discretize(om_s2, k_omega)
                    mu_snapped = snap_mu(commanded_roll)

                    action = get_action(az_d, omega_d, mu_snapped)
                    action = apply_dead_band(commanded_roll, action)

                    new_target = float(np.clip(mu_snapped + action, -30, 30))
                    ramp.set_target(commanded_roll, new_target)
                    last_action_t = now
                    print(f"  az={az_d:+d}(K={k_az:.3f})  w={omega_d:+d}(K={k_omega:.3f})  "
                          f"mu={mu_snapped:+d} deg -> dmu={action:+d} deg  "
                          f"target={new_target:.1f} deg")

                send_attitude(master, commanded_roll, PHI_D, st.yaw_rad)

            elif st.mode == "GUIDED" and not st.sensors_fresh():
                print("[WARN] ATTITUDE/VFR_HUD verisi bayat - komut gonderilmiyor")

            sleep_left = DT - (time.time() - loop_start)
            if sleep_left > 0:
                time.sleep(sleep_left)

    except KeyboardInterrupt:
        print("\n[CIKIS] Kullanici durdurdu.")


if __name__ == "__main__":
    main()
