"""
Thermal Soaring - 2D Top-Down Visualizer + Altitude/Climb Graph
----------------------------------------------------------------
Terminal 1: python main.py
Terminal 2: python thermal_viz.py

Kontroller:
  +/-    zoom
  R      trail sifirla
  ESC    cikis
"""

import pygame
import math
import json
import os
import time
from collections import deque

# ------------------------------------------------------------------ #
#  CONFIG                                                              #
# ------------------------------------------------------------------ #
MAP_W    = 700
PANEL_W  = 260
GRAPH_H  = 200      # alt grafik yüksekliği
TOP_H    = 600      # harita + panel yüksekliği
WIDTH    = MAP_W + PANEL_W
HEIGHT   = TOP_H + GRAPH_H
FPS      = 30
GRAPH_SAMPLES = 999999  # tüm uçuş boyunca sakla

STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "soaring_state.json")

CENTER_LAT =  39.9483187
CENTER_LON =  32.6899477
SCALE      = 5.0

# Renkler
BG        = (15,  15,  25)
PANEL_BG  = (10,  10,  20)
GRAPH_BG  = (10,  12,  22)
GRID_C    = (30,  30,  50)
SEP_C     = (45,  45,  70)
TEXT_C    = (220, 220, 220)
DIM_C     = (90,  90, 120)
POS_C     = (  0, 255, 100)
NEG_C     = (255,  80,  80)
WARN_C    = (255, 200,   0)
STALL_C   = (255,  50,  50)
ALT_C     = (  0, 180, 255)   # irtifa çizgisi
CLIMB_C   = (  0, 255, 120)   # tırmanış çizgisi
UPDRAFT_C = (255, 140,   0)   # updraft çizgisi
ZERO_C    = (60,  60,  90)    # sıfır çizgisi

THERMAL_C = [(255, 80, 20), (255, 160, 0), (100, 200, 255)]
MODE_C    = {"MANUAL":    (100, 180, 255),
             "RL_SOARING":(  0, 255, 120),
             "TAKEOFF":   (255, 200,   0)}

FNT_BIG = FNT_MED = FNT_SML = FNT_TINY = None

def init_fonts():
    global FNT_BIG, FNT_MED, FNT_SML, FNT_TINY
    FNT_BIG  = pygame.font.SysFont("monospace", 17, bold=True)
    FNT_MED  = pygame.font.SysFont("monospace", 15)
    FNT_SML  = pygame.font.SysFont("monospace", 13)
    FNT_TINY = pygame.font.SysFont("monospace", 11)


# ------------------------------------------------------------------ #
#  HELPERS                                                             #
# ------------------------------------------------------------------ #
def latlon_to_px(lat, lon):
    dx = (lon - CENTER_LON) * 111320.0 * math.cos(math.radians(CENTER_LAT))
    dy = (lat - CENTER_LAT) * 110540.0
    return int(MAP_W/2 + dx/SCALE), int(TOP_H/2 - dy/SCALE)

def m2px(m):
    return max(1, int(m / SCALE))


# ------------------------------------------------------------------ #
#  DRAW: GRID                                                          #
# ------------------------------------------------------------------ #
def draw_grid(surf):
    gp = m2px(500)
    for i in range(-10, 11):
        x = MAP_W//2 + i*gp
        y = TOP_H//2 + i*gp
        pygame.draw.line(surf, GRID_C, (x, 0),    (x, TOP_H), 1)
        pygame.draw.line(surf, GRID_C, (0, y),    (MAP_W, y),  1)
        if i != 0:
            surf.blit(FNT_TINY.render(f"{i*500}m", True, (45,45,70)),
                      (x+2, TOP_H//2+2))


# ------------------------------------------------------------------ #
#  DRAW: THERMALS                                                      #
# ------------------------------------------------------------------ #
def draw_thermals(surf, alpha_surf, thermals):
    for t in thermals:
        cx, cy = latlon_to_px(t["lat"], t["lon"])
        if not (-300 < cx < MAP_W+300 and -300 < cy < TOP_H+300):
            continue
        s    = t["strength_ms"]
        r_px = m2px(t["radius_m"] * 2.5)
        col  = THERMAL_C[0] if s >= 4 else THERMAL_C[1] if s >= 2.5 else THERMAL_C[2]

        for ring in range(5, 0, -1):
            rr = int(r_px * ring / 5)
            rs = pygame.Surface((rr*2, rr*2), pygame.SRCALPHA)
            pygame.draw.circle(rs, (*col, int(55*(1-ring/6))), (rr,rr), rr)
            alpha_surf.blit(rs, (cx-rr, cy-rr))

        core = max(4, m2px(t["radius_m"]*0.3))
        pygame.draw.circle(surf, col, (cx, cy), core)
        pygame.draw.circle(surf, (255,255,255), (cx, cy), core, 1)
        surf.blit(FNT_SML.render(f"{s:.1f}m/s", True, col), (cx+core+4, cy-8))


# ------------------------------------------------------------------ #
#  DRAW: AIRCRAFT                                                      #
# ------------------------------------------------------------------ #
def draw_aircraft(surf, trail, ax, ay, psi, roll_deg, mode):
    col = MODE_C.get(mode, (0,255,120))
    if len(trail) > 1:
        for i in range(1, len(trail)):
            fade = max(0, 180 - (len(trail)-i)*2)
            pygame.draw.line(surf, (0, fade, fade//3), trail[i-1], trail[i], 1)
    sz  = 14
    pts = [(ax + sz*math.sin(psi),         ay - sz*math.cos(psi)),
           (ax + sz*.5*math.sin(psi+2.4),  ay - sz*.5*math.cos(psi+2.4)),
           (ax + sz*.5*math.sin(psi-2.4),  ay - sz*.5*math.cos(psi-2.4))]
    pygame.draw.polygon(surf, col, pts)
    pygame.draw.polygon(surf, (255,255,255), pts, 1)
    rr = 24
    sa = math.radians(90 - roll_deg - 20)
    ea = math.radians(90 - roll_deg + 20)
    if sa < ea:
        pygame.draw.arc(surf, WARN_C, (ax-rr, ay-rr, rr*2, rr*2), sa, ea, 3)


# ------------------------------------------------------------------ #
#  DRAW: MAP HUD                                                       #
# ------------------------------------------------------------------ #
def draw_map_hud(surf, mode):
    badge = FNT_BIG.render(f"[ {mode} ]", True, MODE_C.get(mode, TEXT_C))
    surf.blit(badge, (MAP_W//2 - badge.get_width()//2, 8))
    bar_px = m2px(500)
    bx, by = MAP_W - bar_px - 12, TOP_H - 26
    pygame.draw.line(surf, TEXT_C, (bx, by), (bx+bar_px, by), 2)
    pygame.draw.line(surf, TEXT_C, (bx, by-4), (bx, by+4), 2)
    pygame.draw.line(surf, TEXT_C, (bx+bar_px, by-4), (bx+bar_px, by+4), 2)
    surf.blit(FNT_TINY.render("500 m", True, TEXT_C), (bx+bar_px//2-20, by-16))


# ------------------------------------------------------------------ #
#  DRAW: SAG PANEL                                                     #
# ------------------------------------------------------------------ #
def draw_panel(surf, state, rl_log):
    x0 = MAP_W
    pygame.draw.rect(surf, PANEL_BG, (x0, 0, PANEL_W, TOP_H))
    pygame.draw.line(surf, SEP_C, (x0, 0), (x0, TOP_H), 1)

    mode    = state.get("mode",        "---")
    alt     = state.get("alt_m",       0)
    speed   = state.get("speed_ms",    0)
    climb   = state.get("climb_ms",    0)
    roll    = state.get("roll_deg",    0)
    pitch   = state.get("pitch_deg",   0)
    updraft = state.get("updraft_ms",  0)
    heading = state.get("heading_deg", 0)
    az_s    = state.get("az_smooth",   0)
    om_s    = state.get("omega_smooth",0)
    stall   = state.get("stall_guard", False)

    mc = MODE_C.get(mode, TEXT_C)
    px = x0 + 10
    y  = 10

    def sep():
        nonlocal y
        y += 3
        pygame.draw.line(surf, SEP_C, (x0+6, y), (x0+PANEL_W-6, y), 1)
        y += 5

    def row(label, val, col=TEXT_C):
        nonlocal y
        surf.blit(FNT_SML.render(label, True, DIM_C),  (px,    y))
        surf.blit(FNT_MED.render(str(val), True, col), (px+80, y))
        y += 22

    # Mod badge
    badge = FNT_BIG.render(f"[ {mode} ]", True, mc)
    surf.blit(badge, (x0 + PANEL_W//2 - badge.get_width()//2, y))
    y += 30
    sep()

    # Ucus durumu
    surf.blit(FNT_BIG.render("UCUS", True, DIM_C), (px, y)); y += 22
    row("ALT",   f"{alt:.0f} m")
    row("HIZ",   f"{speed:.1f} m/s")
    row("TIRM",  f"{climb:+.2f} m/s",  POS_C if climb > 0 else NEG_C)
    row("ROLL",  f"{roll:+.1f} deg")
    row("PITCH", f"{pitch:+.1f} deg")
    row("HDG",   f"{heading:.0f} deg")
    row("UPDR",  f"{updraft:.2f} m/s", (255,160,0) if updraft > 0.5 else TEXT_C)

    sep()

    # Sensor cue'lari
    surf.blit(FNT_BIG.render("SENSOR", True, DIM_C), (px, y)); y += 22
    row("az",    f"{az_s:+.4f}",  POS_C if az_s > 0 else NEG_C)
    row("omega", f"{om_s:+.4f}",  POS_C if om_s > 0 else NEG_C)

    if stall:
        y += 4
        surf.blit(FNT_BIG.render("!! STALL !!", True, STALL_C), (px, y))
        y += 26

    sep()

    # RL karar gecmisi
    surf.blit(FNT_BIG.render("RL KARAR", True, (0,200,100)), (px, y)); y += 22

    max_show = min(4, len(rl_log))
    if max_show == 0:
        surf.blit(FNT_SML.render("(henuz karar yok)", True, DIM_C), (px, y)); y += 18
    else:
        for i, entry in enumerate(reversed(rl_log[-max_show:])):
            fresh = (i == 0)
            bc  = (0, 230, 100) if fresh else (70, 130, 70)
            dc  = (160,160,160) if fresh else DIM_C
            ac  = entry['action']
            acc = POS_C if ac > 0 else NEG_C if ac < 0 else DIM_C

            surf.blit(FNT_MED.render(
                f"az={entry['az_d']:+d}  w={entry['om_d']:+d}", True, bc), (px, y))
            y += 19
            surf.blit(FNT_MED.render(
                f"mu={entry['mu']:+d}  D={ac:+d} deg", True, acc), (px, y))
            y += 19
            surf.blit(FNT_MED.render(
                f"hedef={entry['target']:+.1f} deg", True, dc), (px, y))
            y += 19
            if i < max_show-1:
                pygame.draw.line(surf, (35,35,55),
                                 (px, y+2), (x0+PANEL_W-10, y+2), 1)
                y += 7

    sep()

    # Reset butonu
    btn = pygame.Rect(px, y, PANEL_W-20, 32)
    pygame.draw.rect(surf, (50,30,80), btn, border_radius=6)
    pygame.draw.rect(surf, (120,60,180), btn, 2, border_radius=6)
    lbl = FNT_MED.render("R — Trail Sifirla", True, (200,150,255))
    surf.blit(lbl, (btn.x + btn.w//2 - lbl.get_width()//2, btn.y+7))
    y += 40

    surf.blit(FNT_TINY.render("+/-=zoom  ESC=cikis", True, (50,50,80)),
              (px, TOP_H-18))

    return btn


# ------------------------------------------------------------------ #
#  DRAW: ALT / CLIMB GRAFIGI                                           #
# ------------------------------------------------------------------ #
def draw_graph(surf, hist_alt, y_offset):
    """
    Uçuş boyunca uzayan irtifa grafiği.
    x ekseni = zaman (uçuş başından bu yana)
    y ekseni = irtifa (m)
    Çizgi rengi: tırmanışta mavi parlak, alçalışta soluk mavi.
    """
    gw = WIDTH
    gh = GRAPH_H - 8
    gx = 0
    gy = y_offset + 4

    pygame.draw.rect(surf, GRAPH_BG, (gx, gy, gw, gh))
    pygame.draw.line(surf, SEP_C, (gx, gy), (gx+gw, gy), 1)

    n = len(hist_alt)

    # Başlık
    surf.blit(FNT_SML.render("IRTIFA GRAFIGI (ucus boyunca)", True, ALT_C),
              (gx+10, gy+4))

    if n < 2:
        surf.blit(FNT_MED.render("Veri bekleniyor...", True, DIM_C),
                  (gx+gw//2-80, gy+gh//2))
        return

    alt_vals = hist_alt
    alt_min  = min(alt_vals) - 30
    alt_max  = max(alt_vals) + 30
    alt_rng  = max(1.0, alt_max - alt_min)

    # Grafik alanı kenar boşlukları
    left_margin  = 55
    right_margin = 60
    top_margin   = 24
    bot_margin   = 20
    plot_w = gw - left_margin - right_margin
    plot_h = gh - top_margin  - bot_margin

    def to_px(i, a):
        x = gx + left_margin + int(i / max(1, n-1) * plot_w)
        y = gy + top_margin  + plot_h - int((a - alt_min) / alt_rng * plot_h)
        return x, y

    # Yatay grid çizgileri + irtifa etiketleri
    step_m = max(50, int(alt_rng / 5 / 50) * 50)
    ga = int(alt_min / step_m) * step_m
    while ga <= alt_max + step_m:
        _, gy_line = to_px(0, ga)
        if gy + top_margin <= gy_line <= gy + top_margin + plot_h:
            pygame.draw.line(surf, (28,28,48),
                             (gx+left_margin, gy_line),
                             (gx+left_margin+plot_w, gy_line), 1)
            surf.blit(FNT_TINY.render(f"{ga}m", True, (70,90,130)),
                      (gx+3, gy_line-7))
        ga += step_m

    # Dikey grid (zaman bölmeleri, yaklaşık 5 adet)
    for frac in [0.2, 0.4, 0.6, 0.8]:
        vx = gx + left_margin + int(frac * plot_w)
        pygame.draw.line(surf, (28,28,48),
                         (vx, gy+top_margin),
                         (vx, gy+top_margin+plot_h), 1)
        # Zaman etiketi (adım sayısı)
        step_label = int(frac * (n-1))
        surf.blit(FNT_TINY.render(f"t={step_label}", True, (50,50,80)),
                  (vx-12, gy+top_margin+plot_h+3))

    # İrtifa çizgisi — segment segment, tırmanışta parlak mavi alçalışta soluk
    pts = [to_px(i, a) for i, a in enumerate(alt_vals)]
    for i in range(1, len(pts)):
        rising = alt_vals[i] >= alt_vals[i-1]
        col = (0, 160, 255) if rising else (0, 80, 160)
        pygame.draw.line(surf, col, pts[i-1], pts[i], 2)

    # Mevcut irtifa noktası (canlı)
    cx, cy = pts[-1]
    pygame.draw.circle(surf, (0, 220, 255), (cx, cy), 5)
    pygame.draw.circle(surf, (255,255,255), (cx, cy), 5, 1)
    cur = alt_vals[-1]
    surf.blit(FNT_MED.render(f"{cur:.0f} m", True, (0,220,255)),
              (min(cx+8, gx+gw-right_margin-10), max(gy+top_margin, cy-10)))

    # Başlangıç noktası
    sx, sy = pts[0]
    pygame.draw.circle(surf, (100,100,200), (sx, sy), 4)
    surf.blit(FNT_TINY.render(f"{alt_vals[0]:.0f}m", True, (100,100,200)),
              (sx+6, sy-8))

    # Max / min irtifa çizgileri
    max_alt = max(alt_vals)
    min_alt = min(alt_vals)
    max_i   = alt_vals.index(max_alt)
    min_i   = alt_vals.index(min_alt)

    _, y_max = to_px(0, max_alt)
    _, y_min = to_px(0, min_alt)
    x_max, _ = to_px(max_i, max_alt)
    x_min, _ = to_px(min_i, min_alt)

    pygame.draw.line(surf, (0,60,100),
                     (gx+left_margin, y_max),
                     (gx+left_margin+plot_w, y_max), 1)
    pygame.draw.line(surf, (80,30,30),
                     (gx+left_margin, y_min),
                     (gx+left_margin+plot_w, y_min), 1)

    surf.blit(FNT_TINY.render(f"MAX {max_alt:.0f}m", True, (0,150,220)),
              (gx+left_margin+plot_w+3, y_max-7))
    surf.blit(FNT_TINY.render(f"MIN {min_alt:.0f}m", True, (180,60,60)),
              (gx+left_margin+plot_w+3, y_min-7))

    # Toplam kazanım
    gain = cur - alt_vals[0]
    col  = POS_C if gain >= 0 else NEG_C
    surf.blit(FNT_SML.render(f"Kazanim: {gain:+.0f} m  |  n={n}",
                              True, col),
              (gx+gw//2-80, gy+top_margin+plot_h+4))


# ------------------------------------------------------------------ #
#  MAIN                                                                #
# ------------------------------------------------------------------ #
def main():
    global SCALE
    pygame.init()
    init_fonts()

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Thermal Soaring — Map + RL + Graph")
    clock  = pygame.time.Clock()

    trail          = []
    rl_log         = []
    last_rl_action = None
    MAX_TRAIL      = 500

    # Grafik geçmişi
    hist_alt     = []   # tüm uçuş boyunca irtifa geçmişi

    print(f"[VIZ] State: {STATE_FILE}")
    print("[VIZ] R=trail sifirla  +/-=zoom  ESC=cikis")

    btn_rect = None

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit(); return
                if event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    SCALE = max(1.0, SCALE * 0.8)
                if event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    SCALE = min(50.0, SCALE * 1.25)
                if event.key == pygame.K_r:
                    trail.clear()
                    print("[VIZ] Trail sifirlandi")
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if btn_rect and btn_rect.collidepoint(event.pos):
                    trail.clear()
                    print("[VIZ] Trail sifirlandi (buton)")

        # ── state oku ──────────────────────────────────────────────
        state = {}
        if os.path.exists(STATE_FILE):
            try:
                with open(STATE_FILE) as f:
                    state = json.load(f)
            except Exception:
                pass

        # RL karar log
        last_dec = state.get("last_rl_decision")
        if last_dec and last_dec != last_rl_action:
            rl_log.append(last_dec)
            last_rl_action = last_dec
            if len(rl_log) > 50:
                rl_log.pop(0)

        # Grafik geçmişine ekle
        if state:
            hist_alt.append(state.get("alt_m", 1300))

        # ── Çiz ────────────────────────────────────────────────────
        screen.fill(BG)

        # Harita alanı (üst)
        alpha_surf = pygame.Surface((MAP_W, TOP_H), pygame.SRCALPHA)
        draw_grid(screen)
        draw_thermals(screen, alpha_surf, state.get("thermals", []))
        screen.blit(alpha_surf, (0, 0))

        ac_lat  = state.get("lat",         CENTER_LAT)
        ac_lon  = state.get("lon",         CENTER_LON)
        ac_psi  = math.radians(state.get("heading_deg", 0))
        ac_roll = state.get("roll_deg", 0)
        mode    = state.get("mode", "MANUAL")

        ax, ay = latlon_to_px(ac_lat, ac_lon)
        if 0 <= ax <= MAP_W and 0 <= ay <= TOP_H:
            trail.append((ax, ay))
        if len(trail) > MAX_TRAIL:
            trail.pop(0)

        draw_aircraft(screen, trail, ax, ay, ac_psi, ac_roll, mode)
        draw_map_hud(screen, mode)

        # Harita alt sınırı
        pygame.draw.line(screen, SEP_C, (0, TOP_H), (MAP_W, TOP_H), 1)

        # Sağ panel
        btn_rect = draw_panel(screen, state, rl_log)

        # Alt grafik
        draw_graph(screen, hist_alt, TOP_H)

        pygame.display.flip()
        clock.tick(FPS)


if __name__ == "__main__":
    main()