"""
Thermal Soaring - 2D Top-Down Visualizer
-----------------------------------------
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

# ------------------------------------------------------------------ #
#  CONFIG                                                              #
# ------------------------------------------------------------------ #
PANEL_W       = 260
MAP_W         = 800
WIDTH         = MAP_W + PANEL_W
HEIGHT        = 800
FPS           = 30

STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "soaring_state.json")

CENTER_LAT =  39.9483187
CENTER_LON =  32.6899477
SCALE      = 5.0

# Renkler
BG          = (15,  15,  25)
PANEL_BG    = (10,  10,  20)
GRID_C      = (30,  30,  50)
TEXT_C      = (220, 220, 220)
DIM_C       = (90,  90, 120)
SEP_C       = (45,  45,  70)
MODE_C      = {"MANUAL":     (100, 180, 255),
               "RL_SOARING": (  0, 255, 120),
               "TAKEOFF":    (255, 200,   0)}
THERMAL_C   = [(255, 80, 20), (255, 160, 0), (100, 200, 255)]
POS_C       = (  0, 255, 100)
NEG_C       = (255,  80,  80)
WARN_C      = (255, 200,   0)
STALL_C     = (255,  50,  50)

FNT_BIG    = None   # init sonra doldurulur
FNT_MED    = None
FNT_SML    = None


def init_fonts():
    global FNT_BIG, FNT_MED, FNT_SML
    FNT_BIG = pygame.font.SysFont("monospace", 17, bold=True)
    FNT_MED = pygame.font.SysFont("monospace", 15)
    FNT_SML = pygame.font.SysFont("monospace", 13)


# ------------------------------------------------------------------ #
#  HELPERS                                                             #
# ------------------------------------------------------------------ #
def latlon_to_px(lat, lon):
    dx = (lon - CENTER_LON) * 111320.0 * math.cos(math.radians(CENTER_LAT))
    dy = (lat - CENTER_LAT) * 110540.0
    return int(MAP_W/2 + dx/SCALE), int(HEIGHT/2 - dy/SCALE)

def m2px(m):
    return max(1, int(m / SCALE))


# ------------------------------------------------------------------ #
#  DRAW: GRID                                                          #
# ------------------------------------------------------------------ #
def draw_grid(surf):
    gp  = m2px(500)
    fnt = pygame.font.SysFont("monospace", 11)
    for i in range(-10, 11):
        x = MAP_W//2 + i*gp
        y = HEIGHT//2 + i*gp
        pygame.draw.line(surf, GRID_C, (x, 0),    (x, HEIGHT), 1)
        pygame.draw.line(surf, GRID_C, (0, y),    (MAP_W, y),  1)
        if i != 0:
            surf.blit(fnt.render(f"{i*500}m", True, (45,45,70)),
                      (x+2, HEIGHT//2+2))


# ------------------------------------------------------------------ #
#  DRAW: THERMALS                                                      #
# ------------------------------------------------------------------ #
def draw_thermals(surf, alpha_surf, thermals):
    for t in thermals:
        cx, cy = latlon_to_px(t["lat"], t["lon"])
        if cx < -200 or cx > MAP_W+200 or cy < -200 or cy > HEIGHT+200:
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
        lbl = FNT_SML.render(f"{s:.1f}m/s", True, col)
        surf.blit(lbl, (cx+core+4, cy-8))


# ------------------------------------------------------------------ #
#  DRAW: AIRCRAFT + TRAIL                                              #
# ------------------------------------------------------------------ #
def draw_aircraft(surf, trail, ax, ay, psi, roll_deg, mode):
    col = MODE_C.get(mode, (0, 255, 120))

    # trail
    if len(trail) > 1:
        for i in range(1, len(trail)):
            fade = max(0, 180 - (len(trail)-i)*2)
            pygame.draw.line(surf, (0, fade, fade//3),
                             trail[i-1], trail[i], 1)

    # uçak üçgeni
    sz  = 14
    pts = [(ax + sz*math.sin(psi),         ay - sz*math.cos(psi)),
           (ax + sz*.5*math.sin(psi+2.4),  ay - sz*.5*math.cos(psi+2.4)),
           (ax + sz*.5*math.sin(psi-2.4),  ay - sz*.5*math.cos(psi-2.4))]
    pygame.draw.polygon(surf, col, pts)
    pygame.draw.polygon(surf, (255,255,255), pts, 1)

    # roll arc
    rr = 24
    sa = math.radians(90 - roll_deg - 20)
    ea = math.radians(90 - roll_deg + 20)
    if sa < ea:
        pygame.draw.arc(surf, WARN_C, (ax-rr, ay-rr, rr*2, rr*2), sa, ea, 3)


# ------------------------------------------------------------------ #
#  DRAW: MAP HUD (mod badge + scale bar)                              #
# ------------------------------------------------------------------ #
def draw_map_hud(surf, mode):
    badge = FNT_BIG.render(f"[ {mode} ]", True, MODE_C.get(mode, TEXT_C))
    surf.blit(badge, (MAP_W//2 - badge.get_width()//2, 8))

    bar_px = m2px(500)
    bx, by = MAP_W - bar_px - 16, HEIGHT - 28
    pygame.draw.line(surf, TEXT_C, (bx, by), (bx+bar_px, by), 2)
    pygame.draw.line(surf, TEXT_C, (bx, by-4), (bx, by+4), 2)
    pygame.draw.line(surf, TEXT_C, (bx+bar_px, by-4), (bx+bar_px, by+4), 2)
    surf.blit(FNT_SML.render("500 m", True, TEXT_C),
              (bx+bar_px//2-20, by-18))


# ------------------------------------------------------------------ #
#  DRAW: SAG PANEL                                                     #
# ------------------------------------------------------------------ #
def draw_panel(surf, state, rl_log):
    x0 = MAP_W
    pygame.draw.rect(surf, PANEL_BG, (x0, 0, PANEL_W, HEIGHT))
    pygame.draw.line(surf, SEP_C, (x0, 0), (x0, HEIGHT), 1)

    mode    = state.get("mode",       "---")
    alt     = state.get("alt_m",      0)
    speed   = state.get("speed_ms",   0)
    climb   = state.get("climb_ms",   0)
    roll    = state.get("roll_deg",   0)
    pitch   = state.get("pitch_deg",  0)
    updraft = state.get("updraft_ms", 0)
    heading = state.get("heading_deg",0)
    stall   = state.get("stall_guard",False)

    mc = MODE_C.get(mode, TEXT_C)
    px = x0 + 10
    y  = 10

    def sep():
        nonlocal y
        y += 4
        pygame.draw.line(surf, SEP_C, (x0+6, y), (x0+PANEL_W-6, y), 1)
        y += 6

    def row(label, val, col=TEXT_C):
        nonlocal y
        surf.blit(FNT_SML.render(label, True, DIM_C), (px,   y))
        surf.blit(FNT_MED.render(str(val), True, col), (px+75, y))
        y += 22

    # MOD badge
    badge = FNT_BIG.render(f"[ {mode} ]", True, mc)
    surf.blit(badge, (x0 + PANEL_W//2 - badge.get_width()//2, y))
    y += 30
    sep()

    # Uçuş durumu
    surf.blit(FNT_BIG.render("UCUS", True, DIM_C), (px, y)); y += 22
    row("ALT",   f"{alt:.0f} m")
    row("HIZ",   f"{speed:.1f} m/s")
    row("TIRM",  f"{climb:+.2f} m/s", POS_C if climb > 0 else NEG_C)
    row("ROLL",  f"{roll:+.1f} deg")
    row("PITCH", f"{pitch:+.1f} deg")
    row("HDG",   f"{heading:.0f} deg")
    row("UPDR",  f"{updraft:.2f} m/s", (255,160,0) if updraft > 0.5 else TEXT_C)

    if stall:
        surf.blit(FNT_BIG.render("!! STALL !!", True, STALL_C), (px, y))
        y += 26

    sep()

    # RL kararlar
    surf.blit(FNT_BIG.render("RL KARAR", True, (0,200,100)), (px, y)); y += 24

    max_show = min(5, len(rl_log))
    if max_show == 0:
        surf.blit(FNT_SML.render("(henuz karar yok)", True, DIM_C), (px, y))
        y += 18
    else:
        for i, entry in enumerate(reversed(rl_log[-max_show:])):
            age   = i + 1   # 1=en yeni, max_show=en eski
            fresh = age == 1

            base_col = (0, 230, 100) if fresh else (80, 140, 80)
            dim      = DIM_C        if not fresh else (160,160,160)

            # az / omega
            surf.blit(FNT_MED.render(
                f"az={entry['az_d']:+d}  w={entry['om_d']:+d}",
                True, base_col), (px, y)); y += 20

            # mu → action
            ac = entry['action']
            ac_col = POS_C if ac > 0 else NEG_C if ac < 0 else DIM_C
            surf.blit(FNT_MED.render(
                f"mu={entry['mu']:+d}  D={ac:+d} deg",
                True, ac_col), (px, y)); y += 20

            # target
            surf.blit(FNT_MED.render(
                f"hedef={entry['target']:+.1f} deg",
                True, dim), (px, y)); y += 20

            if i < max_show - 1:
                pygame.draw.line(surf, (35,35,55),
                                 (px, y+2), (x0+PANEL_W-10, y+2), 1)
                y += 8

    sep()

    # Reset butonu
    btn_rect = pygame.Rect(px, y, PANEL_W-20, 34)
    pygame.draw.rect(surf, (50, 30, 80), btn_rect, border_radius=6)
    pygame.draw.rect(surf, (120, 60, 180), btn_rect, 2, border_radius=6)
    btn_lbl = FNT_MED.render("R — Trail Sifirla", True, (200, 150, 255))
    surf.blit(btn_lbl, (btn_rect.x + btn_rect.w//2 - btn_lbl.get_width()//2,
                         btn_rect.y + 7))
    y += 42

    # Hint
    surf.blit(FNT_SML.render("+/-=zoom  ESC=cikis", True, (50,50,80)),
              (px, HEIGHT-20))

    return btn_rect   # click detection icin


# ------------------------------------------------------------------ #
#  MAIN                                                                #
# ------------------------------------------------------------------ #
def main():
    global SCALE
    pygame.init()
    init_fonts()

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Thermal Soaring — Map + RL Panel")
    clock  = pygame.time.Clock()

    trail          = []
    rl_log         = []
    last_rl_action = None
    MAX_TRAIL      = 500

    print(f"[VIZ] State: {STATE_FILE}")
    print("[VIZ] R = trail sifirla | +/- = zoom | ESC = cikis")

    while True:
        btn_rect = None

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
                    print("[VIZ] Trail sıfırlandı")
            if event.type == pygame.MOUSEBUTTONDOWN:
                if btn_rect and btn_rect.collidepoint(event.pos):
                    trail.clear()
                    print("[VIZ] Trail sıfırlandı (buton)")

        # ── state oku ──────────────────────────────────────────────
        state = {}
        if os.path.exists(STATE_FILE):
            try:
                with open(STATE_FILE) as f:
                    state = json.load(f)
            except Exception:
                pass

        last_dec = state.get("last_rl_decision")
        if last_dec and last_dec != last_rl_action:
            rl_log.append(last_dec)
            last_rl_action = last_dec
            if len(rl_log) > 50:
                rl_log.pop(0)

        # ── ciz ────────────────────────────────────────────────────
        screen.fill(BG)
        alpha_surf = pygame.Surface((MAP_W, HEIGHT), pygame.SRCALPHA)

        draw_grid(screen)
        draw_thermals(screen, alpha_surf, state.get("thermals", []))
        screen.blit(alpha_surf, (0, 0))

        ac_lat  = state.get("lat",         CENTER_LAT)
        ac_lon  = state.get("lon",         CENTER_LON)
        ac_psi  = math.radians(state.get("heading_deg", 0))
        ac_roll = state.get("roll_deg", 0)
        mode    = state.get("mode", "MANUAL")

        ax, ay = latlon_to_px(ac_lat, ac_lon)
        if 0 <= ax <= MAP_W and 0 <= ay <= HEIGHT:
            trail.append((ax, ay))
        if len(trail) > MAX_TRAIL:
            trail.pop(0)

        draw_aircraft(screen, trail, ax, ay, ac_psi, ac_roll, mode)
        draw_map_hud(screen, mode)
        btn_rect = draw_panel(screen, state, rl_log)

        pygame.display.flip()
        clock.tick(FPS)


if __name__ == "__main__":
    main()