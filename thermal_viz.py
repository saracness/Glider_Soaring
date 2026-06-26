"""
Thermal Soaring - 2D Top-Down Visualizer
-----------------------------------------
main.py ile aynı anda çalıştır:
    Terminal 1: python main.py
    Terminal 2: python thermal_viz.py

Shared memory (file) üzerinden state okur.
main.py state.json yazar, viz okur.
"""

import pygame
import math
import json
import os
import time

# ------------------------------------------------------------------ #
#  CONFIG                                                              #
# ------------------------------------------------------------------ #
WIDTH, HEIGHT = 800, 800
FPS           = 30
STATE_FILE    = "/tmp/soaring_state.json"

# Ankara merkez
CENTER_LAT =  39.9483187
CENTER_LON =  32.6899477

# Görüntü ölçeği: kaç metre = 1 piksel
SCALE = 5.0   # 5m/px → 800px = 4km görüş alanı

# Renkler
BG_COLOR        = (15, 15, 25)
GRID_COLOR      = (30, 30, 50)
THERMAL_COLORS  = [
    (255, 80,  20),   # güçlü — turuncu/kırmızı
    (255, 160,  0),   # orta  — sarı
    (100, 200, 255),  # zayıf — mavi
]
AIRCRAFT_COLOR  = (0, 255, 120)
TRAIL_COLOR     = (0, 180, 80)
TEXT_COLOR      = (220, 220, 220)
MODE_COLORS     = {
    "MANUAL"    : (100, 180, 255),
    "RL_SOARING": (0,   255, 120),
    "TAKEOFF"   : (255, 200,   0),
}


# ------------------------------------------------------------------ #
#  COORDINATE HELPERS                                                  #
# ------------------------------------------------------------------ #
def latlon_to_px(lat, lon, center_lat, center_lon, scale, w, h):
    """Convert lat/lon to screen pixel (center of screen = center coords)."""
    dx = (lon - center_lon) * 111320.0 * math.cos(math.radians(center_lat))
    dy = (lat - center_lat) * 110540.0
    px = int(w / 2 + dx / scale)
    py = int(h / 2 - dy / scale)   # y inverted on screen
    return px, py


def meters_to_px(meters, scale):
    return max(1, int(meters / scale))


# ------------------------------------------------------------------ #
#  DRAW FUNCTIONS                                                      #
# ------------------------------------------------------------------ #
def draw_grid(surf, center_lat, center_lon, scale, w, h):
    """Draw km grid lines."""
    grid_spacing_m = 500   # 500m grid
    grid_px = meters_to_px(grid_spacing_m, scale)
    font_small = pygame.font.SysFont("monospace", 11)

    for i in range(-10, 11):
        x = w // 2 + i * grid_px
        y = h // 2 + i * grid_px
        pygame.draw.line(surf, GRID_COLOR, (x, 0), (x, h), 1)
        pygame.draw.line(surf, GRID_COLOR, (0, y), (w, y), 1)
        if i != 0:
            label = font_small.render(f"{i*500}m", True, (50, 50, 80))
            surf.blit(label, (x + 2, h // 2 + 2))


def draw_thermal(surf, thermal, center_lat, center_lon, scale, w, h, alpha_surf):
    """Draw a thermal as concentric transparent circles."""
    cx, cy = latlon_to_px(
        thermal["lat"], thermal["lon"],
        center_lat, center_lon, scale, w, h)

    strength = thermal["strength_ms"]
    radius_m = thermal["radius_m"]
    height_m = thermal["height_m"]
    r_px     = meters_to_px(radius_m * 2.5, scale)   # show 2.5σ radius

    # Color based on strength
    if strength >= 4.0:
        base_color = THERMAL_COLORS[0]
    elif strength >= 2.5:
        base_color = THERMAL_COLORS[1]
    else:
        base_color = THERMAL_COLORS[2]

    # Draw concentric rings (transparent)
    for ring in range(5, 0, -1):
        ring_r   = int(r_px * ring / 5)
        alpha    = int(60 * (1 - ring / 6))
        color    = (*base_color, alpha)
        ring_surf = pygame.Surface((ring_r * 2, ring_r * 2), pygame.SRCALPHA)
        pygame.draw.circle(ring_surf, color, (ring_r, ring_r), ring_r)
        alpha_surf.blit(ring_surf, (cx - ring_r, cy - ring_r))

    # Core circle (solid)
    core_r = max(3, meters_to_px(radius_m * 0.3, scale))
    pygame.draw.circle(surf, base_color, (cx, cy), core_r)
    pygame.draw.circle(surf, (255, 255, 255), (cx, cy), core_r, 1)

    # Updraft arrows (↑)
    font = pygame.font.SysFont("monospace", 14, bold=True)
    arrow_surf = font.render("↑", True, base_color)
    surf.blit(arrow_surf, (cx - 5, cy - core_r - 18))

    # Label
    font_s = pygame.font.SysFont("monospace", 11)
    label  = font_s.render(f"{strength:.1f}m/s  {int(height_m)}m", True, base_color)
    surf.blit(label, (cx + core_r + 4, cy - 8))


def draw_aircraft(surf, trail, ax, ay, heading_rad, roll_deg, mode):
    color = MODE_COLORS.get(mode, AIRCRAFT_COLOR)

    # Trail
    if len(trail) > 1:
        for i in range(1, len(trail)):
            alpha = int(200 * i / len(trail))
            c = (0, max(0, 180 - (len(trail)-i)*3), max(0, 80 - (len(trail)-i)*2))
            pygame.draw.line(surf, c, trail[i-1], trail[i], 1)

    # Aircraft triangle (top-down)
    size = 12
    pts = [
        (ax + size * math.sin(heading_rad),
         ay - size * math.cos(heading_rad)),
        (ax + size * 0.5 * math.sin(heading_rad + 2.4),
         ay - size * 0.5 * math.cos(heading_rad + 2.4)),
        (ax + size * 0.5 * math.sin(heading_rad - 2.4),
         ay - size * 0.5 * math.cos(heading_rad - 2.4)),
    ]
    pygame.draw.polygon(surf, color, pts)
    pygame.draw.polygon(surf, (255, 255, 255), pts, 1)

    # Roll indicator (small arc)
    roll_r = 20
    pygame.draw.arc(surf,
                    (255, 200, 0),
                    (ax - roll_r, ay - roll_r, roll_r*2, roll_r*2),
                    math.radians(90 - roll_deg - 15),
                    math.radians(90 - roll_deg + 15), 3)


def draw_hud(surf, state, w, h):
    font   = pygame.font.SysFont("monospace", 14)
    font_b = pygame.font.SysFont("monospace", 16, bold=True)

    mode    = state.get("mode", "---")
    alt     = state.get("alt_m", 0)
    speed   = state.get("speed_ms", 0)
    climb   = state.get("climb_ms", 0)
    roll    = state.get("roll_deg", 0)
    updraft = state.get("updraft_ms", 0)

    mode_color = MODE_COLORS.get(mode, TEXT_COLOR)

    lines = [
        (f"MOD   : {mode}", mode_color),
        (f"ALT   : {alt:.1f} m", TEXT_COLOR),
        (f"HIZ   : {speed:.1f} m/s", TEXT_COLOR),
        (f"TIRMAN: {climb:+.2f} m/s", (0, 255, 100) if climb > 0 else (255, 100, 100)),
        (f"ROLL  : {roll:+.1f}°", TEXT_COLOR),
        (f"TERMAL: {updraft:.2f} m/s", (255, 160, 0) if updraft > 0.5 else TEXT_COLOR),
    ]

    # HUD box
    box_w, box_h = 210, len(lines) * 22 + 16
    box = pygame.Surface((box_w, box_h), pygame.SRCALPHA)
    box.fill((0, 0, 0, 160))
    surf.blit(box, (10, 10))

    for i, (text, color) in enumerate(lines):
        label = font.render(text, True, color)
        surf.blit(label, (18, 18 + i * 22))

    # Mode badge
    badge = font_b.render(f"[ {mode} ]", True, mode_color)
    surf.blit(badge, (w // 2 - badge.get_width() // 2, 10))

    # Scale bar
    bar_m  = 500
    bar_px = meters_to_px(bar_m, SCALE)
    bx, by = w - bar_px - 20, h - 30
    pygame.draw.line(surf, TEXT_COLOR, (bx, by), (bx + bar_px, by), 2)
    pygame.draw.line(surf, TEXT_COLOR, (bx, by - 4), (bx, by + 4), 2)
    pygame.draw.line(surf, TEXT_COLOR, (bx + bar_px, by - 4), (bx + bar_px, by + 4), 2)
    scale_label = font.render("500 m", True, TEXT_COLOR)
    surf.blit(scale_label, (bx + bar_px // 2 - 20, by - 20))


# ------------------------------------------------------------------ #
#  MAIN VIZ LOOP                                                       #
# ------------------------------------------------------------------ #
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Thermal Soaring — Top-Down View")
    clock  = pygame.time.Clock()

    trail  = []
    MAX_TRAIL = 300

    print("[VIZ] Başlatıldı. main.py çalışıyor olmalı.")
    print(f"[VIZ] State dosyası: {STATE_FILE}")

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    return
                if event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    global SCALE
                    SCALE = max(1.0, SCALE * 0.8)
                if event.key == pygame.K_MINUS:
                    SCALE = min(50.0, SCALE * 1.25)

        # --- state oku ---
        state = {}
        if os.path.exists(STATE_FILE):
            try:
                with open(STATE_FILE) as f:
                    state = json.load(f)
            except Exception:
                pass

        # --- draw ---
        screen.fill(BG_COLOR)

        # alpha surface for transparent thermals
        alpha_surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)

        draw_grid(screen, CENTER_LAT, CENTER_LON, SCALE, WIDTH, HEIGHT)

        # thermals
        thermals = state.get("thermals", [])
        for t in thermals:
            draw_thermal(screen, t, CENTER_LAT, CENTER_LON,
                         SCALE, WIDTH, HEIGHT, alpha_surf)

        screen.blit(alpha_surf, (0, 0))

        # aircraft
        ac_lat = state.get("lat", CENTER_LAT)
        ac_lon = state.get("lon", CENTER_LON)
        ac_psi = math.radians(state.get("heading_deg", 0))
        ac_roll = state.get("roll_deg", 0)
        mode    = state.get("mode", "MANUAL")

        ax, ay = latlon_to_px(ac_lat, ac_lon,
                              CENTER_LAT, CENTER_LON,
                              SCALE, WIDTH, HEIGHT)

        trail.append((ax, ay))
        if len(trail) > MAX_TRAIL:
            trail.pop(0)

        draw_aircraft(screen, trail, ax, ay, ac_psi, ac_roll, mode)

        # HUD
        draw_hud(screen, state, WIDTH, HEIGHT)

        # info
        font_s = pygame.font.SysFont("monospace", 11)
        hint = font_s.render("+/- zoom   ESC quit", True, (60, 60, 90))
        screen.blit(hint, (WIDTH - 160, HEIGHT - 18))

        pygame.display.flip()
        clock.tick(FPS)


if __name__ == "__main__":
    main()