# -*- coding: utf-8 -*-
"""
SGS -> Ranger 2400 olcek donusumu
----------------------------------
JSBSim'in hazir SGS sailplane modelini alir, Volantex Ranger 2400'un
fiziksel parametrelerine olcekler.

Aerodinamik katsayilar boyutsuz oldugu icin (CL, CD, Cm...) aynen kalir;
sadece geometri (metrics) ve kutle (mass_balance) degisir. JSBSim zaten
kuvvetleri qbar * Sw * bw seklinde hesapladigi icin olcek otomatik gecer.

Calistir:  python make_ranger.py
Cikti:     ./aircraft/ranger2400/ranger2400.xml
"""

import os
import shutil
import xml.etree.ElementTree as ET
import jsbsim

# ------------------------------------------------------------------ #
#  RANGER 2400 HEDEF PARAMETRELERI                                    #
# ------------------------------------------------------------------ #
TARGET = {
    "wingspan_m"  : 2.40,    # kanat acikligi
    "chord_m"     : 0.24,    # ortalama kord
    "length_m"    : 1.23,    # govde uzunlugu
    "mass_kg"     : 4.00,    # ucus agirligi
}
TARGET["wingarea_m2"] = TARGET["wingspan_m"] * TARGET["chord_m"]   # 0.576 m2

# Atalet momentleri (kg*m^2) — 2.4m / 4kg RC planor icin tipik degerler
# Ixx (roll)  ~ m * (b/2)^2 / 8   = 4 * 1.44 / 8   = 0.72
# Iyy (pitch) ~ m * (L/2)^2 / 3   = 4 * 0.378 / 3  = 0.50  (daha kucuk alindi)
# Izz (yaw)   ~ Ixx + Iyy
TARGET["ixx"] = 0.55
TARGET["iyy"] = 0.28
TARGET["izz"] = 0.78

M2FT  = 3.280839895
KG2LB = 2.20462262
# kg*m^2 -> slug*ft^2
KGM2_TO_SLUGFT2 = 0.737562149 / 1.0  # 1 kg*m2 = 0.7376 slug*ft2


def find_sgs():
    root = jsbsim.get_default_root_dir()
    p = os.path.join(root, "aircraft", "SGS", "SGS.xml")
    if not os.path.exists(p):
        raise FileNotFoundError(f"SGS.xml bulunamadi: {p}")
    return root, p


def get_unit(elem, default="FT"):
    return (elem.get("unit") or default).upper()


def to_meters(val, unit):
    u = unit.upper()
    if u in ("M", "METERS"):   return val
    if u in ("FT", "FEET"):    return val * 0.3048
    if u in ("IN", "INCHES"):  return val * 0.0254
    return val


def to_m2(val, unit):
    u = unit.upper()
    if u in ("M2", "M^2"):     return val
    if u in ("FT2", "FT^2"):   return val * 0.09290304
    return val


def set_len_m(elem, meters):
    """Elemani METRE cinsinden yaz, unit=M yap."""
    elem.set("unit", "M")
    elem.text = f" {meters:.5f} "


def set_area_m2(elem, m2):
    elem.set("unit", "M2")
    elem.text = f" {m2:.5f} "


def main():
    root_dir, sgs_path = find_sgs()
    print(f"[SGS] Kaynak model: {sgs_path}")

    tree = ET.parse(sgs_path)
    xml  = tree.getroot()

    # ---------------- METRICS ----------------
    metrics = xml.find("metrics")
    if metrics is None:
        raise RuntimeError("metrics blogu yok")

    # Mevcut SGS degerlerini oku (olcek faktoru icin)
    span_el  = metrics.find("wingspan")
    area_el  = metrics.find("wingarea")
    chord_el = metrics.find("chord")

    sgs_span_m = to_meters(float(span_el.text), get_unit(span_el))
    sgs_area   = to_m2(float(area_el.text),  get_unit(area_el))
    sgs_chord  = to_meters(float(chord_el.text), get_unit(chord_el))

    k = TARGET["wingspan_m"] / sgs_span_m     # geometrik olcek faktoru
    print(f"[SGS] span={sgs_span_m:.2f}m  area={sgs_area:.2f}m2  chord={sgs_chord:.2f}m")
    print(f"[OLCEK] k = {k:.4f}  (uzunluklar xk, alanlar xk^2)")

    set_len_m(span_el,  TARGET["wingspan_m"])
    set_area_m2(area_el, TARGET["wingarea_m2"])
    set_len_m(chord_el, TARGET["chord_m"])

    # Kuyruk alanlari ve kollari — k ile olcekle
    for tag, kind in [("htailarea", "area"), ("vtailarea", "area"),
                      ("htailarm",  "len"),  ("vtailarm",  "len")]:
        el = metrics.find(tag)
        if el is None:
            continue
        u = get_unit(el)
        v = float(el.text)
        if kind == "area":
            new = to_m2(v, u) * k * k
            set_area_m2(el, new)
            print(f"  {tag:10s} -> {new:.5f} m2")
        else:
            new = to_meters(v, u) * k
            set_len_m(el, new)
            print(f"  {tag:10s} -> {new:.5f} m")

    # Konumlar (AERORP, EYEPOINT, VRP) — k ile olcekle
    for loc in metrics.findall("location"):
        u = get_unit(loc)
        for ax in ("x", "y", "z"):
            e = loc.find(ax)
            if e is None:
                continue
            m = to_meters(float(e.text), u) * k
            e.text = f" {m:.5f} "
        loc.set("unit", "M")

    # ---------------- MASS BALANCE ----------------
    mb = xml.find("mass_balance")
    if mb is not None:
        for tag, val in [("ixx", TARGET["ixx"]),
                         ("iyy", TARGET["iyy"]),
                         ("izz", TARGET["izz"])]:
            el = mb.find(tag)
            if el is not None:
                el.set("unit", "KG*M2")
                el.text = f" {val:.4f} "
                print(f"  {tag:10s} -> {val:.4f} kg*m2")

        ew = mb.find("emptywt")
        if ew is not None:
            ew.set("unit", "KG")
            ew.text = f" {TARGET['mass_kg']:.3f} "
            print(f"  emptywt    -> {TARGET['mass_kg']:.2f} kg")

        # CG ve pointmass konumlari
        for loc in mb.findall(".//location"):
            u = get_unit(loc)
            for ax in ("x", "y", "z"):
                e = loc.find(ax)
                if e is None:
                    continue
                m = to_meters(float(e.text), u) * k
                e.text = f" {m:.5f} "
            loc.set("unit", "M")

        # Pilot / balast pointmass'leri kaldir (RC ucakta yok)
        for pm in list(mb.findall("pointmass")):
            mb.remove(pm)
            print(f"  pointmass '{pm.get('name')}' kaldirildi")

    # ---------------- GROUND REACTIONS ----------------
    gr = xml.find("ground_reactions")
    if gr is not None:
        for contact in gr.findall("contact"):
            loc = contact.find("location")
            if loc is not None:
                u = get_unit(loc)
                for ax in ("x", "y", "z"):
                    e = loc.find(ax)
                    if e is None:
                        continue
                    m = to_meters(float(e.text), u) * k
                    e.text = f" {m:.5f} "
                loc.set("unit", "M")
            # Yay/sonum katsayilarini kutle oraniyla olcekle
            for tag, factor in [("spring_coeff", k*k), ("damping_coeff", k*k)]:
                e = contact.find(tag)
                if e is not None:
                    try:
                        e.text = f" {float(e.text) * factor:.2f} "
                    except ValueError:
                        pass

    # ---------------- YAZ ----------------
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "aircraft", "ranger2400")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "ranger2400.xml")

    # fdm_config name degistir
    xml.set("name", "ranger2400")
    fh = xml.find("fileheader")
    if fh is not None:
        d = fh.find("description")
        if d is not None:
            d.text = ("Volantex Ranger 2400 - SGS aerodinamiginden olceklendi "
                      f"(span {TARGET['wingspan_m']}m, {TARGET['mass_kg']}kg)")

    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    print()
    print(f"[OK] Yazildi: {out_path}")

    # SGS'nin systems/engine klasorleri varsa kopyala
    sgs_dir = os.path.dirname(sgs_path)
    for sub in ("Systems", "Engines"):
        src = os.path.join(sgs_dir, sub)
        if os.path.isdir(src):
            dst = os.path.join(out_dir, sub)
            if not os.path.exists(dst):
                shutil.copytree(src, dst)
                print(f"[OK] {sub}/ kopyalandi")

    # ---------------- DOGRULAMA ----------------
    print()
    print("=== DOGRULAMA ===")
    proj_root = os.path.dirname(os.path.abspath(__file__))
    fdm = jsbsim.FGFDMExec(proj_root)
    fdm.set_debug_level(0)
    ok = fdm.load_model("ranger2400")
    if not ok:
        print("[HATA] Model yuklenemedi!")
        return

    fdm['ic/h-sl-ft']   = 1000.0
    fdm['ic/vt-kts']    = 30.0
    fdm['ic/theta-deg'] = 0.0
    fdm.run_ic()

    print(f"  wingspan : {fdm['metrics/bw-ft']*0.3048:.3f} m")
    print(f"  wingarea : {fdm['metrics/Sw-sqft']*0.09290304:.4f} m2")
    print(f"  chord    : {fdm['metrics/cbarw-ft']*0.3048:.3f} m")
    print(f"  weight   : {fdm['inertia/weight-lbs']/2.20462:.3f} kg")
    print()

    # Trim elevator tarama
    print("  Trim taramasi (300 adim, h-dot ~ 0 arananiyor):")
    import math
    for elev in [-0.30, -0.25, -0.20, -0.15, -0.10, -0.05, 0.0]:
        f2 = jsbsim.FGFDMExec(proj_root)
        f2.set_debug_level(0)
        f2.load_model("ranger2400")
        f2['ic/h-sl-ft']   = 1000.0
        f2['ic/vt-kts']    = 30.0
        f2['ic/theta-deg'] = 0.0
        f2.run_ic()
        for _ in range(400):
            f2['fcs/elevator-cmd-norm'] = elev
            f2.run()
        hd    = f2['velocities/h-dot-fps'] * 0.3048
        pitch = math.degrees(f2['attitude/pitch-rad'])
        spd   = f2['velocities/vt-fps'] * 0.3048
        print(f"    elev={elev:+.2f}  pitch={pitch:+6.1f}deg  "
              f"h-dot={hd:+6.2f}m/s  spd={spd:5.1f}m/s")

    print()
    print("main.py'de kullanmak icin:")
    print("  fdm = jsbsim.FGFDMExec(os.path.dirname(os.path.abspath(__file__)))")
    print("  fdm.load_model('ranger2400')")


if __name__ == "__main__":
    main()
