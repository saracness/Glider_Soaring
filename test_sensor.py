# -*- coding: utf-8 -*-
"""
KOK NEDEN DUZELTMESI: yanlis ruzgar property adi
=================================================
wind_test.py kaniti:

  atmosphere/wind-d-fps      ->  property YOK (JSBSim sessizce yok sayiyor)
  atmosphere/wind-down-fps   ->  ETKILI

  Zaman serisi:
    rüzgarsiz       alt 1366 -> 1356   (bati)
    UPDRAFT 3 m/s   alt 1356 -> 1391   (TIRMANIS +35 m)
    rüzgarsiz       alt 1391 -> 1387   (yine bati)

Bu yuzden tum oturum boyunca termaller ucagi hic etkilemedi:
  - az sinyali sadece phugoid gurultusuydu
  - ucak hicbir termalde tirmanamadi
  - 'Updr: 7.87' sadece bir sayiydi, fizige girmiyordu

Calistir: python fix_wind.py
"""
import io

p = 'main.py'
s = io.open(p, encoding='utf-8').read()

n = s.count("atmosphere/wind-d-fps")
if n == 0:
    print("[--] 'atmosphere/wind-d-fps' bulunamadi (zaten duzeltilmis olabilir)")
else:
    s = s.replace("atmosphere/wind-d-fps", "atmosphere/wind-down-fps")
    print(f"[OK] {n} yerde  wind-d-fps -> wind-down-fps")

# Guvenlik: dogru yazildigini bir kez daha kontrol et
if "atmosphere/wind-down-fps" in s:
    print("[OK] wind-down-fps kodda mevcut")

io.open(p, 'w', encoding='utf-8').write(s)

import py_compile
try:
    py_compile.compile(p, doraise=True)
    print("[OK] Syntax OK")
except Exception as e:
    print("SYNTAX HATASI:", e)

print()
print("=" * 58)
print("Artik termaller GERCEKTEN ucagi kaldiracak.")
print("Calistir: python main.py  ->  2 tusu ile RL moduna gec")
print("=" * 58)