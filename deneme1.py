#!/usr/bin/env python3
"""
Otonom Mühimmat Dağıtım Görevi
Sakarya Uygulamalı Bilimler Üniversitesi - İHA Yarışması
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

# ╔══════════════════════════════════════════════════════════════╗
# ║                      YAPILANDIRMA                           ║
# ╠══════════════════════════════════════════════════════════════╣
# ║  Tüm ayarlar buradan yapılır, başka bir yere dokunma.       ║
# ╚══════════════════════════════════════════════════════════════╝

# --- Bağlantı ---
BAGLANTI_ADRESI     = "udp:127.0.0.1:14550"    # Gazebo SITL adresi

# --- Uçuş ---
GOREV_IRTIFASI      = 10.0      # metre   | uçuş irtifası
KALKIS_BEKLEME      = 2.0       # saniye  | takeoff sonrası stabilizasyon süresi
WAYPOINT_YAKINLIK   = 1.0       # metre   | hedefe ulaşıldı sayılma eşiği
HOME_YAKINLIK       = 1.5       # metre   | home'ye ulaşıldı sayılma eşiği

# --- İniş ---
INIS_ADIM           = 1.0       # metre   | her adımda bu kadar in (hızlı faz)
INIS_ADIM_TOLERANS  = 0.5       # metre   | adım irtifasına bu kadar yaklaşınca sonraki adıma geç
LAND_BASLANGIC      = 2.0       # metre   | bu irtifanın altında LAND moduna geç
LAND_SPEED          = 50        # cm/s    | LAND modunda iniş hızı
INIS_KONTROL_HIZI   = 0.05      # saniye  | iniş döngüsü polling hızı

# --- Görev ---
HEDEFTE_BEKLEME     = 3.0       # saniye  | mühimmat bırakma bekleme süresi
BATARYA_KRITIK      = 20        # %       | bu seviyenin altında acil RTH

# --- Hedef Koordinatlar ---
# Rota: home -> hedef_1 -> home -> hedef_2 -> home -> ...
# (enlem, boylam) formatında, istediğin kadar ekle veya çıkar
HEDEF_KOORDINATLAR = [
    (-35.3635567, 149.1649536),   # hedef_1
    (-35.3635558, 149.1651442),   # hedef_2
]

# ══════════════════════════════════════════════════════════════
#  BURADAN AŞAĞI DOKUNMA
# ══════════════════════════════════════════════════════════════


# ─────────────────────────────────────────────
#  YARDIMCI FONKSİYONLAR
# ─────────────────────────────────────────────

def mesafe_hesapla(konum1, konum2):
    """İki GPS koordinatı arasındaki yatay mesafeyi hesaplar (metre)."""
    if hasattr(konum1, 'lat'):
        lat1, lon1 = math.radians(konum1.lat), math.radians(konum1.lon)
        lat2, lon2 = math.radians(konum2.lat), math.radians(konum2.lon)
    else:
        lat1, lon1 = math.radians(konum1[0]), math.radians(konum1[1])
        lat2, lon2 = math.radians(konum2[0]), math.radians(konum2[1])

    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 6371000 * 2 * math.asin(math.sqrt(a))


def batarya_kontrol(drone):
    """Batarya seviyesi kritik eşiğin altındaysa True döner."""
    seviye = drone.battery.level
    if seviye is not None and seviye < BATARYA_KRITIK:
        print(f"[BATARYA] Kritik seviye: %{seviye} — görev durduruluyor!")
        return True
    return False


# ─────────────────────────────────────────────
#  GÖREV FONKSİYONLARI
# ─────────────────────────────────────────────

def kalkis(drone, irtifa):
    """
    Drone'u arm edip belirtilen irtifaya kaldırır.
    Zaten havadaysa tekrar arm etmez.
    """
    drone.mode = VehicleMode("GUIDED")
    while drone.mode.name != "GUIDED":
        time.sleep(0.3)

    mevcut_irtifa = drone.location.global_relative_frame.alt or 0.0

    if mevcut_irtifa > 1.0:
        print(f"[KALKIŞ] Zaten havada ({mevcut_irtifa:.1f}m) — {irtifa}m'ye çıkılıyor...")
        drone.simple_goto(LocationGlobalRelative(
            drone.location.global_relative_frame.lat,
            drone.location.global_relative_frame.lon,
            irtifa
        ))
        baslangic = time.time()
        while time.time() - baslangic < 30:
            if (drone.location.global_relative_frame.alt or 0.0) >= irtifa * 0.95:
                break
            time.sleep(0.2)
    else:
        print(f"[KALKIŞ] Arm ediliyor...")
        drone.armed = True
        while not drone.armed:
            print("  Arm bekleniyor...")
            time.sleep(1)

        print(f"[KALKIŞ] {irtifa}m irtifaya çıkılıyor...")
        drone.simple_takeoff(irtifa)

        baslangic = time.time()
        while time.time() - baslangic < 30:
            if (drone.location.global_relative_frame.alt or 0.0) >= irtifa * 0.95:
                break
            time.sleep(0.2)

    time.sleep(KALKIS_BEKLEME)
    print(f"[KALKIŞ] Tamamlandı — {drone.location.global_relative_frame.alt:.1f}m")


def hedefe_git(drone, lat, lon, irtifa, yakinlik_esigi=None):
    """
    Belirtilen GPS koordinatına gider.
    Gidilirken anlık mesafe ve irtifa terminale yazılır.
    """
    if yakinlik_esigi is None:
        yakinlik_esigi = WAYPOINT_YAKINLIK

    drone.simple_goto(LocationGlobalRelative(lat, lon, irtifa))
    print(f"[NAVİGASYON] Hedefe gidiliyor: ({lat:.7f}, {lon:.7f})")

    while True:
        mevcut    = drone.location.global_relative_frame
        mesafe    = mesafe_hesapla((mevcut.lat, mevcut.lon), (lat, lon))
        irtifa_an = mevcut.alt or 0.0
        print(f"  Mesafe: {mesafe:.1f}m | İrtifa: {irtifa_an:.1f}m", end="\r")

        if mesafe < yakinlik_esigi:
            print(f"\n[NAVİGASYON] Hedefe ulaşıldı — {mesafe:.2f}m")
            break
        time.sleep(0.3)


def inis(drone, lat, lon):
    """
    İki aşamalı iniş:
      1) Hızlı faz: GUIDED modda adım adım simple_goto ile LAND_BASLANGIC'a kadar in
      2) Yavaş faz: LAND moduna geç, otomatik disarm bekle
    İniş süresi ve anlık irtifa terminale yazılır.
    """
    print(f"[İNİŞ] Başlıyor — ({lat:.7f}, {lon:.7f})")
    inis_baslangic = time.time()

    # Hızlı faz — adım adım in
    mevcut_irtifa = drone.location.global_relative_frame.alt or 0.0

    while mevcut_irtifa > LAND_BASLANGIC:
        hedef_irtifa = max(mevcut_irtifa - INIS_ADIM, LAND_BASLANGIC)
        drone.simple_goto(LocationGlobalRelative(lat, lon, hedef_irtifa))

        # Bu adım irtifasına ulaşana kadar bekle
        while True:
            mevcut_irtifa = drone.location.global_relative_frame.alt or 0.0
            gecen         = time.time() - inis_baslangic
            print(f"  [Hızlı iniş] İrtifa: {mevcut_irtifa:.2f}m | Süre: {gecen:.1f}s", end="\r")

            if mevcut_irtifa <= hedef_irtifa + INIS_ADIM_TOLERANS:
                break
            time.sleep(INIS_KONTROL_HIZI)

    # Yavaş faz — LAND moduna geç
    gecen = time.time() - inis_baslangic
    print(f"\n[İNİŞ] {mevcut_irtifa:.1f}m — LAND moduna geçiliyor... ({gecen:.1f}s)")

    drone.parameters['LAND_SPEED'] = LAND_SPEED
    drone.mode = VehicleMode("LAND")

    while drone.mode.name != "LAND":
        time.sleep(0.05)

    while drone.armed:
        mevcut_irtifa = drone.location.global_relative_frame.alt or 0.0
        gecen         = time.time() - inis_baslangic
        print(f"  [LAND modu]  İrtifa: {mevcut_irtifa:.2f}m | Süre: {gecen:.1f}s", end="\r")
        time.sleep(INIS_KONTROL_HIZI)

    toplam_sure = time.time() - inis_baslangic
    print(f"\n[İNİŞ] Tamamlandı — toplam iniş süresi: {toplam_sure:.1f}s")


def home_don_ve_in(drone, home_lat, home_lon):
    """Home noktasına döner ve iner."""
    print("[HOME] Dönülüyor...")
    hedefe_git(drone, home_lat, home_lon, GOREV_IRTIFASI, yakinlik_esigi=HOME_YAKINLIK)
    print("[HOME] Üzerinde — iniş yapılıyor...")
    inis(drone, home_lat, home_lon)


# ─────────────────────────────────────────────
#  ANA GÖREV DÖNGÜSÜ
# ─────────────────────────────────────────────

def gorevi_baslat():
    print("=" * 50)
    print("  OTONOM MÜH. DAĞITIM GÖREVİ BAŞLIYOR")
    print("=" * 50)

    # Bağlantı
    print(f"[BAĞLANTI] {BAGLANTI_ADRESI} adresine bağlanılıyor...")
    drone = connect(BAGLANTI_ADRESI, wait_ready=True, baud=57600)
    print(f"[BAĞLANTI] Bağlandı")
    print(f"  Mod     : {drone.mode.name}")
    print(f"  GPS     : {drone.gps_0.fix_type} fix, {drone.gps_0.satellites_visible} uydu")
    print(f"  Batarya : %{drone.battery.level}")

    # GPS fix bekle ve home konumunu al
    print("[HOME] GPS fix bekleniyor...")
    while drone.gps_0.fix_type < 2:
        print(f"  Fix type: {drone.gps_0.fix_type} — bekleniyor...")
        time.sleep(1)

    home_lat = drone.location.global_relative_frame.lat
    home_lon = drone.location.global_relative_frame.lon
    print(f"[HOME] Kaydedildi: ({home_lat:.7f}, {home_lon:.7f})")

    # Görev başlangıç zamanı
    gorev_baslangic  = time.time()
    tamamlanan_hedef = 0

    # Rota: home -> hedef_1 -> home -> hedef_2 -> home -> ...
    for i, (lat, lon) in enumerate(HEDEF_KOORDINATLAR):

        if batarya_kontrol(drone):
            break

        print(f"\n{'─' * 40}")
        print(f"  HEDEF #{i + 1} — ({lat:.7f}, {lon:.7f})")
        print(f"{'─' * 40}")

        hedef_baslangic = time.time()

        # Home'den kalkış
        kalkis(drone, GOREV_IRTIFASI)

        # Hedefe git
        hedefe_git(drone, lat, lon, GOREV_IRTIFASI)

        # Hedefte in
        inis(drone, lat, lon)

        # Mühimmat bırak
        print(f"[BEKLEME] {HEDEFTE_BEKLEME}s — mühimmat bırakılıyor...")
        time.sleep(HEDEFTE_BEKLEME)

        # Home'e dön ve in
        kalkis(drone, GOREV_IRTIFASI)
        home_don_ve_in(drone, home_lat, home_lon)

        tamamlanan_hedef += 1
        hedef_sure = time.time() - hedef_baslangic
        print(f"\n[HEDEF #{i + 1}] Tamamlandı ✓ | Süre: {hedef_sure:.1f}s")

        if batarya_kontrol(drone):
            break

    # Görev bitti
    toplam_sure = time.time() - gorev_baslangic
    print(f"\n{'=' * 50}")
    print(f"  GÖREV TAMAMLANDI")
    print(f"  Ziyaret edilen hedef : {tamamlanan_hedef}/{len(HEDEF_KOORDINATLAR)}")
    print(f"  Toplam görev süresi  : {toplam_sure:.1f}s")
    print(f"  Son batarya          : %{drone.battery.level}")
    print(f"{'=' * 50}")

    drone.close()


if __name__ == "__main__":
    gorevi_baslat()