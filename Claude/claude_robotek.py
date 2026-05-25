from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import json
from pymavlink import mavutil

BAGLANTI_ADRESI = "udp:127.0.0.1:14550"

HEDEF_YUKSEKLIK = 5
HEDEF_MESAFE = 2.0          # 🟠 1.0 → 2.0 | Gerçek GPS CEP toleransı
STABIL_MESAFE = 1.5         # 🟠 0.8 → 1.5 | GPS noise toleransı
STABIL_HIZ = 0.6            # 🟠 0.4 → 0.6 | Bounce'a karşı tolerans
STABIL_SURE = 2.5           # 🟠 1.5 → 2.5 | GPS bounce'dan korunmak için

NORMAL_HIZ = 3
YAVAS_HIZ = 1
YAVASLAMA_MESAFESI = 5

HEDEFTE_BEKLEME_SURESI = 3  # 🔴 EKLENDİ | Şartname: iniş sonrası sabit kalma süresi
BATARYA_KRITIK_SEVIYE = 20  # 🟠 EKLENDİ | %20 altında güvenli iniş


# ─────────────────────────────────────────────
# 🔴 ADIM 1: Home'u runtime'da al, hardcoded kullanma
# ─────────────────────────────────────────────
def home_konumunu_kaydet(vehicle):
    print("Home noktası kaydediliyor...")
    konum = vehicle.location.global_relative_frame

    if konum.lat is None or konum.lon is None:
        print("HATA: Home konumu alınamadı, GPS verisi yok.")
        return None

    home = {"isim": "Home", "lat": konum.lat, "lon": konum.lon}
    print(f"Home kaydedildi → Lat: {home['lat']:.8f} | Lon: {home['lon']:.8f}")
    return home


# ─────────────────────────────────────────────
# 🟢 ADIM 4: Koordinatları dosyadan oku
# ─────────────────────────────────────────────
def gorev_noktalarini_yukle(dosya="gorev_noktalari.json"):
    try:
        with open(dosya) as f:
            noktalar = json.load(f)
        print(f"{len(noktalar)} görev noktası yüklendi: {dosya}")
        return noktalar
    except FileNotFoundError:
        print(f"UYARI: {dosya} bulunamadı. Varsayılan test noktası kullanılıyor.")
        return [
            {"isim": "Hedef_1", "lat": -35.36330975, "lon": 149.16497537}
        ]
    except json.JSONDecodeError as e:
        print(f"HATA: JSON parse hatası → {e}")
        return []


def mesafe_hesapla_metre(konum1, konum2):
    dlat = konum2.lat - konum1.lat
    dlon = konum2.lon - konum1.lon
    ortalama_lat = math.radians((konum1.lat + konum2.lat) / 2)
    metre_lat = dlat * 111320
    metre_lon = dlon * 111320 * math.cos(ortalama_lat)
    return math.sqrt(metre_lat ** 2 + metre_lon ** 2)


# ─────────────────────────────────────────────
# 🟠 ADIM 2: Batarya izleme
# ─────────────────────────────────────────────
def batarya_kontrol_et(vehicle, kritik_seviye=BATARYA_KRITIK_SEVIYE):
    seviye = vehicle.battery.level
    voltaj = vehicle.battery.voltage

    if seviye is None:
        print("UYARI: Batarya seviyesi okunamadı.")
        return True  # Bilgi yoksa bloklama, uyar geç

    print(f"Batarya: %{seviye} | {voltaj:.2f}V")

    if seviye < kritik_seviye:
        print(f"KRİTİK: Batarya %{seviye} → Güvenli iniş tetikleniyor!")
        return False

    return True


def konum_hazir_mi(vehicle):
    print("GPS ve EKF kontrol ediliyor...")
    baslangic_zamani = time.time()

    while True:
        konum = vehicle.location.global_relative_frame
        gps = vehicle.gps_0
        lat_lon_var = konum.lat is not None and konum.lon is not None
        gps_fix_var = gps.fix_type is not None and gps.fix_type >= 3
        ekf_hazir = vehicle.ekf_ok

        print(
            f"GPS Fix: {gps.fix_type} | "
            f"EKF: {ekf_hazir} | "
            f"Konum: {konum.lat}, {konum.lon}"
        )

        if lat_lon_var and gps_fix_var and ekf_hazir:
            print("GPS ve EKF hazır.")
            return True

        if time.time() - baslangic_zamani > 30:
            print("GPS veya EKF hazır olmadı.")
            return False

        time.sleep(1)


def arm_ve_takeoff(vehicle, hedef_yukseklik):
    print(f"{hedef_yukseklik} metre kalkış komutu geldi. Drone ARM ediliyor...")

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print("GUIDED mod bekleniyor...")
        time.sleep(1)

    while not vehicle.is_armable:
        print("Drone henüz ARM edilemez.")
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print("ARM olması bekleniyor...")
        vehicle.armed = True
        time.sleep(1)

    print("ARM edildi.")
    print("Mod:", vehicle.mode.name)
    print("Armed:", vehicle.armed)
    print("Konum:", vehicle.location.global_relative_frame)

    print(f"{hedef_yukseklik} metreye kalkış başlıyor...")
    vehicle.simple_takeoff(hedef_yukseklik)

    baslangic_zamani = time.time()

    while True:
        yukseklik = vehicle.location.global_relative_frame.alt

        print(
            f"Anlık yükseklik: {yukseklik:.2f} m | "
            f"Mod: {vehicle.mode.name} | "
            f"ARM: {vehicle.armed}"
        )

        if yukseklik >= hedef_yukseklik * 0.90:
            print("Hedef yüksekliğe yeterince ulaşıldı...")
            return True

        if time.time() - baslangic_zamani > 30:
            if yukseklik >= hedef_yukseklik * 0.85:
                print("Drone hedef yüksekliğe yeterince yaklaştı. Göreve devam ediliyor...")
                return True
            print("Takeoff zaman aşımına girdi. Drone hedef yüksekliğe ulaşamadı.")
            return False

        time.sleep(1)


def hedefe_git(vehicle, hedef_lat, hedef_lon, hedef_yukseklik, hedef_mesafe=HEDEF_MESAFE, zaman_asimi=60):
    print("Hedefe gitme komutu geldi...")

    if vehicle.mode.name != "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.mode.name != "GUIDED":
            print("GUIDED mod bekleniyor...")
            time.sleep(1)

    hedef_konum = LocationGlobalRelative(hedef_lat, hedef_lon, hedef_yukseklik)
    print(f"Hedef konum: Lat={hedef_lat}, Lon={hedef_lon}, Alt={hedef_yukseklik}")

    # 🟠 Dinamik timeout: mesafeye göre hesapla
    mevcut = vehicle.location.global_relative_frame
    tahmini_mesafe = mesafe_hesapla_metre(mevcut, hedef_konum)
    dinamik_timeout = max(zaman_asimi, tahmini_mesafe / NORMAL_HIZ * 2.5)
    print(f"Tahmini mesafe: {tahmini_mesafe:.1f}m | Timeout: {dinamik_timeout:.0f}s")

    vehicle.simple_goto(hedef_konum, groundspeed=NORMAL_HIZ)

    baslangic_zamani = time.time()
    yavas_mod = False

    while True:
        # 🟡 EKF uçuş sırasında izleme
        if not vehicle.ekf_ok:
            print("UYARI: EKF kaybedildi! Güvenli iniş yapılıyor.")
            return False

        # 🟠 Batarya kontrolü
        if not batarya_kontrol_et(vehicle):
            return False

        mevcut_konum = vehicle.location.global_relative_frame
        kalan_mesafe = mesafe_hesapla_metre(mevcut_konum, hedef_konum)

        if kalan_mesafe <= YAVASLAMA_MESAFESI and not yavas_mod:
            print("Hedefe yaklaşıldı. Drone yavaşlatılıyor...")
            vehicle.simple_goto(hedef_konum, groundspeed=YAVAS_HIZ)
            yavas_mod = True

        print(
            f"Kalan mesafe: {kalan_mesafe:.2f} m | "
            f"Konum: {mevcut_konum.lat:.8f}, {mevcut_konum.lon:.8f} | "
            f"Yükseklik: {mevcut_konum.alt:.2f} m | "
            f"Mod: {vehicle.mode.name}"
        )

        if kalan_mesafe <= hedef_mesafe:
            print("Hedef konuma ulaşıldı.")
            return True

        if time.time() - baslangic_zamani > dinamik_timeout:
            print("Hedefe gitme zaman aşımına girdi.")
            return False

        if vehicle.mode.name != "GUIDED":
            print("Drone GUIDED moddan çıktı. Görev iptal.")
            return False

        time.sleep(1)


def hedef_ustunde_stabil_ol(vehicle, hedef_lat, hedef_lon, hedef_yukseklik):
    print("Hedef üstünde stabilizasyon başladı...")

    hedef_konum = LocationGlobalRelative(hedef_lat, hedef_lon, hedef_yukseklik)
    vehicle.simple_goto(hedef_konum, groundspeed=YAVAS_HIZ)

    stabil_baslangic = None
    baslangic_zamani = time.time()

    while True:
        # 🟡 EKF stabilizasyon sırasında izleme
        if not vehicle.ekf_ok:
            print("UYARI: Stabilizasyon sırasında EKF kaybı!")
            return False

        mevcut_konum = vehicle.location.global_relative_frame
        kalan_mesafe = mesafe_hesapla_metre(mevcut_konum, hedef_konum)
        anlik_hiz = vehicle.groundspeed if vehicle.groundspeed is not None else 99

        print(
            f"Stabil kontrol | "
            f"Mesafe: {kalan_mesafe:.2f} m | "
            f"Hız: {anlik_hiz:.2f} m/s"
        )

        if kalan_mesafe <= STABIL_MESAFE and anlik_hiz <= STABIL_HIZ:
            if stabil_baslangic is None:
                stabil_baslangic = time.time()
            if time.time() - stabil_baslangic >= STABIL_SURE:
                print("Drone hedef üstünde stabil.")
                return True
        else:
            stabil_baslangic = None
            vehicle.simple_goto(hedef_konum, groundspeed=YAVAS_HIZ)

        if time.time() - baslangic_zamani > 30:
            print("Stabilizasyon zaman aşımına girdi.")
            return False

        if vehicle.mode.name != "GUIDED":
            print("Drone GUIDED moddan çıktı. Stabilizasyon iptal.")
            return False

        time.sleep(0.5)


def hiz_komutu_gonder(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)


# ─────────────────────────────────────────────
# 🔴 ADIM 1: Force disarm — param2 = 21196 zorunlu
# ─────────────────────────────────────────────
def zorla_disarm(vehicle):
    print("MAVLink FORCE DISARM komutu gönderiliyor...")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,      # param1: 0 = disarm
        21196,  # param2: ArduPilot force disarm magic number — 0 ile çalışmaz!
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)


def land_ol(vehicle, zaman_asimi=35):
    """
    GUIDED modda sadece velocity komutuyla kontrollü iniş yapar.
    simple_goto KULLANILMIYOR — ikisi aynı anda gönderilince çakışıyordu.
    Velocity komutu (vx=0, vy=0) position hold'u flight controller'a bırakır,
    sadece dikey hızı biz veriyoruz. Son metrede LAND moduna geçilir.
    """
    print("Kontrollü iniş başlıyor...")

    if vehicle.mode.name != "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.mode.name != "GUIDED":
            print("GUIDED mod bekleniyor...")
            time.sleep(0.5)

    baslangic_zamani = time.time()

    while True:
        yukseklik = vehicle.location.global_relative_frame.alt

        print(
            f"Kontrollü iniş | "
            f"Yükseklik: {yukseklik:.2f} m | "
            f"ARM: {vehicle.armed} | "
            f"Mod: {vehicle.mode.name}"
        )

        if not vehicle.armed:
            print("Drone zaten DISARM durumda.")
            return True

        if vehicle.mode.name != "GUIDED":
            print("Drone GUIDED moddan çıktı.")
            return False

        if yukseklik > 2.0:
            hiz_komutu_gonder(vehicle, 0, 0, 1.4)
        elif yukseklik > 0.6:
            hiz_komutu_gonder(vehicle, 0, 0, 0.8)
        elif yukseklik > 0.12:
            hiz_komutu_gonder(vehicle, 0, 0, 0.35)
        else:
            print("Son metreye girildi. LAND moduna geçiliyor...")
            break

        if time.time() - baslangic_zamani > zaman_asimi:
            print("Kontrollü iniş zaman aşımına girdi. LAND moduna geçiliyor...")
            break

        time.sleep(0.2)

    vehicle.mode = VehicleMode("LAND")
    while vehicle.mode.name != "LAND":
        print("LAND mod bekleniyor...")
        time.sleep(0.2)

    sifir_irtifa_baslangic = None
    land_baslangic = time.time()
    son_disarm_deneme = 0

    while True:
        yukseklik = vehicle.location.global_relative_frame.alt

        print(
            f"LAND son aşama | "
            f"Yükseklik: {yukseklik:.2f} m | "
            f"ARM: {vehicle.armed} | "
            f"Mod: {vehicle.mode.name}"
        )

        if not vehicle.armed:
            print("Drone indi ve DISARM oldu.")
            return True

        if yukseklik <= 0.05:
            if sifir_irtifa_baslangic is None:
                sifir_irtifa_baslangic = time.time()
            if time.time() - sifir_irtifa_baslangic > 0.6:
                if time.time() - son_disarm_deneme > 1.0:
                    print("Drone yerde. Force DISARM gönderiliyor...")
                    zorla_disarm(vehicle)
                    son_disarm_deneme = time.time()
        else:
            sifir_irtifa_baslangic = None

        if time.time() - land_baslangic > 8:
            print("LAND son aşama zaman aşımına girdi.")
            return False

        time.sleep(0.2)


def gorev_iptal_guvenli_inis(vehicle, sebep):
    print(f"Görev iptal edildi: {sebep}")
    if vehicle.armed:
        print("Drone güvenli inişe alınıyor...")
        land_ol(vehicle)
    return False


def tek_hedef_gorevi(vehicle, hedef, home_noktasi):
    baslangic_zamani = time.time()
    hedef_isim = hedef["isim"]
    hedef_lat = hedef["lat"]
    hedef_lon = hedef["lon"]
    home_lat = home_noktasi["lat"]
    home_lon = home_noktasi["lon"]

    print("\n================================")
    print(f"{hedef_isim} görevi başlıyor.")
    print("================================\n")

    # 🟠 Batarya kontrolü — göreve başlamadan önce
    if not batarya_kontrol_et(vehicle):
        print("Batarya yetersiz, görev başlatılmadı.")
        return False

    if not arm_ve_takeoff(vehicle, HEDEF_YUKSEKLIK):
        return gorev_iptal_guvenli_inis(vehicle, "Kalkış başarısız")

    if not hedefe_git(vehicle, hedef_lat, hedef_lon, HEDEF_YUKSEKLIK):
        return gorev_iptal_guvenli_inis(vehicle, "Hedefe gidilemedi")

    if not hedef_ustunde_stabil_ol(vehicle, hedef_lat, hedef_lon, HEDEF_YUKSEKLIK):
        return gorev_iptal_guvenli_inis(vehicle, "Hedef üstünde stabil olunamadı")

    if not land_ol(vehicle):
        print("Hedefte iniş başarısız.")
        return False

    # ─────────────────────────────────────────────
    # 🔴 ADIM 1: Şartname gereği iniş sonrası bekleme
    # ─────────────────────────────────────────────
    print(f"Hedefte {HEDEFTE_BEKLEME_SURESI} saniye bekleniyor... (şartname gereği)")
    time.sleep(HEDEFTE_BEKLEME_SURESI)

    # 🟠 Batarya kontrolü — home'a dönmeden önce
    if not batarya_kontrol_et(vehicle):
        print("Batarya kritik, home'a dönüş iptal.")
        return False

    if not arm_ve_takeoff(vehicle, HEDEF_YUKSEKLIK):
        return gorev_iptal_guvenli_inis(vehicle, "Hedeften tekrar kalkış başarısız")

    if not hedefe_git(vehicle, home_lat, home_lon, HEDEF_YUKSEKLIK):
        return gorev_iptal_guvenli_inis(vehicle, "Home noktasına dönülemedi")

    if not hedef_ustunde_stabil_ol(vehicle, home_lat, home_lon, HEDEF_YUKSEKLIK):
        return gorev_iptal_guvenli_inis(vehicle, "Home üstünde stabil olunamadı")

    if not land_ol(vehicle):
        print("Home inişi başarısız.")
        return False

    # Home'da da bekleme (gerekirse açılır)
    # time.sleep(HEDEFTE_BEKLEME_SURESI)

    bitis_zamani = time.time()
    gorev_suresi = bitis_zamani - baslangic_zamani

    print(f"{hedef_isim} görevi tamamlandı.")
    print(f"Görev {gorev_suresi:.2f} saniye sürdü.")
    return True


def Gorevler(vehicle, home_noktasi, gorev_noktalari):
    print("Görevler fonksiyonu başladı.")

    if not konum_hazir_mi(vehicle):
        print("Konum hazır değil. Görev başlatılmadı.")
        return

    print("\nHOME NOKTASI")
    print(f"Home Lat: {home_noktasi['lat']:.8f}")
    print(f"Home Lon: {home_noktasi['lon']:.8f}\n")

    basarili_gorev_sayisi = 0

    for hedef in gorev_noktalari:
        # 🟠 Her hedef öncesi batarya kontrolü
        if not batarya_kontrol_et(vehicle):
            print("Batarya kritik seviyede. Görev döngüsü durduruluyor.")
            break

        sonuc = tek_hedef_gorevi(vehicle, hedef, home_noktasi)

        if sonuc:
            basarili_gorev_sayisi += 1
            print(f"Başarılı görev sayısı: {basarili_gorev_sayisi}")
        else:
            print("Bir hedef görevi başarısız oldu. Görev döngüsü durduruluyor.")
            break

    print("\n================================")
    print("TÜM GÖREV AKIŞI BİTTİ")
    print(f"Toplam başarılı hedef: {basarili_gorev_sayisi}")
    print("================================\n")


if __name__ == "__main__":
    print("Drone bağlanıyor...")
    print("Bağlantı adresi:", BAGLANTI_ADRESI)

    vehicle = connect(BAGLANTI_ADRESI, wait_ready=True, timeout=60)

    print("Drone bağlandı.")
    print("Mod:", vehicle.mode.name)
    print("Armed:", vehicle.armed)
    print("Konum:", vehicle.location.global_relative_frame)

    # 🔴 ADIM 1: Home'u runtime'da kaydet
    home_noktasi = home_konumunu_kaydet(vehicle)
    if home_noktasi is None:
        print("Home konumu alınamadı. Program sonlandırılıyor.")
        vehicle.close()
        exit(1)

    # 🟢 ADIM 4: Koordinatları JSON dosyasından oku
    gorev_noktalari = gorev_noktalarini_yukle("gorev_noktalari.json")
    if not gorev_noktalari:
        print("Görev noktası yok. Program sonlandırılıyor.")
        vehicle.close()
        exit(1)

    try:
        Gorevler(vehicle, home_noktasi, gorev_noktalari)

    except KeyboardInterrupt:
        print("Kullanıcı tarafından durduruldu.")
        if vehicle.armed:
            print("Drone LAND moduna alınıyor...")
            vehicle.mode = VehicleMode("LAND")

    finally:
        print("Vehicle kapatılıyor...")
        vehicle.close()