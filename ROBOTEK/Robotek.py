from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from pymavlink import mavutil

BAGLANTI_ADRESI = "udp:127.0.0.1:14550"

HEDEF_YUKSEKLIK = 5
HEDEF_MESAFE = 1.2
STABIL_MESAFE = 1.2
STABIL_HIZ = 0.4
STABIL_SURE = 1.0

NORMAL_HIZ = 3
YAVAS_HIZ = 1
YAVASLAMA_MESAFESI = 5

INIS_SONRASI_BEKLEME = 0

#####################################################
FORCE_DISARM_AKTIF = True
#####################################################

WATCHDOG_AKTIF = True
BATARYA_KONTROL_AKTIF = False
RTL_KULLAN = True

MAKS_HOME_MESAFESI = 80
MAKS_IRTIFA = 20
MIN_BATARYA_VOLTAJ = 14.4

WPNAV_INIS_HIZI = 250

HOME_NOKTASI = {
    "isim": "Home",
    "lat": -35.36349560,
    "lon": 149.16502250
}

GOREV_NOKTALARI = [
    {
        "isim": "Hedef_1",
        "lat": -35.36372998,
        "lon": 149.16459479
    },
    {
        "isim": "Hedef_2",
        "lat": -35.36387660,
        "lon": 149.16493805
    },
    {
        "isim": "Hedef_3",
        "lat": -35.36381440,
        "lon": 149.16536847
    },
    {
        "isim": "Hedef_4",
        "lat": -35.36356780,
        "lon": 149.16559458
    }
]

def mesafe_hesapla_metre(konum1, konum2):  # iki GPS konumu arasındaki mesafeyi metre cinsinden hesaplar
    dlat = konum2.lat - konum1.lat
    dlon = konum2.lon - konum1.lon

    ortalama_lat = math.radians((konum1.lat + konum2.lat) / 2)

    metre_lat = dlat * 111320
    metre_lon = dlon * 111320 * math.cos(ortalama_lat)

    return math.sqrt(metre_lat ** 2 + metre_lon ** 2)


def konum_hazir_mi(vehicle):  # GPS ve EKF verisi hazır olana kadar bekler
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
            print("GPS ve EKF hazir.")
            return True

        if time.time() - baslangic_zamani > 30:
            print("GPS veya EKF hazir olmadi.")
            return False

        time.sleep(1)


def ardupilot_home_ayarla(vehicle, home_noktasi):  # ArduPilot iç home noktasını ana home koordinatına ayarlar
    home_alt = home_noktasi.get("alt", None)

    if home_alt is None:
        home_alt = vehicle.location.global_frame.alt

    if home_alt is None:
        home_alt = 0

    msg = vehicle.message_factory.command_long_encode(
        0,
        0,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        0,
        0,
        0,
        0,
        home_noktasi["lat"],
        home_noktasi["lon"],
        home_alt
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

    print(
        f"ArduPilot HOME ayarlandi: "
        f"{home_noktasi['lat']:.8f}, {home_noktasi['lon']:.8f}, Alt={home_alt:.2f}"
    )


def arm_ve_takeoff(vehicle, hedef_yukseklik, home_noktasi=None):  # GUIDED moda geçer, ARM eder ve kalkış yapar
    print(f"{hedef_yukseklik} metre kalkis komutu geldi. Drone ARM ediliyor...")

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode.name != "GUIDED":
        print("GUIDED mod bekleniyor...")
        time.sleep(1)

    while not vehicle.is_armable:
        print("Drone henuz ARM edilemez.")
        time.sleep(1)

    if home_noktasi is not None:
        ardupilot_home_ayarla(vehicle, home_noktasi)
        time.sleep(0.5)

    vehicle.armed = True

    arm_baslangic_zamani = time.time()

    while not vehicle.armed:
        print("ARM olmasi bekleniyor...")

        if time.time() - arm_baslangic_zamani > 30:
            print("ARM zaman asimina girdi. Drone ARM edilemedi.")
            return False

        time.sleep(1)

    print("ARM edildi.")
    print("Mod:", vehicle.mode.name)
    print("Armed:", vehicle.armed)
    print("Konum:", vehicle.location.global_relative_frame)

    print(f"{hedef_yukseklik} metreye kalkis basliyor...")
    vehicle.simple_takeoff(hedef_yukseklik)

    baslangic_zamani = time.time()

    while True:
        yukseklik = vehicle.location.global_relative_frame.alt

        print(
            f"Anlik yukseklik: {yukseklik:.2f} m | "
            f"Mod: {vehicle.mode.name} | "
            f"ARM: {vehicle.armed}"
        )

        if yukseklik >= hedef_yukseklik * 0.90:
            print("Hedef yukseklige yeterince ulasildi...")
            return True

        if time.time() - baslangic_zamani > 30:
            if yukseklik >= hedef_yukseklik * 0.85:
                print("Drone hedef yukseklige yeterince yaklasti. Goreve devam ediliyor...")
                return True

            print("Takeoff zaman asimina girdi. Drone hedef yukseklige ulasamadi.")
            return False

        time.sleep(1)


def failsafe_durum_kontrol(vehicle, home_noktasi):  # görev sırasında kritik güvenlik durumlarını kontrol eder
    if not WATCHDOG_AKTIF:
        return True, "Watchdog kapali"

    mevcut = vehicle.location.global_relative_frame

    if vehicle.mode.name not in ["GUIDED", "LAND", "RTL"]:
        return False, f"Beklenmeyen mod: {vehicle.mode.name}"

    if mevcut.lat is None or mevcut.lon is None:
        return False, "GPS konumu yok"

    if vehicle.gps_0.fix_type is not None and vehicle.gps_0.fix_type < 3:
        return False, f"GPS fix zayif: {vehicle.gps_0.fix_type}"

    if not vehicle.ekf_ok:
        return False, "EKF sagliksiz"

    if mevcut.alt > MAKS_IRTIFA:
        return False, f"Irtifa limiti asildi: {mevcut.alt:.2f} m"

    home_konum = LocationGlobalRelative(home_noktasi["lat"], home_noktasi["lon"], 0)
    home_uzaklik = mesafe_hesapla_metre(mevcut, home_konum)

    if home_uzaklik > MAKS_HOME_MESAFESI:
        return False, f"Alan disi riski: {home_uzaklik:.2f} m"

    if BATARYA_KONTROL_AKTIF and vehicle.battery is not None and vehicle.battery.voltage is not None:
        if vehicle.battery.voltage < MIN_BATARYA_VOLTAJ:
            return False, f"Batarya dusuk: {vehicle.battery.voltage:.2f} V"

    return True, "Guvenli"


def failsafe_uygula(vehicle, sebep):  # failsafe durumunda aracı RTL veya LAND moduna alır
    print(f"FAILSAFE TETIKLENDI: {sebep}")

    if not vehicle.armed:
        print("Drone zaten armed degil.")
        return False

    if RTL_KULLAN and ("Batarya" in sebep or "Alan disi" in sebep):
        print("RTL moduna geciliyor...")
        vehicle.mode = VehicleMode("RTL")
        return False

    print("LAND moduna geciliyor...")
    vehicle.mode = VehicleMode("LAND")
    return False


def gorev_iptal_guvenli_mod(vehicle, sebep):  # görev iptal olursa drone havadaysa güvenli moda alır
    print(f"Gorev iptal edildi: {sebep}")

    if not vehicle.armed:
        return False

    if vehicle.mode.name in ["RTL", "LAND"]:
        print(f"Drone zaten guvenli modda: {vehicle.mode.name}")
        return False

    return failsafe_uygula(vehicle, sebep)


def hedefe_git(vehicle, hedef_lat, hedef_lon, hedef_yukseklik, hedef_mesafe=1.0, zaman_asimi=60, home_noktasi=None):  # verilen GPS koordinatına gider
    print("Hedefe gitme komutu geldi...")

    if vehicle.mode.name != "GUIDED":
        print("Drone GUIDED moda aliniyor...")
        vehicle.mode = VehicleMode("GUIDED")

        while vehicle.mode.name != "GUIDED":
            print("GUIDED mod bekleniyor...")
            time.sleep(1)

    hedef_konum = LocationGlobalRelative(
        hedef_lat,
        hedef_lon,
        hedef_yukseklik
    )

    print(f"Hedef konum: Lat={hedef_lat}, Lon={hedef_lon}, Alt={hedef_yukseklik}")

    vehicle.simple_goto(hedef_konum, groundspeed=NORMAL_HIZ)

    baslangic_zamani = time.time()
    yavas_mod = False

    while True:
        mevcut_konum = vehicle.location.global_relative_frame
        kalan_mesafe = mesafe_hesapla_metre(mevcut_konum, hedef_konum)

        if home_noktasi is not None:
            guvenli_mi, sebep = failsafe_durum_kontrol(vehicle, home_noktasi)
            if not guvenli_mi:
                return failsafe_uygula(vehicle, sebep)

        if kalan_mesafe <= YAVASLAMA_MESAFESI and not yavas_mod:
            print("Hedefe yaklasildi. Drone yavaslatiliyor...")
            vehicle.simple_goto(hedef_konum, groundspeed=YAVAS_HIZ)
            yavas_mod = True

        print(
            f"Kalan mesafe: {kalan_mesafe:.2f} m | "
            f"Konum: {mevcut_konum.lat:.8f}, {mevcut_konum.lon:.8f} | "
            f"Yukseklik: {mevcut_konum.alt:.2f} m | "
            f"Mod: {vehicle.mode.name}"
        )

        if kalan_mesafe <= hedef_mesafe:
            print("Hedef konuma ulasildi.")
            return True

        if time.time() - baslangic_zamani > zaman_asimi:
            print("Hedefe gitme zaman asimina girdi.")
            return False

        if vehicle.mode.name != "GUIDED":
            print("Drone GUIDED moddan cikti. Gorev iptal.")
            return False

        time.sleep(1)


def hedef_ustunde_stabil_ol(vehicle, hedef_lat, hedef_lon, hedef_yukseklik, home_noktasi=None):  # iniş öncesi hedef üstünde hız ve konum hatasını düşürür
    print("Hedef ustunde stabilizasyon basladi...")

    hedef_konum = LocationGlobalRelative(
        hedef_lat,
        hedef_lon,
        hedef_yukseklik
    )

    vehicle.simple_goto(hedef_konum, groundspeed=YAVAS_HIZ)

    stabil_baslangic = None
    baslangic_zamani = time.time()

    while True:
        mevcut_konum = vehicle.location.global_relative_frame
        kalan_mesafe = mesafe_hesapla_metre(mevcut_konum, hedef_konum)

        if home_noktasi is not None:
            guvenli_mi, sebep = failsafe_durum_kontrol(vehicle, home_noktasi)
            if not guvenli_mi:
                return failsafe_uygula(vehicle, sebep)

        anlik_hiz = vehicle.groundspeed
        if anlik_hiz is None:
            anlik_hiz = 99

        print(
            f"Stabil kontrol | "
            f"Mesafe: {kalan_mesafe:.2f} m | "
            f"Hiz: {anlik_hiz:.2f} m/s"
        )

        if kalan_mesafe <= STABIL_MESAFE and anlik_hiz <= STABIL_HIZ:
            if stabil_baslangic is None:
                stabil_baslangic = time.time()

            if time.time() - stabil_baslangic >= STABIL_SURE:
                print("Drone hedef ustunde stabil.")
                return True
        else:
            stabil_baslangic = None
            vehicle.simple_goto(hedef_konum, groundspeed=YAVAS_HIZ)

        if time.time() - baslangic_zamani > 30:
            print("Stabilizasyon zaman asimina girdi.")
            return False

        if vehicle.mode.name != "GUIDED":
            print("Drone GUIDED moddan cikti. Stabilizasyon iptal.")
            return False

        time.sleep(0.5)


def wpnav_speed_dn_ayarla(vehicle, hiz_cm_s):  # WPNAV_SPEED_DN iniş hız parametresini ayarlar
    if hiz_cm_s < 10:
        hiz_cm_s = 10

    if hiz_cm_s > 500:
        hiz_cm_s = 500

    print(f"WPNAV_SPEED_DN ayarlaniyor: {hiz_cm_s} cm/s")

    msg = vehicle.message_factory.param_set_encode(
        0,
        0,
        b"WPNAV_SPEED_DN",
        float(hiz_cm_s),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()
    time.sleep(0.5)

def zorla_disarm(vehicle):  # yerdeyken ayara bağlı olarak MAVLink disarm komutu gönderir
    if not FORCE_DISARM_AKTIF:
        print("Force disarm kapali. ArduPilot otomatik disarm bekleniyor...")
        return

    print("MAVLink DISARM komutu gonderiliyor...")

    msg = vehicle.message_factory.command_long_encode(
        0,
        0,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        21196,
        0,
        0,
        0,
        0,
        0
    )

    vehicle.send_mavlink(msg)


def land_ol(vehicle, zaman_asimi=40):  # simple_goto ile kademeli iniş yapar ve son aşamada LAND moduna geçer
    print("Inis basliyor...")

    if vehicle.mode.name != "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")

        while vehicle.mode.name != "GUIDED":
            print("GUIDED mod bekleniyor...")
            time.sleep(0.5)

    mevcut = vehicle.location.global_relative_frame

    if mevcut.lat is None or mevcut.lon is None:
        print("Inis icin mevcut konum alinamadi.")
        return False

    inis_lat = mevcut.lat
    inis_lon = mevcut.lon

    print("1 metreye iniliyor...")
    hedef_1m = LocationGlobalRelative(inis_lat, inis_lon, 1.0)
    vehicle.simple_goto(hedef_1m)

    baslangic_zamani = time.time()

    while True:
        yukseklik = vehicle.location.global_relative_frame.alt

        print(
            f"Simple_goto inis 1m | "
            f"Yukseklik: {yukseklik:.2f} m | "
            f"ARM: {vehicle.armed} | "
            f"Mod: {vehicle.mode.name}"
        )

        if not vehicle.armed:
            print("Drone zaten DISARM durumda.")
            return True

        if yukseklik <= 1.20:
            print("1 metre bolgesine ulasildi.")
            break

        if time.time() - baslangic_zamani > zaman_asimi:
            print("1 metreye inis zaman asimina girdi.")
            break

        if vehicle.mode.name != "GUIDED":
            print("Drone GUIDED moddan cikti. Inis iptal.")
            return False

        time.sleep(0.3)

    print("0.2 metreye iniliyor...")
    hedef_02m = LocationGlobalRelative(inis_lat, inis_lon, 0.2)
    vehicle.simple_goto(hedef_02m)

    ikinci_asama_baslangic = time.time()

    while True:
        yukseklik = vehicle.location.global_relative_frame.alt

        print(
            f"Simple_goto inis 0.2m | "
            f"Yukseklik: {yukseklik:.2f} m | "
            f"ARM: {vehicle.armed} | "
            f"Mod: {vehicle.mode.name}"
        )

        if not vehicle.armed:
            print("Drone zaten DISARM durumda.")
            return True

        if yukseklik <= 0.35:
            print("Drone yere yaklasti. LAND moduna geciliyor...")
            break

        if time.time() - ikinci_asama_baslangic > 15:
            print("0.2 metreye inis zaman asimina girdi. LAND moduna geciliyor...")
            break

        if vehicle.mode.name != "GUIDED":
            print("Drone GUIDED moddan cikti. Inis iptal.")
            return False

        time.sleep(0.3)

    vehicle.mode = VehicleMode("LAND")

    while vehicle.mode.name != "LAND":
        print("LAND mod bekleniyor...")
        time.sleep(0.2)

    sifir_irtifa_baslangic = None
    land_baslangic = time.time()

    while True:
        yukseklik = vehicle.location.global_relative_frame.alt

        print(
            f"LAND son asama | "
            f"Yukseklik: {yukseklik:.2f} m | "
            f"ARM: {vehicle.armed} | "
            f"Mod: {vehicle.mode.name}"
        )

        if not vehicle.armed:
            print("Drone indi ve DISARM oldu.")
            return True

        if yukseklik <= 0.20:
            if sifir_irtifa_baslangic is None:
                sifir_irtifa_baslangic = time.time()

            if time.time() - sifir_irtifa_baslangic > 0.6:
                print("Drone yerde. Hizli DISARM deneniyor...")
                zorla_disarm(vehicle)
        else:
            sifir_irtifa_baslangic = None

        if time.time() - land_baslangic > 15:
            print("LAND son asama zaman asimina girdi.")
            return False

        time.sleep(0.2)

def inis_sonrasi_bekle(sure, nokta_adi):  # inişten sonra ayarlanan süre kadar yerde bekler
    if sure <= 0:
        return

    print(f"{nokta_adi} noktasinda {sure} saniye bekleniyor...")

    for kalan in range(sure, 0, -1):
        print(f"Kalan sure: {kalan}")
        time.sleep(1)

    print("Bekleme tamamlandi.")


def tek_hedef_gorevi(vehicle, hedef, home_noktasi):  # bir hedef için git-in-kalk-home dön-in görev döngüsünü çalıştırır
    baslangic_zamani = time.time()

    hedef_isim = hedef["isim"]
    hedef_lat = hedef["lat"]
    hedef_lon = hedef["lon"]

    home_lat = home_noktasi["lat"]
    home_lon = home_noktasi["lon"]

    print("\n================================")
    print(f"{hedef_isim} gorevi basliyor.")
    print("================================\n")

    if not arm_ve_takeoff(vehicle, HEDEF_YUKSEKLIK, home_noktasi):
        return gorev_iptal_guvenli_mod(vehicle, "Kalkis basarisiz")

    if not hedefe_git(vehicle, hedef_lat, hedef_lon, HEDEF_YUKSEKLIK, HEDEF_MESAFE, home_noktasi=home_noktasi):
        return gorev_iptal_guvenli_mod(vehicle, "Hedefe gidilemedi")

    if not hedef_ustunde_stabil_ol(vehicle, hedef_lat, hedef_lon, HEDEF_YUKSEKLIK, home_noktasi=home_noktasi):
        return gorev_iptal_guvenli_mod(vehicle, "Hedef ustunde stabil olunamadi")

    if not land_ol(vehicle):
        print("Hedefte inis basarisiz.")
        return False

    inis_sonrasi_bekle(INIS_SONRASI_BEKLEME, hedef_isim)

    if not arm_ve_takeoff(vehicle, HEDEF_YUKSEKLIK, home_noktasi):
        return gorev_iptal_guvenli_mod(vehicle, "Hedeften tekrar kalkis basarisiz")

    if not hedefe_git(vehicle, home_lat, home_lon, HEDEF_YUKSEKLIK, HEDEF_MESAFE, home_noktasi=home_noktasi):
        return gorev_iptal_guvenli_mod(vehicle, "Home noktasina donulemedi")

    if not hedef_ustunde_stabil_ol(vehicle, home_lat, home_lon, HEDEF_YUKSEKLIK, home_noktasi=home_noktasi):
        return gorev_iptal_guvenli_mod(vehicle, "Home ustunde stabil olunamadi")

    if not land_ol(vehicle):
        print("Home inisi basarisiz.")
        return False

    inis_sonrasi_bekle(INIS_SONRASI_BEKLEME, "Home")

    bitis_zamani = time.time()
    gorev_suresi = bitis_zamani - baslangic_zamani

    print(f"{hedef_isim} gorevi tamamlandi.")
    print(f"Gorev {gorev_suresi:.2f} saniye surdu.")

    return True


def Gorevler(vehicle):  # tüm görev noktalarını sırayla çalıştırır
    print("Gorevler fonksiyonu basladi.")
    
    if not konum_hazir_mi(vehicle):
        print("Konum hazir degil. Gorev baslatilmadi.")
        return

    HOME_NOKTASI["alt"] = vehicle.location.global_frame.alt

    if HOME_NOKTASI["alt"] is None:
        print("Home global irtifasi alinamadi. Gorev baslatilmadi.")
        return

    print(f"Home global irtifasi kaydedildi: {HOME_NOKTASI['alt']:.2f} m")

    print("\nHOME NOKTASI")
    print(f"Home Lat: {HOME_NOKTASI['lat']:.8f}")
    print(f"Home Lon: {HOME_NOKTASI['lon']:.8f}")
    print(f"Home Alt: {HOME_NOKTASI['alt']:.2f}\n")

    wpnav_speed_dn_ayarla(vehicle, WPNAV_INIS_HIZI)

    basarili_gorev_sayisi = 0

    for hedef in GOREV_NOKTALARI:
        sonuc = tek_hedef_gorevi(vehicle, hedef, HOME_NOKTASI)

        if sonuc:
            basarili_gorev_sayisi += 1
            print(f"Basarili gorev sayisi: {basarili_gorev_sayisi}")
        else:
            print("Bir hedef gorevi basarisiz oldu. Gorev dongusu durduruluyor.")
            break

    print("\n================================")
    print("TUM GOREV AKISI BITTI")
    print(f"Toplam basarili hedef: {basarili_gorev_sayisi}")
    print("================================\n")


if __name__ == "__main__":
    print("Drone baglaniyor...")
    print("Baglanti adresi:", BAGLANTI_ADRESI)

    vehicle = connect(BAGLANTI_ADRESI, wait_ready=True, timeout=60)

    print("Drone baglandi.")
    print("Mod:", vehicle.mode.name)
    print("Armed:", vehicle.armed)
    print("Konum:", vehicle.location.global_relative_frame)

    try:
        Gorevler(vehicle)

    except KeyboardInterrupt:
        print("Kullanici tarafindan durduruldu.")
        if vehicle.armed:
            print("Drone LAND moduna aliniyor...")
            vehicle.mode = VehicleMode("LAND")

    finally:
        print("Vehicle kapatiliyor...")
        vehicle.close()