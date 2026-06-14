from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from pymavlink import mavutil
import sys
import termios
import tty
import select

BAGLANTI_ADRESI = "tcp:192.168.30.3:5760"   # Gerçek drone bağlantısı

# Uçuş ayarları
HEDEF_YUKSEKLIK = 10          # Kalkış ve seyir irtifası
HEDEF_MESAFE = 1.2            # Hedefe bu mesafe altında ulaşıldı kabul edilir
INIS_SONRASI_BEKLEME = 0      # LAND sonrası yerde ekstra bekleme

STABIL_MESAFE = 1.2           # Stabil kabul için hedef mesafe eşiği
STABIL_HIZ = 0.4              # Stabil kabul için hız eşiği
STABIL_SURE = 2.0             # Şartlar bu süre sağlanırsa stabil kabul edilir

WPNAV_INIS_HIZI = 100         # WPNAV_SPEED_DN, cm/s

YERDE_KABUL_IRTIFA = 0.55
YERDE_KABUL_HIZ = 0.35
YERDE_ONAY_SURESI = 1.0

WATCHDOG_AKTIF = True
RTL_KULLAN = True
MAKS_HOME_MESAFESI = 400
MAKS_IRTIFA = 40

# LAND/DISARM sonrası kullanıcıya karar vermesi için verilecek süre.
# Bu süre içinde tuş gelmezse normal akış devam eder.
KOMUT_BEKLEME_SURESI = 5.0

HOME_NOKTASI = {
    "isim": "Home",
    "lat": 40.74403631,
    "lon": 30.33869061
}

GOREV_NOKTALARI = [
    {"isim": "Hedef_1", "lat": 40.74421326, "lon": 30.33874879},
    {"isim": "Hedef_2", "lat": 40.74434805, "lon": 30.33848807},
    {"isim": "Hedef_3", "lat": 40.74453955, "lon": 30.33866349},
    {"isim": "Hedef_4", "lat": 40.74467783, "lon": 30.33840338},
]


def mesafe_hesapla_metre(konum1, konum2):
    if konum1.lat is None or konum1.lon is None or konum2.lat is None or konum2.lon is None:
        return float("inf")

    dlat = konum2.lat - konum1.lat
    dlon = konum2.lon - konum1.lon
    ortalama_lat = math.radians((konum1.lat + konum2.lat) / 2)

    metre_lat = dlat * 111320
    metre_lon = dlon * 111320 * math.cos(ortalama_lat)

    return math.sqrt(metre_lat ** 2 + metre_lon ** 2)


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
            print("GPS ve EKF hazir.")
            return True

        if time.time() - baslangic_zamani > 120:
            print("GPS veya EKF hazir olmadi.")
            return False

        time.sleep(1)


def ardupilot_home_ayarla(vehicle, home_noktasi):
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


def arm_ve_takeoff(vehicle, hedef_yukseklik, home_noktasi=None):
    print(f"{hedef_yukseklik} metre kalkis komutu geldi. Drone ARM ediliyor...")

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode.name != "GUIDED":
        print("GUIDED mod bekleniyor...")
        time.sleep(1)

    if home_noktasi is not None:
        ardupilot_home_ayarla(vehicle, home_noktasi)
        time.sleep(0.5)

    if not vehicle.armed:
        # LAND sonrası hemen tekrar kalkışta ArduPilot'un landed state'i toparlaması için kısa bekleme.
        time.sleep(1.0)

        while not vehicle.is_armable:
            print("Drone henuz ARM edilemez.")
            time.sleep(1)

        vehicle.armed = True
        arm_baslangic_zamani = time.time()

        while not vehicle.armed:
            print("ARM olmasi bekleniyor...")

            if time.time() - arm_baslangic_zamani > 30:
                print("ARM zaman asimina girdi. Drone ARM edilemedi.")
                return False

            time.sleep(1)

        print("ARM edildi.")
    else:
        print("Drone zaten ARM durumda. Tekrar ARM edilmeyecek.")

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

        if not vehicle.armed:
            print("Takeoff sirasinda drone DISARM oldu.")
            return False

        if yukseklik >= hedef_yukseklik * 0.90:
            print("Hedef yukseklige yeterince ulasildi...")
            return True

        if time.time() - baslangic_zamani > 30:
            if yukseklik >= hedef_yukseklik * 0.75:
                print("Drone hedef yukseklige yeterince yaklasti. Goreve devam ediliyor...")
                return True

            print("Takeoff zaman asimina girdi. Drone hedef yukseklige ulasamadi.")
            return False

        time.sleep(1)


def failsafe_durum_kontrol(vehicle, home_noktasi):
    if not WATCHDOG_AKTIF:
        return True, "Watchdog kapali"

    mevcut = vehicle.location.global_relative_frame

    if mevcut.lat is None or mevcut.lon is None:
        print("UYARI: GPS konumu yok | Goreve devam ediliyor.")
        return True, "GPS konumu yok ama devam"

    if vehicle.gps_0.fix_type is not None and vehicle.gps_0.fix_type < 3:
        print(f"UYARI: GPS fix zayif: {vehicle.gps_0.fix_type} | Goreve devam ediliyor.")

    if not vehicle.ekf_ok:
        print("UYARI: EKF sagliksiz gorunuyor | Goreve devam ediliyor.")

    if mevcut.alt > MAKS_IRTIFA:
        return False, f"Irtifa limiti asildi: {mevcut.alt:.2f} m"

    home_konum = LocationGlobalRelative(home_noktasi["lat"], home_noktasi["lon"], 0)
    home_uzaklik = mesafe_hesapla_metre(mevcut, home_konum)

    if home_uzaklik > MAKS_HOME_MESAFESI:
        return False, f"Alan disi riski: {home_uzaklik:.2f} m"

    return True, "Guvenli"


def failsafe_uygula(vehicle, sebep):
    print(f"FAILSAFE TETIKLENDI: {sebep}")

    if not vehicle.armed:
        print("Drone zaten armed degil.")
        return False

    if RTL_KULLAN and "Alan disi" in sebep:
        print("RTL moduna geciliyor...")
        vehicle.mode = VehicleMode("RTL")
        return False

    print("LAND moduna geciliyor...")
    vehicle.mode = VehicleMode("LAND")
    return False


def hedefe_git(vehicle, hedef_lat, hedef_lon, hedef_yukseklik, hedef_mesafe=1.0, zaman_asimi=60, home_noktasi=None):
    print("Hedefe gitme komutu geldi...")

    if vehicle.mode.name != "GUIDED":
        print("Drone GUIDED moda aliniyor...")
        vehicle.mode = VehicleMode("GUIDED")

        while vehicle.mode.name != "GUIDED":
            print("GUIDED mod bekleniyor...")
            time.sleep(1)

    hedef_konum = LocationGlobalRelative(hedef_lat, hedef_lon, hedef_yukseklik)

    print(f"Hedef konum: Lat={hedef_lat}, Lon={hedef_lon}, Alt={hedef_yukseklik}")

    simpleGotoMavlink(vehicle, hedef_konum, hedef_konum.alt)

    baslangic_zamani = time.time()
    son_komut_zamani = time.time()

    while True:
        mevcut_konum = vehicle.location.global_relative_frame

        if mevcut_konum.lat is None or mevcut_konum.lon is None:
            print("UYARI: Hedefe giderken GPS konumu yok. Sonraki olcum bekleniyor.")
            time.sleep(1)
            continue

        kalan_mesafe = mesafe_hesapla_metre(mevcut_konum, hedef_konum)

        if home_noktasi is not None:
            guvenli_mi, sebep = failsafe_durum_kontrol(vehicle, home_noktasi)
            if not guvenli_mi:
                return failsafe_uygula(vehicle, sebep)

        if time.time() - son_komut_zamani > 2.0:
            simpleGotoMavlink(vehicle, hedef_konum, hedef_konum.alt)
            son_komut_zamani = time.time()

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


def hedef_ustunde_stabil_ol(vehicle, hedef_lat, hedef_lon, hedef_yukseklik, home_noktasi=None):
    print("Hedef ustunde stabilizasyon basladi...")

    hedef_konum = LocationGlobalRelative(hedef_lat, hedef_lon, hedef_yukseklik)
    simpleGotoMavlink(vehicle, hedef_konum, hedef_konum.alt)

    stabil_baslangic = None
    baslangic_zamani = time.time()

    while True:
        mevcut_konum = vehicle.location.global_relative_frame

        if mevcut_konum.lat is None or mevcut_konum.lon is None:
            print("UYARI: Stabilizasyonda GPS konumu yok. Sonraki olcum bekleniyor.")
            time.sleep(0.5)
            continue

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
            simpleGotoMavlink(vehicle, hedef_konum, hedef_konum.alt)

        if time.time() - baslangic_zamani > 30:
            print("Stabilizasyon zaman asimina girdi.")
            return False

        if vehicle.mode.name != "GUIDED":
            print("Drone GUIDED moddan cikti. Stabilizasyon iptal.")
            return False

        time.sleep(0.5)


def wpnav_speed_dn_ayarla(vehicle, hiz_cm_s):
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


def land_ol(vehicle, bekleme_suresi=0, zaman_asimi=90):
    print("Direkt LAND inisi basliyor...")
    print("GUIDED ile alcalma yok. Direkt LAND moduna geciliyor...")

    vehicle.mode = VehicleMode("LAND")

    land_mod_baslangic = time.time()

    while vehicle.mode.name != "LAND":
        print("LAND mod bekleniyor...")

        if time.time() - land_mod_baslangic > 5:
            print("LAND moda gecilemedi.")
            return False

        time.sleep(0.2)

    yerde_aday_baslangic = None
    bekleme_baslangic = None
    land_baslangic = time.time()
    arm_bekleme_mesaji_yazildi = False

    while True:
        yukseklik = vehicle.location.global_relative_frame.alt
        hiz = vehicle.groundspeed

        if hiz is None:
            hiz = 0

        print(
            f"LAND | "
            f"Yukseklik: {yukseklik:.2f} m | "
            f"Hiz: {hiz:.2f} m/s | "
            f"ARM: {vehicle.armed} | "
            f"Mod: {vehicle.mode.name}"
        )

        if not vehicle.armed:
            print("Drone DISARM oldu. Inis tamamlandi.")
            return True

        yerde_gibi = yukseklik <= YERDE_KABUL_IRTIFA and hiz <= YERDE_KABUL_HIZ

        if yerde_gibi:
            if yerde_aday_baslangic is None:
                yerde_aday_baslangic = time.time()
                print("Drone yerde gibi. Yerde onay suresi basladi...")

            yerde_aday_suresi = time.time() - yerde_aday_baslangic

            if yerde_aday_suresi >= YERDE_ONAY_SURESI and bekleme_baslangic is None:
                bekleme_baslangic = time.time()
                print("Bekleme sayaci basladi.")

            if bekleme_baslangic is not None:
                yerde_gecen_sure = time.time() - bekleme_baslangic

                if yerde_gecen_sure >= bekleme_suresi:
                    if vehicle.armed:
                        if not arm_bekleme_mesaji_yazildi:
                            print(
                                "Yerde bekleme tamamlandi ama drone hala ARM durumda. "
                                "ArduPilot otomatik DISARM bekleniyor..."
                            )
                            arm_bekleme_mesaji_yazildi = True
                    else:
                        print("Inis ve yerde bekleme tamamlandi.")
                        return True
        else:
            yerde_aday_baslangic = None
            bekleme_baslangic = None
            arm_bekleme_mesaji_yazildi = False

        if time.time() - land_baslangic > zaman_asimi + bekleme_suresi:
            print("LAND zaman asimina girdi.")
            return False

        time.sleep(0.2)


def simpleGotoMavlink(vehicle, hedefKonum, irtifa):
    current_heading = vehicle.heading

    if current_heading is None:
        current_heading = 0

    heading_radians = math.radians(current_heading)

    print(f"Hedefe yoneliliyor. Mevcut yon sabiti: {current_heading} derece")

    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        3064,
        int(hedefKonum.lat * 1e7),
        int(hedefKonum.lon * 1e7),
        irtifa,
        0,
        0,
        0,
        0,
        0,
        0,
        heading_radians,
        0
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()


def b_tusu_bekle():
    print("\n================================")
    print(">> b tusu bekleniyor...")
    print(">> Gorevi baslatmak icin terminalde b tusuna bas")
    print("================================\n")

    fd = sys.stdin.fileno()
    eski_ayar = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)

        while True:
            tus = sys.stdin.read(1).lower()

            if tus == "b":
                print("\n>> b tusu algilandi.")
                print(">> Gorev baslatiliyor...\n")
                return

            else:
                print(">> Sadece b tusuna basinca gorev baslar.")

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, eski_ayar)


def sureli_tus_kontrolu(izinli_tuslar, sure_saniye):
    """
    Belirlenen süre içinde r/h gelirse döndürür.
    Tuş gelmezse None döner ve normal akış devam eder.
    """
    print("\n--------------------------------")
    print(f"Komut penceresi: {sure_saniye:.1f} sn")
    print("r = ayni HEDEF konumuna tekrar git")
    print("h = HOME konumuna git / HOME'u tekrar dene")
    print("Tusa basmazsan normal akis devam eder.")
    print("--------------------------------\n")

    fd = sys.stdin.fileno()
    eski_ayar = termios.tcgetattr(fd)
    baslangic = time.time()

    try:
        tty.setcbreak(fd)

        while time.time() - baslangic < sure_saniye:
            kalan = sure_saniye - (time.time() - baslangic)
            print(f"Komut bekleniyor... kalan: {kalan:.1f} sn", end="\r")

            okunabilir, _, _ = select.select([sys.stdin], [], [], 0.1)

            if okunabilir:
                tus = sys.stdin.read(1).lower()

                if tus in izinli_tuslar:
                    print(f"\nKomut alindi: {tus}\n")
                    return tus

                print(f"\nGecersiz tus: {tus}. Sadece {izinli_tuslar}\n")

        print("\nKomut gelmedi. Normal akis devam ediyor.\n")
        return None

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, eski_ayar)


def konuma_git_ve_land(vehicle, nokta, home_noktasi):
    print("\n================================")
    print(f"{nokta['isim']} denemesi basliyor.")
    print("================================\n")

    if not arm_ve_takeoff(vehicle, HEDEF_YUKSEKLIK, home_noktasi):
        print(f"{nokta['isim']} icin kalkis basarisiz.")
        return False

    if not hedefe_git(
        vehicle,
        nokta["lat"],
        nokta["lon"],
        HEDEF_YUKSEKLIK,
        HEDEF_MESAFE,
        home_noktasi=home_noktasi
    ):
        print(f"{nokta['isim']} konumuna gidilemedi.")
        return False

    if not hedef_ustunde_stabil_ol(
        vehicle,
        nokta["lat"],
        nokta["lon"],
        HEDEF_YUKSEKLIK,
        home_noktasi=home_noktasi
    ):
        print(f"{nokta['isim']} ustunde stabil olunamadi.")
        return False

    if not land_ol(vehicle, bekleme_suresi=INIS_SONRASI_BEKLEME):
        print(f"{nokta['isim']} LAND basarisiz.")
        return False

    print(f"{nokta['isim']} LAND tamamlandi.")
    return True


def Gorevler(vehicle):
    print("Gorevler fonksiyonu basladi.")

    genel_baslangic_zamani = time.time()

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

    basarili_hedef_sayisi = 0

    for hedef in GOREV_NOKTALARI:
        durum = "HEDEF"  # HEDEF -> HOME -> siradaki hedef

        while True:
            if durum == "HEDEF":
                sonuc = konuma_git_ve_land(vehicle, hedef, HOME_NOKTASI)

                if not sonuc:
                    print("Hedef denemesi basarisiz oldu. Gorev durduruluyor.")
                    return

                tus = sureli_tus_kontrolu(["r", "h"], KOMUT_BEKLEME_SURESI)

                if tus == "r":
                    print(f"Operator komutu: {hedef['isim']} yanlis/eksik. Ayni hedef tekrar deneniyor.")
                    durum = "HEDEF"
                    continue

                if tus == "h":
                    print("Operator komutu: HOME'a gidiliyor.")
                    durum = "HOME"
                    continue

                print("Komut yok: normal akis geregi HOME'a gidiliyor.")
                durum = "HOME"
                continue

            if durum == "HOME":
                sonuc = konuma_git_ve_land(vehicle, HOME_NOKTASI, HOME_NOKTASI)

                if not sonuc:
                    print("HOME denemesi basarisiz oldu. Gorev durduruluyor.")
                    return

                tus = sureli_tus_kontrolu(["r", "h"], KOMUT_BEKLEME_SURESI)

                if tus == "h":
                    print("Operator komutu: HOME yanlis/eksik. HOME tekrar deneniyor.")
                    durum = "HOME"
                    continue

                if tus == "r":
                    print(f"Operator komutu: {hedef['isim']} tekrar deneniyor.")
                    durum = "HEDEF"
                    continue

                print("Komut yok: normal akis geregi siradaki hedefe geciliyor.")
                basarili_hedef_sayisi += 1
                print(f"Tamamlanan hedef sayisi: {basarili_hedef_sayisi}")
                break

    genel_bitis_zamani = time.time()
    toplam_gorev_suresi = genel_bitis_zamani - genel_baslangic_zamani

    print("\n================================")
    print("TUM GOREV AKISI BITTI")
    print(f"Toplam tamamlanan hedef: {basarili_hedef_sayisi}")
    print(f"Genel gorev suresi: {toplam_gorev_suresi:.2f} saniye")
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
        b_tusu_bekle()
        for i in range(3):
            Gorevler(vehicle)

    except KeyboardInterrupt:
        print("Kullanici tarafindan durduruldu.")
        print("Mevcut moda dokunulmuyor.")

    finally:
        print("Vehicle kapatiliyor...")
        vehicle.close()