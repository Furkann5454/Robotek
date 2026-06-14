from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from pymavlink import mavutil


BAGLANTI_ADRESI = "tcp:192.168.30.3:5760"   # Gerçek drone bağlantısı


HEDEF_YUKSEKLIK = 20          # Geçen gerçek uçuşta kullanılan görev irtifası
HEDEF_MESAFE = 1.2            # Hedefe 1.2 m yaklaşınca ulaşıldı kabul edilir
INIS_SONRASI_BEKLEME = 6     # Hedefe indikten sonra yerde 3 sn bekleme


STABIL_MESAFE = 1.2           # Hedef üstünde stabil kabul için mesafe eşiği
STABIL_HIZ = 0.4              # Hedef üstünde stabil kabul için hız eşiği
STABIL_SURE = 1.0             # Bu şartlar 1 sn sağlanırsa stabil kabul edilir


WPNAV_INIS_HIZI = 250         # Aşağı iniş hızı, 250 cm/s = 2.5 m/s

YAKIN_INIS_IRTIFA = 0.50      # LAND öncesi GUIDED ile gidilecek yakın iniş hedefi # 0.30
LAND_GECIS_IRTIFA = 0.60      # 0.60 m altına düşünce LAND moduna geçilir          # 0.40



YERDE_KABUL_IRTIFA = 0.55     # 0.30 m altı ve düşük hızda drone yerde gibi kabul edilir 
BEKLEME_BASLAT_IRTIFA = 0.55  # Yerde bekleme sayacı 0.30 m altında başlar  
YERDE_KABUL_HIZ = 0.35        # Yerde kabul için hız eşiği
YERDE_ONAY_SURESI = 1.0       # Yerde gibi durumu 1 sn sürerse onaylanır
ERKEN_ARM_KALAN_SURE = 3.0    # Eski kodda erken ARM için kalan süre ayarı


LAND_TAKILMA_SURESI = 3.0        # LAND modunda 3 sn irtifa değişmezse takılma kontrolü
LAND_KURTARMA_IRTIFA = 0.22      # Takılırsa GUIDED ile 0.22 m hedeflenir
LAND_KURTARMA_SURESI = 5.0       # Kurtarma için maksimum süre
LAND_KURTARMA_MAX_DENEME = 1     # Gerçek uçuşta en fazla 1 kurtarma denemesi 
LAND_TAKILMA_IRTIFA_FARKI = 0.03 # 3 cm'den az değişim varsa takılma sayılır



WATCHDOG_AKTIF = True        # Kod içi güvenlik kontrolü açık
RTL_KULLAN = True            # Alan dışı gibi durumda RTL kullanılabilir
MAKS_HOME_MESAFESI = 80      # Home'dan 80 m uzaklaşırsa failsafe
MAKS_IRTIFA = 30             # 30 m üstü irtifa limiti

# Home noktası
HOME_NOKTASI = {
    "isim": "Home", # 30 home uzak
    "lat": 40.74457400,
    "lon": 30.33828620
}


# Görev hedefleri
GOREV_NOKTALARI = [
    {
    "isim": "Hedef_3",
    "lat": 40.74434990,
    "lon": 30.33849760
    },
    {
    "isim": "Hedef_2", # 50 orta uzak
    "lat": 40.74441170,
    "lon": 30.33832840
    },
    {
    "isim": "Hedef_1",  # 40 sol uzak
    "lat": 40.74451190,
    "lon": 30.33845470
    }
]

# ROTA 30 HOME uzak -> 40 SOL uzak -> 50 ORTA uzak -> 30 HOME uzak  
def mesafe_hesapla_metre(konum1, konum2):
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


def gorev_iptal_guvenli_mod(vehicle, sebep):
    print(f"Gorev iptal edildi: {sebep}")

    if not vehicle.armed:
        return False

    if vehicle.mode.name in ["RTL", "LAND"]:
        print(f"Drone zaten guvenli modda: {vehicle.mode.name}")
        return False

    print("Gorev durduruldu. Mevcut moda dokunulmuyor.")
    return False


def hedefe_git(vehicle, hedef_lat, hedef_lon, hedef_yukseklik, hedef_mesafe=1.0, zaman_asimi=60, home_noktasi=None):
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

    simpleGotoMavlink(vehicle, hedef_konum, hedef_konum.alt)

    baslangic_zamani = time.time()
    son_komut_zamani = time.time()

    while True:
        mevcut_konum = vehicle.location.global_relative_frame
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

    hedef_konum = LocationGlobalRelative(
        hedef_lat,
        hedef_lon,
        hedef_yukseklik
    )

    simpleGotoMavlink(vehicle, hedef_konum, hedef_konum.alt)

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


def land_takilma_kurtarma(vehicle, inis_lat, inis_lon):
    print("LAND modunda alcalma durdu. Kisa GUIDED kurtarma uygulanıyor...")

    vehicle.mode = VehicleMode("GUIDED")

    guided_baslangic = time.time()

    while vehicle.mode.name != "GUIDED":
        print("GUIDED mod bekleniyor...")

        if time.time() - guided_baslangic > 5:
            print("GUIDED moda gecilemedi. Kurtarma basarisiz.")
            return False

        time.sleep(0.2)

    hedef_kurtarma = LocationGlobalRelative(
        inis_lat,
        inis_lon,
        LAND_KURTARMA_IRTIFA
    )

    kurtarma_baslangic = time.time()
    son_komut_zamani = 0

    while time.time() - kurtarma_baslangic < LAND_KURTARMA_SURESI:
        if time.time() - son_komut_zamani > 0.5:
            simpleGotoMavlink(vehicle, hedef_kurtarma, hedef_kurtarma.alt)
            son_komut_zamani = time.time()

        yukseklik = vehicle.location.global_relative_frame.alt

        print(
            f"GUIDED kurtarma | "
            f"Yukseklik: {yukseklik:.2f} m | "
            f"ARM: {vehicle.armed} | "
            f"Mod: {vehicle.mode.name}"
        )

        if not vehicle.armed:
            print("Drone DISARM oldu. Kurtarma sonlandiriliyor.")
            return True

        if yukseklik <= YERDE_KABUL_IRTIFA:
            print("GUIDED kurtarma yeterince alcaltildi. LAND moduna donuluyor...")
            break

        time.sleep(0.2)

    vehicle.mode = VehicleMode("LAND")

    land_baslangic = time.time()

    while vehicle.mode.name != "LAND":
        print("LAND mod bekleniyor...")

        if time.time() - land_baslangic > 5:
            print("LAND moda geri gecilemedi.")
            return False

        time.sleep(0.2)

    print("Kurtarma tamamlandi. LAND moduna geri donuldu.")
    return True


def land_ol(vehicle, inis_lat, inis_lon, bekleme_suresi=0, zaman_asimi=40):
    print("Inis basliyor...")

    if vehicle.mode.name != "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")

        while vehicle.mode.name != "GUIDED":
            print("GUIDED mod bekleniyor...")
            time.sleep(0.5)

    if inis_lat is None or inis_lon is None:
        print("Inis icin hedef konum alinamadi.")
        return False

    print(f"Inis hedef koordinati: Lat={inis_lat:.8f}, Lon={inis_lon:.8f}")

    print("1 metreye iniliyor...")
    hedef_1m = LocationGlobalRelative(inis_lat, inis_lon, 1.0)
    simpleGotoMavlink(vehicle, hedef_1m, hedef_1m.alt)

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

    print(f"{YAKIN_INIS_IRTIFA:.2f} metre civarinda LAND gecisi hazirlaniyor...")
    hedef_yakin = LocationGlobalRelative(inis_lat, inis_lon, YAKIN_INIS_IRTIFA)
    simpleGotoMavlink(vehicle, hedef_yakin, hedef_yakin.alt)

    ikinci_asama_baslangic = time.time()
    son_komut_zamani = time.time()

    while True:
        yukseklik = vehicle.location.global_relative_frame.alt

        print(
            f"Simple_goto inis {YAKIN_INIS_IRTIFA:.2f}m | "
            f"Yukseklik: {yukseklik:.2f} m | "
            f"ARM: {vehicle.armed} | "
            f"Mod: {vehicle.mode.name}"
        )

        if not vehicle.armed:
            print("Drone zaten DISARM durumda.")
            return True

        if yukseklik <= LAND_GECIS_IRTIFA:
            print("Drone yere yaklasti. LAND moduna geciliyor...")
            break

        if time.time() - son_komut_zamani > 1.0:
            simpleGotoMavlink(vehicle, hedef_yakin, hedef_yakin.alt)
            son_komut_zamani = time.time()

        if time.time() - ikinci_asama_baslangic > 20:
            print("Yakin inis zaman asimina girdi. LAND moduna geciliyor...")
            break

        if vehicle.mode.name != "GUIDED":
            print("Drone GUIDED moddan cikti. Inis iptal.")
            return False

        time.sleep(0.3)

    vehicle.mode = VehicleMode("LAND")

    while vehicle.mode.name != "LAND":
        print("LAND mod bekleniyor...")
        time.sleep(0.2)

    yerde_aday_baslangic = None
    bekleme_baslangic = None
    land_baslangic = time.time()
    disarm_bekleme_suresi = 0
    disarm_yazildi = False
    arm_bekleme_mesaji_yazildi = False

    son_land_yukseklik = None
    son_land_kontrol_zamani = time.time()
    land_kurtarma_deneme_sayisi = 0

    while True:
        yukseklik = vehicle.location.global_relative_frame.alt
        hiz = vehicle.groundspeed

        if hiz is None:
            hiz = 0

        print(
            f"LAND son asama | "
            f"Yukseklik: {yukseklik:.2f} m | "
            f"Hiz: {hiz:.2f} m/s | "
            f"ARM: {vehicle.armed} | "
            f"Mod: {vehicle.mode.name}"
        )

        yerde_gibi = yukseklik <= YERDE_KABUL_IRTIFA and hiz <= YERDE_KABUL_HIZ

        if (
            vehicle.armed and
            vehicle.mode.name == "LAND" and
            not yerde_gibi and
            land_kurtarma_deneme_sayisi < LAND_KURTARMA_MAX_DENEME
        ):
            if son_land_yukseklik is None:
                son_land_yukseklik = yukseklik
                son_land_kontrol_zamani = time.time()
            else:
                gecen_sure = time.time() - son_land_kontrol_zamani
                irtifa_farki = abs(son_land_yukseklik - yukseklik)

                if gecen_sure >= LAND_TAKILMA_SURESI:
                    if yukseklik > YERDE_KABUL_IRTIFA and irtifa_farki <= LAND_TAKILMA_IRTIFA_FARKI:
                        land_kurtarma_deneme_sayisi += 1

                        kurtarma_sonuc = land_takilma_kurtarma(vehicle, inis_lat, inis_lon)

                        if not kurtarma_sonuc:
                            print("LAND takilma kurtarma basarisiz.")
                            return False

                        yerde_aday_baslangic = None
                        bekleme_baslangic = None
                        arm_bekleme_mesaji_yazildi = False
                        son_land_yukseklik = vehicle.location.global_relative_frame.alt
                        son_land_kontrol_zamani = time.time()
                        land_baslangic = time.time()
                        continue

                    son_land_yukseklik = yukseklik
                    son_land_kontrol_zamani = time.time()

        if yerde_gibi:
            if yerde_aday_baslangic is None:
                yerde_aday_baslangic = time.time()
                print("Drone yerde gibi. Yerde onay suresi basladi...")

            yerde_aday_suresi = time.time() - yerde_aday_baslangic

            bekleme_baslat_sarti = (
                yerde_aday_suresi >= YERDE_ONAY_SURESI and
                (
                    yukseklik <= BEKLEME_BASLAT_IRTIFA or
                    not vehicle.armed
                )
            )

            if bekleme_baslat_sarti and bekleme_baslangic is None:
                bekleme_baslangic = time.time()
                print("Bekleme sayaci basladi.")

            if not vehicle.armed and not disarm_yazildi:
                disarm_bekleme_suresi = time.time() - yerde_aday_baslangic
                print(
                    f"Drone DISARM oldu. "
                    f"Yerde kabulden disarm'a kadar gecen sure: {disarm_bekleme_suresi:.2f} sn"
                )
                disarm_yazildi = True

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
                        print(
                            f"Inis ve yerde bekleme tamamlandi. "
                            f"Istenen bekleme: {bekleme_suresi:.2f} sn | "
                            f"Yerde gecen sure: {yerde_gecen_sure:.2f} sn | "
                            f"ARM: {vehicle.armed}"
                        )
                        return True

        else:
            yerde_aday_baslangic = None
            bekleme_baslangic = None
            arm_bekleme_mesaji_yazildi = False

        if time.time() - land_baslangic > zaman_asimi + bekleme_suresi + 20:
            print("LAND son asama zaman asimina girdi.")
            return False

        time.sleep(0.2)


def tek_hedef_gorevi(vehicle, hedef, home_noktasi):
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

    if not hedefe_git(
        vehicle,
        hedef_lat,
        hedef_lon,
        HEDEF_YUKSEKLIK,
        HEDEF_MESAFE,
        home_noktasi=home_noktasi
    ):
        return gorev_iptal_guvenli_mod(vehicle, "Hedefe gidilemedi")

    if not hedef_ustunde_stabil_ol(
        vehicle,
        hedef_lat,
        hedef_lon,
        HEDEF_YUKSEKLIK,
        home_noktasi=home_noktasi
    ):
        return gorev_iptal_guvenli_mod(vehicle, "Hedef ustunde stabil olunamadi")

    inis_sonuc = land_ol(
        vehicle,
        hedef_lat,
        hedef_lon,
        bekleme_suresi=INIS_SONRASI_BEKLEME
    )

    if not inis_sonuc:
        print("Hedefte inis basarisiz.")
        return False

    if not arm_ve_takeoff(vehicle, HEDEF_YUKSEKLIK, home_noktasi):
        return gorev_iptal_guvenli_mod(vehicle, "Hedeften tekrar kalkis basarisiz")

    if not hedefe_git(
        vehicle,
        home_lat,
        home_lon,
        HEDEF_YUKSEKLIK,
        HEDEF_MESAFE,
        home_noktasi=home_noktasi
    ):
        return gorev_iptal_guvenli_mod(vehicle, "Home noktasina donulemedi")

    if not hedef_ustunde_stabil_ol(
        vehicle,
        home_lat,
        home_lon,
        HEDEF_YUKSEKLIK,
        home_noktasi=home_noktasi
    ):
        return gorev_iptal_guvenli_mod(vehicle, "Home ustunde stabil olunamadi")

    inis_sonuc = land_ol(
        vehicle,
        home_lat,
        home_lon,
        bekleme_suresi=0
    )

    if not inis_sonuc:
        print("Home inisi basarisiz.")
        return False

    bitis_zamani = time.time()
    gorev_suresi = bitis_zamani - baslangic_zamani

    print(f"{hedef_isim} gorevi tamamlandi.")
    print(f"Gorev {gorev_suresi:.2f} saniye surdu.")

    return True


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

    basarili_gorev_sayisi = 0

    for hedef in GOREV_NOKTALARI:
        sonuc = tek_hedef_gorevi(vehicle, hedef, HOME_NOKTASI)

        if sonuc:
            basarili_gorev_sayisi += 1
            print(f"Basarili gorev sayisi: {basarili_gorev_sayisi}")
        else:
            print("Bir hedef gorevi basarisiz oldu. Gorev dongusu durduruluyor.")
            break

    genel_bitis_zamani = time.time()
    toplam_gorev_suresi = genel_bitis_zamani - genel_baslangic_zamani

    print("\n================================")
    print("TUM GOREV AKISI BITTI")
    print(f"Toplam basarili hedef: {basarili_gorev_sayisi}")
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
        Gorevler(vehicle)

    except KeyboardInterrupt:
        print("Kullanici tarafindan durduruldu.")
        if vehicle.armed:
            print("Drone LAND moduna aliniyor...")
            vehicle.mode = VehicleMode("LAND")

    finally:
        print("Vehicle kapatiliyor...")
        vehicle.close()