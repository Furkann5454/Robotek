from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from pymavlink import mavutil

# Bağlantı
BAGLANTI_ADRESI = "udp:127.0.0.1:14550"

# Görev genel ayarları
HEDEF_YUKSEKLIK = 10
HEDEF_MESAFE = 1.2
INIS_SONRASI_BEKLEME = 0

# Stabilizasyon ayarları
STABIL_MESAFE = 1.2
STABIL_HIZ = 0.4
STABIL_SURE = 1.0

# İniş ayarları
WPNAV_INIS_HIZI = 250

# Failsafe ayarları
WATCHDOG_AKTIF = True        
RTL_KULLAN = True            
MAKS_HOME_MESAFESI = 80      
MAKS_IRTIFA = 20             

# Home noktası
HOME_NOKTASI = {
    "isim": "Home",
    "lat": -35.36349560,  
    "lon": 149.16502250
}

# Saf Koordinat (Tuple) Listesi
GOREV_NOKTALARI = [
    (-35.3635567, 149.1649536),   # hedef_1
    (-35.3635558, 149.1651442),   # hedef_2
]


def ardupilot_inis_parametrelerini_ayarla(vehicle):
    """Firmware destekliyorsa hizli disarm parametresini yazar"""
    print("\n[BILGI] ArduPilot DISARM_DELAY kontrol ediliyor...")
    try:
        if 'DISARM_DELAY' in vehicle.parameters:
            vehicle.parameters['DISARM_DELAY'] = 1
            print("  -> [BASARILI] DISARM_DELAY = 1 olarak ayarlandi.")
        else:
            print("  -> [BILGI] DISARM_DELAY parametresi varsayilan degerinde.")
    except Exception as e:
        print(f"  -> [UYARI] Parametre yazilamadi: {e}")


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

        print(f"GPS Fix: {gps.fix_type} | EKF: {ekf_hazir} | Konum: {konum.lat}, {konum.lon}")

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
        0, 0, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0,
        home_noktasi["lat"], home_noktasi["lon"], home_alt
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print(f"ArduPilot HOME ayarlandi: {home_noktasi['lat']:.8f}, {home_noktasi['lon']:.8f}, Alt={home_alt:.2f}")


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

    print(f"{hedef_yukseklik} metreye kalkis basliyor...")
    vehicle.simple_takeoff(hedef_yukseklik)
    baslangic_zamani = time.time()

    while True:
        yukseklik = vehicle.location.global_relative_frame.alt
        print(f"Anlik yukseklik: {yukseklik:.2f} m | Mod: {vehicle.mode.name} | ARM: {vehicle.armed}")

        if yukseklik >= hedef_yukseklik * 0.90:
            print("Hedef yukseklige yeterince ulasildi...")
            return True

        if time.time() - baslangic_zamani > 30:
            if yukseklik >= hedef_yukseklik * 0.75:
                print("Drone hedef yukseklige yeterince yaklasti. Goreve devam ediliyor...")
                return True
            print("Takeoff zaman asimina girdi.")
            return False
        time.sleep(1)


def failsafe_durum_kontrol(vehicle, home_noktasi):
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
    return failsafe_uygula(vehicle, sebep)


def hedefe_git(vehicle, hedef_lat, hedef_lon, hedef_yukseklik, hedef_mesafe=1.0, zaman_asimi=60, home_noktasi=None):
    print("Hedefe gitme komutu geldi...")
    if vehicle.mode.name != "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.mode.name != "GUIDED":
            print("GUIDED mod bekleniyor...")
            time.sleep(1)

    hedef_konum = LocationGlobalRelative(hedef_lat, hedef_lon, hedef_yukseklik)
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

        print(f"Kalan mesafe: {kalan_mesafe:.2f} m | Yukseklik: {mevcut_konum.alt:.2f} m")

        if kalan_mesafe <= hedef_mesafe:
            print("Hedef konuma ulasildi.")
            return True

        if time.time() - baslangic_zamani > zaman_asimi:
            print("Hedefe gitme zaman asimina girdi.")
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
        kalan_mesafe = mesafe_hesapla_metre(mevcut_konum, hedef_konum)
        anlik_hiz = vehicle.groundspeed if vehicle.groundspeed is not None else 99

        if home_noktasi is not None:
            guvenli_mi, sebep = failsafe_durum_kontrol(vehicle, home_noktasi)
            if not guvenli_mi:
                return failsafe_uygula(vehicle, sebep)

        print(f"Stabil kontrol | Mesafe: {kalan_mesafe:.2f} m | Hiz: {anlik_hiz:.2f} m/s")

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
        time.sleep(0.5)


def land_ol(vehicle, bekleme_suresi=0, zaman_asimi=40):
    """
    Agresif Dalis ve Otonom Inis Modeli.
    Karari tamamen ArduPilot kendi sensor süzgecine gore verir.
    """
    print("\n--- Agresif Dalis ve Inis Operasyonu Basliyor ---")
    
    mevcut_konum = vehicle.location.global_relative_frame
    hedef_yer = LocationGlobalRelative(mevcut_konum.lat, mevcut_konum.lon, 0)
    simpleGotoMavlink(vehicle, hedef_yer, 0)

    baslangic_zamani = time.time()
    
    # 1. ASAMA: 1.5 metreye kadar tam gaz inis
    while True:
        yukseklik = vehicle.location.global_relative_frame.alt
        print(f"Dalis Asamasi | Yukseklik: {yukseklik:.2f} m")
        
        if yukseklik <= 1.50:
            print("1.5 metre siniri gecildi! Otonom yumusak inis icin LAND moduna geciliyor...")
            vehicle.mode = VehicleMode("LAND")
            break
            
        if time.time() - baslangic_zamani > zaman_asimi:
            print("Dalis zamandasimina ugradi, guvenlik amacli LAND moduna zorlaniyor.")
            vehicle.mode = VehicleMode("LAND")
            break
        time.sleep(0.1)

    # LAND moduna gecis onayi bekleniyor
    while vehicle.mode.name != "LAND":
        time.sleep(0.05)

    land_baslangic = time.time()
    disarm_yazildi = False

    # 2. ASAMA: LAND Modunda sadece ArduPilot'un kendi kendine disarm etmesini takip ediyoruz
    while True:
        yukseklik = vehicle.location.global_relative_frame.alt
        hiz = vehicle.groundspeed if vehicle.groundspeed is not None else 0
        print(f"LAND Son Temas | Yukseklik: {yukseklik:.2f} m | Hiz: {hiz:.2f} m/s | ARM: {vehicle.armed}")

        if not vehicle.armed:
            if not disarm_yazildi:
                print(f"Drone basariyla DISARM oldu (ArduPilot). Toplam inis suresi: {time.time() - baslangic_zamani:.2f} sn")
                disarm_yazildi = True
            
            if bekleme_suresi > 0:
                print(f"{bekleme_suresi} saniye muhimmat mekanizmasi operasyonu icin bekleniyor...")
                time.sleep(bekleme_suresi)
                
            return True, 0

        if time.time() - land_baslangic > zaman_asimi:
            print("LAND zaman asimina girdi!")
            return False, 0
        time.sleep(0.1)


def tek_hedef_gorevi(vehicle, hedef, home_noktasi):
    baslangic_zamani = time.time()
    hedef_isim = hedef["isim"]
    hedef_lat = hedef["lat"]
    hedef_lon = hedef["lon"]

    print(f"\n================================ {hedef_isim} GOREVI ================================")

    if not arm_ve_takeoff(vehicle, HEDEF_YUKSEKLIK, home_noktasi):
        return gorev_iptal_guvenli_mod(vehicle, "Kalkis basarisiz")

    if not hedefe_git(vehicle, hedef_lat, hedef_lon, HEDEF_YUKSEKLIK, HEDEF_MESAFE, home_noktasi=home_noktasi):
        return gorev_iptal_guvenli_mod(vehicle, "Hedefe gidilemedi")

    if not hedef_ustunde_stabil_ol(vehicle, hedef_lat, hedef_lon, HEDEF_YUKSEKLIK, home_noktasi=home_noktasi):
        return gorev_iptal_guvenli_mod(vehicle, "Hedef ustunde stabil olunamadi")

    # Hedef noktasına agresif iniş
    inis_sonuc, disarm_suresi = land_ol(vehicle, bekleme_suresi=INIS_SONRASI_BEKLEME)

    if not inis_sonuc:
        print("Hedefte inis basarisiz.")
        return False

    if not arm_ve_takeoff(vehicle, HEDEF_YUKSEKLIK, home_noktasi):
        return gorev_iptal_guvenli_mod(vehicle, "Hedeften tekrar kalkis basarisiz")

    if not hedefe_git(vehicle, home_noktasi["lat"], home_noktasi["lon"], HEDEF_YUKSEKLIK, HEDEF_MESAFE, home_noktasi=home_noktasi):
        return gorev_iptal_guvenli_mod(vehicle, "Home noktasina donulemedi")

    if not hedef_ustunde_stabil_ol(vehicle, home_noktasi["lat"], home_noktasi["lon"], HEDEF_YUKSEKLIK, home_noktasi=home_noktasi):
        return gorev_iptal_guvenli_mod(vehicle, "Home ustunde stabil olunamadi")

    # Home noktasına iniş (Beklemesiz)
    inis_sonuc, disarm_suresi = land_ol(vehicle, bekleme_suresi=0)

    if not inis_sonuc:
        print("Home inisi basarisiz.")
        return False

    bitis_zamani = time.time()
    print(f"{hedef_isim} gorevi toplam {bitis_zamani - baslangic_zamani:.2f} saniyede tamamlandi.\n")
    return True


def simpleGotoMavlink(vehicle, hedefKonum, irtifa):
    current_heading = vehicle.heading if vehicle.heading is not None else 0
    heading_radians = math.radians(current_heading)

    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 3064,
        int(hedefKonum.lat * 1e7), int(hedefKonum.lon * 1e7), irtifa,
        0, 0, 0, 0, 0, 0, heading_radians, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def Gorevler(vehicle):
    print("Gorevler fonksiyonu basladi.")
    genel_baslangic_zamani = time.time()

    if not konum_hazir_mi(vehicle):
        print("Konum hazir degil. Gorev baslatilmadi.")
        return

    HOME_NOKTASI["alt"] = vehicle.location.global_frame.alt if vehicle.location.global_frame.alt is not None else 0
    print(f"\n[BILGI] Home Irtifasi Kaydedildi: {HOME_NOKTASI['alt']:.2f} m")

    ardupilot_inis_parametrelerini_ayarla(vehicle)

    basarili_gorev_sayisi = 0

    # Dongu icindeki temiz eslesme yapildi
    for index, koordinat in enumerate(GOREV_NOKTALARI):
        hedef_sozluk = {
            "isim": f"Hedef_{index + 1}",
            "lat": koordinat[0],
            "lon": koordinat[1]
        }
        
        if tek_hedef_gorevi(vehicle, hedef_sozluk, HOME_NOKTASI):
            basarili_gorev_sayisi += 1
        else:
            print("Gorev hatasi! Dongu durduruluyor.")
            break

    toplam_sure = time.time() - genel_baslangic_zamani
    print("\n================================")
    print(f"TUM GOREV AKISI BITTI | Basarili Hedef: {basarili_gorev_sayisi} | Toplam Sure: {toplam_sure:.2f} sn")
    print("================================\n")


if __name__ == "__main__":
    print("Drone baglaniyor...")
    print("Baglanti adresi:", BAGLANTI_ADRESI)

    vehicle = connect(BAGLANTI_ADRESI, wait_ready=True, timeout=60)

    print("Drone baglandi.")
    print(f"Mod: {vehicle.mode.name} | Armed: {vehicle.armed}")

    try:
        Gorevler(vehicle)
    except KeyboardInterrupt:
        print("Kullanici tarafindan durduruldu.")
        if vehicle.armed:
            vehicle.mode = VehicleMode("LAND")
    finally:
        print("Vehicle kapatiliyor...")
        vehicle.close()