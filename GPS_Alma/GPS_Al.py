from dronekit import connect
import time
import sys

BAGLANTI_ADRESI = sys.argv[1] if len(sys.argv) > 1 else "tcp:192.168.30.3:5760"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else None

ORNEK_SAYISI = 20
ORNEK_ARALIGI = 0.2


def baglan():
    print("Drone'a baglaniliyor...")
    print("Baglanti:", BAGLANTI_ADRESI)

    if BAUD is None:
        vehicle = connect(BAGLANTI_ADRESI, wait_ready=True, timeout=60)
    else:
        vehicle = connect(BAGLANTI_ADRESI, wait_ready=True, baud=BAUD, timeout=60)

    print("Baglandi.")
    return vehicle


def gps_hazir_bekle(vehicle):
    print("GPS hazir olmasi bekleniyor...")

    while True:
        konum = vehicle.location.global_relative_frame
        gps = vehicle.gps_0

        print(
            f"GPS Fix: {gps.fix_type} | "
            f"Uydu: {gps.satellites_visible} | "
            f"EKF: {vehicle.ekf_ok} | "
            f"Lat: {konum.lat} | Lon: {konum.lon}"
        )

        if (
            konum.lat is not None and
            konum.lon is not None and
            gps.fix_type is not None and
            gps.fix_type >= 3 and
            vehicle.ekf_ok
        ):
            print("GPS hazir.")
            return

        time.sleep(1)


def gps_ortalama_al(vehicle):
    latlar = []
    lonlar = []

    print(f"{ORNEK_SAYISI} adet GPS ornegi aliniyor...")

    for i in range(ORNEK_SAYISI):
        konum = vehicle.location.global_relative_frame

        if konum.lat is not None and konum.lon is not None:
            latlar.append(konum.lat)
            lonlar.append(konum.lon)

        print(
            f"{i + 1}/{ORNEK_SAYISI} | "
            f"Lat: {konum.lat} | Lon: {konum.lon}"
        )

        time.sleep(ORNEK_ARALIGI)

    if len(latlar) == 0:
        print("GPS verisi alinamadi.")
        return None, None

    lat_ort = sum(latlar) / len(latlar)
    lon_ort = sum(lonlar) / len(lonlar)

    return lat_ort, lon_ort


def main():
    vehicle = baglan()

    try:
        gps_hazir_bekle(vehicle)

        sayac = 1

        while True:
            komut = input("\nNoktaya drone'u koy. GPS almak icin ENTER, cikmak icin q yaz: ")

            if komut.lower() == "q":
                break

            lat, lon = gps_ortalama_al(vehicle)

            if lat is None:
                continue

            print("\n==============================")
            print(f"NOKTA_{sayac}")
            print(f"lat = {lat:.8f}")
            print(f"lon = {lon:.8f}")
            print("------------------------------")
            print("Python'a yapistirmalik:")
            print("{")
            print(f'    "isim": "Hedef_{sayac}",')
            print(f'    "lat": {lat:.8f},')
            print(f'    "lon": {lon:.8f}')
            print("}")
            print("==============================\n")

            sayac += 1

    finally:
        print("Baglanti kapatiliyor...")
        vehicle.close()


if __name__ == "__main__":
    main()