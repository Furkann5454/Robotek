from dronekit import connect
import time
import sys

BAGLANTI = sys.argv[1] if len(sys.argv) > 1 else "udp:127.0.0.1:14550"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else None
ORNEK = 1
ARALIK = 0.2

print("Baglaniliyor:", BAGLANTI)
vehicle = connect(BAGLANTI, wait_ready=True, baud=BAUD, timeout=60) if BAUD else connect(BAGLANTI, wait_ready=True, timeout=60)
print("Baglandi.")

print("GPS bekleniyor...")
while True:
    k = vehicle.location.global_relative_frame
    g = vehicle.gps_0
    print(f"Fix:{g.fix_type} Uydu:{g.satellites_visible} EKF:{vehicle.ekf_ok} Lat:{k.lat} Lon:{k.lon}")
    if k.lat and k.lon and g.fix_type >= 3 and vehicle.ekf_ok:
        print("GPS hazir.")
        break
    time.sleep(1)

n = 1
try:
    while True:
        girdi = input("\nDrone'u noktaya koy, ENTER = konum al, q = cik: ")
        if girdi.lower() == "q":
            break

        latlar, lonlar = [], []
        for i in range(ORNEK):
            k = vehicle.location.global_relative_frame
            if k.lat and k.lon:
                latlar.append(k.lat)
                lonlar.append(k.lon)
            print(f"{i+1}/{ORNEK} Lat:{k.lat} Lon:{k.lon}")
            time.sleep(ARALIK)

        if not latlar:
            print("Veri alinamadi.")
            continue

        lat = sum(latlar) / len(latlar)
        lon = sum(lonlar) / len(lonlar)

        print(f"\n=== NOKTA_{n} ===")
        print(f"lat={lat:.8f}, lon={lon:.8f}")
        print(f'{{"lat": {lat:.8f}, "lon": {lon:.8f}}}')
        n += 1

finally:
    vehicle.close()
    print("Kapatildi.")