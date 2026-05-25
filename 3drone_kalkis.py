from dronekit import connect,VehicleMode
import time
import sys

def arm_ve_takeoff(vehicle,hedef_yukseklik,drone_numarasi):
    print(f"{drone_numarasi} icin 5 metre Kalkis komutu geldi.Drone ARM ediliyor...")
    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode.name != "GUIDED": # Mod adı GUIDED değilse bu loopa girer.
        print(f"{drone_numarasi}GUIDED mod bekleniyor...")
        time.sleep(1)
    
    while not vehicle.is_armable: # ARM edilebilecek durumda değilse is_armable false olduğundan döngüye girer.
        print(f"{drone_numarasi}Drone henüz ARM edilemez.")
        time.sleep(1)
    
    vehicle.armed = True # Drone ARM edildi.
    while not vehicle.armed:
        print(f"{drone_numarasi}ARM olmasi bekleniyor...")
        vehicle.armed = True
        time.sleep(1)

    print(f"{drone_numarasi}ARM edildi.")
    print(f"{drone_numarasi}Mod:", vehicle.mode.name)
    print(f"{drone_numarasi}Armed:", vehicle.armed)
    print(f"{drone_numarasi}Konum:", vehicle.location.global_relative_frame)

    print(f"{hedef_yukseklik} metreye kalkis basliyor ...")
    vehicle.simple_takeoff(hedef_yukseklik) # Takeoff verme fonksiyonu

    baslangic_zamani = time.time()
    while True:
        yukseklik = vehicle.location.global_relative_frame.alt
        print(
            f"{drone_numarasi} Anlik yukseklik: {yukseklik:.2f} m | "
            f"Mod: {vehicle.mode.name} | "
            f"ARM: {vehicle.armed}"
        )

        if yukseklik >= hedef_yukseklik*0.95:
            print(f"{drone_numarasi} Hedef yukseklikteyiz...")
            break
        if time.time() - baslangic_zamani > 20:
            print("Takeoff zaman asimina girdi. Drone hedef yukseklige ulasamadi.")
            break

        time.sleep(1)
    
def land_ve_disarm(vehicle,drone_numarasi):
    print(f"{drone_numarasi}LAND moduna geciliyor...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.mode.name != "LAND":
        print(f"{drone_numarasi}LAND moda gecmesi bekleniyor...")
        print("Mevcut mod:", vehicle.mode.name)
        time.sleep(1)
    
    print(f"{drone_numarasi} İnis basladi ...")

    while True:
        yukseklik = vehicle.location.global_relative_frame.alt
        print(
            f"{drone_numarasi} Anlik yukseklik: {yukseklik:.2f} m | "
            f"Mod: {vehicle.mode.name} | "
            f"ARM: {vehicle.armed}"
        )
        if yukseklik <= 0.2:
            print("İnis basarili.")
            break

        time.sleep(1)
        
    print(f"{drone_numarasi}DISARM ediliyor...")
    vehicle.armed = False
    while vehicle.armed:
        print(f"{drone_numarasi}Drone indi ama hala ARM modda. DISARM olmasi bekleniyor...")
        vehicle.armed = False
        time.sleep(1)
    
    print("Drone DISARM oldu.")

drone_no = int(sys.argv[1])

droneler = [
    {"id" : 1 , "SITL_port" : "udp:127.0.0.1:14550"},
    {"id" : 2 , "SITL_port" : "udp:127.0.0.1:14560"},
    {"id" : 3 , "SITL_port" : "udp:127.0.0.1:14570"}
]

secili_drone = None

for drone in droneler:
    if drone["id"] == drone_no:
        secili_drone = drone
        break

if secili_drone is None:
    print("Gecersiz drone numarasi. 1, 2 veya 3 gir.")
    sys.exit()

baglanma_portu = secili_drone["SITL_port"]
print(f"Drone {drone_no} baglaniyor...")
print(f"Baglanma portu: {baglanma_portu}")

vehicle = connect(secili_drone["SITL_port"],wait_ready=True,timeout=60)

print(f"Drone {drone_no} baglandi.")
print("Mevcut durum:", vehicle.mode.name)
print("ARM durumu:", vehicle.armed)

arm_ve_takeoff(vehicle, 5, drone_no)

time.sleep(3)

land_ve_disarm(vehicle, drone_no)

print(f"Drone {drone_no} baglantisi kapatiliyor...")
vehicle.close()

print("Program bitti.")







