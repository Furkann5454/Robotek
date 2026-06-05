from dronekit import connect,VehicleMode
import time
import sys

def arm_ve_takeoff(vehicle,hedef_yukseklik):
    print("5 metre Kalkis komutu geldi.Drone ARM ediliyor...")
    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode.name != "GUIDED": # Mod adı GUIDED değilse bu loopa girer.
        print("GUIDED mod bekleniyor...")
        time.sleep(1)
    
    while not vehicle.is_armable: # ARM edilebilecek durumda değilse is_armable false olduğundan döngüye girer.
        print("Drone henüz ARM edilemez.")
        time.sleep(1)
    
    vehicle.armed = True # Drone ARM edildi.
    while not vehicle.armed:
        print("ARM olmasi bekleniyor...")
        vehicle.armed = True
        time.sleep(1)

    print("ARM edildi.")
    print("Mod:", vehicle.mode.name)
    print("Armed:", vehicle.armed)
    print("Konum:", vehicle.location.global_relative_frame)

    print(f"{hedef_yukseklik} metreye kalkis basliyor ...")
    vehicle.simple_takeoff(hedef_yukseklik) # Takeoff verme fonksiyonu

    baslangic_zamani = time.time()
    while True:
        yukseklik = vehicle.location.global_relative_frame.alt
        print(
            f"Anlik yukseklik: {yukseklik:.2f} m | "
            f"Mod: {vehicle.mode.name} | "
            f"ARM: {vehicle.armed}"
        )

        if yukseklik >= hedef_yukseklik*0.95:
            print("Hedef yukseklikteyiz...")
            break
        if time.time() - baslangic_zamani > 20:
            print("Takeoff zaman asimina girdi. Drone hedef yukseklige ulasamadi.")
            break

        time.sleep(1)

def hedefe_git(vehicle):
    vehicle.simple_goto()




def land_ve_disarm(vehicle):
    print("LAND moduna geciliyor...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.mode.name != "LAND":
        print("LAND moda gecmesi bekleniyor...")
        print("Mevcut mod:", vehicle.mode.name)
        time.sleep(1)
    
    print("İnis basladi ...")

    while True:
        yukseklik = vehicle.location.global_relative_frame.alt
        print(
            f"Anlik yukseklik: {yukseklik:.2f} m | "
            f"Mod: {vehicle.mode.name} | "
            f"ARM: {vehicle.armed}"
        )
        if yukseklik <= 0.2:
            print("İnis basarili.")
            break

        time.sleep(1)