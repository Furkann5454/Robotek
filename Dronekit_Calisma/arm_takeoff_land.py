from dronekit import connect,VehicleMode
import time



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

        

    print("DISARM ediliyor...")
    vehicle.armed = False
    while vehicle.armed:
        print("Drone indi ama hala ARM modda. DISARM olmasi bekleniyor...")
        vehicle.armed = False
        time.sleep(1)
    
    print("Drone DISARM oldu.")


baglanma_portu = "udp:127.0.0.1:14550"

print("Drone 1'e baglaniyor...")

vehicle = connect(baglanma_portu,wait_ready= True)
    # Burada Dronekit UDP port bağlantisini acar, İlgili adresi dinler ve oradan MAVlink
    # mesajı bekler.Gelen heartbeat sonrasinda vehicle nesnesini verir.

print("Mevcut durum :",vehicle.mode.name)
print("ARM drumu :",vehicle.armed)

arm_ve_takeoff(vehicle,5)
time.sleep(1)

land_ve_disarm(vehicle)

print("Baglanti kapatildi.")
vehicle.close()


