from dronekit import connect 
from dronekit import VehicleMode , LocationGlobalRelative
from pymavlink import mavutil
import math
import time

PORT = "127.0.0.1:14550" 
IRTIFA = 10 
HEDEFLER = {
        0: LocationGlobalRelative( -35.36318007, 149.16513974, IRTIFA),  #örnek konumlar  -35.36318007 149.16513974
        #0: LocationGlobalRelative( 40.74438225, 30.33836027, IRTIFA),  #örnek konumlar 40.74438225 30.33836027 
        #1: LocationGlobalRelative( 40.74421937, 30.33843507, IRTIFA), #40.74421937 30.33843507
        1: LocationGlobalRelative( -35.36330631, 149.16522084, IRTIFA), #-35.36330631 149.16522084 
        #2: LocationGlobalRelative(40.74450123, 30.33856928, IRTIFA), #40.74450123 30.33856928
        2: LocationGlobalRelative(-35.36334179, 149.16524393, IRTIFA), #-35.36334179 149.16524393 
        #3: LocationGlobalRelative(40.74405399, 30.33845291, IRTIFA), #40.74405399 30.33845291 
        3: LocationGlobalRelative(-35.36334261, 149.16518804, IRTIFA), #-35.36334261 149.16518804
    } 
HEDEF_NOKTA_SAYISI = 4
BEKLEME_SURESI = 3 

class IHA : 
    def __init__(self):
        print("ihaya bağlanıyor...")
        self.vehicle = connect(PORT, wait_ready=True)
        self.homeKonum = None
         
    def setMode(self , mode):
            self.vehicle.mode = VehicleMode(mode)
            while self.vehicle.mode.name != mode:
                print(f"{mode} olarak ayarlanıyor.")
                time.sleep(1.5)
            print(f"Mode {mode} olarak ayarlandı.")

    def armVeTakeOff(self, irtifa):
        while not self.vehicle.is_armable:
            print("İHA hazır değil.")
            time.sleep(1)
        print("İHA hazır.")
        self.setMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("ARM bekleniyor.")
            time.sleep(1)
        print("ARM olundu.")
        self.vehicle.simple_takeoff(irtifa)
        while self.vehicle.location.global_relative_frame.alt <= irtifa * 0.9:
            print(f"İrtifa: {self.vehicle.location.global_relative_frame.alt}")
            time.sleep(1)
        print("Takeoff gerçekleşti.")
        self.yaw = self.vehicle.attitude.yaw
        self.vehicle.groundspeed = 2

    def simpleGotoMavlink(self , hedefKonum , irtifa) : 
        current_heading = self.vehicle.heading
        heading_radians = math.radians(current_heading)
        
        print(f"Hedefe yöneliliyor. Mevcut yön sabiti: {current_heading} derece")

        # MAVLink mesajını oluştur
        # Maske (3064): Pozisyon (X,Y,Z) ve Yaw aktif, diğerleri pasif
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 
            3064,    # type_mask
            int(hedefKonum.lat * 1e7), 
            int(hedefKonum.lon * 1e7), 
            irtifa, 
            0, 0, 0, # vx, vy, vz
            0, 0, 0, # afx, afy, afz
            heading_radians, 
            0        # yaw_rate
        )
    
        self.vehicle.send_mavlink(msg)

    def LandMavlink(self , velo) :
        """
        DroneKit message_factory kullanarak WPNAV_SPEED_DN parametresini günceller.
        hiz_cm_s: 10 ile 500 cm/s arasında olmalıdır.
        """
        # Değeri güvenli aralıkta (10-500) sınırla
        print(f"Otonom iniş hızı ayarlanıyor... Hedef: {velo} cm/s")

        # MAVLink PARAM_SET mesajını oluştur
        msg = self.vehicle.message_factory.param_set_encode(
            0, # target_system (0 = yayın/broadcast)
            0, # target_component (0 = yayın/broadcast)
            b'WPNAV_SPEED_DN', # Parametre adı (Mutlaka byte formatında olmalı)
            float(velo), # Parametre değeri
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32 # Parametrenin veri tipi
        )
    
        # Mesajı araca gönder
        self.vehicle.send_mavlink(msg)
        time.sleep(0.4)

    def oklidFark(self, loc1, loc2):
        dlat  = loc2.lat - loc1.lat
        dlong = loc2.lon - loc1.lon
        return math.sqrt(dlat * dlat + dlong * dlong) * 1.113195e5
    
    def hedefeGit(self , hedefKonum):
        mevcut_konum = self.vehicle.location.global_relative_frame
        self.simpleGotoMavlink(hedefKonum , IRTIFA)

        while True:
            mevcut_konum = self.vehicle.location.global_relative_frame
            mesafe = self.oklidFark(mevcut_konum , hedefKonum)
            print(f"hedefe gidiliyor . kalan mesafe : {mesafe}")
            if mesafe <= 0.8 : 
                print("hedefe varıldı . ")
                break
            time.sleep(1)
        print("hedefe ulaşıldı . ") 
        time.sleep(0.5)
    
    def hedefeInis(self):
        print("iniş başladı")
        self.setMode("LAND")
        while True :
            irtifa = self.vehicle.location.global_relative_frame.alt
            print(f"iniş yapılyor . irtifa : {irtifa}")
            if irtifa <= 0.2 : 
                print("iniş tamamlandı . ") 
                break

            time.sleep(0.5)
        print("hedefe ulaşıldi")
        time.sleep(BEKLEME_SURESI)

    def hedefeInisMavlink(self ,velocity,irto) :
        mevcut_konum_lat = self.vehicle.location.global_relative_frame.lat 
        mevcut_konum_lon = self.vehicle.location.global_relative_frame.lon
        konum = LocationGlobalRelative(mevcut_konum_lat,mevcut_konum_lon,irto)
        print("iniş başladı . ") 
        self.LandMavlink(velocity)
        self.simpleGotoMavlink(konum , irto)
        while True :
            irtifa = self.vehicle.location.global_relative_frame.alt
            print(f"iniş yapılyor . irtifa : {irtifa}")
            if irtifa <= irto  + 0.2 : 
                print("iniş tamamlandı . ") 
                break
            time.sleep(0.5)
        time.sleep(0.5)
        print("hedefe ulaşıldi")

    def bekle(self): 
        time.sleep(BEKLEME_SURESI)
    def disArm(self): 
        print("Zorla (Force) DİSARM komutu gönderiliyor...")
        
        # MAV_CMD_COMPONENT_ARM_DISARM mesajı
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,                                          # target system, target component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Komut ID
            0,                                             # Confirmation
            0,                                             # Param 1: 0 = Disarm, 1 = Arm
            21196,                                         # Param 2: 21196 = Zorla (Force) kodu
            0, 0, 0, 0, 0                                  # Param 3-7 (Kullanılmıyor)
        )
        self.vehicle.send_mavlink(msg)
        
        while self.vehicle.armed:
            print("DİSARM bekleniyor...")
            time.sleep(1)
            
        print("Başarıyla DİSARM olundu.")

    def gorev(self ,hedefKonum) : 
        self.armVeTakeOff(IRTIFA) 
        self.hedefeGit(hedefKonum)  
        self.hedefeInisMavlink(250 ,1)
        self.hedefeInisMavlink(250,0)
        self.disArm()
        self.bekle()
        self.armVeTakeOff(IRTIFA)
        self.hedefeGit(self.homeKonum)
        self.hedefeInisMavlink(250 ,1)
        self.hedefeInisMavlink(250 ,0)
        self.disArm()
        self.bekle()


if __name__ == "__main__":
     
    node = IHA()

    while not node.vehicle.location.global_relative_frame:
        time.sleep(1)
    node.homeKonum = node.vehicle.location.global_relative_frame

    sayac = 0 
    while True : 
        node.gorev(HEDEFLER[sayac % HEDEF_NOKTA_SAYISI])
        sayac += 1