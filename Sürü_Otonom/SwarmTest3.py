import time
import threading
import zmq
import json
import math
from dronekit import connect, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil
import numpy as np

nodes = [
    {"id": "1", "sitl_port": "tcp:192.168.30.3:5760", "pub": "5555", "subs": ["127.0.0.1:5556", "127.0.0.1:5557"]},
    {"id": "2", "sitl_port": "tcp:192.168.30.7:5760", "pub": "5556", "subs": ["127.0.0.1:5555", "127.0.0.1:5557"]},
    #{"id": "3", "sitl_port": "tcp:127.0.0.1:5782", "pub": "5557", "subs": ["127.0.0.1:5555", "127.0.0.1:5556"]},
]

YKI_PUB_PORT = "5599"
YKI_ADDR = "127.0.0.1:5599"

DRONE_IRTIFALARI = {
    "1":  10.0,
    "2":  12.5,
    "3": 10.0,
}
# ─────────────────────────────────────────────
#  Drone Durum Sabitleri
# ─────────────────────────────────────────────
DURUM_BEKLE      = "BEKLE"
DURUM_FORMASYON  = "FORMASYON"
DURUM_NAVIGASYON = "NAVIGASYON"
DURUM_MANEVRA    = "MANEVRA"


def get_location_metres(original_location, dNorth, dEast, dUp=0.0):
    R = 6378137.0
    dLat = dNorth / R
    dLon = dEast / (R * math.cos(math.pi * original_location.lat / 180))
    return LocationGlobalRelative(
        original_location.lat + math.degrees(dLat),
        original_location.lon + math.degrees(dLon),
        original_location.alt + dUp,
    )


class SwarmNode:
    def __init__(self, drone_id):
        self.drone_id = str(drone_id)
        self.sitl_port = None
        self.pub_port = None
        self.subs_port = None
        self.peer_states = {}
        self.merkezLoc = None

        self.peer_lock    = threading.Lock()
        self.ihale_lock   = threading.Lock()
        self.durum_lock   = threading.Lock()   
        self.manevra_lock = threading.Lock()

        # ── Drone Durum Makinesi ──────────────────────────────────────────
        # BEKLE | FORMASYON | NAVIGASYON | MANEVRA
        self._durum = DURUM_BEKLE

        # ── Formasyon değişkenleri ────────────────────────────────────────
        self.formasyon_tipi   = None          # "V" | "OK" | "DUZ"
        self.aktif_offset_id  = None
        self.noktalar         = {}
        self.hedefKonumlar    = {}
        self.formasyonMatris  = None
        self.maliyetDizi      = np.zeros(1)

        # Formasyon thread'i
        self._formasyon_thread    = None
        self._formasyon_dur_event = threading.Event()

        # ── Navigasyon değişkenleri ───────────────────────────────────────
        self.sanal_merkez_loc     = None
        self.sanal_merkez_heading = 0.0
        self.hedef_heading        = 0.0
        self.rotasyon_hizi        = 15.0
        self.nihai_hedef_loc      = None
        self.suru_hizi_ms         = 2.0
        self.aktif_hedef_id       = None

        # Navigasyon thread'i
        self._navigasyon_thread    = None
        self._navigasyon_dur_event = threading.Event()

        # ── Manevra değişkenleri ──────────────────────────────────────────
        self.euler_offset      = np.zeros(3)
        self.son_manevra       = None
        self.islenen_manevra_id = -1

        # ── Ortak değişkenler ─────────────────────────────────────────────
        self.yaw         = None
        self.droneSay    = None
        self.aktif_dronelar = []

        self.ihale_hedefim  = -1
        self.ihale_teklifim = 9999.0
        self.ihale_panosu   = {}

        self.hazir_lock      = threading.Lock()
        self.takeoff_irtifa  = None
        self.takeoff_yapildi = False


        self.benim_irtifam = DRONE_IRTIFALARI.get(self.drone_id, 10.0)
        print(f"[Drone-{self.drone_id}] Katman irtifası: {self.benim_irtifam} m")

        # Ağdan gelen son komut (kilitleme sorununu önlemek için kuyruk)
        self._komut_kuyruk      = []
        self._komut_kuyruk_lock = threading.Lock()

        for n in nodes:
            if self.drone_id == n["id"]:
                self.sitl_port = n["sitl_port"]
                self.pub_port  = n["pub"]
                self.subs_port = n["subs"]

        print(f"{self.drone_id} SITL'e bağlanıyor.")
        self.vehicle = connect(self.sitl_port, wait_ready=True)
        self.context = zmq.Context()

    # ═══════════════════════════════════════════════════════════════════
    #  DURUM YÖNETİMİ
    # ═══════════════════════════════════════════════════════════════════

    @property
    def durum(self):
        with self.durum_lock:
            return self._durum

    @durum.setter
    def durum(self, yeni):
        with self.durum_lock:
            self._durum = yeni
        print(f"[Drone-{self.drone_id}] Durum → {yeni}")

    # ═══════════════════════════════════════════════════════════════════
    #  ZMQ PUBLISH
    # ═══════════════════════════════════════════════════════════════════

    def publish(self):
        pub_socket = self.context.socket(zmq.PUB)
        pub_socket.bind(f"tcp://*:{self.pub_port}")

        while True:
            loc = self.vehicle.location.global_relative_frame
            data = {
                "id": self.drone_id,
                "lat": loc.lat,
                "lon": loc.lon,
                "alt": loc.alt,
                "durum": self.durum,
                "formasyon_tipi": self.formasyon_tipi,
                "hazir": self.takeoff_yapildi,
                "ihale_hedef": self.ihale_hedefim,
                "ihale_maliyet": self.ihale_teklifim,
                "hedef_id": self.aktif_hedef_id,
                "zaman": time.time(),
                "manevra": self.son_manevra,
            }
            pub_socket.send_json(data)
            time.sleep(0.3)

    # ═══════════════════════════════════════════════════════════════════
    #  ZMQ SUBSCRIBE  –  HİÇBİR ŞEYİ BLOKLAMAZ
    # ═══════════════════════════════════════════════════════════════════

    def subscribe(self):
        sub_socket = self.context.socket(zmq.SUB)
        for addr in self.subs_port:
            sub_socket.connect(f"tcp://{addr}")
            print(f"Drone-{self.drone_id}, {addr} adresini dinliyor...")
        sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        while True:
            peer_data = sub_socket.recv_json()
            # Peer durumunu sakla
            self.sakla(peer_data)

            # Formasyon komutu
            gelen_tip = peer_data.get("formasyon_tipi")
            if gelen_tip and gelen_tip != self.formasyon_tipi:
                self._formasyon_komutu_isle(gelen_tip)

            # Manevra
            gelen_manevra = peer_data.get("manevra")
            if gelen_manevra:
                self._manevra_uygula_eger_yeni(gelen_manevra)

    # ═══════════════════════════════════════════════════════════════════
    #  YKİ DİNLE  –  HİÇBİR ŞEYİ BLOKLAMAZ
    # ═══════════════════════════════════════════════════════════════════

    def yki_dinle(self):
        yki_sub = self.context.socket(zmq.SUB)
        yki_sub.connect(f"tcp://{YKI_ADDR}")
        yki_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        print(f"[Drone-{self.drone_id}] YKİ dinliyor.")

        while True:
            try:
                veri = yki_sub.recv_json()
                komut = veri.get("komut", "")

                if komut == "BAŞLAT" and not self.takeoff_yapildi:
                    self.takeoff_irtifa = self.benim_irtifam
                    print("Başlat Komutu Alındı.")
                    threading.Thread(target=self.senkron_Takeoff, daemon=True).start()

                elif komut == "FORMASYON":
                    tip = veri.get("tip", "")
                    if tip:
                        self._formasyon_komutu_isle(tip)

                elif komut == "NAVIGASYON":
                    hedef = veri.get("hedef")      # LocationGlobalRelative dict
                    hedef_id = veri.get("hedef_id")
                    if hedef:
                        loc = LocationGlobalRelative(hedef["lat"], hedef["lon"], hedef["alt"])
                        self._navigasyon_komutu_isle(loc, hedef_id)

                elif komut == "RTL":
                    pass

            except Exception as e:
                print(f"[Drone-{self.drone_id}] YKİ komut hatası: {e}")

    # ═══════════════════════════════════════════════════════════════════
    #  FORMASYON – Ayrı Thread
    # ═══════════════════════════════════════════════════════════════════

    def _formasyon_komutu_isle(self, tip):
        """Subscribe/YKI thread'inden çağrılır, hemen döner."""

        with self.durum_lock: 
            if self._durum != DURUM_BEKLE : 
                print(f"[Drone-{self.drone_id}] Formasyon komutu reddedildi. Drone şu an '{self._durum}' durumunda.")
                return
            
            self._durum = DURUM_FORMASYON
            print(f"[Drone-{self.drone_id}] Durum → {self._durum}")

        threading.Thread(
            target=self._formasyon_baslat,
            args=(tip,),
            daemon=True
        ).start()

    def _formasyon_baslat(self, tip):
        """
        Formasyon thread'i:
          1. Eski formasyon thread'ini durdur
          2. Sanal merkezi belirle
          3. İhale ile offset belirle
          4. Hedefe simple_goto ile git
          5. Hedefe varınca kapat, durum → BEKLE
        """
        # Eski formasyon thread'ini durdur
        self._formasyon_dur_event.set()
        if self._formasyon_thread and self._formasyon_thread.is_alive():
            self._formasyon_thread.join(timeout=2.0)
        self._formasyon_dur_event.clear()

        # Navigasyon thread'ini de durdur (formasyon geçişi sırasında)
        self._navigasyon_dur_event.set()
        if self._navigasyon_thread and self._navigasyon_thread.is_alive():
            self._navigasyon_thread.join(timeout=2.0)
        self._navigasyon_dur_event.clear()

        self.formasyon_tipi = tip

        # Peer verilerini bekle
        self.tumVeriyiBekle()
        with self.peer_lock:
            snapshot = dict(self.peer_states)

        # Sanal merkez
        self.sanalMerkeziBelirle(snapshot)
        if not self.sanal_merkez_loc:
            print(f"[Drone-{self.drone_id}] Sanal merkez yok, formasyon iptal.")
            self.durum = DURUM_BEKLE
            return

        # Matris & noktalar
        if tip == "V":
            self.matrisYapV(self.droneSay)
            self.noktalar = self.mesafeleriHesapla(self.formasyonMatris, 2.5 * math.sqrt(3), 7.5)
        elif tip == "OK":
            self.matrisYapok(self.droneSay)
            self.noktalar = self.mesafeleriHesapla(self.formasyonMatris, 2.5 * math.sqrt(3), 7.5)
        else:  # DUZ
            self.matrisYapDuz(self.droneSay)
            self.noktalar = self.mesafeleriHesapla(self.formasyonMatris, 5, 15)

        with self.manevra_lock:
            self.euler_offset = np.zeros(3)

        self.noktalarıKonumaÇevir()
        self.MatrisHesap(snapshot)

        # İhale ile offset belirle
        benim_offset_id = self.hedefKarar()
        self.aktif_offset_id = benim_offset_id

        # Hedef konumu hesapla
        hedef_konum = self._offset_konumunu_hesapla()
        if not hedef_konum:
            print(f"[Drone-{self.drone_id}] Hedef konum hesaplanamadı.")
            self.durum = DURUM_BEKLE
            return

        # Formasyon thread'ini başlat
        self._formasyon_thread = threading.Thread(
            target=self._formasyon_konumuna_git,
            args=(hedef_konum,),
            daemon=True
        )
        self._formasyon_thread.start()

    def _formasyon_konumuna_git(self, hedef_konum):
        """
        Formasyon thread'i:
        Drone formasyon konumuna gider, hedefe varınca kapanır.
        """
        print(f"[Drone-{self.drone_id}] Formasyon konumuna gidiliyor. Offset: {self.aktif_offset_id}")
        dongu_suresi = 0.3
        self.vehicle.groundspeed = 3

        while not self._formasyon_dur_event.is_set():
            # Euler offset uygulayarak güncel hedef konumu
            guncel_hedef = self._offset_konumunu_hesapla()
            if not guncel_hedef:
                break
            self.simpleGotoMavlink(guncel_hedef)

            # Hedefe ulaştık mı?
            mesafe = self.oklidFark(
                self.vehicle.location.global_relative_frame, guncel_hedef
            )
            if mesafe < 1.0:
                print(f"[Drone-{self.drone_id}] Formasyon konumuna ulaşıldı. Thread kapanıyor.")
                break

            time.sleep(dongu_suresi)

        # Formasyon tamamlandı — u BEKLE'ye çek
        # (Navigasyon aktifse o thread zaten bağımsız çalışır)
        if not self._navigasyon_dur_event.is_set():
            if self.durum == DURUM_FORMASYON:
                self.durum = DURUM_BEKLE

    # ═══════════════════════════════════════════════════════════════════
    #  NAVİGASYON – Ayrı Thread
    # ═══════════════════════════════════════════════════════════════════

    def _navigasyon_komutu_isle(self, hedef_loc, hedef_id=None):
        """YKI/keyboard'dan çağrılır. BEKLE durumunda ve formasyon çalışmıyorsa kabul edilir."""
        with self.durum_lock:
            # 1. Drone BEKLE durumunda mı?
            if self._durum != DURUM_BEKLE:
                print(f"[Drone-{self.drone_id}]  Navigasyon komutu reddedildi. Drone şu an '{self._durum}' durumunda.")
                return
            
            # 2. Formasyon thread'i hala çalışıyor mu?
            if self._formasyon_thread and self._formasyon_thread.is_alive():
                print(f"[Drone-{self.drone_id}]  Navigasyon komutu reddedildi. Formasyon işlemi hala devam ediyor!")
                return
                
            # 3. Formasyon başarıyla kurulmuş mu? (Merkez ve offset var mı?)
            if self.aktif_offset_id is None or not self.sanal_merkez_loc:
                print(f"[Drone-{self.drone_id}]  Navigasyon komutu reddedildi. Önce formasyon kurulmalı!")
                return

            # Tüm şartlar sağlandıysa durumu Navigasyon yap
            self._durum = DURUM_NAVIGASYON
            print(f"[Drone-{self.drone_id}] Durum → {self._durum}")

        # Şartlar sağlandı, thread'i başlat
        threading.Thread(
            target=self._navigasyon_baslat,
            args=(hedef_loc, hedef_id),
            daemon=True
        ).start()

    def _navigasyon_baslat(self, hedef_loc, hedef_id=None):
        """
        Navigasyon thread'i:
          1. Eski navigasyon thread'ini durdur
          2. Sürü merkezini hedefe doğru hareket ettir
          3. Hedefe varınca kapat, durum → BEKLE
        """

        # Eski navigasyon thread'ini durdur (Güvenlik amaçlı bırakıldı)
        self._navigasyon_dur_event.set()
        if self._navigasyon_thread and self._navigasyon_thread.is_alive():
            self._navigasyon_thread.join(timeout=2.0)
        self._navigasyon_dur_event.clear()

        self.nihai_hedef_loc = hedef_loc
        self.aktif_hedef_id  = hedef_id
        
        # NOT: self.durum = DURUM_NAVIGASYON atamasını sildik çünkü yukarıda yapıldı.

        print(f"[Drone-{self.drone_id}] Navigasyon başladı. Hedef ID: {hedef_id}")

        self._navigasyon_thread = threading.Thread(
            target=self._navigasyon_dongusu,
            daemon=True
        )
        self._navigasyon_thread.start()

    def _navigasyon_dongusu(self):
        """
        Navigasyon thread'i:
        Sanal merkezi hedefe doğru kaydırır.
        Her adımda drone kendi offset konumuna simple_goto ile gider.
        Sürü merkezi hedefe ulaşınca thread kapanır.
        """
        dongu_suresi = 0.2
        print(f"[Drone-{self.drone_id}] Navigasyon döngüsü aktif.")

        # --- YENİ EKLENEN DEĞİŞKEN ---
        bekleme_baslangic = None 

        while not self._navigasyon_dur_event.is_set():
            if not self.sanal_merkez_loc or not self.nihai_hedef_loc:
                time.sleep(dongu_suresi)
                continue

            # Hedef yönünü hesapla
            self.hedef_heading = self.hedefeAciHesapla(
                self.sanal_merkez_loc, self.nihai_hedef_loc
            )

            # Heading'i hedefe doğru döndür
            fark = self.aciFarki(self.hedef_heading, self.sanal_merkez_heading)
            max_donus = self.rotasyon_hizi * dongu_suresi

            if abs(fark) <= max_donus:
                self.sanal_merkez_heading = self.hedef_heading
            else:
                yon = 1 if fark > 0 else -1
                self.sanal_merkez_heading = (
                    self.sanal_merkez_heading + yon * max_donus
                ) % 360
            
            time.sleep(0.05)

            # Sanal merkezi ilerlet
            if abs(fark) <= 5.0:
                # --- YENİ BEKLEME MANTIĞI ---
                # Eğer hedeflenen açıya yeni ulaştıysak, zamanlayıcıyı başlat
                if bekleme_baslangic is None:
                    bekleme_baslangic = time.time()
                    print(f"[Drone-{self.drone_id}] Yönelim tamamlandı, droneların yerleşmesi için 1 saniye bekleniyor...")

                # Zamanlayıcı 1 saniyeyi geçtiyse ilerlemeye başla
                if time.time() - bekleme_baslangic >= 1.0:
                    mesafe = self.oklidFark(self.sanal_merkez_loc, self.nihai_hedef_loc)
                    if mesafe > 0.5:
                        adim = min(self.suru_hizi_ms * dongu_suresi, mesafe)
                        d_kuzey = adim * math.cos(math.radians(self.hedef_heading))
                        d_dogu  = adim * math.sin(math.radians(self.hedef_heading))
                        yeni_konum = get_location_metres(self.sanal_merkez_loc, d_kuzey, d_dogu)
                        # Sadece yatay konumu güncelle, irtifayı sabitle
                        self.sanal_merkez_loc = LocationGlobalRelative(
                            yeni_konum.lat, yeni_konum.lon, self.benim_irtifam
                        )
                    else:
                        # Hedefe ulaşıldı
                        print(f"[Drone-{self.drone_id}] Navigasyon tamamlandı. Thread kapanıyor.")
                        self.nihai_hedef_loc = None
                        self.aktif_hedef_id  = None
                        self.durum = DURUM_BEKLE
                        break
            else:
                # Açı farkı 5 dereceden büyükse (dönüş devam ediyorsa veya yeni hedef verildiyse) sayacı sıfırla
                bekleme_baslangic = None

            # Drone'u güncel offset konumuna gönder
            # Bekleme süresi boyunca burası çalışmaya devam edeceği için dronelar yeni açılarına uçup yerleşecekler
            guncel_hedef = self._offset_konumunu_hesapla()
            if guncel_hedef:
                self.simpleGotoMavlink(guncel_hedef)

            time.sleep(dongu_suresi)

    # ═══════════════════════════════════════════════════════════════════
    #  MANEVRA – Kısa süreli, ayrı thread gerekmiyor
    # ═══════════════════════════════════════════════════════════════════

    def _manevra_yayinla(self, tip: str, aci: float):
        zaman_id = time.time()
        self.son_manevra = {
            "tip": tip,
            "aci": aci,
            "kaynak": self.drone_id,
            "ts": zaman_id,
        }
        self.islenen_manevra_id = zaman_id
        self._manevra_hesapla_ve_uygula(tip, aci)

    def _manevra_uygula_eger_yeni(self, manevra: dict):
        if not manevra:
            return
        ts     = manevra.get("ts", -1)
        kaynak = manevra.get("kaynak", "")
        if kaynak == self.drone_id:
            return
        if abs(ts - self.islenen_manevra_id) < 0.001:
            return
        self.islenen_manevra_id = ts
        tip = manevra.get("tip", "")
        aci = manevra.get("aci", 0.0)
        self._manevra_hesapla_ve_uygula(tip, aci)

    def _manevra_hesapla_ve_uygula(self, tip: str, aci: float):
        if self.aktif_offset_id is None or not self.sanal_merkez_loc:
            print(f"[Drone-{self.drone_id}] Manevra: önce formasyon kur.")
            return

        # 1. SIFIRLA KONTROLÜ
        if tip == "SIFIRLA":
            with self.manevra_lock:
                self.euler_offset = np.zeros(3)
            print(f"[Drone-{self.drone_id}] Euler offset sıfırlandı.")

        # 2. YAW KONTROLÜ (Kesinlikle elif olmalı)
        elif tip == "YAW":
            self.sanal_merkez_heading = (self.sanal_merkez_heading + math.degrees(aci)) % 360
            print(f"[Drone-{self.drone_id}] YAW uygulandı → heading {self.sanal_merkez_heading:.1f}°")

        # 3. ROLL VEYA PITCH KONTROLÜ (Kesinlikle elif olmalı)
        elif tip == "ROLL" or tip == "PITCH":
            if tip == "ROLL":
                phi = aci
                R = np.array([
                    [1, 0,           0           ],
                    [0, np.cos(phi), -np.sin(phi)],
                    [0, np.sin(phi),  np.cos(phi)],
                ])
            else: # PITCH
                theta = aci
                R = np.array([
                    [ np.cos(theta), 0, np.sin(theta)],
                    [0,              1, 0             ],
                    [-np.sin(theta), 0, np.cos(theta) ],
                ])

            mesafe_ofset = self.noktalar[self.aktif_offset_id]
            rk, rd = self.rotasyonUygula(
                mesafe_ofset["kuzey"], mesafe_ofset["dogu"], self.sanal_merkez_heading
            )
            with self.manevra_lock:
                e = self.euler_offset.copy()

            V     = np.array([rk + e[0], rd + e[1], e[2]])
            V_new = R @ V

            with self.manevra_lock:
                self.euler_offset = np.array([
                    V_new[0] - rk,
                    V_new[1] - rd,
                    V_new[2],
                ])
            print(f"[Drone-{self.drone_id}] {tip} uygulandı → euler_offset {self.euler_offset}")
            
        # 4. HİÇBİRİ DEĞİLSE
        else:
            print(f"[Drone-{self.drone_id}] Bilinmeyen manevra tipi: {tip}")
            return # Sadece hatalı (örneğin yazım yanlışı olan) bir komut gelirse buradan çıkar, aşağıya gitmez.

        # 5. DRONE HAREKET KISMI (Else'e düşmezse tüm komutlar için burası çalışır)
        if self.durum == DURUM_BEKLE:
            yeni_hedef = self._offset_konumunu_hesapla()
            if yeni_hedef:
                self.simpleGotoMavlink(yeni_hedef)
    def roll_manevra(self, phi):
        print(f"[Drone-{self.drone_id}] ROLL {math.degrees(phi):.1f}° yayınlanıyor.")
        self._manevra_yayinla("ROLL", phi)

    def pitch_manevra(self, theta):
        print(f"[Drone-{self.drone_id}] PITCH {math.degrees(theta):.1f}° yayınlanıyor.")
        self._manevra_yayinla("PITCH", theta)

    def yaw_manevra(self, psi):
        print(f"[Drone-{self.drone_id}] YAW {math.degrees(psi):.1f}° yayınlanıyor.")
        self._manevra_yayinla("YAW", psi)

    def euler_sifirla(self):
        print(f"[Drone-{self.drone_id}] SIFIRLA yayınlanıyor.")
        self._manevra_yayinla("SIFIRLA", 0.0)

    # ═══════════════════════════════════════════════════════════════════
    #  YARDIMCI: Offset Konumu Hesapla
    # ═══════════════════════════════════════════════════════════════════

    def _offset_konumunu_hesapla(self):
        """Aktif offset + euler_offset → hedef konumu döndürür."""
        if self.aktif_offset_id is None or not self.sanal_merkez_loc:
            return None
        if self.aktif_offset_id not in self.noktalar:
            return None

        mesafe_ofset = self.noktalar[self.aktif_offset_id]
        r_kuzey, r_dogu = self.rotasyonUygula(
            mesafe_ofset["kuzey"], mesafe_ofset["dogu"], self.sanal_merkez_heading
        )
        with self.manevra_lock:
            e = self.euler_offset.copy()

        hedef_yatay = get_location_metres(
            self.sanal_merkez_loc, r_kuzey + e[0], r_dogu + e[1]
        )
        return LocationGlobalRelative(
            hedef_yatay.lat, hedef_yatay.lon, self.benim_irtifam
        )

    # ═══════════════════════════════════════════════════════════════════
    #  TAKEOFF
    # ═══════════════════════════════════════════════════════════════════

    def simpleGotoMavlink(self , hedefKonum) : 
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
            hedefKonum.alt, 
            0, 0, 0, # vx, vy, vz
            0, 0, 0, # afx, afy, afz
            heading_radians, 
            0        # yaw_rate
        )
        
        # Mesajı araca gönder
        self.vehicle.send_mavlink(msg)

    def senkron_Takeoff(self):
        beklenen = len(nodes)
        while not self.vehicle.is_armable:
            print(f"[Drone-{self.drone_id}] İHA Hazır değil.")
            time.sleep(1)
        print(f"[Drone-{self.drone_id}] İHA hazır.")

        self.setMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(f"[Drone-{self.drone_id}] ARM bekleniyor.")
            time.sleep(1)
        self.takeoff_yapildi = True  
        print(f"[Drone-{self.drone_id}] ARM OK.")

        print(f"[Drone-{self.drone_id}] Diğer droneları bekliyor.")

        timeout   = 15.0
        baslangic = time.time()
        while True:
            simdi = time.time()
            with self.peer_lock:
                guncel_hazirlar = {
                    k: v for k, v in self.peer_states.items()
                    if v.get("hazir", False) and simdi - v.get("timestamp", 0) < 3.0
                }
            if len(guncel_hazirlar) >= beklenen - 1:
                break
            if simdi - baslangic > timeout:
                print(f"[Drone-{self.drone_id}] UYARI: Timeout! {len(guncel_hazirlar)+1}/{beklenen} drone ile takeoff.")
                break
            time.sleep(0.1)

        self.vehicle.simple_takeoff(self.takeoff_irtifa)
        while self.vehicle.location.global_relative_frame.alt <= self.takeoff_irtifa * 0.9:
            print(f"[Drone-{self.drone_id}] İrtifa: {self.vehicle.location.global_relative_frame.alt:.1f}m")
            time.sleep(1)
          

        print(f"[Drone-{self.drone_id}] Takeoff tamamlandı.")
        self.yaw = self.vehicle.attitude.yaw
        self.vehicle.groundspeed = 2

    # ═══════════════════════════════════════════════════════════════════
    #  START
    # ═══════════════════════════════════════════════════════════════════

    def start(self):
        threading.Thread(target=self.publish,   daemon=True).start()
        threading.Thread(target=self.subscribe, daemon=True).start()
        threading.Thread(target=self.yki_dinle, daemon=True).start()

    # ═══════════════════════════════════════════════════════════════════
    #  PEER DURUM YÖNETİMİ
    # ═══════════════════════════════════════════════════════════════════

    def tumVeriyiBekle(self, timeout=1.0):
        beklenen  = len(nodes) - 1
        baslangic = time.time()
        while True:
            simdi = time.time()
            with self.peer_lock:
                guncel = {k: v for k, v in self.peer_states.items()
                          if simdi - v.get("timestamp", 0) < 2.0}
            if len(guncel) >= beklenen:
                break
            if simdi - baslangic > timeout:
                print("Timeout: bazı dronelardan veri gelmedi, mevcut veriyle devam.")
                break
            time.sleep(0.05)

    def sakla(self, data):
        peer_id = data.get("id")
        with self.peer_lock:
            self.peer_states[peer_id] = {
                "lat":       data.get("lat", 0),
                "lon":       data.get("lon", 0),
                "alt":       data.get("alt", 0),
                "hazir":     data.get("hazir", False),
                "timestamp": time.time(),
            }
        with self.ihale_lock:
            self.ihale_panosu[peer_id] = {
                "hedef":   data.get("ihale_hedef",   -1),
                "maliyet": data.get("ihale_maliyet", 9999.0),
            }

    # ═══════════════════════════════════════════════════════════════════
    #  FORMASYON – Matris & Nokta Hesapları
    # ═══════════════════════════════════════════════════════════════════

    def matrisYapV(self, n):
        is_tek = None
        if n % 2 == 0:
            n = n + 1
            is_tek = False
        else:
            is_tek = True
        satir = int((n // 2) + 1)
        sutun = int(n)
        self.formasyonMatris = np.zeros((satir, sutun))
        for i in range(satir):
            sol = i
            sag = (n - 1) - i
            if sol >= 0:
                self.formasyonMatris[i, sol] = 1
            if sag < sutun and sol != sag:
                if not is_tek and i == 0 :
                    continue
                self.formasyonMatris[i, sag] = 1

        return self.formasyonMatris

    def matrisYapok(self, n):
        is_tek = None
        if n % 2 == 0:
            n = n + 1
            is_tek = False
        else:
            is_tek = True
        satir = int((n // 2) + 1)
        sutun = int(n)
        self.formasyonMatris = np.zeros((satir, sutun))
        orta_sutun = sutun // 2
        for i in range(satir):
            sol = orta_sutun - i
            sag = orta_sutun + i
            if sol >= 0:
                self.formasyonMatris[i, orta_sutun - i] = 1
            if sag < sutun and sol != sag:
                if not is_tek and i == satir -1: 
                    continue
                self.formasyonMatris[i, orta_sutun + i] = 1
        return self.formasyonMatris

    def matrisYapDuz(self, n):
        sutun = int(n)
        self.formasyonMatris = np.zeros((1, sutun))
        for i in range(sutun):
            self.formasyonMatris[0, i] = 1
        return self.formasyonMatris

    def mesafeleriHesapla(self, matris, k, d):
        toplam_satir, toplam_sutun = matris.shape
        orta_satir = (toplam_satir - 1) / 2
        orta_sutun = (toplam_sutun - 1) / 2
        nokta_sozluk = {}
        bir_konumlar = np.argwhere(matris == 1)
        for idx, konum in enumerate(bir_konumlar, start=1):
            i, j = int(konum[0]), int(konum[1])
            nokta_sozluk[idx] = {
                "kuzey": (orta_satir - i) * k,
                "dogu":  (j - orta_sutun) * d,
            }
        return nokta_sozluk

    def noktalarıKonumaÇevir(self):
        if not self.sanal_merkez_loc or not self.noktalar:
            return
        self.hedefKonumlar = {}
        for idx, mesafe in self.noktalar.items():
            r_kuzey, r_dogu = self.rotasyonUygula(
                mesafe["kuzey"], mesafe["dogu"], self.sanal_merkez_heading
            )
            hedef_yatay = get_location_metres(self.sanal_merkez_loc, r_kuzey, r_dogu)
            self.hedefKonumlar[idx] = LocationGlobalRelative(
                hedef_yatay.lat, hedef_yatay.lon, self.benim_irtifam
            )

    def MatrisHesap(self, snapshot):
        if not self.hedefKonumlar:
            return
        aktif_dronelar = sorted(list(snapshot.keys()) + [self.drone_id])
        n = len(aktif_dronelar)
        self.aktif_dronelar = aktif_dronelar
        self.maliyetDizi = np.zeros(n)
        my_loc = self.vehicle.location.global_relative_frame
        if my_loc:
            for j in range(n):
                h_loc   = self.hedefKonumlar[j + 1]
                mesafe  = round(self.oklidFark(my_loc, h_loc) * 2) / 2
                self.maliyetDizi[j] = mesafe
        else:
            for j in range(n):
                self.maliyetDizi[j] = 9999.0

    # ═══════════════════════════════════════════════════════════════════
    #  İHALE
    # ═══════════════════════════════════════════════════════════════════

    def hedefKarar(self):
        n = len(self.aktif_dronelar)
        self.ihale_hedefim  = -1
        self.ihale_teklifim = 9999.0
        with self.ihale_lock:
            for peer_id in self.aktif_dronelar:
                if peer_id != self.drone_id:
                    self.ihale_panosu[peer_id] = {"hedef": -1, "maliyet": 9999.0}

        my_maliyetlerim = {i + 1: self.maliyetDizi[i] for i in range(n)}
        ihale_tamamlandi = False

        while not ihale_tamamlandi:
            if self.ihale_hedefim == -1:
                en_iyi_hedef  = None
                en_iyi_maliyet = 9999.0
                for hdf, mlyt in my_maliyetlerim.items():
                    if mlyt < en_iyi_maliyet:
                        en_iyi_maliyet = mlyt
                        en_iyi_hedef   = hdf
                if en_iyi_hedef is not None:
                    self.ihale_hedefim  = en_iyi_hedef
                    self.ihale_teklifim = en_iyi_maliyet
                    print(f"[Drone-{self.drone_id}] Hedef {en_iyi_hedef} teklifi: {en_iyi_maliyet:.1f}m")

            time.sleep(0.4)

            with self.ihale_lock:
                pano_snapshot = dict(self.ihale_panosu)

            for peer_id, veri in pano_snapshot.items():
                if peer_id not in self.aktif_dronelar:
                    continue
                peer_hedef   = veri["hedef"]
                peer_maliyet = veri["maliyet"]
                if peer_hedef != -1 and peer_hedef == self.ihale_hedefim:
                    if (peer_maliyet < self.ihale_teklifim) or (
                        peer_maliyet == self.ihale_teklifim
                        and int(peer_id) < int(self.drone_id)
                    ):
                        print(f"[Drone-{self.drone_id}] ÇATIŞMA: Hedef {self.ihale_hedefim} kaybedildi (Drone-{peer_id}).")
                        if self.ihale_hedefim in my_maliyetlerim:
                            del my_maliyetlerim[self.ihale_hedefim]
                        self.ihale_hedefim  = -1
                        self.ihale_teklifim = 9999.0
                        break

            atanan_hedefler = set()
            if self.ihale_hedefim != -1:
                atanan_hedefler.add(self.ihale_hedefim)
            for peer_id, veri in pano_snapshot.items():
                if peer_id in self.aktif_dronelar and veri["hedef"] != -1:
                    atanan_hedefler.add(veri["hedef"])
            if len(atanan_hedefler) == n:
                ihale_tamamlandi = True

        print(f"[Drone-{self.drone_id}] >>> İHALE SONUCU: Hedef {self.ihale_hedefim} KAZANILDI! <<<")
        return self.ihale_hedefim

    # ═══════════════════════════════════════════════════════════════════
    #  GEOMETRİ YARDIMCILARI
    # ═══════════════════════════════════════════════════════════════════

    def oklidFark(self, loc1, loc2):
        dlat  = loc2.lat - loc1.lat
        dlong = loc2.lon - loc1.lon
        return math.sqrt(dlat * dlat + dlong * dlong) * 1.113195e5

    def rotasyonUygula(self, kuzey, dogu, aci_derece):
        aci_radyan = math.radians(aci_derece)
        yeni_kuzey = kuzey * math.cos(aci_radyan) - dogu * math.sin(aci_radyan)
        yeni_dogu  = kuzey * math.sin(aci_radyan) + dogu * math.cos(aci_radyan)
        return yeni_kuzey, yeni_dogu

    def sanalMerkeziBelirle(self, snapshot):
        my_loc = self.vehicle.location.global_relative_frame
        if not my_loc:
            return
        toplam_lat = my_loc.lat
        toplam_lon = my_loc.lon
        toplam_alt = my_loc.alt
        sayac = 1
        for i in snapshot.values():
            toplam_lat += i["lat"]
            toplam_lon += i["lon"]
            sayac += 1
        self.sanal_merkez_loc = LocationGlobalRelative(
            toplam_lat / sayac, toplam_lon / sayac, self.benim_irtifam
        )
        self.droneSay = sayac
        print(f"[Drone-{self.drone_id}] Sanal Merkez Sabitlendi.")

    def hedefeAciHesapla(self, loc1, loc2):
        dLon = loc2.lon - loc1.lon
        y = math.sin(math.radians(dLon)) * math.cos(math.radians(loc2.lat))
        x = (math.cos(math.radians(loc1.lat)) * math.sin(math.radians(loc2.lat))
             - math.sin(math.radians(loc1.lat)) * math.cos(math.radians(loc2.lat))
             * math.cos(math.radians(dLon)))
        aci = math.atan2(y, x)
        return (math.degrees(aci) + 360) % 360

    def aciFarki(self, hedef, guncel):
        fark = (hedef - guncel) % 360
        if fark > 180:
            fark -= 360
        return fark

    # ═══════════════════════════════════════════════════════════════════
    #  MOD & ARM YARDIMCILARI
    # ═══════════════════════════════════════════════════════════════════

    def setMode(self, mode):
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


# ═══════════════════════════════════════════════════════════════════════
#  ANA PROGRAM
# ═══════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import sys

    node = SwarmNode(sys.argv[1])
    node.start()

    # ── Ortak koordinatlar ──────────────────────────────────────────────
    merkez_koordinat = LocationGlobalRelative(-35.36334548, 149.16514083, 0)
    manuel_hedefler = {
        1: LocationGlobalRelative( 40.74443846, 30.33848941, 0), # 1. Hedef koordinatı40.74443846 30.33848941 
        2: LocationGlobalRelative( 40.74454597 , 30.33829182, 0), # 2. Hedef koordinatı40.74454597 30.33829182 
        3: LocationGlobalRelative(40.74443846, 30.33848941, 0), # 3. Hedef koordinatı
        4: LocationGlobalRelative(40.74443846, 30.33848941, 0), # 4. Hedef koordinatı
    }
    yaricap_m = 20.0

    # Altıgen köşe noktaları (navigasyon hedefleri)
    altigen_hedefler = {}
    for i in range(1, 7):
        aci_derece = (i - 1) * 60
        d_kuzey = yaricap_m * math.cos(math.radians(aci_derece))
        d_dogu  = yaricap_m * math.sin(math.radians(aci_derece))
        altigen_hedefler[i] = get_location_metres(merkez_koordinat, d_kuzey, d_dogu)

    # ═══════════════════════════════════════════════════════════════════
    #  GÖREV FONKSİYONLARI  –  Her biri bağımsız bir görevi tanımlar.
    #  YKİ'den "GOREV" komutu alındığında sırayla çalıştırılır.
    #  Her fonksiyon droneun BEKLE durumuna dönmesini bekleyerek biter.
    # ═══════════════════════════════════════════════════════════════════

    def gorev_bekle_durum(zaman_asimi=120):
        """Drone BEKLE durumuna geçene kadar bekler."""
        baslangic = time.time()
        while node.durum != DURUM_BEKLE:
            if time.time() - baslangic > zaman_asimi:
                print(f"[GÖREV] UYARI: {zaman_asimi}s zaman aşımı, bir sonraki göreve geçiliyor.")
                break
            time.sleep(0.5)

    def gorev_1_V_formasyon():
        """Görev 1 — V formasyonu kur."""
        print("\n[GÖREV-1] V Formasyonu kuruluyor...")
        node._formasyon_komutu_isle("V")
        gorev_bekle_durum()
        print("[GÖREV-1] Tamamlandı.")

    def gorev_2_kose_1_git():
        """Görev 2 — Altıgenin 1. köşesine git."""
        print("\n[GÖREV-2] Altıgenin 1. köşesine gidiliyor...")
        node._navigasyon_komutu_isle(manuel_hedefler[2], hedef_id=1)
        gorev_bekle_durum()
        print("[GÖREV-2] Tamamlandı.")

    def gorev_3_OK_formasyon():
        """Görev 3 — Ok formasyonuna geç."""
        print("\n[GÖREV-3] Ok Formasyonu kuruluyor...")
        node._formasyon_komutu_isle("OK")
        gorev_bekle_durum()
        print("[GÖREV-3] Tamamlandı.")

    def gorev_4_kose_4_git():
        """Görev 4 — Altıgenin 4. köşesine git (karşı taraf)."""
        print("\n[GÖREV-4] Altıgenin 4. köşesine gidiliyor...")
        node._navigasyon_komutu_isle(manuel_hedefler[1], hedef_id=4)
        gorev_bekle_durum()
        print("[GÖREV-4] Tamamlandı.")

    def gorev_5_DUZ_formasyon():
        """Görev 5 — Düz hat formasyonuna geç."""
        print("\n[GÖREV-5] Düz Hat Formasyonu kuruluyor...")
        node._formasyon_komutu_isle("DUZ")
        gorev_bekle_durum()
        print("[GÖREV-5] Tamamlandı.")

    def gorev_6_merkeze_don():
        """Görev 6 — Başlangıç merkezine dön."""
        print("\n[GÖREV-6] Başlangıç merkezine dönülüyor...")
        node._navigasyon_komutu_isle(merkez_koordinat, hedef_id=0)
        gorev_bekle_durum()
        print("[GÖREV-6] Tamamlandı.")

    def gorev_7_yaw_saga():
        """Görev 7 — Formasyon 90° sağa döner."""
        print("\n[GÖREV-7] Formasyon 90° sağa döndürülüyor...")
        node.yaw_manevra(math.radians(90))
        time.sleep(3)   # Kısa manevra, bekleme süresi yeterli
        print("[GÖREV-7] Tamamlandı.")

    def gorev_8_kose_2_3_arasi():
        """Görev 8 — Altıgenin 2. köşesine git."""
        print("\n[GÖREV-8] Altıgenin 2. köşesine gidiliyor...")
        node._navigasyon_komutu_isle(manuel_hedefler[3], hedef_id=2)
        gorev_bekle_durum()
        print("[GÖREV-8] Tamamlandı.")

    def gorev_9_euler_sifirla():
        """Görev 9 — Euler offset sıfırla, düzeltme yap."""
        print("\n[GÖREV-9] Euler offset sıfırlanıyor...")
        node.euler_sifirla()
        time.sleep(3)
        print("[GÖREV-9] Tamamlandı.")

    def gorev_10_V_formasyon_don():
        """Görev 10 — Tekrar V formasyonuna geç ve merkeze dön."""
        print("\n[GÖREV-10] V Formasyonuna dönülüyor...")
        node._formasyon_komutu_isle("V")
        gorev_bekle_durum()
        node._navigasyon_komutu_isle(merkez_koordinat, hedef_id=0)
        gorev_bekle_durum()
        print("[GÖREV-10] Tamamlandı. Tüm görevler bitti.")

    # ── Görev sırası — YKİ'den her "GOREV" komutu bir sonrakini çalıştırır ──
    GOREV_SIRASI = [
        #gorev_1_V_formasyon,
        #gorev_3_OK_formasyon,
        gorev_5_DUZ_formasyon,
        gorev_2_kose_1_git,
        #gorev_1_V_formasyon,
        gorev_4_kose_4_git,
        #gorev_6_merkeze_don,
        #gorev_8_kose_2_3_arasi,
        #gorev_9_euler_sifirla,
        #gorev_10_V_formasyon_don,
    ]

    # Görev sayacı — thread-safe erişim için lock ile korunur
    _gorev_index      = 0
    _gorev_index_lock = threading.Lock()
    _gorev_aktif      = False   # Bir görev zaten çalışıyorken yeni komutu reddeder

    def gorev_ilerlet():
        """YKİ'den gelen GOREV komutuyla çağrılır. Sıradaki görevi başlatır."""
        global _gorev_index, _gorev_aktif

        with _gorev_index_lock:
            if _gorev_aktif:
                print("[GÖREV] Önceki görev henüz bitmedi, komut reddedildi.")
                return
            if _gorev_index >= len(GOREV_SIRASI):
                print("[GÖREV] Tüm görevler tamamlandı. Yeni görev yok.")
                return
            idx = _gorev_index
            _gorev_index += 1
            _gorev_aktif = True

        def _calistir():
            global _gorev_aktif
            gorev_fn = GOREV_SIRASI[idx]
            print(f"\n[GÖREV] ▶ Görev {idx + 1}/{len(GOREV_SIRASI)}: {gorev_fn.__doc__.strip()}")
            try:
                gorev_fn()
            except Exception as e:
                print(f"[GÖREV] HATA: {e}")
            finally:
                _gorev_aktif = False
                with _gorev_index_lock:
                    kalan = len(GOREV_SIRASI) - _gorev_index
                print(f"[GÖREV] ✓ Görev {idx + 1} bitti. Kalan: {kalan} görev. YKİ'den 'g' ile devam edin.")

        threading.Thread(target=_calistir, daemon=True).start()

    # ── YKİ'den gelen GOREV komutunu dinle ──────────────────────────────
    def yki_gorev_dinle():
        """Ana YKİ soketine ek olarak GOREV komutlarını yakalar."""
        ctx    = zmq.Context()
        sub    = ctx.socket(zmq.SUB)
        sub.connect(f"tcp://{YKI_ADDR}")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        print(f"[GÖREV-DİNLEYİCİ] Hazır. YKİ'den 'g' tuşu bekleniyor...")
        while True:
            try:
                veri   = sub.recv_json()
                komut  = veri.get("komut", "")
                if komut == "GOREV":
                    gorev_ilerlet()
            except Exception as e:
                print(f"[GÖREV-DİNLEYİCİ] Hata: {e}")

    threading.Thread(target=yki_gorev_dinle, daemon=True).start()

    while True:
        time.sleep(1)