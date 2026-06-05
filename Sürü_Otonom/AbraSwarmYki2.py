import zmq
import time

# ─── Ayarlar ────────────────────────────────────────────────────
YKI_PUB_PORT    = "5599"
VARSAYILAN_IRTIFA = 10   # metre
# ────────────────────────────────────────────────────────────────


class YKI:
    def __init__(self):
        self.context    = zmq.Context()
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{YKI_PUB_PORT}")
        print(f"[YKİ] Yayın soketi başlatıldı: tcp://*:{YKI_PUB_PORT}")
        time.sleep(0.5)   # Abone bağlantılarının oturması için

        # Kaçıncı göreve ulaşıldığını göstermek için sayaç (bilgi amaçlı)
        self._gonderilen_gorev = 0

    # ── Genel gönderici ─────────────────────────────────────────
    def gonder(self, veri: dict):
        self.pub_socket.send_json(veri)

    # ── b tuşu: Bağlan + Takeoff ─────────────────────────────────
    def baslatKomutu(self, irtifa: int = VARSAYILAN_IRTIFA):
        """Tüm dronelara senkron takeoff komutu gönderir."""
        print(f"\n[YKİ] *** BAŞLAT komutu gönderiliyor → {irtifa}m ***")
        self.gonder({"komut": "BAŞLAT", "irtifa": irtifa})

    # ── g tuşu: Sıradaki görevi tetikle ──────────────────────────
    def gorevIlerlet(self):
        """
        Dronelara GOREV komutu gönderir.
        Drone tarafında bir görev listesi tutulur; her komutta
        sıradaki görev çalışır. Önceki görev bitmeden komut
        reddedilir — yani bu tuşa görev bitmeden basılmaz.
        """
        self._gonderilen_gorev += 1
        print(f"\n[YKİ] *** GÖREV komutu gönderiliyor (#{self._gonderilen_gorev}) ***")
        self.gonder({"komut": "GOREV"})

    # ── Ana döngü ────────────────────────────────────────────────
    def calistir(self):
        irtifa = VARSAYILAN_IRTIFA

        yardim = (
            "\n─────────────────────────────────────────\n"
            "  b  →  Tüm dronelara Bağlan + Takeoff\n"
            "  g  →  Sıradaki Görevi Başlat / İlerlet\n"
            "  q  →  Çıkış\n"
            "  h  →  Bu yardım menüsü\n"
            "─────────────────────────────────────────"
        )
        print(yardim)

        while True:
            try:
                girdi = input("\nKomut > ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print("\n[YKİ] Kapatılıyor...")
                break

            if not girdi:
                continue

            if girdi == "b":
                self.baslatKomutu(irtifa)

            elif girdi == "g":
                self.gorevIlerlet()

            elif girdi == "q":
                print("[YKİ] Çıkılıyor.")
                break

            elif girdi == "h":
                print(yardim)

            else:
                print(f"[YKİ] Bilinmeyen komut: '{girdi}'  (yardım için: h)")


if __name__ == "__main__":
    yki = YKI()
    yki.calistir()
