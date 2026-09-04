/* ===========================================================================
 *  ARAC KODU - ESC UC NOKTA KALIBRASYONU
 *  Dosya : arac_esc_kalibrasyon.ino
 *
 *  AMAC
 *    Dort ESC'ye ayni gaz araligini (1000-2000 us) ogretir. Kalibrasyon
 *    yapilmazsa her ESC kendi fabrika araligini kullanir; ayni PWM degeri
 *    farkli motorlarda farkli devir uretir ve arac havada yana kayar.
 *
 *  PROSEDUR
 *    1. Batarya TAKILI DEGIL. Pervaneler TAKILI DEGIL.
 *    2. Kodu yukle, seri monitoru ac (115200).
 *    3. "MAX sinyal aktif" yazisini gorunce bataryayi tak.
 *    4. Bip seslerini dinle - ESC ust noktayi kaydeder.
 *    5. 10 sn sonra kod alt noktaya gecer, onay bipleri gelir.
 *    6. Kalibrasyon ESC hafizasinda kalicidir, tekrar gerekmez.
 * ======================================================================== */

#include <ESP32Servo.h>

Servo esc[4];
const int PIN[4] = { 12, 13, 14, 27 };

void hepsineYaz(uint16_t us) {
  for (int i = 0; i < 4; i++) esc[i].writeMicroseconds(us);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < 4; i++) {
    esc[i].setPeriodHertz(50);
    esc[i].attach(PIN[i], 1000, 2000);
  }

  hepsineYaz(2000);
  Serial.println(F("=== MAX sinyal aktif (2000 us) ==="));
  Serial.println(F("SIMDI bataryayi tak. Bip seslerini bekle."));
  delay(10000);

  hepsineYaz(1000);
  Serial.println(F("=== MIN sinyal aktif (1000 us) ==="));
  Serial.println(F("Onay bipleri gelmeli."));
  delay(5000);

  Serial.println(F("KALIBRASYON BITTI."));
}

void loop() {
  // Sinyal surdurulur. Kesilirse ESC "sinyal yok" hata bipi calar.
  hepsineYaz(1000);
  delay(20);
}
