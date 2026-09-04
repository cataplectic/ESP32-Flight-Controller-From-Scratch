/* ===========================================================================
 *  ARAC KODU - TEKIL MOTOR VE DONUS YONU TESTI
 *  Dosya : arac_motor_test.ino
 *
 *  AMAC
 *    Her motoru tek tek dusuk gazda dondurerek donus yonunu dogrular.
 *    X konfigurasyonunda capraz motorlar ayni yone, komsu motorlar ters
 *    yone donmelidir. Aksi halde yaw ekseninde net tork olusur ve arac
 *    havada kendi etrafinda doner.
 *
 *  KULLANIM
 *    Seri monitorden 1-4 : ilgili motoru 3 saniye dondurur
 *                     0 : tum motorlari durdurur
 *
 *  !! PERVANELER TAKILI OLMAYACAK !!
 *  Sasi masaya sabitlenmis olmali, eller motorlardan uzak tutulmalidir.
 * ======================================================================== */

#include <ESP32Servo.h>

Servo esc[4];
const int PIN[4] = { 12, 13, 14, 27 };
const uint16_t TEST_GAZ = 1150;    // rolanti uzeri - yon gormeye yeter

void setup() {
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < 4; i++) {
    esc[i].setPeriodHertz(50);
    esc[i].attach(PIN[i], 1000, 2000);
    esc[i].writeMicroseconds(1000);
  }

  Serial.println(F("=== MOTOR TESTI ==="));
  Serial.println(F("1-4 : o motoru 3 saniye dondur"));
  Serial.println(F("0   : hepsini durdur"));
  Serial.println(F("PERVANE TAKILI OLMAYACAK"));
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();

  if (c >= '1' && c <= '4') {
    int m = c - '1';
    Serial.print(F(">> Motor ")); Serial.print(m + 1);
    Serial.println(F(" donuyor (3 sn)"));
    esc[m].writeMicroseconds(TEST_GAZ);
    delay(3000);
    esc[m].writeMicroseconds(1000);
    Serial.println(F("   durdu"));
  }

  if (c == '0') {
    for (int i = 0; i < 4; i++) esc[i].writeMicroseconds(1000);
    Serial.println(F(">> HEPSI DURDU"));
  }
}
