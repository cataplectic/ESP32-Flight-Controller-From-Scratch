/* ===========================================================================
 *  ESP32 QUADCOPTER - UCUS KONTROL YAZILIMI (ALICI)
 *  Dosya   : alici_v3_ucus_kontrol.ino
 *  Donanim : ESP32 DevKit V1 (38 pin) + nRF24L01+ PA/LNA + MPU6050 + 4x30A ESC
 *  Surum   : v3.0
 *
 *  KAPSAM
 *    - 2.4 GHz telsiz alicisi (4 kanal, 50 Hz)
 *    - Atalet olcum birimi ile roll / pitch / yaw kestirimi
 *    - Arming (silahlanma) mantigi ve cok katmanli emniyet kilitleri
 *    - Failsafe: baglanti kaybinda gaz kesme
 *    - Kaskatsiz PID (aci modu: roll/pitch, hiz modu: yaw)
 *    - X konfigurasyonu motor karistirma matrisi
 *
 *  !! EMNIYET !!
 *    PID kazanclari FABRIKA CIKISI SIFIRDIR. Pervane takili olarak ilk
 *    calistirmayi YAPMAYIN. Ayar sirasi ve test standi prosedürü icin
 *    "Sistem Tasarim Dokumani" Bolum 8'e bakiniz.
 * ======================================================================== */

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <ESP32Servo.h>

/* ---------------------------------------------------------------- PINLER */
#define PIN_NRF_CE    4
#define PIN_NRF_CSN   5
//      NRF SCK 18 | MISO 19 | MOSI 23  (VSPI varsayilan)
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22

// Motor numaralandirmasi - X konfigurasyonu, USTTEN bakis
//
//        ON (kirmizi kollar)
//     M4  \             /  M1
//      (CCW)           (CW)
//          \  ______  /
//           |      |
//           |      |
//          /  ------  \
//      (CW)            (CCW)
//     M3  /             \  M2
//        ARKA (siyah kollar)
//
#define PIN_M1   12   // on-sag , CW
#define PIN_M2   13   // arka-sag, CCW
#define PIN_M3   14   // arka-sol, CW
#define PIN_M4   27   // on-sol , CCW

/* ------------------------------------------------------------- SABITLER */
const byte     ADRES[6]        = "DRN01";
const uint8_t  RF_KANAL        = 100;      // 2.4 GHz WiFi bandindan uzak
const uint16_t PWM_MIN         = 1000;     // us - motor durdu
const uint16_t PWM_MAX         = 2000;     // us - tam gaz
const uint16_t PWM_ARM         = 1100;     // us - rolanti (armed, yerde)
const uint16_t PWM_TAVAN       = 1900;     // us - PID icin bas payi
const uint16_t GAZ_ESIK        = 1050;     // altinda "gaz kapali" sayilir
const uint16_t BAGLANTI_ZAMAN  = 500;      // ms - failsafe esigi
const float    DONGU_HZ        = 250.0;    // Hz - kontrol dongusu
const uint32_t DONGU_US        = (uint32_t)(1000000.0 / DONGU_HZ);

// Kumanda komut olcekleri
const float MAX_ACI      = 25.0;   // derece - roll/pitch komut siniri
const float MAX_YAW_HIZ  = 120.0;  // derece/s - yaw komut siniri

/* ------------------------------------------------ PID KAZANCLARI (AYAR) */
// Ayar sirasi: once Kp, sonra Kd, en son Ki. Detay: Tasarim Dok. Bolum 8.3
struct Kazanc { float kp, ki, kd; };

Kazanc G_ROLL  = { 0.0f, 0.0f, 0.0f };   // baslangic: 1.30 / 0.00 / 12.0
Kazanc G_PITCH = { 0.0f, 0.0f, 0.0f };   // roll ile ayni alinir (simetrik)
Kazanc G_YAW   = { 0.0f, 0.0f, 0.0f };   // baslangic: 2.50 / 0.00 /  0.0

const float I_SINIR   = 120.0f;   // integral windup siniri (us)
const float PID_SINIR = 320.0f;   // eksen basina toplam cikis siniri (us)

/* -------------------------------------------------------- GLOBAL NESNE */
RF24     radio(PIN_NRF_CE, PIN_NRF_CSN);
MPU6050  mpu(Wire);
Servo    motor[4];
const int MOTOR_PIN[4] = { PIN_M1, PIN_M2, PIN_M3, PIN_M4 };

/* ------------------------------------------------------- VERI YAPILARI */
struct Paket {                 // telsiz uzerinden gelen cerceve (10 bayt)
  uint16_t kanal[4];           // 0:gaz  1:yaw  2:pitch  3:roll  (1000-2000)
  uint16_t sayac;              // paket sirasi - kayip tespiti icin
};
Paket gelen;

uint16_t kanal[4] = { 1000, 1500, 1500, 1500 };   // guvenli baslangic
uint32_t sonPaket  = 0;
uint16_t sonSayac  = 0;
uint32_t kayipPaket = 0;
bool     baglanti  = false;
bool     armed     = false;
uint32_t donguSon  = 0;
float    dt        = 1.0f / DONGU_HZ;

struct PidDurum {
  float integral;        // birikmis integral terimi (us)
  float oncekiOlculen;   // bir onceki dongudeki olculen deger
  bool  ilkTur;          // true ise turev hesaplanmaz (bkz. pidHesapla)
};
PidDurum dRoll  = { 0.0f, 0.0f, true };
PidDurum dPitch = { 0.0f, 0.0f, true };
PidDurum dYaw   = { 0.0f, 0.0f, true };

/* ====================================================================== */
/*  YARDIMCI FONKSIYONLAR                                                 */
/* ====================================================================== */

/* Kanal degerini (1000-2000) simetrik komut araligina donusturur.
   1500 us -> 0.0 ; 2000 us -> +limit ; 1000 us -> -limit               */
float kanalKomut(uint16_t us, float limit) {
  float x = ((float)us - 1500.0f) / 500.0f;      // -1.0 .. +1.0
  x = constrain(x, -1.0f, 1.0f);
  return x * limit;
}

/* Tek eksen PID. Turev, olculen degerden alinir (derivative-on-measurement)
   - boylece kumanda komutu aniden degistiginde turev sicramasi olmaz.   */
float pidHesapla(PidDurum &d, const Kazanc &g,
                 float hedef, float olculen, bool bisikletle) {
  float hata = hedef - olculen;

  // Integral: sadece armed ve gaz acikken birikir, aksi halde sifirlanir
  if (bisikletle) {
    d.integral += g.ki * hata * dt;
    d.integral = constrain(d.integral, -I_SINIR, I_SINIR);
  } else {
    d.integral = 0.0f;
  }

  /* Turev. Ilk turda hesaplanmaz: oncekiOlculen henuz gecerli bir olcum
     degil, sifirdir. Sifirdan turev almak (olculen - 0) / dt gibi devasa
     bir deger uretir ve arming aninda motorlara darbe gonderirdi.       */
  float turev = 0.0f;
  if (d.ilkTur) {
    d.ilkTur = false;
  } else {
    turev = (olculen - d.oncekiOlculen) / dt;
  }
  d.oncekiOlculen = olculen;

  float cikis = g.kp * hata + d.integral - g.kd * turev;
  return constrain(cikis, -PID_SINIR, PID_SINIR);
}

/* Tum PID durumlarini sifirlar - disarm ve failsafe aninda cagrilir */
void pidSifirla() {
  dRoll  = { 0.0f, 0.0f, true };
  dPitch = { 0.0f, 0.0f, true };
  dYaw   = { 0.0f, 0.0f, true };
}

/* Dort motora ayni degeri yazar */
void motorlariYaz(uint16_t us) {
  for (int i = 0; i < 4; i++) motor[i].writeMicroseconds(us);
}

/* ====================================================================== */
/*  KURULUM                                                               */
/* ====================================================================== */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== ESP32 UCUS KONTROL v3.0 ==="));

  /* --- 1. ADIM: motorlar once susturulur -------------------------------
     ESC'ler enerji aldiginda gecerli bir dusuk sinyal gormezse hata bipi
     calar veya beklenmedik sekilde donebilir. Bu yuzden ilk is motor
     cikislarini PWM_MIN'e sabitlemektir.                                */
  for (int i = 0; i < 4; i++) {
    motor[i].setPeriodHertz(50);
    motor[i].attach(MOTOR_PIN[i], PWM_MIN, PWM_MAX);
    motor[i].writeMicroseconds(PWM_MIN);
  }
  Serial.println(F("[OK] Motor cikislari PWM_MIN'e sabitlendi"));

  /* --- 2. ADIM: atalet olcum birimi ----------------------------------- */
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);                       // hizli I2C - dongu butcesi icin
  byte durum = mpu.begin();
  if (durum != 0) {
    Serial.print(F("[HATA] MPU6050 bulunamadi, kod: "));
    Serial.println(durum);
    while (1) { motorlariYaz(PWM_MIN); delay(200); }   // guvenli kilit
  }
  Serial.println(F("[..] Kart DUZ ve HAREKETSIZ dursun, kalibre ediliyor"));
  delay(1200);
  mpu.calcOffsets();                           // gyro + ivme sapma giderme
  Serial.println(F("[OK] MPU6050 hazir"));

  /* --- 3. ADIM: telsiz ------------------------------------------------ */
  if (!radio.begin()) {
    Serial.println(F("[HATA] nRF24 bulunamadi"));
    while (1) { motorlariYaz(PWM_MIN); delay(200); }
  }
  radio.openReadingPipe(0, ADRES);
  radio.setPALevel(RF24_PA_LOW);               // ucusta RF24_PA_MAX yapilir
  radio.setDataRate(RF24_250KBPS);             // dusuk hiz = uzun menzil
  radio.setChannel(RF_KANAL);
  radio.startListening();
  Serial.println(F("[OK] nRF24 dinlemede"));

  sonPaket = millis();
  donguSon = micros();
  Serial.println(F("=== HAZIR - arming icin gaz kolunu en alta cekin ===\n"));
}

/* ====================================================================== */
/*  ANA DONGU - sabit 250 Hz                                              */
/* ====================================================================== */
void loop() {

  /* ---------- 1. TELSIZ: gelen cerceveyi al ve dogrula ---------------- */
  if (radio.available()) {
    radio.read(&gelen, sizeof(gelen));

    // Aralik disi deger = bozuk cerceve. Bu filtre olmadan tek bir hatali
    // paket motorlara sicrama komutu olarak gidebilir.
    bool gecerli = true;
    for (int i = 0; i < 4; i++)
      if (gelen.kanal[i] < 950 || gelen.kanal[i] > 2050) gecerli = false;

    if (gecerli) {
      // paket kaybi istatistigi (telemetri / menzil analizi icin)
      if (sonSayac != 0 && (uint16_t)(gelen.sayac - sonSayac) > 1)
        kayipPaket += (uint16_t)(gelen.sayac - sonSayac) - 1;
      sonSayac = gelen.sayac;

      for (int i = 0; i < 4; i++) kanal[i] = gelen.kanal[i];
      sonPaket  = millis();
      baglanti  = true;
    }
  }

  /* ---------- 2. FAILSAFE: baglanti kaybi ----------------------------- */
  if (millis() - sonPaket > BAGLANTI_ZAMAN) {
    if (baglanti) Serial.println(F(">>> FAILSAFE - baglanti kesildi"));
    baglanti = false;
    armed    = false;                 // silahsizlan
    kanal[0] = PWM_MIN;               // gaz kesildi
    kanal[1] = kanal[2] = kanal[3] = 1500;
    pidSifirla();
    motorlariYaz(PWM_MIN);
  }

  /* ---------- 3. ARMING MANTIGI --------------------------------------- *
     Silahlanma sarti: baglanti VAR  +  gaz kolu en altta.
     Silahsizlanma  : gaz kolu esigin altina inince.
     Bu, kod resetlendiginde veya kumanda yeniden baglandiginda motorlarin
     kendiliginden calismasini engelleyen birincil emniyet katmanidir.   */
  if (!armed && baglanti && kanal[0] <= GAZ_ESIK) {
    armed = true;
    pidSifirla();
    Serial.println(F(">>> ARMED - motorlar aktif"));
  }
  if (armed && kanal[0] <= GAZ_ESIK) {
    // yerde bekleme: PID birikmesin
    pidSifirla();
  }

  /* ---------- 4. SABIT DONGU ZAMANLAMASI ------------------------------ *
     PID turev ve integral terimleri sabit dt varsayar. Dongu suresi
     degiskense kazanclar da fiilen degisir ve ayar tutmaz.              */
  uint32_t simdi = micros();
  if (simdi - donguSon < DONGU_US) return;
  dt = (simdi - donguSon) / 1000000.0f;
  donguSon = simdi;

  /* ---------- 5. ATALET OLCUMU ---------------------------------------- */
  mpu.update();
  float rollOlculen  = mpu.getAngleX();      // derece
  float pitchOlculen = mpu.getAngleY();      // derece
  float yawHizi      = mpu.getGyroZ();       // derece/s (aci degil!)

  /* ---------- 6. KUMANDA KOMUTLARI ------------------------------------ */
  float rollHedef  = kanalKomut(kanal[3], MAX_ACI);
  float pitchHedef = kanalKomut(kanal[2], MAX_ACI);
  float yawHedef   = kanalKomut(kanal[1], MAX_YAW_HIZ);
  uint16_t gaz     = constrain(kanal[0], PWM_MIN, PWM_TAVAN);

  /* ---------- 7. PID -------------------------------------------------- *
     Roll ve pitch  : ACI modu   - hedef aci, olculen aci
     Yaw            : HIZ modu   - hedef donus hizi, olculen gyro hizi
     Yaw'da aci kullanilmaz cunku manyetometre olmadan yaw acisi surekli
     kayar (drift). Donus hizi ise driftten etkilenmez.                  */
  bool integralAktif = armed && (gaz > GAZ_ESIK + 60);

  float uRoll  = pidHesapla(dRoll,  G_ROLL,  rollHedef,  rollOlculen,  integralAktif);
  float uPitch = pidHesapla(dPitch, G_PITCH, pitchHedef, pitchOlculen, integralAktif);
  float uYaw   = pidHesapla(dYaw,   G_YAW,   yawHedef,   yawHizi,      integralAktif);

  /* ---------- 8. MOTOR KARISTIRMA (X konfigurasyonu) ------------------ *
     Isaret mantigi:
       roll  olculen > hedef  (saga fazla yatik) -> sag motorlar artar
       pitch olculen > hedef  (burun fazla yukari) -> on motorlar artar
       yaw   saga donus komutu -> CCW motorlar artar (tepki torku)
     M1 on-sag CW | M2 arka-sag CCW | M3 arka-sol CW | M4 on-sol CCW    */
  int m[4];
  m[0] = gaz - uRoll - uPitch - uYaw;   // M1 on-sag   (CW)
  m[1] = gaz - uRoll + uPitch + uYaw;   // M2 arka-sag (CCW)
  m[2] = gaz + uRoll + uPitch - uYaw;   // M3 arka-sol (CW)
  m[3] = gaz + uRoll - uPitch + uYaw;   // M4 on-sol   (CCW)

  /* ---------- 9. CIKIS SINIRLAMA VE YAZMA ----------------------------- */
  if (!armed || gaz <= GAZ_ESIK) {
    // Silahsiz veya gaz kapali: PID ne derse desin motorlar durur.
    // Bu, arming kontrolunden BAGIMSIZ ikinci bir emniyet kapisidir.
    motorlariYaz(PWM_MIN);
    for (int i = 0; i < 4; i++) m[i] = PWM_MIN;   // telemetri dogru gostersin
  } else {
    for (int i = 0; i < 4; i++) {
      m[i] = constrain(m[i], PWM_ARM, PWM_MAX);
      motor[i].writeMicroseconds((uint16_t)m[i]);
    }
  }

  /* ---------- 10. TELEMETRI (10 Hz) ----------------------------------- */
  static uint32_t yaz = 0;
  if (millis() - yaz > 100) {
    yaz = millis();
    Serial.print(baglanti ? F("OK  ") : F("KOP "));
    Serial.print(armed ? F("ARM ") : F("--- "));
    Serial.print(F("T:"));  Serial.print(kanal[0]);
    Serial.print(F(" R:")); Serial.print(rollOlculen, 1);
    Serial.print(F(" P:")); Serial.print(pitchOlculen, 1);
    Serial.print(F(" Yr:")); Serial.print(yawHizi, 1);
    Serial.print(F(" | M "));
    for (int i = 0; i < 4; i++) { Serial.print(m[i]); Serial.print(' '); }
    Serial.print(F("| kayip:")); Serial.println(kayipPaket);
  }
}
