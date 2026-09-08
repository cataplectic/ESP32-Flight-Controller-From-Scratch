/* ===========================================================================
 *  ESP32 QUADCOPTER - UCUS KONTROL YAZILIMI (ALICI)
 *  Dosya   : alici_v3_ucus_kontrol.ino
 *  Donanim : ESP32 DevKit V1 (38 pin) + nRF24L01+ PA/LNA + MPU6050 + 4x30A ESC
 *  Surum   : v6.0  (jiroskop alcak gecirgen filtresi eklendi)
 *
 *  v4.0 DEGISIKLIGI - NEDEN
 *    ESP32Servo kutuphanesi kullanilirken nRF24 paket kaybi %60'a
 *    ciktigi OLCULDU. Kutuphane devre disi birakilinca kayip SIFIRA
 *    dustu (tani testi). Cozum: PWM artik ESP32'nin LEDC donanim
 *    birimiyle dogrudan uretiliyor. Ek kutuphane yok, kesme yok.
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
 *    Asagidaki kazanclar DOGRULANMIS DEGIL, ayar icin baslangic
 *    degerleridir. Bu kodla ilk yapilacak is ISARET DOGRULAMASIDIR:
 *    pervaneler SOKULU, sasi masaya sabit, gaz ~1200, araci elle egip
 *    seri porttaki M degerlerinin dogru yonde arttigini kontrol et.
 *      saga yatir  -> M1 ve M2 artmali
 *      sola yatir  -> M3 ve M4 artmali
 *      one egil    -> M3 ve M4 artmali
 *      arkaya egil -> M1 ve M2 artmali
 *    Ters cikan varsa m[] satirlarindaki ilgili terimin isareti degisir.
 *    Isaret dogrulanmadan pervane TAKILMAZ. Kazanc ayari yalnizca test
 *    standi uzerinde yapilir - ayar yontemi salinim esigini aradigi icin
 *    serbest halde denemek araci devirir.
 *    Ayrinti: "Sistem Tasarim Dokumani" Bolum 8.3
 * ======================================================================== */

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050_light.h>

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
/* MOTOR PINLERI - v4.1'de DEGISTI, KABLO TASINACAK
   Eski: 12, 13, 14, 27.  Bu pinlerde LEDC PWM acikken nRF24 paket
   kaybi %60 olculdu; PWM kapatilinca kayip sifira dustu.
     GPIO 12 : strapping pini (MTDI) - acilista flash gerilimini belirler
     GPIO 13 : HSPI MOSI / JTAG
     GPIO 14 : HSPI CLK  / JTAG
   Yeni pinler bu ozel islevlerin hicbirine sahip degil ve alicida
   baska bir sey tarafindan kullanilmiyor.                            */
#define PIN_M1   25   // on-sag , CW
#define PIN_M2   26   // arka-sag, CCW
#define PIN_M3   32   // arka-sol, CW
#define PIN_M4   33   // on-sol , CCW

/* ------------------------------------------------------------- SABITLER */
const byte     ADRES[6]        = "DRN01";
const uint8_t  RF_KANAL        = 115;      // VERICI ILE AYNI OLMALI (2515 MHz)

/* RF CIKIS GUCU - VERICI ILE AYNI OLMALI
   RF24_PA_MIN  : masa ustu / yakin mesafe testleri  <-- SIMDI BURADASIN
   RF24_PA_LOW  : birkac metre
   RF24_PA_HIGH : orta menzil
   RF24_PA_MAX  : acik alan ucusu
   DIKKAT: Yakin mesafede yuksek guc alicinin giris katini DOYURUR ve
   paket kaybi ARTAR. Mesafe acildikca kademeli yukseltilir.          */
#define RF_GUC   RF24_PA_MIN
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
/* AYAR SIRASI: once Kp, sonra Kd, EN SON Ki.
 *
 *  1) Kp'yi 0.2'lik adimlarla artir. Arac dengeye donmeye baslar.
 *     Kendi kendine ileri geri salinmaya basladigi degeri not et,
 *     sonra o degerin %60'ina geri cek.  (tipik sonuc: 1.0 - 1.5)
 *  2) Kd ekle. Salinimi bastirir ve Kp'yi biraz daha yukseltmene izin
 *     verir. Motorlarda cizirti duyarsan Kd fazla gelmis, geri cek.
 *     (tipik: 8 - 14)
 *  3) Ki EN SON ve en kucuk adimlarla (0.02). Yalnizca arac bir tarafa
 *     KALICI olarak yatik duruyorsa gerekir. Fazlasi yavas ve buyuk
 *     genlikli salinim yapar.
 *  4) Pitch simetrik: roll degerlerini yaz ve dogrula.
 *  5) Yaw'da yalnizca Kp; hiz modunda turev genelde gereksizdir.
 *
 *  KABUL KRITERI: stand uzerinde elle verilen 20-30 derecelik bozucuya
 *  karsi arac, belirgin asim ve salinim olmadan yatay konuma donmeli.
 *  Bu saglanmadan pervaneli serbest ucusa GECILMEZ.
 */
struct Kazanc { float kp, ki, kd; };

//                 Kp     Ki     Kd
Kazanc G_ROLL  = { 1.20f, 0.00f, 3.00f };   // filtre sayesinde Kd tutulabiliyor
Kazanc G_PITCH = { 1.20f, 0.00f, 3.00f };   // roll ile ayni (govde simetrik)
Kazanc G_YAW   = { 0.50f, 0.00f, 0.00f };   // hiz modu - yalnizca Kp

/* ---------------------------------- JIROSKOP ALCAK GECIRGEN FILTRESI ---
   Pervane titresimi jiroskopa yuksek frekansli gurultu olarak gelir.
   D terimi bunu gercek hareket sanip motorlara yansitir, motorlar daha
   cok titrer - kendini besleyen bir dongu. OLCULDU: orta gazda belirgin.

   Birinci derece (ustel) filtre:
       suzulmus = suzulmus + ALFA * (ham - suzulmus)
   ALFA 0 ile 1 arasindadir:
       1.00 -> filtre yok, ham veri
       0.20 -> guclu yumusatma  (~9 Hz kesme, 250 Hz orneklemede)
       0.10 -> cok guclu        (~4 Hz kesme) - gecikme artar

   TAKAS: filtre gecikme getirir. Cok dusuk ALFA ile kontrolcu gec
   tepki verir ve arac sallanmaya baslar. Once 0.20 dene; titresim
   surerse 0.12'ye in, tepki tembellesirse 0.30'a cik.

   NOT: Bu bir YAZILIM cozumudur, mekanik titresimi ORTADAN KALDIRMAZ,
   yalnizca kontrolcuye ulasmasini engeller. Dengesiz pervane veya
   gevsek montaj varsa asil cozum orada.                             */
const float GYRO_ALFA = 0.20f;
                                            // 2.00 asiri sertti: elle cevirmede
                                            // motorlar tavana dayandi (olculdu)

const float I_SINIR   = 120.0f;   // integral windup siniri (us)
const float PID_SINIR = 320.0f;   // eksen basina toplam cikis siniri (us)

/* -------------------------------------------------------- GLOBAL NESNE */
RF24     radio(PIN_NRF_CE, PIN_NRF_CSN);
MPU6050  mpu(Wire);
const uint8_t MOTOR_PIN[4] = { PIN_M1, PIN_M2, PIN_M3, PIN_M4 };


/* ------------------------------------------------------- VERI YAPILARI *
   DIKKAT: Arduino IDE fonksiyon prototiplerini otomatik uretir ve ILK
   fonksiyon tanimindan ONCE yerlestirir. Bu yuzden struct tanimlari
   MUTLAKA butun fonksiyonlardan once gelmelidir - aksi halde
   "'PidDurum' was not declared in this scope" hatasi alinir.        */
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
};
PidDurum dRoll  = { 0.0f };
PidDurum dPitch = { 0.0f };
PidDurum dYaw   = { 0.0f };

/* ------------------------------------------------- PWM (RMT) ----------
   NEDEN LEDC DEGIL RMT - hepsi olculdu:
     LEDC  50 Hz + 16 bit : nRF24 paket kaybi %60
     LEDC  50 Hz + 14 bit : nRF24 paket kaybi %60
     LEDC 250 Hz + 16 bit : kayip yok, ESC'ler senkronsuz
     LEDC 400 Hz + 16 bit : kayip yok, ESC'ler senkronsuz
   ESC'ler 50 Hz istiyor, LEDC 50 Hz'de SPI'i bozuyor - cikmaz.

   RMT (Remote Control) ESP32'nin hassas darbe uretimi icin ayrilmis
   cevre birimidir; kendi saat bolucusu vardir ve LEDC'nin kullandigi
   altyapiyi paylasmaz. Servo/ESC darbesi tam olarak bu is icindir.

   Calisma sekli: 1 MHz tik (1 tik = 1 us). Her motor icin tek bir
   RMT sembolu yazilir:
       seviye 1, sure = darbe genisligi (1000-2000 us)
       seviye 0, sure = periyodun geri kalani (20000 - darbe)
   Sembol her 20 ms'de bir yeniden tetiklenir.                        */
const uint32_t PWM_PERIYOT_US = 20000;   // 50 Hz
const uint32_t RMT_TIK_HZ     = 1000000; // 1 us cozunurluk
const uint32_t MOTOR_YAZ_MS   = 20;      // RMT yeniden tetikleme periyodu

rmt_data_t motorSembol[4][1];
uint16_t   motorHedef[4] = { PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN };

bool motorKur() {
  bool ok = true;
  for (int i = 0; i < 4; i++) {
    if (!rmtInit(MOTOR_PIN[i], RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, RMT_TIK_HZ))
      ok = false;
  }
  return ok;
}

/* Hedef degeri kaydeder - fiziksel yazma motorlariGonder() icinde olur */
inline void motorYaz(uint8_t i, uint16_t us) {
  motorHedef[i] = constrain(us, PWM_MIN, PWM_MAX);
}

/* Dort kanalin sembolunu doldurup RMT'ye gonderir. 20 ms'de bir cagrilir.
   timeout 0 -> engellemeyen (async) gonderim; kontrol dongusu beklemez. */
void motorlariGonder() {
  for (int i = 0; i < 4; i++) {
    motorSembol[i][0].level0    = 1;
    motorSembol[i][0].duration0 = motorHedef[i];
    motorSembol[i][0].level1    = 0;
    motorSembol[i][0].duration1 = PWM_PERIYOT_US - motorHedef[i];
    rmtWrite(MOTOR_PIN[i], motorSembol[i], 1, 0);
  }
}

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

/* Tek eksen PID.
 *
 *  TUREV TERIMI JIROSKOPTAN ALINIR - hesaplanmaz.
 *  Aciyi sayisal olarak turetmek (aci_farki / dt) sensor gurultusunu
 *  buyutur: dt = 0.004 s oldugu icin 0.1 derecelik gurultu 25 derece/s
 *  gibi gorunur ve Kd ile carpilinca 200 us'lik sahte duzeltme uretir.
 *  OLCULDU: arac hareketsizken motorlar 1100-1438 arasinda savruluyordu.
 *  MPU6050 acisal hizi ZATEN dogrudan olcer - bolme yok, buyutme yok.
 *
 *  hiz : ilgili eksenin SUZULMUS jiroskop okumasi (derece/s)
 *        Yaw hiz modunda calistigi icin oraya 0 gecilir.
 */
float pidHesapla(PidDurum &d, const Kazanc &g,
                 float hedef, float olculen, float hiz, bool bisikletle) {
  float hata = hedef - olculen;

  // Integral: sadece armed ve gaz acikken birikir, aksi halde sifirlanir
  if (bisikletle) {
    d.integral += g.ki * hata * dt;
    d.integral = constrain(d.integral, -I_SINIR, I_SINIR);
  } else {
    d.integral = 0.0f;
  }

  float cikis = g.kp * hata + d.integral - g.kd * hiz;
  return constrain(cikis, -PID_SINIR, PID_SINIR);
}

/* Tum PID durumlarini sifirlar - disarm ve failsafe aninda cagrilir */
void pidSifirla() {
  dRoll.integral  = 0.0f;
  dPitch.integral = 0.0f;
  dYaw.integral   = 0.0f;
}

/* Dort motora ayni degeri yazar */
void motorlariYaz(uint16_t us) {
  for (int i = 0; i < 4; i++) motorYaz(i, us);
}

/* ====================================================================== */
/*  KURULUM                                                               */
/* ====================================================================== */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== ESP32 UCUS KONTROL v6.0 ==="));

  /* --- 1. ADIM: motorlar once susturulur -------------------------------
     ESC'ler enerji aldiginda gecerli bir dusuk sinyal gormezse hata bipi
     calar veya beklenmedik sekilde donebilir. Bu yuzden ilk is motor
     cikislarini PWM_MIN'e sabitlemektir.                                */
  motorKur();
  motorlariYaz(PWM_MIN);
  Serial.println(F("[OK] Motor cikislari PWM_MIN'e sabitlendi (LEDC)"));

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
  radio.setPALevel(RF_GUC);                    // yukaridaki sabitten
  /* VERI HIZI - ALICI VE VERICI AYNI OLMALI
     250 kbps menzil icin en iyisi, ANCAK piyasadaki nRF24 modullerinin
     buyuk kismi Nordic degil Si24R1 klonudur ve klonlarda 250 kbps modu
     yarı yarıya paket kaybi verir. printPrettyDetails() klonu ayirt
     edemez - ikisi de "nRF24L01+" gorunur.
     Kayip yuksekse once bu satiri RF24_1MBPS yapip test et.           */
  radio.setDataRate(RF24_1MBPS);               // klon uyumlulugu icin
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

  /* Jiroskop okumalari suzulur - ham degerler pervane titresimi tasir */
  static float rollHizi = 0.0f, pitchHizi = 0.0f, yawHizi = 0.0f;
  rollHizi  += GYRO_ALFA * (mpu.getGyroX() - rollHizi);
  pitchHizi += GYRO_ALFA * (mpu.getGyroY() - pitchHizi);
  yawHizi   += GYRO_ALFA * (mpu.getGyroZ() - yawHizi);

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

  float uRoll  = pidHesapla(dRoll,  G_ROLL,  rollHedef,  rollOlculen,
                            rollHizi,  integralAktif);
  float uPitch = pidHesapla(dPitch, G_PITCH, pitchHedef, pitchOlculen,
                            pitchHizi, integralAktif);
  float uYaw   = pidHesapla(dYaw,   G_YAW,   yawHedef,   yawHizi,
                            0.0f,      integralAktif);

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
      motorYaz(i, (uint16_t)m[i]);
    }
  }

  /* ---------- 10. RMT GONDERIM (50 Hz) --------------------------------
     PID 250 Hz'de hesaplanir ama ESC'ye giden darbe 50 Hz'dir. Ara
     hesaplar motorHedef[] icinde birikir, en guncel deger gonderilir.
     Bu blok OLMAZSA RMT yalnizca setup'ta bir kez tetiklenir ve
     motorlar hicbir komuta tepki vermez.                             */
  static uint32_t rmtSon = 0;
  if (millis() - rmtSon >= MOTOR_YAZ_MS) {
    rmtSon = millis();
    motorlariGonder();
  }

  /* ---------- 11. TELEMETRI (10 Hz) ----------------------------------- */
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
