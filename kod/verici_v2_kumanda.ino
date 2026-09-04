/* ===========================================================================
 *  ESP32 QUADCOPTER - EL KUMANDASI (VERICI)
 *  Dosya   : verici_v2_kumanda.ino
 *  Donanim : ESP32 DevKit V1 (38 pin) + nRF24L01+ PA/LNA + 2x analog joystick
 *  Surum   : v2.1
 *
 *  OZELLIKLER
 *    - 4 kanal, 50 Hz cerceve hizi (millis tabanli, kayma yapmaz)
 *    - Eksen basina merkez kalibrasyonu ve olu bolge
 *    - Gaz ekseninde "kilitli gaz" modu (yay kisiti icin yazilim cozumu)
 *    - Karesel tepki egrisi: kucuk hareket = hassas, tam hareket = hizli
 *    - ACIL GAZ KESME: gaz kolu tam asagi -> gaz aninda 1000'e duser
 *    - Baglanti kalitesi olcumu (ACK basari orani)
 *
 *  v2.1 DEGISIKLIKLERI
 *    - radio.begin() donus degeri kontrol ediliyor (sessiz ariza engellendi)
 *    - setRetries sinirlandi: kotu baglantida dongu blokaji onlendi
 *    - Acil gaz kesme eklendi (kilitli gaz modunun emniyet karsiligi)
 *    - delay(20) yerine millis() tabanli sabit periyot
 *    - Seri cikti 10 Hz'e seyreltildi, baglanti kalitesi eklendi
 * ======================================================================== */

#include <SPI.h>
#include <RF24.h>

/* ---------------------------------------------------------------- PINLER */
#define PIN_NRF_CE   4
#define PIN_NRF_CSN  5
#define PIN_GAZ     32     // sol joystick  - dikey   (ADC1)
#define PIN_YAW     33     // sol joystick  - yatay   (ADC1)
#define PIN_PITCH   34     // sag joystick  - dikey   (ADC1)
#define PIN_ROLL    35     // sag joystick  - yatay   (ADC1)
// NOT: Dort eksen de ADC1 pinlerindedir. ADC2 telsizle ayni anda
//      kullanilamaz (ESP32 donanim kisiti).

/* ------------------------------------------- KALIBRASYON (OLCUM SONUCU) */
// Her joystick serbest birakildiginda okunan ham ADC degeri.
// Fabrikasyon toleransi nedeniyle 2048 degil, eksen basina farklidir.
const uint16_t MERKEZ_GAZ   = 1850;
const uint16_t MERKEZ_YAW   = 1900;
const uint16_t MERKEZ_PITCH = 1860;
const uint16_t MERKEZ_ROLL  = 1880;

const uint16_t OLU_BOLGE     = 80;     // +/- ADC birimi - titremeyi susturur
const uint16_t GAZ_OLU_BOLGE = 200;    // gaz icin daha genis bant
const float    GAZ_HIZI      = 12.0;   // us / dongu - gaz degisim hizi
const float    ACIL_KES_ESIK = 0.90;   // bu oranin uzerinde asagi = acil kesme

const uint16_t CERCEVE_MS    = 20;     // 50 Hz cerceve hizi
const uint16_t TELEMETRI_MS  = 100;    // 10 Hz seri cikti

/* -------------------------------------------------------- GLOBAL NESNE */
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const byte ADRES[6] = "DRN01";

struct Paket {
  uint16_t kanal[4];    // 0:gaz 1:yaw 2:pitch 3:roll
  uint16_t sayac;
};
Paket veri;

float    gaz        = 1000.0;  // kilitli gaz - guc kesilene kadar korunur
uint32_t cerceveSon = 0;
uint32_t yazSon     = 0;
uint16_t gonderilen = 0;       // son pencerede gonderilen cerceve sayisi
uint16_t basarili   = 0;       // son pencerede ACK donen cerceve sayisi
uint8_t  kalite     = 0;       // baglanti kalitesi, yuzde

/* ======================================================================
 *  kanalOku - ham ADC degerini 1000-2000 us bandina cevirir
 *
 *  Merkezin iki yaninda AYRI olcekleme yapilir. Boylece merkez degeri
 *  2048'den sapmis olsa bile cikis tam olarak 1500'de merkezlenir.
 *  Tek bir map() kullanilsaydi merkez kacikligi dogrudan cikisa tasinirdi.
 * ==================================================================== */
uint16_t kanalOku(uint8_t pin, uint16_t merkez, bool ters) {
  int ham = analogRead(pin);
  int cikis;

  if (ham > merkez + OLU_BOLGE) {
    cikis = map(ham, merkez + OLU_BOLGE, 4095, 1500, 2000);
  } else if (ham < merkez - OLU_BOLGE) {
    cikis = map(ham, 0, merkez - OLU_BOLGE, 1000, 1500);
  } else {
    cikis = 1500;                       // olu bolge icinde: tam merkez
  }

  cikis = constrain(cikis, 1000, 2000);
  if (ters) cikis = 3000 - cikis;       // ekseni yazilimda tersine cevir
  return (uint16_t)cikis;
}

/* ======================================================================
 *  gaziGuncelle - kilitli gaz mantigi
 *
 *  Joystick modulunde tek merkezleme yayi iki ekseni birden kontrol
 *  ediyordu; gaz icin yay sokuldugunde yaw ekseni de merkezlemesini
 *  kaybetti. Cozum: yay yerinde birakildi, gaz yazilimda kilitlendi.
 *    kol yukari  -> gaz artar
 *    kol serbest -> gaz son degerinde kalir
 *    kol TAM asagi -> ACIL KESME, gaz aninda 1000
 * ==================================================================== */
void gaziGuncelle() {
  int hamGaz = analogRead(PIN_GAZ);

  if (hamGaz > MERKEZ_GAZ + GAZ_OLU_BOLGE) {
    /* --- YUKARI: gaz artar, karesel tepki egrisiyle.
       Karesel egri sayesinde kolun az itildigi bolgede degisim cok
       yavas, tam itildiginde hizlidir. Pratik karsiligi, aracin havada
       asili kaldigi gaz noktasi cevresinde ince ayar yapilabilmesidir. */
    float oran = (float)(hamGaz - MERKEZ_GAZ - GAZ_OLU_BOLGE)
               / (4095.0 - MERKEZ_GAZ - GAZ_OLU_BOLGE);
    oran = constrain(oran, 0.0, 1.0);
    gaz += GAZ_HIZI * oran * oran;

  } else if (hamGaz < MERKEZ_GAZ - GAZ_OLU_BOLGE) {
    float oran = (float)(MERKEZ_GAZ - GAZ_OLU_BOLGE - hamGaz)
               / (MERKEZ_GAZ - GAZ_OLU_BOLGE);
    oran = constrain(oran, 0.0, 1.0);

    if (oran > ACIL_KES_ESIK) {
      /* --- ACIL GAZ KESME
         Kilitli gaz modunun emniyet karsiligi. Normal azaltmayla gazi
         2000'den 1000'e indirmek kolu ~1.7 saniye asagida tutmayi
         gerektirir; acil durumda bu sure uzundur. Kol TAM asagi
         itildiginde gaz dogrudan sifirlanir.
         Yay merkezleme yaptigi icin bu konuma kazara gelinmez -
         kasitli ve tam bir hareket gerekir.                            */
      gaz = 1000.0;
    } else {
      gaz -= GAZ_HIZI * oran * oran;
    }
  }
  /* olu bolge icinde: hicbir sey yapilmaz, gaz son degerinde KILITLI kalir */

  gaz = constrain(gaz, 1000, 2000);
}

/* ====================================================================== */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== ESP32 EL KUMANDASI v2.1 ==="));

  /* Telsiz baslatma. Donus degeri KONTROL EDILIR: modul bulunamazsa kod
     sessizce calismaya devam eder ve hicbir sey gondermez. Bu, sahada
     "alici bozuk" gibi gorunen yaniltici bir ariza uretir.              */
  if (!radio.begin()) {
    Serial.println(F("[HATA] nRF24 bulunamadi - kablolamayi kontrol edin"));
    Serial.println(F("       CE=4  CSN=5  SCK=18  MISO=19  MOSI=23"));
    Serial.println(F("       Besleme 3.3 V olmali, 5 V modulu bozar."));
    while (1) delay(500);               // guvenli durus
  }

  radio.openWritingPipe(ADRES);
  radio.setPALevel(RF24_PA_LOW);        // ucusta RF24_PA_MAX yapilir
  radio.setDataRate(RF24_250KBPS);      // dusuk hiz = uzun menzil
  radio.setChannel(100);                // 2.4 GHz WiFi bandindan uzak

  /* Yeniden deneme siniri. Varsayilan ayar kotu baglantida tek bir
     write() cagrisini birkac milisaniye bloklar; bu 20 ms'lik cerceve
     butcesinin buyuk bolumunu yer ve cerceve hizini duzensizlestirir.
     5 x 250 us gecikme, 3 deneme = en fazla ~4 ms.                     */
  radio.setRetries(5, 3);

  radio.stopListening();

  gaz        = 1000.0;                  // her acilista gaz sifirdan baslar
  veri.sayac = 0;
  cerceveSon = millis();
  yazSon     = millis();

  Serial.println(F("[OK] nRF24 hazir - kanal 100, 250 kbps"));
  Serial.println(F("[OK] Gaz 1000'den basliyor (kilitli mod)"));
  Serial.println(F("     ACIL KESME: gaz kolunu TAM asagi it\n"));
}

/* ====================================================================== */
void loop() {
  /* ---------- 1. SABIT 50 Hz CERCEVE ZAMANLAMASI ----------------------
     delay(20) yerine millis() kullanilir. delay, kendisinden onceki
     islerin suresini hesaba katmaz; gercek cerceve hizi 50 Hz'in altina
     duser ve islem yuku arttikca kayar.                                */
  uint32_t simdi = millis();
  if (simdi - cerceveSon < CERCEVE_MS) return;
  cerceveSon = simdi;

  /* ---------- 2. EKSENLERI OKU ---------------------------------------- */
  gaziGuncelle();
  veri.kanal[0] = (uint16_t)gaz;
  veri.kanal[1] = kanalOku(PIN_YAW,   MERKEZ_YAW,   false);
  veri.kanal[2] = kanalOku(PIN_PITCH, MERKEZ_PITCH, false);
  veri.kanal[3] = kanalOku(PIN_ROLL,  MERKEZ_ROLL,  false);

  /* ---------- 3. GONDER ------------------------------------------------
     sayac her cercevede artar ve asla sifirlanmaz. Alici tarafta paket
     kaybi bu sayacin sureklilgi uzerinden olculur.                     */
  veri.sayac++;
  bool ok = radio.write(&veri, sizeof(veri));   // ACK donerse true

  gonderilen++;
  if (ok) basarili++;

  /* ---------- 4. TELEMETRI (10 Hz) ------------------------------------
     Her cercevede yazmak 115200 baud'da butcenin onemli bir kismini
     yerdi. Ayrica baglanti kalitesi yuzdesi burada hesaplanir - menzil
     testinde tek bakilacak sayi budur.                                 */
  if (simdi - yazSon >= TELEMETRI_MS) {
    yazSon = simdi;

    if (gonderilen > 0) kalite = (uint8_t)((basarili * 100UL) / gonderilen);
    gonderilen = 0;
    basarili   = 0;

    Serial.print(F("T:"));  Serial.print(veri.kanal[0]);
    Serial.print(F(" Y:")); Serial.print(veri.kanal[1]);
    Serial.print(F(" P:")); Serial.print(veri.kanal[2]);
    Serial.print(F(" R:")); Serial.print(veri.kanal[3]);
    Serial.print(F("  | link:")); Serial.print(kalite);
    Serial.println(kalite > 0 ? F("%  [BAGLI]") : F("%  [KOPUK]"));
  }
}
