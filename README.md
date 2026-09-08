# ESP32 Flight Controller From Scratch

> **English summary.** A quadcopter flight control system written from scratch on
> an ESP32 — no off-the-shelf flight controller. Includes a custom 2.4 GHz radio
> protocol (nRF24L01+), MPU6050 attitude estimation, a 250 Hz hard real-time
> control loop, PID control (angle mode for roll/pitch, rate mode for yaw) and a
> ten-layer safety architecture. Developed following a V-Model lifecycle with 19
> traceable requirements, SysML architecture models, a 12-item FMEA hazard
> analysis and a verification matrix. **Documentation is in Turkish**; the source
> code comments are in Turkish as well. Current status: all subsystems verified,
> first takeoff achieved, PID gain tuning in progress.

---

## Türkçe

Hazır bir uçuş kontrol kartı (Pixhawk, Betaflight FC vb.) kullanmadan, **ESP32
mikrodenetleyici üzerinde sıfırdan yazılmış** bir quadcopter uçuş kontrol
sistemi. Telsiz haberleşme protokolü, atalet ölçüm zinciri, emniyet mantığı ve
PID kontrol yasası proje kapsamında tasarlanıp gerçeklenmiştir.

Projenin amacı yalnızca uçan bir araç ortaya koymak değil; savunma ve havacılık
sektöründe kullanılan **sistem mühendisliği pratiklerini** — gereksinim
ayrıştırma, arayüz tanımı, tehlike analizi, artımlı doğrulama — küçük ölçekli
ama gerçek bir platformda uygulamaktır.

---

## Öne çıkanlar

| Ölçüt | Değer |
|---|---|
| Kontrol döngüsü | 250 Hz sabit periyot |
| Motor darbe hızı | 50 Hz (RMT çevre birimi) |
| Telsiz çerçeve hızı | 50 Hz, 10 baytlık çerçeve, 250 kbps |
| Statik açı sapması (roll/pitch, 3 dk) | < 0.1° |
| Yaw açısal sürüklenmesi | 0.1°/dk |
| Failsafe tepki süresi | < 500 ms |
| Kanal merkez kararlılığı | 0 µs titreşim |
| Zamanlama bütçesi kullanımı | ~%30 (yaklaşık %70 rezerv) |
| Yazılım hacmi | ~900 satır C++ / 4 modül + 3 tanı kodu |
| Paket kaybı (masa üstü) | 0 / saniyede 50 çerçeve |
| Belgelenen tehlike (FMEA) | 12 arıza modu, azaltım önlemleriyle |

---

## Olgunluk durumu

| Alt sistem | Durum | Doğrulama kanıtı |
|---|---|---|
| Telsiz haberleşme (2.4 GHz) | ✅ Tamamlandı | Kesintisiz ACK, 0 paket kaybı |
| Kumanda kanal işleme | ✅ Tamamlandı | 4 kanal 1000–2000 µs, ±0 µs merkez sapması |
| Atalet ölçüm birimi | ✅ Tamamlandı | Roll/pitch drift < 0.1°, yaw 0.1°/dk |
| Emniyet mantığı (failsafe) | ✅ Tamamlandı | 500 ms içinde gaz kesme, doğrulandı |
| Güç dağıtımı ve tahrik | ✅ Tamamlandı | Süreklilik + kutup testi, 4 motor yön doğrulaması |
| İşaret doğrulaması | ✅ Tamamlandı | Dört eksen de doğru yönde düzeltiyor |
| Motor sürüş katmanı (RMT) | ✅ Tamamlandı | 50 Hz darbe, ESC'ler senkron, 0 paket kaybı |
| Kontrol yasası (PID) | 🟡 Kazanç ayarı sürüyor | Roll/pitch 1.20/0/3.00, yaw 0.50/0/0 |
| İlk kalkış | 🟡 Gerçekleşti | Araç havalanıyor |
| Kararlı uçuş | ⚪ Parça bekliyor | Pervane civatası eğri — mekanik titreşim kaynağı |

> **Not:** Depodaki PID kazançları **ayar için başlangıç değerleridir ve henüz
> doğrulanmamıştır** (roll/pitch 1.20 / 0 / 8.0, yaw 2.00 / 0 / 0).
> Bu kodla yapılacak ilk iş **işaret doğrulamasıdır**: pervaneler sökülü,
> şasi sabit, gaz ~1200; aracı elle eğip motor komutlarının doğru yönde
> arttığı seri porttan kontrol edilir. Ayar prosedürü için
> `docs/01_Sistem_Tasarim_Dokumani.pdf` Bölüm 8.3'e bakınız.

---

## Doküman seti

Bu depo yalnızca kod değil, tam bir mühendislik kaydı içerir.

| Doküman | Kod | İçerik |
|---|---|---|
| [Sistem Tasarım Dokümanı](docs/01_Sistem_Tasarim_Dokumani.pdf) | SDD-QC-001 | Gereksinimler, SysML mimarisi (BDD/IBD), durum makinesi, FMEA, test matrisi, PID ayar prosedürü, V-Model, çıkarılan dersler |
| [Yazılım Dokümantasyonu](docs/02_Yazilim_Dokumantasyonu.pdf) | SWD-QC-002 | Katman mimarisi, telsiz protokolü, satır düzeyinde kod açıklamaları, emniyet mekanizmaları, zamanlama bütçesi |
| [Proje Özeti ve CV Eki](docs/03_CV_Eki_Proje_Ozeti.pdf) | PRJ-QC-003 | Yetkinlik matrisi, sayısal çıktılar, mülakat hazırlığı |

---

## Sistem mimarisi

```
┌──────────────────┐   2.4 GHz    ┌──────────────────┐   4×PWM   ┌──────────────┐
│  EL KUMANDASI    │  250 kbps    │ UÇUŞ KONTROLCÜSÜ │           │ TAHRİK GRUBU │
│                  │─────────────▶│                  │──────────▶│              │
│ ESP32-WROOM-32   │◀ ─ ─ ACK ─ ─ │ ESP32-WROOM-32   │           │ 4 × 30 A ESC │
│ 2× analog joystick│              │ MPU6050 IMU      │           │ 4 × BLDC 2212│
│ nRF24L01+ (TX)   │              │ nRF24L01+ (RX)   │           │ 3S 11.1 V    │
│ 4 kanal @ 50 Hz  │              │ PID @ 250 Hz     │           │ X konfigür.  │
└──────────────────┘              └──────────────────┘           └──────────────┘
```

**Yazılım katmanları** (her katman bir üsttekine geçilmeden bağımsız doğrulandı):

```
Kontrol yasası     PID · motor karıştırma            ← ayar bekliyor
Emniyet mantığı    arming · failsafe · sınırlama     ✅
Kestirim           MPU6050 · açı ve açısal hız       ✅
Haberleşme         nRF24 · çerçeve doğrulama         ✅
Donanım soyutlama  SPI · I²C · LEDC PWM              ✅
```

---

## Donanım

| Bileşen | Model | Adet |
|---|---|---|
| Mikrodenetleyici | ESP32-WROOM-32 DevKit V1 (**38 pin**) | 2 |
| Telsiz | nRF24L01+ PA/LNA | 2 |
| Atalet ölçüm birimi | MPU6050 (6-DOF) | 1 |
| Motor sürücü | 30 A ESC (BEC'li) | 4 |
| Motor | BLDC 2212 sınıfı | 4 |
| Gövde | F450 quadrotor şasi (tümleşik güç katı) | 1 |
| Batarya | 3S 11.1 V 2200 mAh LiPo | 1 |
| Güç konnektörü | XT60 | 1 çift |
| Girdi cihazı | 2 eksenli analog joystick modülü | 2 |

**Neden 38 pinli ESP32:** 30 pinli sürüm GPIO 34–39 hatlarını dışarı vermez.
Dört analog eksenin ADC1'e (GPIO 32–35) yerleştirilmesi zorunludur, çünkü
**ADC2 telsiz yığını etkinken kullanılamaz.**

### Pin haritası

**Verici (el kumandası)**

| GPIO | İşlev | Çevre birimi |
|---|---|---|
| 4 / 5 | nRF24 CE / CSN | GPIO |
| 18 / 19 / 23 | SCK / MISO / MOSI | VSPI |
| 32 | Gaz | ADC1 |
| 33 | Yaw | ADC1 |
| 34 | Pitch | ADC1 (yalnızca giriş) |
| 35 | Roll | ADC1 (yalnızca giriş) |

**Alıcı (uçuş kontrolcüsü)**

| GPIO | İşlev | Çevre birimi |
|---|---|---|
| 4 / 5 | nRF24 CE / CSN | GPIO |
| 18 / 19 / 23 | SCK / MISO / MOSI | VSPI |
| 21 / 22 | SDA / SCL (MPU6050) | I²C @ 400 kHz |
| 25 | Motor 1 — ön-sağ (CW) | RMT |
| 26 | Motor 2 — arka-sağ (CCW) | RMT |
| 32 | Motor 3 — arka-sol (CW) | RMT |
| 33 | Motor 4 — ön-sol (CCW) | RMT |

> **Pinler v4.1'de taşındı.** Eski değerler 12/13/14/27 idi. GPIO 12 bir
> strapping pini (MTDI), 13 ve 14 ise HSPI/JTAG hatları. Bu pinlerde PWM
> üretilirken telsizde paket kaybı ölçüldü.

### Motor yerleşimi (X konfigürasyonu, üstten bakış)

```
        ÖN
   M4 \       / M1
   (CCW)     (CW)
       \ ___ /
        |   |
        |___|
       /     \
   (CW)       (CCW)
   M3 /       \ M2
       ARKA
```

Karıştırma denklemleri:

```
M1 = gaz − u_roll − u_pitch − u_yaw     (ön-sağ,   CW)
M2 = gaz − u_roll + u_pitch + u_yaw     (arka-sağ, CCW)
M3 = gaz + u_roll + u_pitch − u_yaw     (arka-sol, CW)
M4 = gaz + u_roll − u_pitch + u_yaw     (ön-sol,   CCW)
```

⚠️ İşaretler IMU'nun gövdeye montaj yönelimine bağlıdır. Ayar öncesinde
**işaret doğrulaması** yapılmalıdır (aşağıya bakınız).

---

## Kurulum

### Araç zinciri

| Bileşen | Sürüm / ayar |
|---|---|
| Arduino IDE | 2.x |
| ESP32 kart paketi | `esp32` by Espressif Systems 3.3.x |
| Kart seçimi | ESP32 Dev Module |
| Upload Speed | **115200** (921600'de seri gürültü hatası gözlendi) |
| USB köprü sürücüsü | Silicon Labs CP210x VCP |
| Seri monitör | 115200 baud |

> Sürücü kurulmadan port hiç görünmez. Windows'ta Aygıt Yöneticisi'nde
> "bilinmeyen aygıt" görüyorsanız eksik olan CP210x sürücüsüdür.

### Kütüphaneler

| Kütüphane | Yazar | Not |
|---|---|---|
| RF24 | **TMRh20** | Aynı adla başka paketler var, yalnızca bu uyumlu |
| MPU6050_light | **rfetick** | — |
| Wire, SPI | çekirdek | — |

> **ESP32Servo kullanılmıyor.** PWM üretimi doğrudan ESP32'nin RMT çevre
> birimiyle yapılıyor; ek kütüphane gerekmez. Gerekçe aşağıda.

---

## Devreye alma sırası

Bu sıra **atlanmamalıdır.** Her adım bir sonrakinin ön koşuludur.

1. **Bataryayı çıkar.** Kod yükleme sırasında ESC'nin BEC hattı ile USB
   beslemesi çakışır ve seri iletişim bozulur.
2. `kod/arac_esc_kalibrasyon.ino` yükle, seri monitörü aç.
3. "MAX sinyal aktif" mesajını görünce bataryayı tak, bip dizisini bekle.
4. `kod/arac_motor_test.ino` yükle. Seri porttan `1`–`4` göndererek motorları
   tek tek çalıştır, dönüş yönlerini doğrula.
5. Yönler doğrulandıktan **sonra** motor kablolarını kalıcı olarak lehimle.
6. `kod/verici_v2_kumanda.ino` verici karta, `kod/alici_v3_ucus_kontrol.ino`
   alıcı karta yüklenir.
7. Kazançlar sıfırken bağlantı, arming ve failsafe davranışını doğrula —
   **pervanesiz.**
8. Test standına geç: işaret doğrulaması → roll/pitch kazanç ayarı → yaw ayarı.
9. Kabul kriteri sağlandıktan sonra serbest uçuş.

### İşaret doğrulaması (adım 8'in ilk işi)

Tüm kazançlar sıfırken, aracı stand üzerinde elle eğip seri porttaki motor
komutlarını izleyin:

- Sağa yatırıldığında **sağ motorların** komutu artmalı
- İleri eğildiğinde **ön motorların** komutu artmalı

Yön tersse, ilgili terimin karıştırma denklemindeki işareti değiştirilir.

---

## ⚠️ Emniyet

Bu bölüm süs değildir. Pervaneli bir quadcopter ciddi yaralanma üretebilir.

- **Pervaneler en sona takılır.** Test standı üzerinde kararlılık
  ispatlanmadan pervane takılmaz. Tüm masa testleri pervanesiz yapılır.
- **Her kod yüklemesinden önce batarya çıkarılır.**
- **Tek BEC kuralı.** Yalnızca bir ESC'nin kırmızı (5 V) hattı kontrolcüye
  bağlanır, diğer üçü yalıtılır. Toprak hatları ortak referans için tümüyle
  bağlıdır.
- **Batarya bağlamadan önce multimetre.** XT60 pinleri arası süreklilik *yok*,
  artı ve eksi hatlarda ESC'lere doğru süreklilik *var* olmalıdır. Ters kutup,
  dört ESC'nin birden kaybı demektir.
- **LiPo.** Hücre gerilimlerini izleyin, 3.80 V civarında depolayın, şişmiş
  veya darbe almış paketi kullanmayın, yanmaz muhafazada saklayın.
- **Motor testleri sırasında** şasi masaya sabitlenmiş, eller motorlardan
  uzak olmalıdır.
- **Acil gaz kesme.** Kilitli gaz modunda gaz kolunu *tam aşağı* itmek gazı
  anında 1000 µs'ye düşürür. Yay merkezleme yaptığı için bu konuma kazara
  gelinmez. Uçuş öncesi bu davranışı pervanesiz doğrulayın.

---

## Emniyet mimarisi (kod düzeyinde)

Tek bir mekanizmaya güvenilmez. On bağımsız katman:

| # | Mekanizma | Konum |
|---|---|---|
| 1 | Başlangıçta motor çıkışlarının asgariye sabitlenmesi | `setup()` 1. adım |
| 2 | IMU / telsiz başlatma hatasında güvenli kilit | `setup()` 2.–3. adım |
| 3 | Çerçeve aralık süzgeci (950–2050 µs) | `loop()` 1. blok |
| 4 | 500 ms zaman aşımlı failsafe | `loop()` 2. blok |
| 5 | Failsafe'te otomatik silahsızlanma | `loop()` 2. blok |
| 6 | Arming zorunluluğu (gaz asgari şartı) | `loop()` 3. blok |
| 7 | Koşullu integral birikimi + windup sınırı | `pidHesapla()` |
| 8 | Eksen başına PID çıkış sınırlaması (±320 µs) | `pidHesapla()` |
| 9 | Gaz tavanı (1900 µs) ile düzeltme baş payı | `loop()` 6. blok |
| 10 | Silahsız / gaz kapalı durumda koşulsuz asgari çıkış | `loop()` 9. blok |

10 numara bilinçli bir fazlalıktır — 6 numara zaten aynı işi yapar. Ancak
emniyet-kritik yolda tek kontrole güvenmek, o kontroldeki bir mantık hatasının
doğrudan tehlikeye dönüşmesi demektir.

---

## Dizin yapısı

```
.
├── README.md
├── docs/
│   ├── 01_Sistem_Tasarim_Dokumani.pdf     SDD-QC-001
│   ├── 02_Yazilim_Dokumantasyonu.pdf      SWD-QC-002
│   └── 03_CV_Eki_Proje_Ozeti.pdf          PRJ-QC-003
└── kod/
    ├── verici_v2_kumanda.ino              el kumandası (v2.4, kalıcı)
    ├── alici_v3_ucus_kontrol.ino          uçuş kontrolcüsü (v6.0, kalıcı)
    ├── arac_esc_kalibrasyon.ino           ESC kalibrasyonu (v3.0, RMT)
    ├── arac_motor_test.ino                motor yön doğrulaması
    └── test/                              tanı kodları
        ├── test_telsiz_verici.ino         minimal telsiz testi (TX)
        ├── test_telsiz_alici.ino          minimal telsiz testi (RX)
        └── tani_alici_motorsuz.ino        PWM izolasyon testi
```

---

## Öne çıkan tasarım kararları

**Yaw ekseninde açı değil açısal hız kontrolü.** MPU6050'de manyetometre yok;
ivmeölçer yatay düzlemdeki dönüşü göremediği için yaw açısının düzeltme
referansı bulunmuyor ve jiroskop integrali sürükleniyor (ölçülen: 0.1°/dk).
Açısal hız integralden gelmediği için bu sürüklenmeden etkilenmez. Donanım
eksikliği, kontrol değişkeni seçimiyle mimari düzeyde telafi edilmiştir.

**Türev terimi hatadan değil ölçülen değerden.** Pilot komutu ani değiştiğinde
hata sıçrar ve türev büyük bir darbe üretir (*derivative kick*). Ölçülen açıdan
türev almak bunu ortadan kaldırır — aracın fiziksel açısı ani sıçrayamaz.

**PWM tavanı 2000 değil 1900.** PID'nin düzeltme yapabilmesi için tepe gazın
altında baş payı kalmalıdır. Tüm motorlar tavana dayanmışsa kontrolcü artık
hiçbir eksende düzeltme üretemez.

**Kilitli gaz, yazılımda.** Kumanda modülünde tek merkezleme yayı iki ekseni
birden merkeziyordu; gaz için yay sökülünce yaw da merkezlemesini kaybetti.
Mekanik çözüm geri alındı, sorun yazılım katmanında çözüldü — kol itildiği
sürece gaz artar, bırakılınca son değerde kilitlenir. Karesel tepki eğrisi
küçük hareketlerde hassasiyet sağlar.

**Emniyet blokları zamanlama kapısından önce.** Kontrol yasası 4 ms'de bir
güncellenir, ancak failsafe ve arming kontrolleri her turda koşulsuz çalışır.

---

## Çıkarılan dersler

Ayrıntılı kayıtlar (D-01…D-06) `docs/01_Sistem_Tasarim_Dokumani.pdf` Bölüm
10'dadır. Kısaca:

- **Boş çerçeveler geçerli veri sanılabilir.** Verici beslemesiz kalınca alıcı
  0/65535 salınımı gösterdi. Ayırt edici ipucu, paket sayacının da bozulmasıydı
  — sayaç kodda yalnızca artar. *Bir hatanın kaynağını bulmak için, hata
  sinyalinin etkilememesi gereken değişkenlere bakmak en hızlı yoldur.*
- **Bağlantı kopmaları çoğu zaman protokol değil güç bütünlüğü sorunudur.**
  nRF24 PA/LNA verici moduna geçerken yüksek akım çeker; MCU'nun 3.3 V
  regülatörü bunu karşılayamaz. Ayrı regülatörlü adaptör veya 10 µF kondansatör
  çözer. *Yazılımda saatlerce aranan hata, donanımda bir kondansatörle
  çözülebilir.*
- **Varsayılan ayarlar genel amaçlıdır.** nRF24 fabrika çıkışında 1 Mbps ve
  WiFi bandının ortasındaki 76. kanaldadır. 250 kbps + kanal 100 menzili
  belirgin artırır; veri yükü zaten çok düşüktür.
- **Geri döndürülemez adımlar doğrulamadan sonraya bırakılır.** Motor kabloları,
  dönüş yönü geçici bağlantıyla doğrulanana kadar lehimlenmedi.
- **İki çevre birimi görünürde ilgisizken birbirini bozabilir.** PWM üretimi
  için kullanılan LEDC birimi, telsizin SPI hattında %60 paket kaybına yol
  açtı. Teşhis sistematik elemeyle yapıldı: önce minimal test kodu (yalnızca
  telsiz → %100 başarı), sonra tek tek değişken kapatma. Suçlu bulununca üç
  frekans/çözünürlük kombinasyonu denendi; hiçbiri hem ESC senkronunu hem
  telsiz temizliğini sağlamadı. Çözüm LEDC'yi tümüyle bırakıp **RMT** çevre
  birimine geçmek oldu. *Bir bileşen çalışıyor görünüyorsa bile, komşusunu
  bozup bozmadığını ölçmek gerekir.*
- **Sensör gürültüsünü türetmek felakettir.** D terimi başlangıçta açının
  sayısal türeviydi. dt = 4 ms olduğu için 0.1°'lik sensör gürültüsü 25 °/s
  gibi görünüyor, Kd ile çarpılınca 200 µs'lik sahte düzeltme üretiyordu —
  araç hareketsizken motorlar 1100–1438 arasında savruluyordu. Çözüm: türevi
  hesaplamak yerine jiroskopun **zaten ölçtüğü** açısal hızı kullanmak.
- **Mekanik sorunu yazılımla örtme.** Orta gazda başlayan titreşimin kaynağı
  eğri bir pervane civatasıydı. Yazılım filtresi belirtiyi azaltır ama sebebi
  ortadan kaldırmaz.

---

## Yol haritası

**Kısa vadeli**

- [x] Kablo yönetimi, kart montajı, IMU'nun titreşim yalıtımlı bağlanması
- [x] Ağırlık merkezi dengeleme ve batarya konumlandırma
- [ ] Test standı imalatı (tek eksen serbestlik)
- [x] Karıştırma işaret doğrulaması
- [ ] Roll / pitch kazanç ayarı
- [ ] Yaw kazanç ayarı
- [ ] Kararlı uçuş kabul testi

**Orta vadeli**

- [ ] Telemetri geri kanalı (batarya gerilimi, paket kaybı, açı)
- [ ] Kaskatlı PID (dış açı çevrimi + iç açısal hız çevrimi)
- [ ] Batarya gerilim izleme ve kritik eşik uyarısı
- [ ] Çift çekirdek ayrıştırma (FreeRTOS görev ayrımı)
- [ ] Barometre ile irtifa tutma
- [ ] SD kart üzerine kara kutu kaydı
- [ ] Alıcı düğümü için özel PCB tasarımı

---

## Lisans

MIT — eğitim ve kişisel kullanım için serbesttir.

Bu kod eğitim amaçlıdır ve emniyet-kritik uygulamalar için sertifikalı
değildir. Kullanım sorumluluğu kullanıcıya aittir.
