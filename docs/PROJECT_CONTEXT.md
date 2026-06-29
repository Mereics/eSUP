# Context Proiect - Motor Electric Pentru Standup Paddle

Acest document este memoria de lucru a proiectului. Îl actualizăm de fiecare dată când se schimbă hardware-ul, cablajul, deciziile de protocol, testele sau statusul.

## Scop

Construim un sistem electric de propulsie și control pentru o placă de standup paddle.

Sistemul include:

- Motor BLDC de putere mare într-o elice ducted.
- Control direcțional prin ultima secțiune steerable a ductului.
- Telecomandă waterproof, de ținut într-o singură mână, cu display, trigger throttle, butoane și steering pe bază de IMU.
- Un ESP32-S3 receptor lângă flight controller.
- ArduPilot pe un flight controller Skystars H743 HD.
- Telemetrie înapoi către display-ul telecomenzii.

## Concept Mecanic

- Elicea este montată într-un duct.
- Ultima secțiune a ductului se poate roti pentru steering.
- Steeringul se face cu un servo foarte puternic.
- Placa trebuie să suporte steering manual și un mod sport în care înclinarea telecomenzii controlează direcția.

## Propulsie

- Motor: BLDC, aproximativ 4.5 kW.
- ESC: are două intrări PWM:
  - Intrare PWM normală pentru forward throttle.
  - Intrare PWM separată pentru reverse.
- Plajele exacte PWM pentru ESC trebuie măsurate/confirmate în siguranță.
- Telemetria bateriei motorului trebuie afișată pe display-ul telecomenzii.

## Flight Controller

- Placă: Skystars H743 HD.
- Firmware: ArduPilot.
- Responsabilități așteptate de la FC:
  - Output servo pentru ductul steerable.
  - Output PWM pentru ESC.
  - Output pentru reverse, dacă este suportat/necesar.
  - Integrare GPS.
  - Heading hold / moduri asistate.
  - Telemetrie MAVLink către ESP32-ul receptor.

## GPS Și Navigație

- GPS-ul va fi conectat la ArduPilot.
- Date GPS necesare:
  - Ground speed.
  - Număr de sateliți.
  - Heading / course.
  - Funcția de heading hold.

Întrebări deschise:

- Dacă heading hold trebuie implementat în principal în ArduPilot sau parțial în layer-ul ESP remote/receiver.
- Ce vehicle type și ce configurație de mixer/output în ArduPilot sunt cele mai potrivite pentru propulsie + duct steerable.

## Arhitectură Electronică

### Telecomandă / Transmitter

Hardware:

- ESP32-S3 Super Mini.
- Display rotund GC9A01, 240x240.
- Trigger throttle cu senzor Hall analogic.
- Magnet montat în mecanismul triggerului.
- IMU / giroscop / accelerometru pentru înclinarea telecomenzii.
- Butoane capacitive touch, deoarece telecomanda trebuie să fie waterproof.
- Baterie pentru telecomandă.

Responsabilități telecomandă:

- Citește triggerul de throttle.
- Citește butoanele touch.
- Citește înclinarea din IMU.
- Selectează modul: manual / sport / heading hold.
- Trimite date RC/control către ESP32-ul receptor.
- Primește telemetrie de la ESP32-ul receptor.
- Randare UI pe display.

Pinout display actual:

| Display GC9A01 | ESP32-S3 Super Mini |
| --- | --- |
| VCC / VIN | 3V3 |
| GND | GND |
| RST / RES | GPIO5 |
| CS | GPIO4 |
| DC | GPIO12 |
| SDA / MOSI | GPIO2 |
| SCL / SCK | GPIO1 |

Software display actual:

- LovyanGFX.
- Hardware SPI.
- Sprite framebuffer 240x240 pe 16-bit.
- Full-frame push către display pentru update smooth.

UI display actual:

- Gauge curbat sus: steering / turn indicator.
- Gauge curbat stânga: throttle.
- Gauge curbat dreapta: power.
- Gauge curbat jos: procent baterie motor/main battery.
- Centru:
  - Mode: manual/sport.
  - Status armed/disarmed.
  - Heading.
  - Speed.
  - Număr sateliți.
  - Nivel baterie telecomandă.

### Receptor

Hardware:

- ESP32-S3 Super Mini.
- Comunică cu flight controller-ul prin MAVLink.
- Comunică cu telecomanda ESP32 probabil prin ESP-NOW.

Responsabilități receptor:

- Primește pachete de control de la telecomandă.
- Trimite comenzi RC/control către ArduPilot / flight controller.
- Citește telemetrie MAVLink de la flight controller.
- Trimite pachete de telemetrie înapoi către telecomandă.

## Link Wireless

Protocol probabil:

- ESP-NOW între ESP32-S3 transmitter și ESP32-S3 receiver.

Date transmise de la telecomandă la receptor:

- Valoare trigger throttle.
- Butoane manual steering.
- Înclinare IMU / comandă steering sport.
- Comandă arm/disarm.
- Comandă heading hold.
- Selectare mod.
- Heartbeat / packet counter pentru failsafe.

Date transmise de la receptor la telecomandă:

- Tensiune/procent baterie motor/main battery.
- Ground speed.
- Număr sateliți.
- Heading.
- Status armed/disarmed.
- Mod curent.
- Status link/failsafe.
- Confirmări pentru comenzi, dacă este nevoie.

Întrebări deschise:

- Format pachet.
- Update rate.
- Timeout failsafe.
- Dacă pachetele trebuie criptate/semnate.
- Dacă ESP-NOW are range/reliability suficient peste apă.

## Controale

### Throttle

- Input fizic: trigger custom.
- Senzor: Hall analogic.
- Magnetul se mișcă față de senzor când triggerul este apăsat.
- Testul inițial a fost făcut și senzorul funcționează.
- În codul curent, valorile negative față de poziția de repaus sunt mapate la `0%`.

Procesare necesară pentru throttle:

- Citire ADC raw.
- Filtrare.
- Calibrare.
- Dead zone lângă poziția de repaus.
- Mapare către RC PWM sau comandă MAVLink.
- Failsafe dacă senzorul se deconectează sau raportează valori invalide.

### Steering

Mod manual:

- Butoane touch pentru steering stânga/dreapta.
- Comportamentul de center/trim este încă TBD.

Mod sport:

- Steering bazat pe înclinarea telecomenzii citită din IMU.
- Scop: riderul stă în picioare pe placă și controlează direcția dinamic prin înclinarea telecomenzii ținute într-o mână.

Heading hold:

- Buton dedicat.
- Folosește date GPS/heading.
- Implementarea exactă este TBD.

### Arm / Disarm

- Buton touch dedicat.
- Trebuie proiectat atent pentru a evita armarea accidentală.
- Probabil necesită long press, double press sau feedback UI clar.

## Butoane Capacitive Touch

Motiv:

- Telecomanda trebuie să fie waterproof.
- Butoanele capacitive touch evită deschiderile mecanice.

Teste necesare:

- Suport ESP32-S3 pentru capacitive touch și pinii disponibili pe placa aceasta.
- Comportament prin materialul carcasei.
- Comportament cu mâini ude/apă.
- Rezistență la false touch.
- Dacă este nevoie de controller capacitive touch extern în cazul în care touch-ul built-in nu este suficient de stabil.

## Status Repository Curent

Rol repository acum:

- Prototip transmitter/display pentru ESP32-S3 Super Mini.
- Proiect PlatformIO Arduino.

Status curent:

- Display-ul rotund GC9A01 funcționează.
- Randarea cu LovyanGFX este smooth.
- Proiectul PlatformIO este împărțit în două environment-uri:
  - `transmitter`, cu surse în `src/transmitter/`.
  - `receiver`, cu surse în `src/receiver/`.
- Dashboard-ul LovyanGFX a fost pus înapoi ca UI principal.
- Throttle-ul este citit live din senzorul Hall 49E/AH49E pe `GPIO6`.
- Steering-ul este pregătit pentru citire live din MPU-9250/6500/9255 pe axa Y.
- MPU-ul folosește I2C pe `GPIO7` SDA și `GPIO8` SCL, cu `AD0` la GND pentru adresa `0x68`.
- Butonul ARM TTP223 este pregătit pe `GPIO9`, momentary active-high, cu toggle armed/disarmed în firmware.
- Butonul Cruise TTP223 este pregătit pe `GPIO10`, momentary active-high, cu toggle cruise control în firmware.
- Power on/off se face temporar cu switch mecanic; ulterior se va folosi un switch magnetic/reed/hall potrivit pentru carcasa waterproof.
- Tensiunea bateriei telecomenzii este citită pe `GPIO11` prin divizor rezistiv `100k/100k`, cu factor `2.0`; B+ nu se leagă direct la ADC, pentru că o celulă Li-ion plină ajunge la aproximativ `4.2V`.
- Cruise control tine throttle-ul comandat curent pana la urmatoarea apasare.
- Throttle-ul trimis spre receiver este mapat software: `0%` ramane `0%`, iar valori peste deadband sunt mapate la `15..100%`; motorul incepe fizic sa se invarta de la aproximativ `20%`, deci ramane putin deadzone.
- Transmitter-ul trimite control catre receiver prin ESP-NOW: throttle, steering, arm toggle, mode placeholder.
- Receiver-ul trimite inapoi telemetrie prin ESP-NOW: armed real din FC, speed, heading, sateliti, baterie, link quality, RSSI.
- Receiver-ul trimite catre FC MAVLink heartbeat, RC override si command long pentru arm/disarm.
- Failsafe receiver: daca nu primeste control >500 ms, throttle `1000`, steering `1500`.
- Receiver-ul activ din `src/receiver/main.cpp` este codul MAVLink/UART către FC.
- Vechiul test ESP-NOW RTT + dashboard web este în `backup/receiver_espnow_rtt_dashboard.cpp`.
- Backup-ul testerului Hall este în `backup/hall_sensor_test.cpp`.
- Backup-ul dashboard-ului anterior este în `backup/dashboard_lovyangfx_mockup.cpp`.
- Display-ul pătrat ST7789 a fost testat și a funcționat cu SPI mode 3 la 8 MHz, dar nu mai este display-ul ales.

Dependințe curente:

- PlatformIO.
- Arduino framework.
- LovyanGFX.

Board target curent:

- `esp32-s3-devkitm-1`
- Flash size configurat la 4 MB.

## Plan Pe Termen Scurt

1. Cablat MPU-9250/6500/9255 la ESP32.
2. Test citire axa Y pentru steering.
3. Verificare sens steering; dacă este invers, schimbare `STEERING_INVERT`.
4. Ajustare `STEERING_DEADZONE_DEG`.
5. Ajustare `STEERING_FULL_SCALE_DEG`.
6. Integrare steering în modul manual/sport.
7. Test butoane capacitive touch.
8. Începere definire structură pachet transmitter-to-receiver.
9. Proof-of-concept receiver ESP32 + MAVLink.
10. Failsafe și arm/disarm logic.

## Note De Siguranță

- Motorul BLDC de 4.5 kW și ESC-ul sunt hardware de putere mare.
- Testele timpurii pentru throttle/ESC trebuie făcute fără elice sau cu elicea dezactivată în siguranță.
- Arm/disarm și failsafe sunt funcții critice.
- Reverse control trebuie testat atent pentru a evita inversări bruște de thrust.
- Waterproofing-ul și comportamentul touch cu apă trebuie testate real, nu doar pe banc.

## Necunoscute / Decizii De Luat

- Configurația finală ArduPilot.
- Plajele exacte PWM pentru ESC.
- Comportamentul intrării reverse.
- Model servo, alimentare servo, consum și range de control.
- Cablaj receiver-to-FC MAVLink și baud rate.
- Reliability/range ESP-NOW peste apă.
- Tip baterie telecomandă și design încărcare.
- Material carcasă waterproof și layout butoane capacitive.
- Model IMU.
- Confirmare model senzor Hall și curba mecanică trigger/magnet.
