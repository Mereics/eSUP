# Telecomanda ESP32-S3 pentru SUP electric

Firmware PlatformIO pentru telecomanda si receptorul sistemului de propulsie al unui standup paddle electric. Documentul extins de lucru este in [docs/PROJECT_CONTEXT.md](docs/PROJECT_CONTEXT.md).

## Status curent

### Implementat si testat

- Display rotund GC9A01 240x240 controlat prin LovyanGFX.
- Dashboard randat intr-un sprite RGB565 de 240x240 si trimis ca frame complet, fara flicker vizibil.
- Throttle analogic cu senzor Hall KY-035/49E.
- Steering prin inclinarea telecomenzii, citita din accelerometrul MPU-6500/9250/9255.
- Buton capacitiv TTP223 pentru ARM/DISARM.
- Buton capacitiv TTP223 pentru cruise control.
- Control bidirectional ESP-NOW intre transmitter si receiver.
- Trimiterea throttle-ului si steering-ului in ArduPilot prin MAVLink `RC_CHANNELS_OVERRIDE`.
- Comanda ARM/DISARM prin `MAV_CMD_COMPONENT_ARM_DISARM`.
- Telemetrie MAVLink de baza trimisa inapoi la display: armed, viteza, heading, sateliti si bateria principala.
- Failsafe local pe receiver pentru pierderea pachetelor de control.
- Alimentarea placii transmitter de la o celula Li-ion prin pad-urile `B+` si `B-` a fost testata dupa reincarcarea bateriei.

### Urmeaza sa fie facut

- Cablarea masurarii bateriei telecomenzii pe `GPIO11` prin divizor rezistiv.
- Calibrarea tensiunii ADC fata de un multimetru si verificarea estimarii procentului Li-ion.
- Alegerea si montarea intrerupatorului magnetic waterproof pentru alimentarea telecomenzii.
- Implementarea modului SPORT ca mod distinct; campul de protocol exista, dar transmitter-ul trimite momentan mereu modul `0`.
- Implementarea heading hold si a butonului dedicat.
- Implementarea reverse-ului ESC.
- Masurarea RSSI real; campul exista in protocol, dar receiver-ul trimite momentan `0`.
- Teste de raza, pierderi de pachete si comportament deasupra apei.
- Validarea failsafe-ului complet, inclusiv comportamentul de ARM/DISARM la pierderea legaturii.
- Teste cu carcasa finala waterproof si cu butoanele capacitive ude.

## Structura proiectului

```text
src/transmitter/             firmware telecomanda
src/receiver/                firmware receptor conectat la flight controller
src/common/remote_protocol.h protocolul comun ESP-NOW
backup/                      testere si versiuni anterioare pastrate pentru referinta
docs/PROJECT_CONTEXT.md      memoria completa a proiectului
```

Proiectul foloseste doua environment-uri PlatformIO:

```powershell
# Telecomanda
pio run -e transmitter
pio run -e transmitter -t upload
pio device monitor -e transmitter

# Receptor
pio run -e receiver
pio run -e receiver -t upload
pio device monitor -e receiver
```

Environment-ul implicit este `transmitter`.

Configuratie comuna:

- board PlatformIO: `esp32-s3-devkitm-1`
- framework: Arduino
- flash: 4 MB
- USB mode: hardware USB
- USB CDC on boot: activat
- monitor: 115200 baud

Dependinte:

- transmitter: `lovyan03/LovyanGFX`
- receiver: `okalachev/MAVLink`

## Pinout transmitter

| Functie | Modul/pin | ESP32-S3 Super Mini |
| --- | --- | --- |
| Display clock | GC9A01 SCL/SCK | GPIO1 |
| Display data | GC9A01 SDA/MOSI | GPIO2 |
| Display chip select | GC9A01 CS | GPIO4 |
| Display reset | GC9A01 RST/RES | GPIO5 |
| Display data/command | GC9A01 DC | GPIO12 |
| Throttle analogic | KY-035/49E OUT/S | GPIO6 |
| IMU data | MPU SDA | GPIO7 |
| IMU clock | MPU SCL | GPIO8 |
| ARM | TTP223 OUT/SIG | GPIO9 |
| Cruise control | TTP223 OUT/SIG | GPIO10 |
| Baterie TX, planificat | iesire divizor rezistiv | GPIO11 |

Toate modulele au masa comuna. Display-ul, senzorul Hall, IMU-ul si modulele TTP223 sunt alimentate la 3.3V in prototipul actual.

`GPIO3` nu mai este folosit pentru display. DC a fost mutat pe `GPIO12`, deoarece `GPIO3` este strapping pin pe ESP32-S3 si poate afecta pornirea in anumite montaje.

## Display GC9A01

Configuratia LovyanGFX se afla in:

- `src/transmitter/display_gc9a01.h`
- `src/transmitter/display_gc9a01.cpp`

Setari curente:

- rezolutie: 240x240
- SPI host: `SPI2_HOST`
- SPI mode: 0
- frecventa scriere: 40 MHz
- framebuffer: sprite 16-bit, 240x240
- update UI: la fiecare 80 ms, aproximativ 12.5 FPS

Elementele dashboard-ului:

- sus: steering `-100..100`
- stanga: throttle comandat `0..100%`
- dreapta: `PWR`, momentan aceeasi valoare ca throttle-ul comandat; nu este putere masurata
- jos: procentul bateriei principale primit din MAVLink `SYS_STATUS`
- centru: mod local, armed real din FC, heading, viteza, sateliti si procent baterie TX
- deasupra centrului: link quality
- marginea de jos: tensiunea bateriei TX; valoarea nu este valida pana la cablarea divizorului

Daca nu soseste telemetrie timp de 1 secunda, link-ul este considerat inactiv, LQ devine 0, iar viteza, heading-ul, satelitii si bateria principala sunt afisate ca 0. Statusul `armed` ramane ultima valoare primita si trebuie tratat momentan ca informatie posibil invechita.

## Throttle Hall KY-035 / 49E

Pinout:

| KY-035 / 49E | ESP32-S3 |
| --- | --- |
| VCC/+ | 3V3 |
| GND/- | GND |
| OUT/S | GPIO6 |

Procesarea curenta:

- ADC pe 12 biti, cu atenuare `ADC_11db`
- fiecare citire este media a 16 esantioane
- la boot se calibreaza pozitia eliberata timp de 700 ms
- filtrare exponentiala: 88% valoarea anterioara, 12% citirea noua
- deadband raw: 25
- prag minim pentru invatarea maximului: 80 raw peste repaus
- cursa minima folosita la mapare: 700 raw
- valorile sub pozitia de repaus sunt limitate la 0%

Maparea comenzii motorului:

```text
throttle citit <= 2% -> comanda 0%
throttle citit > 2%  -> comanda mapata in intervalul 15..100%
```

Motorul a inceput fizic sa se roteasca in testele initiale in jurul comenzii de 20%. Valoarea minima software a ramas 15% pentru a pastra o zona de tranzitie.

## Steering MPU-6500/9250/9255

Pinout:

| MPU | ESP32-S3 | Observatie |
| --- | --- | --- |
| VCC | 3V3 | alimentare si logica 3.3V |
| GND | GND | masa comuna |
| SDA | GPIO7 | I2C data |
| SCL | GPIO8 | I2C clock |
| AD0 | GND | adresa `0x68` |
| NCS | 3V3 | selecteaza I2C |
| EDA/ECL | neconectat | magistrala auxiliara nefolosita |
| INT | neconectat | polling, fara intrerupere |
| FSYNC | GND sau neconectat | nefolosit |

Montajul mecanic declarat in cod:

- Y in sus
- Z spre dreapta
- X spre fata telecomenzii

Steering-ul este calculat din accelerometru, nu din viteza unghiulara a giroscopului:

```cpp
atan2f(az, ay)
```

Setari curente:

```cpp
STEERING_INVERT = true
STEERING_MOUNT_OFFSET_DEG = 0.0
STEERING_DEADZONE_DEG = 4.0
STEERING_FULL_SCALE_DEG = 35.0
```

Citirile cu magnitudinea acceleratiei in afara intervalului `0.55g..1.45g` sunt respinse. Dupa mai mult de 5 erori consecutive sau 300 ms fara o citire buna, IMU este marcata invalida si steering-ul revine la 0.

## Butoane TTP223

Ambele module sunt folosite in modul momentary active-high si au debounce software de 60 ms. Toggle-ul se face in firmware, nu in configuratia latch a modulului.

### ARM pe GPIO9

- prima atingere schimba cererea locala in ARM
- urmatoarea atingere schimba cererea in DISARM
- transmitter-ul incrementeaza `armToggleCount`
- receiver-ul detecteaza schimbarea contorului si trimite `MAV_CMD_COMPONENT_ARM_DISARM`
- display-ul arata starea reala primita ulterior din heartbeat-ul FC, nu doar cererea butonului

### Cruise control pe GPIO10

- prima atingere salveaza comanda curenta de throttle si activeaza cruise
- miscarile ulterioare ale triggerului sunt ignorate pentru comanda transmisa
- urmatoarea atingere dezactiveaza cruise
- pe display modul devine `CRZ`; in rest este `MAN`
- cruise nu este dezactivat automat la pierderea legaturii, dar receiver-ul aplica throttle 1000 dupa timeout

## ESP-NOW

Configuratie curenta:

- canal Wi-Fi: 1
- control transmitter -> receiver: broadcast, necriptat
- telemetrie receiver -> transmitter: unicast catre MAC-ul invatat din primul pachet valid
- identificator protocol: `0x53555031` (`SUP1`)
- control: la 40 ms, aproximativ 25 Hz
- telemetrie: la 100 ms, aproximativ 10 Hz

Pachetul de control contine:

- secventa si timpul local TX
- throttle `0..100`
- steering `-100..100`
- contorul apasarilor ARM
- mode; momentan este intotdeauna `0`
- flags: IMU valid, ARM cerut, cruise activ

Pachetul de telemetrie contine:

- armed
- numar sateliti
- procent baterie principala
- procent baterie telecomanda; receiver-ul pune momentan valoarea fixa 100, iar display-ul foloseste citirea locala TX
- link quality
- RSSI; momentan valoarea este 0
- tensiune baterie principala
- ground speed in m/s
- heading in grade

Link quality este calculat cumulativ din pachetele de control primite si discontinuitatile contorului de secventa. Implementarea actuala numara o discontinuitate ca un singur pachet ratat, indiferent de marimea saltului, deci LQ este orientativ si va trebui imbunatatit inainte de testele finale.

## Receiver si MAVLink

Pinout UART:

```text
ESP32 receiver GPIO1 TX -> FC RX
ESP32 receiver GPIO2 RX -> FC TX
ESP32 receiver GND      -> FC GND
baud                    = 115200
```

Receiver-ul trimite:

- heartbeat MAVLink la 1 Hz, identificat ca `MAV_TYPE_GCS`
- `RC_CHANNELS_OVERRIDE` la 25 Hz
- `MAV_CMD_COMPONENT_ARM_DISARM` la fiecare schimbare a contorului ARM

Maparea RC actuala:

| Canal | Valoare |
| --- | --- |
| CH1 | throttle `0..100` mapat la `1000..2000 us` |
| CH2 | steering `-100..100` mapat la `1000..2000 us`, centru 1500 |
| CH3 | 1500 fix |
| CH4 | 1000 fix; ARM nu se face prin acest canal |
| CH5 | 1000 fix; mode nu este implementat |
| CH6-CH8 | 1500 fix |
| CH9-CH18 | 0, ignorate |

Mesaje MAVLink citite:

- `HEARTBEAT`: status armed
- `GPS_RAW_INT`: latitudine, longitudine, viteza GPS si sateliti
- `VFR_HUD`: ground speed si heading; aceste valori suprascriu viteza/heading-ul cand mesajul soseste
- `SYS_STATUS`: tensiune si procent baterie principala

Failsafe receiver:

- timeout control: 500 ms
- dupa timeout, throttle devine 1000 us
- dupa timeout, steering revine la 1500 us
- implementarea curenta nu trimite automat DISARM

## Bateria telecomenzii

Alimentarea prin pad-urile `B+` si `B-` ale placii a functionat dupa reincarcarea celulei Li-ion. In testul cu bateria descarcata, rail-ul de 3.3V cobora in jur de 3.0V si placa nu pornea stabil.

Masurarea tensiunii este pregatita in firmware, dar cablajul nu este finalizat. `GPIO11` neconectat produce valori ADC fara semnificatie, de exemplu aproximativ 0.02V.

Schema planificata:

```text
B+ -> 100k -> GPIO11 -> 100k -> GND
B- ---------------------------> GND comun
```

Cu rezistente egale, factorul divizorului este 2.0. La o baterie de 3.8V, pe GPIO11 trebuie sa existe aproximativ 1.9V. B+ nu se conecteaza direct la ADC.

Pasii ramasi:

1. Montarea divizorului.
2. Compararea valorii afisate cu multimetrul.
3. Ajustarea factorului `BATTERY_DIVIDER_RATIO`.
4. Verificarea tabelului tensiune-procent pentru celula folosita si pentru sarcina reala.

## Serial debug

Transmitter-ul are momentan:

```cpp
SERIAL_DEBUG_ENABLED = false
```

Prin urmare, nu initializeaza USB Serial si nu asteapta un monitor serial la boot. Comenzile `t` pentru recalibrarea repausului Hall si `s` pentru recalibrarea orientarii IMU exista in cod, dar sunt indisponibile pana cand flag-ul este pus pe `true` si firmware-ul este recompilat.

Receiver-ul foloseste Serial Monitor la 115200 baud pentru diagnostic si afiseaza heartbeat-ul si telemetria primita de la FC.

## Backup-uri

- `backup/dashboard_lovyangfx_mockup.cpp`: versiune anterioara a dashboard-ului
- `backup/hall_sensor_test.cpp`: tester dedicat senzorului Hall
- `backup/receiver_espnow_rtt_dashboard.cpp`: test anterior ESP-NOW RTT si dashboard web

## Observatii de siguranta

- Motorul de aproximativ 4.5 kW nu se testeaza cu elicea expusa sau cu placa neasigurata.
- Failsafe-ul software actual trebuie validat independent inainte de teste pe apa.
- O comanda DISARM confirmata de FC trebuie preferata unei simple schimbari locale de UI.
- Reverse-ul nu este implementat si nu trebuie conectat pana la definirea tranzitiilor sigure intre forward, zero si reverse.
- TTP223 trebuie testat prin carcasa finala, cu maini ude si apa pe suprafata, pentru a identifica false touch.
