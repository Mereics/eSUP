# Comunicație și maparea canalelor de control

Acest document descrie traseul complet al comenzilor, de la controalele fizice ale telecomenzii până la ieșirile PWM ale flight controller-ului.

## Separarea nivelurilor

Sistemul are trei niveluri distincte de semnal:

1. Telecomanda citește senzorii și butoanele și construiește valori logice normalizate.
2. ESP32 receiver transformă pachetul ESP-NOW în canale RCIN MAVLink trimise către ArduPilot prin `RC_CHANNELS_OVERRIDE`.
3. ArduPilot interpretează RCIN-urile și le mapează către funcțiile și ieșirile fizice RCOUT/PWM ale flight controller-ului.

ESP32 receiver nu comandă direct pinii PWM ai ESC-ului sau ai servourilor. Toate ieșirile fizice sunt generate de ArduPilot.

## GPIO telecomandă

| GPIO | Componentă | Funcție |
| --- | --- | --- |
| 1 | GC9A01 SCL/SCK | clock SPI display |
| 2 | GC9A01 SDA/MOSI | date SPI display |
| 3 | GC9A01 DC | data/command display |
| 4 | GC9A01 CS | chip select display |
| 5 | GC9A01 RST | reset display |
| 6 | Hall 49E OUT | trigger throttle analogic |
| 7 | MPU SDA | date I2C |
| 8 | MPU SCL | clock I2C |
| 9 | TTP223 | SPEED HOLD / cruise local |
| 10 | TTP223 | ARM/DISARM |
| 11 | TTP223 | REVERSE |
| 12 | TTP223 | COURSE HOLD |
| 13 | rezervat | măsurare baterie TX prin divizor, încă dezactivată |

Toate modulele telecomenzii folosesc masă comună.

## Procesarea controalelor în transmitter

### Throttle

- Triggerul Hall produce `0..100%`.
- Sub deadzone comanda este 0%.
- Peste deadzone comanda este mapată la intervalul motorului `15..100%`.
- În forward, valoarea este pusă în `forwardThrottle`, iar `reverseThrottle` este zero.
- În reverse, valoarea este pusă în `reverseThrottle`, iar `forwardThrottle` este zero.
- Cele două comenzi nu pot fi nenule simultan.

### Steering

- MPU produce o comandă normalizată `-100..100`.
- Valoarea este transmisă separat de throttle și devine RCIN3.

### SPEED HOLD

- Buton pe GPIO9.
- Funcția este implementată exclusiv în telecomandă.
- La activare este memorată comanda curentă de throttle.
- Triggerul este ignorat pentru comanda transmisă până la dezactivare.
- SPEED HOLD nu consumă un canal RCIN dedicat.
- Este anulat la activarea reverse sau când telecomanda intră în failsafe.

### ARM/DISARM

- Buton pe GPIO10.
- ARM necesită long press de 2 secunde.
- DISARM este trimis imediat la apăsare dacă ARM este activ sau cerut.
- Evenimentul este transmis prin `armToggleCount` și flag-ul `ARM_REQUESTED`.
- Receiver-ul trimite către FC comanda MAVLink `MAV_CMD_COMPONENT_ARM_DISARM`.
- ARM nu este mapat pe un RCIN PWM.

### REVERSE

- Buton toggle pe GPIO11.
- Schimbarea sensului este acceptată numai după minimum o secundă continuă cu triggerul la 0%.
- Activarea reverse anulează SPEED HOLD.
- În reverse, RCIN1 primește comanda triggerului și RCIN2 rămâne la minim.
- În forward, RCIN2 primește comanda triggerului și RCIN1 rămâne la minim.
- Receiver-ul verifică suplimentar pachetul; dacă forward și reverse sunt simultan nenule, ambele sunt forțate la zero.

### COURSE HOLD

- Buton toggle pe GPIO12.
- Telecomanda transmite starea prin flag-ul `COURSE_HOLD_ACTIVE`.
- Cât timp COURSE HOLD este activ, transmitter-ul trimite steering neutru pe RCIN3; controlul direcției revine modului ArduPilot.
- Receiver-ul generează RCIN4=1000 us pentru OFF și RCIN4=2000 us pentru ON.
- Selectarea și comportamentul modului de course hold sunt configurate în ArduPilot.
- ESP32 nu generează direct ieșirea servourilor și nu implementează în această etapă controller-ul de navigație.

## Pachetul ESP-NOW de control

Pachetul `RemoteControlPacket` este packed și conține:

| Câmp | Tip | Rol |
| --- | --- | --- |
| `magic` | `uint32_t` | identificator protocol `SUP1` |
| `type` | `uint8_t` | tip pachet control |
| `seq` | `uint16_t` | detectarea pachetelor pierdute |
| `txMillis` | `uint32_t` | timpul local al transmitter-ului |
| `forwardThrottle` | `uint8_t` | `0..100`, destinat RCIN2 |
| `reverseThrottle` | `uint8_t` | `0..100`, destinat RCIN1 |
| `steering` | `int8_t` | `-100..100`, destinat RCIN3 |
| `armToggleCount` | `uint8_t` | contor evenimente ARM/DISARM |
| `mode` | `uint8_t` | rezervat |
| `flags` | `uint8_t` | stări binare |

Flag-uri:

- bit 0: IMU valid;
- bit 1: ARM cerut;
- bit 2: SPEED HOLD activ;
- bit 3: COURSE HOLD activ;
- bit 4: REVERSE activ.

Pachetul de control este trimis la aproximativ 25 Hz, o dată la 40 ms.

## Maparea RCIN în receiver

Receiver-ul trimite `RC_CHANNELS_OVERRIDE` la aproximativ 25 Hz:

| RCIN ArduPilot | Semnal | Domeniu MAVLink |
| --- | --- | --- |
| RCIN1 | reverse throttle | 1000..2000 us |
| RCIN2 | forward throttle | 1000..2000 us |
| RCIN3 | steering | 1000..2000 us, centru 1500 us |
| RCIN4 | course hold | 1000 us OFF, 2000 us ON |
| RCIN5 | nefolosit | 1000 us |
| RCIN6-RCIN8 | nefolosite | 1500 us |
| RCIN9-RCIN18 | ignorate | 0 |

## Maparea RCOUT în ArduPilot

Maparea dorită a ieșirilor fizice este:

| RCOUT/PWM FC | Consumator |
| --- | --- |
| PWM1 | intrarea reverse a ESC-ului |
| PWM2 | intrarea forward a ESC-ului |
| PWM3 | servomecanism steering 1 |
| PWM4 | servomecanism steering 2 |

ArduPilot trebuie configurat astfel încât:

- RCIN1 să controleze funcția reverse de pe PWM1;
- RCIN2 să controleze forward throttle de pe PWM2;
- RCIN3 să controleze sincron PWM3 și PWM4;
- RCIN4 să selecteze funcția/modul COURSE HOLD;
- cele două servouri să aibă trim, limite și sens verificate mecanic;
- failsafe-ul ArduPilot să aducă motorul la zero, servourile la centru și sistemul în DISARM.

Parametrii ArduPilot exacți se configurează și se validează în Mission Planner după verificarea mișcărilor fără elice.

## Failsafe de comunicație

- Receiver-ul consideră link-ul pierdut după 500 ms fără pachete de control valide.
- RCIN1 și RCIN2 sunt aduse la 1000 us.
- RCIN3 este adus la 1500 us.
- RCIN4 este adus la 1000 us.
- Receiver-ul trimite o comandă MAVLink DISARM.
- La reconectare este sincronizat contorul ARM fără repetarea vechii comenzi.
- Pentru rearmare este necesar un nou long press pe butonul ARM.

## Telemetrie și radio

- Telemetria este trimisă receiver -> transmitter la aproximativ 10 Hz.
- Controlul și telemetria folosesc ESP-NOW pe canalul Wi-Fi 1.
- Ambele ESP32 solicită puterea radio maximă de 19,5 dBm prin API-ul ESP-IDF.
- LQ este indicatorul principal al calității legăturii; RSSI nu este folosit.

## Funcții rămase

- Cablarea și activarea măsurării bateriei TX pe GPIO13.
- Adăugarea divizorului bateriei principale pe un ADC al receiver-ului.
- Configurarea și validarea parametrilor RCIN/RCOUT în ArduPilot.
- Testarea COURSE HOLD în ArduPilot.
- Testarea controlată a reverse-ului și confirmarea plajei PWM a ESC-ului.
- Upgrade ulterior la vESC pentru curent, tensiune și limitare configurabilă.
