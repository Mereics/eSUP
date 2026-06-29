# ESP32-S3 Super Mini + GC9A01 + Hall + MPU

Memoria proiectului / context complet sistem: [docs/PROJECT_CONTEXT.md](docs/PROJECT_CONTEXT.md)

## Structura Firmware

Proiectul are doua firmware-uri PlatformIO separate:

```text
src/transmitter/   codul telecomenzii cu display, Hall, MPU si ARM touch
src/receiver/      codul ESP32 receiver pentru MAVLink/UART cu FC-ul
```

Upload transmitter:

```powershell
pio run -e transmitter -t upload
pio device monitor -e transmitter
```

Upload receiver:

```powershell
pio run -e receiver -t upload
pio device monitor -e receiver
```

Environment-ul default este `transmitter`, deci `pio run -t upload` va urca firmware-ul telecomenzii.

## Transmitter

Firmware-ul transmitter afiseaza dashboard-ul normal LovyanGFX si citeste live:

- throttle din senzorul Hall KY-035 / 49E pe `GPIO6`
- steering din MPU-9250/6500/9255 pe axa Y, prin I2C
- ARM touch pe `GPIO9`
- cruise control touch pe `GPIO10`
- tensiunea bateriei telecomenzii pe `GPIO11`, prin divizor rezistiv
- telemetrie primita de la receiver prin ESP-NOW

Transmitter trimite prin ESP-NOW, la aproximativ 25 Hz:

- throttle `0..100`
- steering `-100..100`
- arm toggle counter
- mode placeholder

Transmitter primeste de la receiver:

- armed real citit din FC prin MAVLink
- ground speed
- heading
- sateliti
- baterie motor/main battery
- link quality si RSSI

## Hall KY-035 / 49E

Pinout:

| 49E / KY-035 | ESP32-S3 Super Mini |
| --- | --- |
| VCC / + | 3V3 |
| GND / - | GND |
| OUT / S | GPIO6 |

Modulul KY-035 / 49E merge alimentat la 3.3V. Fara camp magnetic puternic, output-ul ar trebui sa fie aproximativ la jumatatea alimentarii, deci in jur de 1.65V.

In codul curent, throttle-ul nu poate scadea sub `0%`. Orice valoare negativa fata de pozitia de repaus este mapata la `0%`, pentru ca spre FC nu trimitem throttle negativ.

Stabilizare throttle:

- la boot se face media senzorului Hall timp de aproximativ 700 ms
- exista deadband raw pentru zgomot in jurul repausului
- max-ul de trigger nu mai este invatat din zgomot mic, ci doar dupa un prag minim
- pana la invatarea unei curse reale, se foloseste un span minim raw ca sa nu sara direct la 100%

Comanda Serial Monitor:

```text
t = recalibreaza throttle rest la valoarea curenta
```

## MPU-9250 / MPU-6500 / MPU-9255

Pinout recomandat, avand deja `GPIO1-6` ocupate:

| MPU pin | ESP32-S3 Super Mini | Observatii |
| --- | --- | --- |
| VCC | 3V3 | Logica sigura pentru ESP32 |
| GND | GND | Masa comuna |
| SCL | GPIO8 | I2C clock |
| SDA | GPIO7 | I2C data |
| EDA | neconectat | I2C auxiliar, nefolosit |
| ECL | neconectat | I2C auxiliar, nefolosit |
| AD0 | GND | Adresa MPU `0x68` |
| INT | neconectat momentan | Optional, poate merge pe GPIO9 mai tarziu |
| NCS | 3V3 | Tine modulul in I2C, nu SPI |
| FSYNC | GND sau neconectat | Neutilizat |

In cod folosim roll pentru steering. Pentru montajul curent:

- `Y` este in sus
- `Z` este spre dreapta
- `X` este spre fata telecomenzii

Roll-ul stanga/dreapta este calculat din `Y/Z`:

```cpp
atan2f(az, ay)
```

Daca axa este buna dar sensul este invers, schimba:

```cpp
constexpr bool STEERING_INVERT = true;
```

in:

```cpp
constexpr bool STEERING_INVERT = false;
```

Setari initiale steering:

```cpp
STEERING_MOUNT_OFFSET_DEG = 90.0
STEERING_DEADZONE_DEG = 4.0
STEERING_FULL_SCALE_DEG = 35.0
```

Serial Monitor afiseaza si `ax`, `ay`, `az`, ca sa putem verifica orientarea reala a modulului daca este nevoie.

Stabilizare IMU:

- I2C timeout este setat la 50 ms
- citirile cu magnitudine accelerometru nerealista sunt respinse
- dupa mai multe erori consecutive, steering-ul revine la 0 in loc sa ramana blocat full stanga/dreapta

## Buton ARM TTP223

Pinout:

| TTP223 | ESP32-S3 Super Mini |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| I/O / OUT / SIG | GPIO9 |

Modulul TTP223 merge la 2.5-5.5V, deci il alimentam la 3.3V. In modul default este de obicei momentary active-high: `LOW` fara atingere, `HIGH` cand este atins.

Pentru ARM folosim modul momentary si facem toggle in firmware:

- prima atingere: `ARMED`
- urmatoarea atingere: `DISARMED`

Nu seta modulul in latching/toggle hardware pentru acest test, altfel vom avea toggle in modul si toggle in cod in acelasi timp.

## Buton Cruise TTP223

Pinout:

| TTP223 | ESP32-S3 Super Mini |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| I/O / OUT / SIG | GPIO10 |

Comportament:

- prima atingere activeaza cruise control
- throttle-ul comandat ramane blocat la valoarea curenta
- urmatoarea atingere dezactiveaza cruise control
- cand cruise este activ, `MODE` pe display devine `CRZ`

## Mapare Throttle Motor

Motorul incepe fizic sa se invarta de la aproximativ `20%`, dar folosim comanda minima de `15%` ca sa existe putin deadzone mecanic/electric. Transmitter-ul trimite catre receiver throttle comandat, nu raw trigger:

```text
trigger <= 2%  -> comanda 0%
trigger > 2%   -> comanda mapata 15..100%
```

Constante relevante in cod:

```cpp
THROTTLE_INPUT_DEADBAND_PERCENT = 2
MOTOR_START_PERCENT = 15
```

## Citire Baterie Telecomanda

ESP32-ul nu poate citi direct tensiunea de pe o celula Li-ion, pentru ca bateria plina are aproximativ `4.2V`, iar ADC-ul ESP32 trebuie tinut sub `3.3V`.

Pinout recomandat cu divizor rezistiv:

```text
B+ baterie -> 100k -> GPIO11 -> 100k -> GND
B- baterie -> GND comun
```

Cu doua rezistente egale, ESP32 vede jumatate din tensiunea bateriei. In cod este setat:

```cpp
BATTERY_DIVIDER_RATIO = 2.0
```

Display-ul afiseaza bateria telecomenzii ca `TX %` in centru si `TX x.xxV` jos. Daca folosesti alte valori pentru rezistente, schimba `BATTERY_DIVIDER_RATIO` cu factorul real al divizorului.

Exemple:

- `100k / 100k`: factor `2.0`, consum permanent aproximativ `21 uA` la baterie plina
- `220k / 220k`: factor `2.0`, consum permanent mai mic, dar citirea ADC poate deveni putin mai zgomotoasa

Comanda Serial Monitor:

```text
s = recalibreaza steering rest la orientarea curenta, suprascriind offset-ul fix
```

## Display GC9A01

Pinout:

| GC9A01 display | ESP32-S3 Super Mini |
| --- | --- |
| VCC / VIN | 3V3 |
| GND | GND |
| RST / RES | GPIO5 |
| CS | GPIO4 |
| DC | GPIO12 |
| SDA / MOSI | GPIO2 |
| SCL / SCK | GPIO1 |

Display-ul foloseste LovyanGFX cu hardware SPI remapat la pinii de mai sus. UI-ul este randat intr-un sprite 240x240 pe 16-bit si apoi trimis ca full frame pentru update smooth.

Codul de configurare display este separat in:

- `src/display_gc9a01.h`
- `src/display_gc9a01.cpp`

## Backup-uri

Backup dashboard LovyanGFX anterior:

- `backup/dashboard_lovyangfx_mockup.cpp`

Backup tester Hall:

- `backup/hall_sensor_test.cpp`

## Receiver

Codul receiver curent este sketch-ul MAVLink care comunica prin UART cu flight controller-ul:

- `src/receiver/main.cpp`

Backup-ul vechiului test ESP-NOW RTT + dashboard web este in:

- `backup/receiver_espnow_rtt_dashboard.cpp`

Pinout curent receiver -> FC din cod:

```text
ESP32 GPIO1 TX -> FC RX
ESP32 GPIO2 RX -> FC TX
GND comun
baud 115200
```

Codul trimite heartbeat MAVLink, `RC_CHANNELS_OVERRIDE` si citeste telemetrie de la FC.

Receiver-ul primeste control prin ESP-NOW de la transmitter si:

- mapeaza throttle `0..100` la PWM `1000..2000`
- mapeaza steering `-100..100` la PWM `1000..2000`, centru `1500`
- trimite `MAV_CMD_COMPONENT_ARM_DISARM` la fiecare apasare ARM primita
- aplica failsafe local: daca nu primeste control >500 ms, throttle devine `1000`, steering `1500`
- trimite inapoi telemetrie catre transmitter la aproximativ 10 Hz
