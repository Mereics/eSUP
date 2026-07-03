# Context Proiect - Motor Electric Pentru Standup Paddle

Acest document este memoria de lucru a proiectului. Îl actualizăm de fiecare dată când se schimbă hardware-ul, cablajul, deciziile de protocol, testele sau statusul.

Maparea tehnică GPIO -> ESP-NOW -> RCIN -> RCOUT este documentată separat în `docs/COMMUNICATION_AND_CHANNEL_MAPPING.md`.

## Scop

Construim un sistem electric de propulsie și control pentru o placă de standup paddle. Obiectivul nu este doar demonstrarea unui prototip de laborator: sistemul final trebuie să fie complet funcțional și utilizabil în condiții reale pe apă.

Funcțiile obligatorii ale sistemului final sunt:

- Control throttle prin trigger cu senzor Hall.
- Steering exclusiv prin înclinarea telecomenzii; nu este prevăzut steering manual prin butoane stânga/dreapta.
- Cruise control care menține comanda de throttle memorată.
- Heading hold bazat pe datele de navigație ale flight controller-ului.
- Control reverse pentru ESC.
- Comunicație bidirecțională între telecomandă și receptor.
- Telemetrie pe display: număr de sateliți, viteză, link quality, tensiune receiver, tensiune transmitter, steering, throttle și heading.
- Controlul motorului BLDC și al ductului steerable prin ArduPilot pe flight controller-ul Skystars H743 HD.
- Telecomandă waterproof, ergonomică și utilizabilă cu o singură mână.

Nu au fost stabilite momentan funcții suplimentare explicit în afara scopului.

## Concept Mecanic

- Elicea este montată într-un ansamblu ducted format din două părți.
- Ductul principal este fix. Acesta are inițial o secțiune constantă, apoi diametrul se reduce ușor spre capăt.
- După ductul fix este montat un duct scurt și mobil, folosit ca duză de steering.
- Ductul mobil este articulat pe doi rulmenți aflați pe axa sa centrală: unul sus și unul jos.
- Cursa mecanică dorită este de aproximativ 30 de grade spre stânga și 30 de grade spre dreapta față de poziția centrală.
- Ductul mobil este acționat simultan de două servomecanisme Feetech `FT5325M`, montate pe părți opuse.
- Fiecare servo are un cuplu nominal declarat de 25 kg-cm.
- Cele două servomecanisme sunt legate de ductul mobil prin pushrod-uri și lucrează împreună pentru a produce aceeași mișcare de steering.
- Steering-ul este comandat prin înclinarea telecomenzii; nu sunt prevăzute butoane pentru steering manual stânga/dreapta.
- La pierderea legăturii cu telecomanda, ArduPilot trebuie configurat să oprească imediat motorul și să comande ambele servomecanisme în poziția centrală, astfel încât SUP-ul să continue drept.
- Centrarea de failsafe este comandată electronic prin flight controller; mecanismul nu are momentan o revenire mecanică pasivă la centru.

## Propulsie

- Motor: Maytech waterproof 6374 outrunner, varianta 80 KV pentru foil board/SUP.
- Pagina motorului: `https://maytech.cn/products/maytech-waterproof-6374-motor-50kv-for-electric-foil-board-sup?variant=43059108708457`.
- Domeniu tensiune motor: 6S-10S LiPo.
- Putere maximă declarată: 4,3 kW.
- Curent nominal declarat: 60 A.
- Curent maxim declarat: 102 A la 42 V.
- Grad de protecție motor: IPX8.
- La pachetul 8S LiFePO4, turația teoretică fără sarcină este aproximativ 2.048 RPM la 25,6 V nominal și 2.336 RPM la 29,2 V complet încărcat.
- ESC: FMS Predator 120A, cod produs `PRESC037`.
- Pagina ESC-ului: `https://www.fmshobby.com/products/predator-esc-120a`.
- Domeniu tensiune ESC: 3S-8S.
- Curent nominal/maxim declarat prin denumirea produsului: 120 A.
- ESC-ul are două fire separate de comandă PWM:
  - un fir pentru forward;
  - un fir pentru reverse.
- Intervalul declarat pentru comanda forward este `1000..2000 us`.
- Intervalul exact pentru reverse și comportamentul ESC-ului când ambele intrări sunt active simultan nu sunt încă verificate.
- Până la verificarea hardware, firmware-ul trebuie să garanteze că forward și reverse nu pot fi comandate simultan.

Acumulator principal:

- Două baterii LiFePO4 de 12 V / 50 Ah conectate în serie.
- Pagina bateriei: `https://www.dbsolar.ro/produs/baterie-lithium-lifepo4-acumulator-50ah/`.
- Fișa tehnică: `https://www.dbsolar.ro/wp-content/uploads/2023/08/12.8V-50AH-LiFePO4-battery-specification_ro.pdf`.
- Configurația rezultată este 8S LiFePO4, aproximativ 25,6 V nominal și 29,2 V încărcată complet, la o capacitate de 50 Ah.
- Energia nominală estimată a ansamblului este aproximativ 1,28 kWh.
- Fiecare baterie are BMS propriu, display fizic și monitorizare prin aplicație mobilă.
- Producătorul declară că bateriile pot fi conectate în serie sau paralel, până la patru unități.
- Tensiune maximă de încărcare pentru fiecare baterie: `14,6 +/- 0,1 V`; pentru cele două baterii în serie, tensiunea maximă totală este aproximativ 29,2 V.
- Curent maxim de încărcare pentru fiecare baterie: 22,5 A.
- Tensiune de deconectare pentru fiecare baterie: 10,4 V.
- Curent maxim continuu de descărcare pentru fiecare baterie/BMS: 90 A.
- Prag protecție la supracurent: minimum 90 A, maximum 112,5 A, cu întârziere declarată de 130 ms.
- Grad de protecție baterie: IP55.
- În conexiune serie, tensiunile se adună, dar capacitatea rămâne 50 Ah și același curent trece prin ambele BMS-uri; limita ansamblului nu devine 180 A, ci rămâne 90 A continuu.
- Obiectivul versiunii actuale este limitarea aproximativă a sistemului la maximum 80 A, păstrând marjă față de limita continuă de 90 A a BMS-urilor.
- La 80 A, puterea electrică teoretică este aproximativ 2,05 kW la tensiunea nominală și 2,34 kW la tensiunea maximă.
- Limita de 4,3 kW a motorului nu va fi utilizată în această configurație.

Monitorizare baterie principală:

- Curentul poate fi urmărit local pe display-urile bateriilor și în aplicația lor, dar nu este transmis momentan către flight controller sau ESP32.
- Se vor face teste controlate pentru a observa relația dintre throttle și curent și pentru a stabili o limită inițială de throttle corespunzătoare unui consum de cel mult aproximativ 80 A.
- O limită fixă de throttle nu garantează însă aceeași limită de curent în toate condițiile, deoarece curentul depinde de sarcina elicei, viteza SUP-ului, tensiune și eventuale blocaje.
- În versiunea actuală nu va fi adăugat un senzor de curent dedicat.
- Este planificat un upgrade ulterior la un vESC care poate măsura direct tensiunea și curentul și oferă parametri suplimentari pentru configurarea, protecția și limitarea motorului.
- Tensiunea va fi măsurată printr-un divizor rezistiv conectat la alimentarea ESC-ului și citit de un ADC al ESP32 receiver.
- Pinul ADC și valorile divizorului de pe receiver vor fi stabilite după verificarea pinout-ului fizic disponibil.
- Skystars H743 HD este specificat pentru `VBAT 7..25 V` / 2S-6S LiPo.
- Pachetul 8S LiFePO4 poate ajunge la aproximativ 29,2 V și nu trebuie conectat direct la intrarea VBAT a flight controller-ului.
- Flight controller-ul trebuie alimentat printr-un regulator/BEC separat, compatibil cu tensiunea maximă a pachetului.
- Pentru citirea tensiunii este necesar un divizor extern dimensionat pentru 29,2 V plus marjă pentru supratensiuni; ieșirea divizorului trebuie să rămână în domeniul ADC-ului ales.
- Procentul bateriei afișat pe telecomandă va fi estimat din tensiune, deoarece nu este disponibilă măsurarea curentului/coulomb counting prin sistemul de control.
- Din cauza platoului foarte plat al chimiei LiFePO4, procentul calculat numai din tensiune este orientativ și nu trebuie interpretat ca SOC precis.
- Tensiunea instantanee va fi afișată permanent.
- Pentru reducerea erorii produse de voltage sag, procentul va fi actualizat preferabil după ce throttle-ul rămâne la zero sau foarte jos pentru o perioadă scurtă; în sarcină mare se păstrează ultima estimare stabilă.
- Mapare inițială propusă pentru pachetul 8S, bazată pe tensiunea de repaus:
  - `>=27,2 V`: 100%;
  - `26,8 V`: 90%;
  - `26,6 V`: 80%;
  - `26,4 V`: 70%;
  - `26,2 V`: 60%;
  - `26,1 V`: 50%;
  - `26,0 V`: 40%;
  - `25,8 V`: 30%;
  - `25,6 V`: 20%;
  - `24,0 V`: 10%;
  - `20,8 V`: 0%, corespunzător tensiunii de deconectare declarate pentru cele două baterii în serie.
- Între punctele tabelului se va folosi interpolare liniară.
- Avertizare baterie principală: culoare portocalie la `<=25,6 V` / aproximativ 20% estimat.
- Avertizare critică: text roșu intermitent la `<=24,0 V` / aproximativ 10% estimat.
- Pragurile și tabelul vor fi recalibrate după teste reale, comparând tensiunea măsurată cu SOC-ul raportat de BMS-urile bateriilor.

## Flight Controller

- Placă: Skystars H743 HD.
- Firmware: ArduRover `4.6.3`.
- Flight controller-ul este alimentat la 12 V de la un power distribution board (PDB), nu direct de la pachetul 8S LiFePO4.
- Mapare ieșiri PWM:
  - PWM1: comanda reverse a ESC-ului;
  - PWM2: comanda forward/throttle a ESC-ului;
  - PWM3: servomecanism steering 1;
  - PWM4: servomecanism steering 2.
- Cele două servomecanisme sunt montate în oglindă, iar geometria mecanică permite folosirea aceleiași comenzi; nu este necesară inversarea software a unuia dintre semnale.
- ESP32 receiver este conectat la UART1 al flight controller-ului și comunică prin MAVLink la 115200 baud, conform configurației firmware actuale.
- Maparea MAVLink/RC dorită pentru versiunea finală este RC1 reverse, RC2 forward throttle și RC3 steering.
- Responsabilități FC:
  - generarea semnalelor PWM pentru forward și reverse;
  - comanda sincronizată a celor două servomecanisme;
  - integrarea GPS și calculul datelor de navigație;
  - implementarea heading hold;
  - aplicarea failsafe-ului la pierderea telecomenzii;
  - furnizarea telemetriei MAVLink către ESP32 receiver.

## GPS și Navigație

- Modul GPS: Holybro M9N bazat pe receptorul u-blox M9N.
- Modulul este deja conectat la UART4 al flight controller-ului.
- Holybro M9N include și magnetometru `IST8310` sau `IST8308`.
- Liniile SDA/SCL ale modulului sunt conectate la magistrala I2C a flight controller-ului, deci ArduPilot poate folosi compasul extern.
- Date folosite și afișate:
  - poziție GPS;
  - ground speed;
  - număr de sateliți;
  - direcție/course over ground;
  - heading/yaw estimat de ArduPilot.

Obiectiv navigație:

- Utilizatorul activează funcția din telecomandă în timp ce SUP-ul se deplasează în direcția dorită.
- La activare se memorează direcția curentă ca direcție țintă.
- Activarea este permisă doar cu poziție GPS validă și o viteză peste sol de minimum aproximativ 1 m/s, pentru ca direcția GPS memorată să fie relevantă.
- Sistemul trebuie să compenseze nu doar rotația SUP-ului, ci și deriva laterală produsă de vânt, valuri și curentul apei.
- Un simplu heading hold menține orientarea provei, dar nu garantează menținerea aceleiași linii peste sol.
- Funcția necesară este în realitate `course/track hold`: menținerea unei linii GPS, cu corecție de cross-track error.
- Soluția planificată este folosirea navigației ArduRover în `Guided`: la activare se salvează poziția și direcția curentă, apoi se generează un punct virtual aflat înainte pe direcția țintă.
- Controller-ul de poziție ArduRover va comanda steering-ul pentru a reveni pe linia dorită dacă SUP-ul este deplasat lateral.
- Ținta virtuală va trebui extinsă sau regenerată înainte să fie atinsă, pentru a permite deplasarea continuă pe aceeași direcție.
- În UI funcția poate rămâne denumită `heading hold`, dar intern trebuie tratată ca menținere de course/track.
- Compass-ul/estimarea AHRS este folosită pentru orientarea instantanee, mai ales la viteză mică, iar GPS-ul este folosit pentru poziție, viteză și corecția derivei peste sol.
- Cât timp funcția este activă, comanda de steering provenită din înclinarea telecomenzii este ignorată complet.
- A doua apăsare a butonului dezactivează funcția și revine imediat la steering prin înclinarea telecomenzii.

## Arhitectură Electronică

### Telecomandă / Transmitter

Hardware:

- ESP32-S3 Super Mini.
- Display rotund GC9A01, 240x240.
- Trigger throttle cu senzor Hall analogic.
- Magnet montat în mecanismul triggerului.
- IMU / giroscop / accelerometru pentru înclinarea telecomenzii.
- Butoane capacitive touch, deoarece telecomanda trebuie să fie waterproof.
- Baterie telecomandă: o singură celulă Li-ion 1S, 2.600 mAh.
- Modul TP4056 folosit în configurația actuală numai pentru protecția bateriei, fără folosirea funcției sale de încărcare.
- Ieșirea modulului TP4056 alimentează pad-urile `B+`/`B-` ale ESP32-S3 Super Mini.
- Conector MT30 cu trei pini încastrat în mâner:
  - minus baterie;
  - plus direct baterie;
  - plus către circuitul TP4056/telecomandă.
- Mufă de funcționare: realizează puntea dintre cele două contacte pozitive și pornește telecomanda.
- Mufă separată de încărcare: folosește doar plusul și minusul direct al bateriei pentru conectarea unui încărcător extern 1S Li-ion.
- Bateria este conectată la intrările de baterie ale modulului, iar telecomanda este alimentată exclusiv din ieșirea protejată.
- Minusul comun al componentelor telecomenzii pleacă din `OUT-`; nu există punte directă între minusul bateriei `B-` și minusul protejat `OUT-`.
- Încărcarea se face separat, direct prin contactele bateriei din MT30, folosind un încărcător extern pentru acumulatori FPV configurat pentru Li-ion 1S / 4,2 V.

Responsabilități telecomandă:

- Citește triggerul de throttle.
- Citește butoanele touch.
- Citește înclinarea din IMU.
- Selectează steering prin înclinare, cruise și course/track hold.
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
| DC | GPIO3 |
| SDA / MOSI | GPIO2 |
| SCL / SCK | GPIO1 |

- Pinout-ul display-ului a fost verificat fizic și este aplicat în firmware.

Software display actual:

- LovyanGFX.
- Hardware SPI.
- Sprite framebuffer 240x240 pe 16-bit.
- Full-frame push către display pentru update smooth.

UI display actual:

- Gauge curbat sus: steering / turn indicator.
- Gauge curbat stânga: throttle.
- Gauge curbat dreapta: power, duplică momentan comanda de throttle și va fi eliminat.
- Gauge curbat jos: procent baterie motor/main battery.
- Centru:
  - Mode: manual/sport.
  - Status armed/disarmed.
  - Heading.
  - Speed.
  - Număr sateliți.
  - Nivel baterie telecomandă.

UI display planificat:

- Gauge-ul din dreapta pentru power va fi eliminat, deoarece afișează aceeași informație ca throttle-ul din stânga.
- Gauge-ul din dreapta va afișa bateria principală de propulsie, atât procentul estimat, cât și tensiunea măsurată.
- În partea de jos va fi adăugat indicatorul de heading.
- Când heading hold este activ, centrul indicatorului de jos reprezintă heading-ul țintă memorat.
- În stânga și dreapta centrului va fi afișată deviația course-ului curent față de course-ul țintă, într-un stil similar indicatorului de steering de sus.
- Domeniul complet al indicatorului inferior va fi `-45..+45 grade`; centrul înseamnă deviație zero.
- Informațiile permanente din centru vor fi:
  - viteza;
  - numărul de sateliți;
  - link quality;
  - tensiunea bateriei telecomenzii;
  - starea ARM/DISARM;
  - MODE.
- Stările reverse, cruise și course/track hold vor fi indicate prin text în zona `MODE`.
- Dacă cruise și course/track hold sunt active simultan, MODE va afișa `CRZ+HLD`.
- Când course/track hold este oprit, indicatorul inferior va afișa heading-ul absolut curent.
- Pentru baterie scăzută și link quality sub prag, UI-ul va schimba culoarea și va afișa o avertizare intermitentă.
- Avertizarea intermitentă afectează doar textul/valoarea relevantă, nu întregul display.
- Prag avertizare baterie telecomandă: 3,7 V.
- Prag avertizare link quality: sub 90%.
- Dashboard-ul final de bază este implementat: PWR a fost eliminat, bateria principală este în dreapta, heading-ul este jos, iar centrul conține viteză, sateliți, LQ, baterie TX, ARM și MODE.
- Afișarea deviației course/track hold rămâne inactivă până la implementarea butonului și logicii Guided.

### Receptor

Hardware:

- ESP32-S3 Super Mini.
- Comunică cu flight controller-ul prin MAVLink.
- Comunică bidirecțional cu telecomanda prin ESP-NOW.

Responsabilități receptor:

- Primește pachete de control de la telecomandă.
- Trimite comenzi RC/control către ArduPilot / flight controller.
- Citește telemetrie MAVLink de la flight controller.
- Trimite pachete de telemetrie înapoi către telecomandă.

### Distribuție alimentare pe SUP

- PDB actual: Matek Mini Power Hub / model magazin `T1016.MATEKPDB`, cu BEC de 5 V și 12 V.
- Pagina produsului: `https://electronicmarket.ro/matek-pdb-placa-de-distributie-cu-bec-5v-12v`.
- Specificații declarate PDB:
  - intrare 9..26 V / 3S-6S;
  - ieșire 5 V, 3 A continuu;
  - ieșire 12 V, 2 A continuu și maximum 3 A timp de 10 secunde pe minut;
  - ieșiri ESC maximum 20 A fiecare.
- Sunt montate două convertoare step-down generice identice, cu intrare declarată `7..32 V`, ieșire reglabilă `0,8..28 V` și curent maxim declarat de 9 A.
- Primul step-down este reglat la 12 V și alimentează PDB-ul.
- Al doilea step-down este dedicat celor două servomecanisme și este reglat la 7,4 V.
- Pachetul complet încărcat ajunge la 29,2 V, lăsând doar 2,8 V marjă până la limita de intrare de 32 V a convertoarelor; comportamentul la tranzienți și temperatura sub sarcină trebuie verificat.
- Flight controller-ul este alimentat la 12 V prin această ramură de alimentare/PDB.
- ESP32 receiver este alimentat din pinul de 5 V al flight controller-ului.
- Ieșirile ESC de 20 A ale PDB-ului nu sunt folosite pentru alimentarea motorului.
- ESC-ul este alimentat direct din bateria principală printr-un întrerupător general montat pe exteriorul cutiei waterproof de electronică.
- Întrerupătorul general are un rating declarat de 350 A.
- Nu este documentată momentan o siguranță principală separată.
- Nu există momentan circuit anti-spark, rezistență de pre-charge sau contactor.
- La conectarea bateriei, întrerupătorul este deschis; alimentarea ESC-ului începe când întrerupătorul este închis.
- Condensatorii de filtrare integrați în ESC produc un curent de inrush la încărcare și nu înlocuiesc un circuit anti-spark/pre-charge.
- Rating-ul de 350 A al întrerupătorului trebuie verificat și pentru tensiune DC, capacitate de comutare și curent de inrush repetat, nu doar pentru curentul continuu.
- Cele două servomecanisme sunt alimentate la 7,4 V de un step-down separat.
- Fiecare servo poate consuma până la aproximativ 3,5 A peak; convertorul trebuie să suporte cel puțin 7 A peak cumulat, plus marjă.

### Protecție la apă

- Motorul și cele două servomecanisme lucrează submersate.
- Servourile sunt declarate waterproof și au fost desfăcute și umplute suplimentar cu silicon pentru etanșare.
- Restul electronicii de pe SUP este montat într-o cutie waterproof aflată pe placă.
- Electronica telecomenzii este acoperită cu conformal coating.
- La asamblarea finală, îmbinările carcasei telecomenzii sunt sigilate cu silicon.
- Conformal coating-ul este tratat ca protecție secundară; etanșarea carcasei rămâne bariera principală împotriva apei.

## Link Wireless

- Protocol: ESP-NOW bidirecțional între ESP32-S3 transmitter și ESP32-S3 receiver.
- Canal Wi-Fi: 1.
- Rată control transmitter -> receiver: aproximativ 25 Hz / un pachet la 40 ms.
- Rată telemetrie receiver -> transmitter: aproximativ 10 Hz / un pachet la 100 ms.
- Telecomanda va fi asociată permanent cu un singur receiver.
- Implementarea actuală folosește broadcast pentru control și receiver-ul învață adresa MAC a primului transmitter care trimite un pachet valid.
- Pentru configurația finală trebuie salvate/introduse explicit adresele MAC ale perechii, astfel încât receiver-ul să nu accepte accidental alt transmitter cu același format de protocol.
- Criptarea ESP-NOW nu va fi activată momentan. Decizia poate fi reevaluată după testele de latență și securitate.
- Raza necesară nu este stabilită numeric; va fi măsurată prin teste reale, inclusiv deasupra apei.

Date transmise de la telecomandă la receptor:

- Valoare trigger throttle.
- Înclinare IMU / comandă steering.
- Comandă arm/disarm.
- Comandă course/track hold.
- Stare cruise control.
- Număr de secvență și timp local pentru detectarea pierderilor și failsafe.

Date transmise de la receptor la telecomandă:

- Tensiune/procent baterie motor/main battery.
- Ground speed.
- Număr sateliți.
- Heading.
- Status armed/disarmed.
- Mod curent.
- Status link/failsafe.
- Confirmări pentru comenzi, dacă este nevoie.

Rată și comportament telemetrie:

- Telemetria receiver -> transmitter rămâne la 10 Hz / un pachet la 100 ms.
- Pachetul actual are aproximativ 29 bytes payload, astfel că rata este suficientă pentru actualizare vizuală fluidă fără consum semnificativ de bandă ESP-NOW.
- Dacă transmitter-ul nu primește telemetrie timp de 1 secundă, valorile provenite de la receiver sunt afișate ca zero: LQ, baterie principală, viteză, heading, sateliți și armed.
- Tensiunea bateriei telecomenzii rămâne disponibilă deoarece este măsurată local pe transmitter.
- Firmware-ul transmitter trece toate valorile remote, inclusiv `armed`, la zero după timeout-ul de telemetrie.

Link quality și avertizare:

- Link quality este calculat din continuitatea numerelor de secvență ale pachetelor de control.
- Pragul de avertizare dorit pe display este sub 90% LQ.
- Algoritmul curent este cumulativ și aproximativ; înainte de testele finale trebuie schimbat cu o fereastră temporală recentă, pentru ca valoarea să reacționeze rapid la degradarea legăturii.

Failsafe wireless:

- Timeout-ul acceptat este 500 ms fără pachete valide de control.
- Imediat după timeout, throttle-ul trebuie comandat la zero, servourile trebuie centrate și flight controller-ul trebuie dezarmat.
- Firmware-ul receiver actual aplică doar throttle 1000 us și steering 1500 us; comanda automată DISARM nu este încă implementată.
- După restabilirea legăturii, sistemul trebuie să rămână dezarmat.
- Reconectarea nu trebuie să repete automat ultima cerere ARM rămasă în transmitter.
- Pentru rearmare este obligatorie o nouă apăsare fizică a butonului ARM după reconectare.
- După rearmare, utilizatorul poate continua numai dacă a verificat că pierderea legăturii nu indică o problemă persistentă.

Failsafe GPS:

- Dacă poziția GPS sau soluția necesară navigației devine invalidă, course/track hold este anulat imediat.
- Pierderea GPS provoacă DISARM, throttle zero și centrarea servourilor, nu revenire automată la steering prin înclinare.
- Sistemul rămâne dezarmat după revenirea GPS-ului și necesită o nouă comandă ARM de la telecomandă.
- Acest failsafe GPS nu este încă implementat în firmware-ul ESP32/ArduPilot și trebuie configurat și testat explicit.

## Controale

### Throttle

- Input fizic: trigger custom.
- Senzor: Hall analogic.
- Magnetul se mișcă față de senzor când triggerul este apăsat.
- Senzorul și mecanismul au fost testate și funcționează.
- Calibrarea poziției eliberate se face automat la fiecare pornire; triggerul trebuie să rămână eliberat în timpul secvenței de boot.
- Valorile negative față de poziția de repaus sunt limitate la `0%`.
- Procesarea include mediere ADC, filtrare, deadzone și mapare la comanda transmisă prin ESP-NOW/MAVLink.
- În repaus comanda rămâne strict `0%`. După filtrul brut de zgomot al senzorului Hall, întreaga cursă utilă este mapată liniar la comanda motorului `15..100%`, fără un deadzone procentual suplimentar.

### Steering

Steering-ul nu va avea comandă manuală prin butoane stânga/dreapta.

- Comanda de steering este bazată pe înclinarea telecomenzii citită din IMU.
- Riderul stă în picioare pe placă și controlează direcția dinamic prin înclinarea telecomenzii ținute într-o mână.
- Parametrii confirmați momentan sunt deadzone de 4 grade și comandă maximă la o înclinare de 35 de grade.

### Course/Track Hold

- Buton touch dedicat pe `GPIO12`.
- Toggle-ul este implementat în firmware și este transmis prin ESP-NOW.
- Receiver-ul îl mapează pe RCIN4: 1000 us OFF și 2000 us ON.
- La activare, steering-ul din înclinarea telecomenzii este ignorat și ArduRover menține traseul GPS memorat.
- A doua apăsare dezactivează funcția și revine la steering prin înclinare.

### Arm / Disarm

- Buton TTP223 dedicat pe `GPIO10`.
- ARM necesită long press de 2 secunde pentru a evita armarea accidentală.
- DISARM se execută imediat la o atingere scurtă.
- Firmware-ul actual face toggle la o atingere scurtă; long press nu este încă implementat.

### Cruise Control

- Buton TTP223 dedicat pe `GPIO9`.
- Prima apăsare memorează throttle-ul curent, iar a doua apăsare dezactivează cruise.
- Cruise se dezactivează automat la reverse, DISARM sau pierderea legăturii wireless.
- Mișcarea/apăsarea triggerului nu dezactivează cruise.
- Firmware-ul actual implementează doar toggle-ul la a doua apăsare; anularea la reverse, DISARM și link loss trebuie adăugată.

### Reverse

- Reverse este comandat prin toggle de la butonul touch conectat la `GPIO11`.
- Activarea reverse trebuie să anuleze imediat cruise control.
- Reverse poate fi activat numai când triggerul este la 0%.
- După activare, triggerul controlează intrarea fizică reverse separată a ESC-ului, iar ieșirea forward rămâne la zero.
- La dezactivarea reverse, triggerul revine la controlul intrării forward, cu trecere obligatorie prin zero.
- La orice schimbare între forward și reverse, triggerul trebuie să rămână la 0% continuu timp de minimum 1 secundă înainte ca schimbarea de sens să fie acceptată.
- În ArduPilot, RC1 reverse va fi rutat către PWM1, RC2 forward către PWM2, iar RC3 steering către PWM3 și PWM4.
- Telecomanda și receiver-ul implementează deja separarea comenzilor forward/reverse și maparea RCIN; configurarea RCOUT rămâne în ArduPilot.

## Butoane Capacitive Touch

- Se folosesc module externe TTP223, nu perifericul touch intern al ESP32-S3.
- Motivul alegerii este eliminarea deschiderilor mecanice din carcasa waterproof.
- Modulele sunt folosite momentary active-high; toggle-ul și long press-ul sunt procesate în firmware.
- GPIO9: SPEED HOLD / cruise control local.
- GPIO10: ARM/DISARM.
- GPIO11: reverse.
- GPIO12: course/track hold.
- Sunt patru butoane capacitive în total.
- Sunt necesare teste prin carcasa finală, cu mâini ude și cu apă pe suprafață, pentru sensibilitate și false touch.

## Status Repository Curent

Rol repository acum:

- Firmware PlatformIO Arduino pentru telecomanda și receptorul sistemului SUP.
- Două build-uri separate: `transmitter` și `receiver`.

Status software curent:

- Display-ul rotund GC9A01 funcționează.
- Randarea cu LovyanGFX este smooth.
- Proiectul PlatformIO este împărțit în două environment-uri:
  - `transmitter`, cu surse în `src/transmitter/`.
  - `receiver`, cu surse în `src/receiver/`.
- Dashboard-ul LovyanGFX a fost pus înapoi ca UI principal.
- Throttle-ul este citit live din senzorul Hall 49E/AH49E pe `GPIO6`.
- Steering-ul este citit live din MPU-9250/6500/9255 folosind orientarea calculată din axele Y/Z ale accelerometrului.
- MPU-ul folosește I2C pe `GPIO7` SDA și `GPIO8` SCL, cu `AD0` la GND pentru adresa `0x68`.
- Butonul ARM TTP223 este pe `GPIO10`; firmware-ul implementează long press 2 secunde pentru ARM și DISARM imediat la apăsare.
- Butonul SPEED HOLD TTP223 este pe `GPIO9`, momentary active-high, cu toggle cruise control local în firmware.
- Butonul reverse este pe `GPIO11`, iar butonul course/track hold pe `GPIO12`; ambele sunt implementate în transmitter și protocolul ESP-NOW.
- Power on/off pentru telecomandă se face prin mufa MT30 cu jumper dedicat de funcționare.
- Citirea bateriei telecomenzii este mutată software pe `GPIO13`, dar rămâne dezactivată până la cablarea și calibrarea divizorului.
- SPEED HOLD ține throttle-ul comandat curent până la următoarea apăsare și este anulat la reverse sau failsafe local.
- Throttle-ul trimis spre receiver este mapat software: repausul rămâne strict `0%`, iar întreaga cursă activă a triggerului este mapată la `15..100%`. Motorul începe fizic să se rotească în jurul comenzii de `20%`; intervalul `15..20%` rămâne o rezervă electrică mică, nu un deadzone introdus în cursa senzorului.
- Transmitter-ul trimite prin ESP-NOW: forward throttle, reverse throttle, steering, ARM/DISARM, SPEED HOLD, COURSE HOLD și starea reverse.
- Receiver-ul trimite înapoi telemetrie prin ESP-NOW: armed real din FC, speed, heading, sateliți, baterie și link quality. Câmpul RSSI rămâne neutilizat.
- Receiver-ul trimite catre FC MAVLink heartbeat, RC override si command long pentru arm/disarm.
- Failsafe receiver: după 500 ms fără control, RCIN1/RCIN2 devin `1000`, RCIN3 devine `1500`, RCIN4 devine `1000` și este trimisă comanda MAVLink DISARM o singură dată.
- La reconectare, receiver-ul sincronizează contorul ARM fără să repete cererea veche; este necesară o nouă comandă ARM.
- Atât transmitter-ul, cât și receiver-ul solicită puterea Wi-Fi maximă suportată de configurația curentă: 19,5 dBm.
- Receiver-ul activ din `src/receiver/main.cpp` este codul MAVLink/UART către FC.
- Vechiul test ESP-NOW RTT + dashboard web este în `backup/receiver_espnow_rtt_dashboard.cpp`.
- Backup-ul testerului Hall este în `backup/hall_sensor_test.cpp`.
- Backup-ul dashboard-ului anterior este în `backup/dashboard_lovyangfx_mockup.cpp`.
- Display-ul pătrat ST7789 a fost testat și a funcționat cu SPI mode 3 la 8 MHz, dar nu mai este display-ul ales.

Status integrare hardware:

- Motorul Maytech a fost testat cu elicea montată și comandă forward.
- Testul motorului a fost făcut fără duct și fără servourile de steering active.
- Reverse-ul ESC nu a fost testat încă.
- Cele două servomecanisme sunt montate fizic, dar pushrod-urile/linkage-urile către ductul mobil nu sunt încă montate.
- Servourile nu au fost încă comandate prin PWM3/PWM4 de la flight controller.
- Ductul, servourile și motorul nu au fost încă testate împreună ca ansamblu complet.
- Holybro M9N obține GPS fix, iar compasul este conectat și calibrat în ArduPilot.
- Sistemul complet nu a fost testat pe apă; până acum au fost testate componente și subsisteme separat.
- Cutia waterproof de pe SUP și carcasa waterproof a telecomenzii sunt încă în dezvoltare.
- Integrarea mecanică și electrică a proiectului este în desfășurare.

Dependințe curente:

- PlatformIO.
- Arduino framework.
- LovyanGFX.

Board target curent:

- `esp32-s3-devkitm-1`
- Flash size configurat la 4 MB.

## TODO După Maparea GPIO

- Montarea divizorului bateriei telecomenzii pe GPIO13 și activarea `TX_BATTERY_MONITOR_ENABLED`.
- Adăugarea divizorului pentru bateria principală pe un ADC liber al ESP32 receiver și trimiterea tensiunii prin telemetrie.
- Configurarea în ArduPilot a RCIN1 reverse, RCIN2 forward, RCIN3 steering și RCIN4 course hold.
- Configurarea RCOUT/PWM1 reverse, PWM2 forward și PWM3/PWM4 pentru cele două servouri.
- Validarea mecanică și electrică a mapării fără elice.
- Testarea funcției course hold în ArduPilot.
- Testarea controlată a reverse-ului; logica din ESP este implementată, dar ESC-ul nu a fost testat în reverse.
- Upgrade ulterior la vESC pentru măsurarea directă a curentului și tensiunii și pentru limitarea configurabilă a motorului.

## Plan Pe Termen Scurt

1. Montarea pushrod-urilor și finalizarea linkage-urilor celor două servouri către ductul mobil.
2. Testarea separată a servourilor prin PWM3/PWM4, fără pornirea motorului.
3. Verificarea centrării mecanice, cursei de `+/-30 grade` și a sensului ambelor servouri.
4. Testarea centrării la failsafe și DISARM.
5. Configurarea și verificarea RCIN/RCOUT în ArduPilot conform documentului de mapare.
6. Verificarea celor patru butoane finale și a comenzilor RCIN în Mission Planner.
7. Testarea reverse-ului inițial fără elice și cu tranziția obligatorie prin zero.
8. Testarea ansamblului motor, duct și steering într-un mediu controlat.
9. Finalizarea și verificarea etanșării carcaselor.
10. Teste progresive pe apă, pornind cu putere și distanță limitate.

## Note De Siguranță

- Motorul BLDC de maximum 4,3 kW și ESC-ul sunt hardware de putere mare.
- Testele timpurii pentru throttle/ESC trebuie făcute fără elice sau cu elicea dezactivată în siguranță.
- Arm/disarm și failsafe sunt funcții critice.
- Reverse control trebuie testat atent pentru a evita inversări bruște de thrust.
- Waterproofing-ul și comportamentul touch cu apă trebuie testate real, nu doar pe banc.

Condiții obligatorii pentru ARM:

- trigger throttle la 0%;
- GPS valid;
- IMU valid;
- senzor Hall valid;
- link ESP-NOW activ;
- link quality de minimum 90%;
- comandă ARM prin apăsare continuă timp de 2 secunde.

Condiții care provoacă DISARM automat:

- lipsa pachetelor de control timp de 500 ms;
- GPS invalid;
- IMU invalid;
- senzor Hall invalid/deconectat;
- tensiune baterie principală `<=21,0 V`;
- tensiune baterie telecomandă `<=3,4 V`.

Comportament la DISARM/failsafe:

- forward și reverse comandate la zero;
- ambele servouri comandate la centru;
- cruise și course/track hold dezactivate;
- sistemul rămâne dezarmat după revenirea senzorului sau legăturii;
- rearmarea necesită satisfacerea din nou a tuturor condițiilor și un nou long press ARM de 2 secunde.

Schimbarea sensului:

- trecerea forward/reverse este permisă numai după minimum 1 secundă continuă cu triggerul la 0%;
- cele două ieșiri forward/reverse nu pot fi active simultan.

Aceste condiții reprezintă cerința finală. Firmware-ul actual nu implementează încă toate validările și toate comenzile automate DISARM.

## Necunoscute / Decizii De Luat

- Pinout-ul fizic final al display-ului și verificarea tuturor conexiunilor telecomenzii.
- Pinul ADC și valorile divizorului pentru tensiunea bateriei principale pe ESP32 receiver.
- Valorile finale ale divizorului pentru bateria telecomenzii pe GPIO13.
- Plaja PWM exactă pentru intrarea reverse și comportamentul ESC-ului dacă ambele intrări primesc accidental semnal.
- Parametrii finali ArduRover pentru rutarea RC1/RC2/RC3 către PWM1-PWM4 și pentru controller-ul Guided/course hold.
- Relația reală throttle-curent și limita de throttle corespunzătoare aproximativ pragului de 80 A.
- Reliability/range ESP-NOW peste apă.
- Rating-ul DC complet al întrerupătorului general și necesitatea unui circuit anti-spark/pre-charge.
- Comportamentul termic al celor două step-down-uri generice la sarcină reală.
- Materialele și procedura finală de etanșare pentru cutia SUP și carcasa telecomenzii.
- Comportamentul butoanelor TTP223 cu apă pe carcasă și mâini ude.
