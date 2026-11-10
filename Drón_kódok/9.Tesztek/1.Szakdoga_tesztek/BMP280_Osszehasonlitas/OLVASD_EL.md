# BMP280 összehasonlító mérés ESP32-n

Az eredeti fájlt nem írja felül ez a külön tesztprogram. Nincs MPU/Kalman-szűrő benne:
a BMP280 mérési zaját, eltolódását, gyakoriságát és mozgáskövetését vizsgálja.

## Hardver és feltöltés

1. Nyisd meg a BMP280_Osszehasonlitas/BMP280_Osszehasonlitas.ino fájlt Arduino IDE-ben.
2. Válaszd ki az ESP32 lapodat. A kód tetején ellenőrizd SDA_PIN=21, SCL_PIN=22,
   BMP_ADDR=0x77 értékeket. Ezek a klasszikus ESP32 alapértékei; S3/C3 vagy eltérő
   bekötés esetén állítsd a tényleges lábakra. A cím az eredeti kódodból származik.
3. Töltsd fel. Soros sebesség: 460800 baud. Ha megváltoztatod, a Python oldalon is módosítsd.
4. Két percig legyen bekapcsolva, lehetőleg a teszthelyzetben. Ez nem garantál termikus
   egyensúlyt; jelentős hőmérséklet-drift esetén hosszabb bemelegítés szükséges.

## Profilok

| Parancs | Nyomás | Hőmérséklet | F5 config | F4 forced indítás |
|---|---|---|---|---|
| 1 | ×16 | ×2 | 0x14 (eredeti bájt) | 0x55 |
| 2 | ×4 | ×1 | 0x08 (IIR 4) | 0x2D |

Mindkét profil FORCED módban fut. Az eredeti 0x57 és az ajánlott 0x2F alsó
módbitjei emiatt megváltoznak; az első profil nem bitazonos az eredeti normal móddal.
A standby mező forced módban nem ütemez. A profil 1 eredeti 0x14 config bájtját
megőrizzük, nem írjuk át más kódolásra. A visszaolvasott F4/F5 szerepel a naplóban.
A két profillal egyszerre változik túlmintavételezés és szűrés: a teljes két beállítást
hasonlítjuk össze, az egyes változtatások külön hatását ebből nem lehet elkülöníteni.

Bosch normal módot javasol belső IIR-szűréshez; forced módot host által ütemezett
méréshez. Itt az utóbbit választottuk a minták egyértelmű azonosítására. A belső
szűrőt nem írjuk újra mintánként, így a memóriája megmarad. A végső normal módú
repülési beállítást később abban az üzemmódban is ellenőrizni kell.

## Parancsok

- `s`: a következő futás nyugalmi teszt (alapértelmezett).
- `d`: a következő futás magasságlépés-teszt.
- `1` vagy `2`: az adott profil tesztjének indítása.
- `m`: mozgáskezdet kézi jelölése a tesztfázisban.
- `0`: futás megszakítása.
- `?`: rövid súgó.

Egy futás: 10 s beállás + 10 s referenciaátlag + 120 s teszt. Az első két fázisban
maradjon mozdulatlan a szenzor. A teszt kezdetét a `#PHASE,test` jelzi.
A statisztikába kizárólag a tesztfázis friss, érvényes mintái kerülnek.
Bármilyen I2C-, konfigurációs vagy mérési hiba megszakítja a futást. A megszakított
futást ne használd a végső összehasonlításhoz.

## Naplózás Pythonból

Python 3.10 vagy újabb szükséges:

```sh
python -m pip install pyserial numpy matplotlib
python bmp280_meres.py capture --port COM5 --output nyugalmi.csv
```

A COM5 helyére a tényleges portot írd; Mac/Linux esetén például /dev/cu.usbserial-...
A Serial Monitort/Plottert zárd be, egyszerre csak egy program használhatja a portot.
A Python terminálba írd a parancsot, majd Enter. `q` befejezi a naplózást.
A fájl tartalmazza a megjegyzéseket is; az elemző kezeli ezeket.
A minták nem öntik el a terminált, csak a státuszok és fázisváltások jelennek meg.

## Nyugalmi teszt

A szenzor a végső beépítési helyzetben legyen, stabil tartón. Ne takard le légmentesen.
Ne legyen nyitott ablak, ventilátor, közvetlen napfény; ne fogd kézben és ne fújj rá.
Minden futás előtt azonos fizikai magasságra tedd. Sorrend:

1. `s`, majd `1`; várd meg a #END sort.
2. `2`; várd meg a végét.
3. `2`; várd meg a végét.
4. `1`; várd meg a végét.

A relatív nulla minden futás saját, friss mintákból képzett referenciaátlaga.
Az abszolút magasságpontosságot ez a teszt nem igazolja. A referenciaátlag
bizonytalanságát a teszt alatti szórás nem tartalmazza.

## Magasságlépés-teszt

Jelölj ki két merev tartási szintet, 50 cm függőleges különbséggel. A kábel ne húzza
el a szenzort. Készíts videót, lehetőleg a mozgásról és a terminálról együtt.
Új naplóba mérj; `d`, majd `1` vagy `2`.

- Beállás és referencia: az alsó szinten mozdulatlanul.
- A teszt indulásától kb. 10 s-ig alsó szint.
- `m` + Enter, majd gyors, de ismételhető áthelyezés a felső szintre.
- Újabb kb. 10 s után `m`, vissza az alsó szintre.
- Folytasd tíz átmenetig, majd maradj az utolsó szinten a teszt végéig.
- Ismételd az 1, 2, 2, 1 profilsorrendet azonos eljárással.

A kézi marker nem pontos fizikai mozgáskezdet: a 75%-os követési idő tartalmazza
az emberi reakciót és a mozgatás időtartamát is. Valódi szenzorkéséshez független,
szinkronizált magasságreferencia kell. A végső 3 s platóátlag és a mért lépésnagyság
kézi tesztnél is összehasonlítható, ha valóban mozdulatlan a szenzor.

## Elemzés

```sh
python bmp280_meres.py analyse nyugalmi.csv --output nyugalmi_eredmenyek
python bmp280_meres.py analyse lepes.csv --output lepes_eredmenyek --height-cm 50
```

PNG ábrák, summary.csv és summary.json készülnek. A teljes futás `complete=true`.
A dinamikus futásból a program nem készít R-becslést: annak varianciájában valódi
mozgás is van.

Nyugalmi értelmezés:
- sd_raw_cm / R_raw_cm2: a mért összes ingadozás / ennek négyzete.
- sd_detrended_cm / R_detrended_cm2: lineáris drift levonása utáni érték; a driftet
  külön is vizsgáld, levonásával nem oldódik meg a fizikai hiba.
- drift_cm_per_min: lineáris magassági eltolódás.
- lag1_correlation: szomszédos, detrendelt minták korrelációja.
- achieved_hz: tényleges kiolvasási gyakoriság, benne kommunikáció/feldolgozás is.
- conversion_median_ms: indítás utáni, kiolvasásig tartó idő; nem tisztán ADC-idő.
- increment_sd_over_sqrt2_cm: különböző időeltolásokon mért különbség-ingadozás;
  korrelált zajnál ezek nem azonosak az egyedi minta szórásával.

Erős IIR-szűrés csökkenti a szórást, de növeli a korrelációt és a követési késést.
Ne válassz profilt kizárólag a kisebb szórás alapján. Az R_cm2 a centiméteres
Kalmanhoz használható kezdő becslés; méteres állapotnál oszd 10000-rel. Ez nem
helyettesíti a késés/korreláció kezelését és az MPU-val közös innovációvizsgálatot.
A végső rendszerben motorjáratás melletti, helyzetében rögzített szenzorral is kell
adat: vibráció és légáram az asztali tesztben nem jelenik meg. Ezt csak megfelelő
biztonságos próbapadon végezd; az asztali eredményt ne tekintsd repülési validációnak.

## Javítások és korlátok

Bosch double-kompenzáció (adatlap 8.1), nulla nevező esetén hibajelzés, bájtszám-
és chip-ID ellenőrzés, friss mérésenkénti kiolvasás, status figyelés és timeout.
A status nulláját nem tekintjük automatikusan új mintának: saját forced indítás után
busy->idle átmenet vagy a legrosszabb konverziós idő eltelte kell.
500 us-os státuszlekérdezési időköz; időbélyeg a kiolvasás végén, pontossága
függ a ciklus és kommunikáció futási idejétől. A beállításváltás röviden blokkolhat;
ez külön tesztprogram, változtatás nélkül ne építsd repülésvezérlőbe.
Hardveren nem volt mód futtatni. Host oldali szintaxis- és szimulációs ellenőrzés
készült; feltöltés és valós mérés az ESP32-n szükséges.

Forrás: Bosch BMP280 adatlap, rev. 1.26, 3.3, 3.6, 3.8, 4.3 és 8.1 fejezet:
https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
