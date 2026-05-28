# Astronomická èelovka s ATtiny25

Praktická èelovka urèená pøedevším pro astronomické pozorování. Obsahuje dvì sady LED (èervenou a bílou) s plynule nastavitelnou intenzitou a extrémnì nízkou spotøebou v reimu spánku.

![obrazek celovky vcetne baterii a gum](https://github.com/astromik/celovka/blob/master/pic/celovka+gumy.jpg?raw=true)

## Hlavní funkce

- Dvì barvy svìtla (èervená + bílá) s 10 úrovnìmi jasu (logaritmická regulace)
- Reim SLEEP s odbìrem pod 1 µA
- Automatické uspání po 10 minutách neaktivity
- Indikace stavu baterie (1–4 bliknutí)
- Kompaktní a spolehlivé ovládání tøemi tlaèítky

## Ovládání

### Tlaèítko 1 – RED
- **Krátkı stisk** (pøi zhasnuto): rozsvítí èervenou sadu LED
- **Krátkı stisk** (pøi svícení èervené): zhasne èervenou sadu
- **Dlouhı stisk** (> 1 s): rozsvítí èervenou, ale po uvolnìní tlaèítka se automaticky zhasne
- V reimu nastavení jasu: **zvyšuje jas**

### Tlaèítko 2 – WHITE
- **Dlouhı stisk** (~0,7 s): rozsvítí bílou sadu LED (ochrana proti náhodnému zapnutí)
- Pokud bílá sada LED svítí, stisk tlaèítka WHITE ji zhasne
- V reimu nastavení jasu: **sniuje jas**

### Tlaèítko 3 – OFF
- **Krátkı stisk**: zhasne právì svítící LED
- **Drení + RED nebo WHITE** (pøi svícení): vstup do reimu nastavení jasu
- **Dlouhı stisk** (2 s) pøi zhasnutıch LED: pøechod do reimu **SLEEP** (nebo probuzení)

### Nastavení jasu
1. Rozsvite poadovanou sadu LED
2. Stisknìte a drte tlaèítko **OFF**
3. Tlaèítkem **RED** zvyšujete jas, tlaèítkem **WHITE** sniujete jas (10 krokù)
4. Uvolnìním tlaèítka **OFF** nastavení ukonèíte (LED zùstane svítit)

### Reim SLEEP
- Pøi zhasnutıch LED stisknìte tlaèítko **OFF** na ~2 sekundy › 2× blikne èervená LED
- Tlaèítka RED a WHITE se zablokují
- Odbìr klesne **pod 1 µA**

**Probuzení:** Podrte tlaèítko **OFF** na ~2 sekundy › 1× blikne èervená LED.

### Autosleep
Pokud je èelovka déle ne ~10 minut neaktivní (LED zhasnuté), automaticky pøejde do reimu SLEEP.

### Test stavu baterie
Pøi zhasnutıch LED stisknìte nejprve tlaèítko **WHITE** a do 0,7 sekundy k nìmu pøidejte tlaèítko **RED**.

- Èelovka zaène blikat poslednì pouitou barvou
- Poèet bliknutí udává stav baterie:
  - **4×** = baterie dobrá / plná
  - **3×** = stav OK
  - **2×** = baterie slabá
  - **1×** = baterie vybitá (doporuèuji vymìnit)
- Blikání ukonèíte stiskem tlaèítka **OFF**

## Technické informace

**Procesor:** ATtiny25 (8 MHz)  
**Napájení:** 3× AA (1,5V alkalické nebo 1,2V NiMH)  
**Low Fuse:** `0xE2`  
**High Fuse:** `0xD7`  
**Velikost programu:** ~1868 bajtù (91 % FLASH)

**Spotøeba:**
- Zhasnuto (aktivní reim): 4–7 mA
- SLEEP reim: < 1 µA

## Schéma zapojení

*[schéma](https://github.com/astromik/celovka/blob/master/schema/celovka-schema.gif)*

## Kompilace a nahrání

Pouijte Arduino IDE + podporu pro ATtiny.  
Doporuèené nastavení:
- Board: ATtiny25/45/85
- Clock: 8 MHz (internal)
- Burn Bootloader (pro správné nastavení fuse bitù)

