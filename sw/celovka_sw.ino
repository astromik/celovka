//============================================
// Astronomicka celovka s obvodem ATtiny25
//               verze 13
//              (28.5.2026)
//============================================
//   3 tlacitka, prepinani cervenych a bilych LED s nastavitelnou intenzitou obou LED.
//   S moznosti prechodu do rezimu minimalni spotreby.
//
//
//
//  Zapojeni:
//=============
//                                                  ATtiny25
//                                                   +-\_/-+
//              nezapojeno (RESET)           - PB5  1|     |8  Vcc
//          tlacitko [WHITE] ( / snizit jas) - PB3  2|     |7  PB2  - INT0     - tlacitko [OFF] ( / SLEEP / WAKE-UP)
//            tlacitko [RED] ( / zvysit jas) - PB4  3|     |6  PB1  OC0B (PWM) - bila LED - spinana pres FET
//                                             GND  4|     |5  PB0  OC0A (PWM) - cervena LED - spinana pres FET  
//                                                   +-----+
//
//                   (Tlacitka jsou spinana proti GND, LED jsou kvuli vyssim proudum ovladane pres N-FET napr. 2N7000)
//   Napajeni je bud pomoci   3 x 1.2V AA NiMh akumulatory = 3.6 V,
//                            4 x 1.2V AA NiMh akumulatory = 4.8 V,
//                       nebo 3 x 1.5V AA baterie          = 4.5 V
//        v pripade pouziti   4 x 1.5V AA baterie          = 6   V,  je treba snizit napeti do ATtiny pomoci seriove zapojene diody na vyvodu Vcc
//
//
// nastaveni FUSE bajtu:
//========================
// LowFUSE  : 0xE2         8MHz pro zrychleni PWM. Pri nastaveni 1MHz bylo pri pohybu celovky videt neprijemne prerusovane blikani LED.
// HighFUSE : 0xD7
//
// velikost prelozeneho programu 2032 Bajtu (99% obsazene PROGMEM)


// Seznam zmen verze 13 proti verzi 12 (z 2.4.2016):
//---------------------------------
// Funkce BLIKAC byla nahrazena funkci testu stavu baterie
//     Soucasnym stiskem tlacitek [WHITE] + [RED] (nejdriv [WHITE] a behem 0.7 sekundy pridat [RED]) se zacne vyblikavat kod 1 az 4 podle stavu napajeciho napeti (1x = slaba baterie ... 4x = baterie v poradku)
//     Barva a jas blikani jsou stejne, jako bylo posledni pouzite sviceni.
//     Ukonceni blikani stiskem tlacitka [OFF]. Sice se ted neda volit rezim blikani, ale alespon omezene blikani bylo zachovano.
// Doplnena moznost jednoduchou upravou kodu prohodit pripojeni tlacitek [RED] a [WHITE]
// Svitici bilou sadu LED je mozne zhasnout stiskem tlacitka [WHITE]. Drive tlacitko [WHITE] pri sviticich bilych LED nic nedelalo a bila se musela hlasnout pomoci [OFF], nebo prepnutim na [RED]
// Tajna funkce:
//     Pokud se pri resetu (pri vkladani baterii) budou drzet vsechna 3 tlacitka stisknuta, nebude se bila sada LED rozsvecovat skokove, ale narust jasu bude pozvolny.
//     Prechod z nuloveho na maximalni jas trva asi 2 sekundy. 
//     Je to jen takova "dyzajnovka", postupne rozsveceni je trochu prijemnejsi na oci.
//     Nevyhoda je pomalejsi reakce. Zhasnuti bile sady LED tlacitkem [WHITE] je mozne az po jejich uplnem rozsviceni.
//     Vraceni puvodni funkce skokoveho rozsvecovani bile sady LED je mozne dalsim resetem (vlozenim baterii) bez stisknutych tlacitek.
//     Nastaveny zpusob rozsveceni bile sady LED si procesor pamatuje i v rezimu spanku.



// Seznam zmen verze 12 proti verzi 11 (z 10.11.2015):
//---------------------------------
//  Tlacitko [RED] ma dve nove funkce:
//       1) pri rozsvicene cervene LED je mozne tuto LED stejnym tlacitkem zase zhasnout - neni tedy treba prehmatavat na tlacitko [OFF]
//       2) pri stisku tohoto tlacitka na dele nez 1 sekundu se pri jeho uvolneni cervena LED automaticky zhasne - tlacitko funguje jako normalni tlacitko
//  Kvuli velikosti programu bylo nutne ve funkci BLIKAC ubrat nejpomalejsi rezim blikani (1 sekunda svetlo, 15 sekund tma)





//=============================================================================================================

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>

#define tlacitko_RED          0b00010000                         // PB4 tlacitka [RED] a [WHITE] je mozne navzajem prohodit
#define tlacitko_WHITE        0b00001000                         // PB3



#define tlacitko_OFF           0b00000100                         // PB2 kvuli probouzeni pres pevy signal INT0 nelze pozici tlacitka menit

#define STISKNUTO(x)          (!(PINB & (x)))                     // stisknute tlacitko je spojene na GND, takze vraci LOW
#define UVOLNENO(x)             (PINB & (x))                      // uvolnene tlacitko je pull-upovane, takze vraci HIGH


// casovaci konstanty v [ms] (kvuli zmene frekvence procesoru na 8MHz je nutne pozadovany cas nasobit 8x)
#define timer_30             8 *   30                             // 30ms pauza pro osetreni zakmitu tlacitek a pro casovanio pruchdu hlavni smyckou
#define bat_led_on           8 *  125                             // pri vyblikavani kodu pro stav baterie urcuje dobu trvani rozsvicene LED
#define bat_led_off          8 *  250                             // pri vyblikavani kodu pro stav baterie urcuje dobu trvani zhasnute LED
#define bat_led_mezera       8 * 1250                             // pri vyblikavani kodu pro stav baterie urcuje dobu mezi opakovanim sekvence bliknuti
#define blikred_led_on       8 *  200                             // delka trvani rozsvicene LED pri signalizaci rucniho uspani nebo probuzeni procesoru
#define blikred_led_off      8 *  200                             // delka trvani zhasnute LED pri signalizaci rucniho uspani nebo probuzeni procesoru
#define longpress_white      8 *  700                             // doba trvani drzeni tlacitka [WHITE] pred skutecnym rozsvicenim bile sady LED
#define change_level_timer   8 *  500                             // pauza mezi zmenami jasu LED v rezimu nastaveni jasu
#define wakeup_timer         8 *  100                             // pauza na zotaveni po probuzeni z rezimu spanku
#define longpress_red        8 * 1000                             // doba trvani drzeni tlacitka [RED] po kterem se tlacitko chova jako tlacitko a po uvolneni LED hned zhasne
#define longpress_off        8 * 2000                             // doba drzeni tlacitka [OFF] nutna k probuzeni nebo uspani procesoru

#define boot_timer           1000                                 // cas necinnosti [ms] pri startu napajeni - osetreni zakmitu pri vkladani baterii do drzaku

#define cykly_autosleep   20000;                                  // pauza v cyklech pred prechodem do automatickeho uspani. Jeden cyklus trva asi 30ms (konstanta "timer_30").
                                                                  // 20000 cyklu odpovida asi 10 minutam


volatile uint8_t level_red = 4;                                   // uroven jasu cervene LED - po zapnuti napajeni se cervena LED nastavi na 50%  (urovne jsou 0 az 9 pro 10% az 100% jasu)
volatile uint8_t level_white = 7;                                 // uroven jasu bile LED - po zapnuti napajeni se bila LED nastavi na 80%  (urovne jsou 0 az 9 pro 10% az 100% jasu)
volatile uint8_t prave_sviti = 0;                                 // pomocna promenna pro indikaci prave svitici LED (0= nesviti zadna; 1= sviti cervena; 2= sviti bila)
volatile uint8_t byla_zmena;                                      // pomocna promenna pro rozliseni, jsetli je uvolneni tlacitka [OFF] navratem z nastaveni, nebo obycejnym zhasnutim
volatile uint8_t zamek = 0;                                       // informace o tom, jestli ma pokracovat SLEEP rezim po nahodnem kratkem stisku tlacitka [OFF], nebo jestli se ma SLEEP ukoncit po dlouhem stisku       
volatile uint32_t autosleep;                                      // automaticky prechod do rezimu SLEEP po 10 minutach, kdy jsou obe LED zhasnute (hodnota casu se nastavuje v podprogramu "obe_off()" )
volatile uint32_t starttime;                                      // pomocna promenna pro zjistovani delky sepnuti tlacitka
volatile uint8_t posledni_barva = 1;                              // promenna pro zapamatovani posledni pouzite barvy LED - kvuli funkci blikace, kdy pak bude blikat stejna LED (1=cervena , 2= bila)

bool slow_white = false;                                          // pomaly nabeh bile sady LED je mozne zapnout pri resetu (pri vkladani baterii) drzenim vsech 3 tlacitek. Defaultne je pomaly nabeh vypnuty.

// ===================================================================
// zmena jasu LED neni provadena linearne, ale logaritmicky.
//    (protoze je dobre znatelny rozdil v jasu mezi cisly 253 a 254, ale neni pozorovatelny zadny rozdil v jasech mezi cisly 10 a 11 )
const byte logaritmy[] = {254,253,252,249,243,233,215,181,117,0}; // 0 = maximalni jas LED ; 255 = zhasnute LED



// ===================================================================
// temhle podprogram je tu jen kvuli uspore mista ve FLASH pameti ATtiny25
// volanim podprogramu "pauza_30()" se proti volani funkce "delay(30)" usetri v tomto programu vice nez 100 bajtu pameti
void pauza_30(void)
  {
    delay(timer_30);
  }



// ===================================================================
// Mereni napeti baterie pres interni bandgap referenci (1.1V)
uint8_t getBatteryLevel(void)
{
  ADMUX = (0 << REFS0) | (1 << MUX3) | (1 << MUX2);               // 1.1V bandgap vs Vcc
  delay(16);                                                      // povinna pauza 2ms * 8
  ADCSRA |= (1 << ADSC);
  while (bit_is_set(ADCSRA, ADSC));

  uint16_t adc = ADC;
                                                                  // procesor funguje i pri napeti pod 2V, ale nektere typy LED uz se nedokazi na takto malem napeti rozsvitit
                                                                  // Velikost napeti ovlivnuje i svitivost LED, takze pri nizsim napajeni je nutne zvysit stupen jasu.

                                                                  // pokusne zjistene hodnoty napeti pro ruzne 'adc' meze.
                                                                  // Navratova hodnota udava pocet bliknuti pri funkci testovani baterie.
  if (adc > 390) return 1;                                        // napeti velmi spatne  ( pod 2.9V)
  if (adc > 340) return 2;                                        // napeti slabe         ( 2.9V az 3.2V)
  if (adc > 280) return 3;                                        // napeti stredni       ( 3.3V az 3.9V)
  return 4;                                                       // napeti v poradku     ( nad 3.9V)
}

// ===================================================================
// rozsviceni cervene LED (PWM kanal A - pin PB0)
// parametrem je index pole logaritmy[]  ; 0 az 9 odpovida 10% az 100% jasu
void red_on(byte level_red)
  {
    OCR0A  = logaritmy[level_red];                                // sirka PWM impulzu
    prave_sviti = 1;                                              // prave sviti cervena LED
  }


// ===================================================================
// rozsviceni bile LED (PWM kanal B - pin PB1)
// parametrem je index pole logaritmy[]  ; 0 az 9 odpovida 10% az 100% jasu
void white_on(byte WHITE)
  {
    OCR0B  = logaritmy[WHITE];                                    // sirka PWM impulzu
    prave_sviti = 2;                                              // prave sviti bila LED
  }

// ===================================================================
// bez parametru se postupne rozsviti bila sada LED (PWM kanal B - pin PB1) na posledni nastaveny jas bile LED
void white_on(void)
  {
    byte zkratka = 0;
    for (byte i = 255 ; i > logaritmy[level_white] ; i--)         // zacina se od nizkeho jasu a postupne se zjasnuje
      {
        OCR0B = i;
        if (i > 150)                                              // dynamicka pauza se dela jen pro nizke jasy 
          {
            zkratka = inter_pauza(i,false);                       // pri parametru 0 se budou testovat obe tlacitka [RED] i [OFF]
            if (zkratka > 0 )                                     // kdyz se behem rozsveceni stiskne [OFF] nebo [RED]
              {
                break;
              }
          }
      }
    
    if (zkratka < 2)                                              // pri zkraceni smycky pomoci tlacitka [OFF] (zkratka=1), nebo pri dokonceni pozvolneho dobehu (zkratka=0) 
      {
        OCR0B = logaritmy[level_white];                           // dojde k okamzitemu rozsviceni na nastavenou uroven (preskoci se pozvolny najezd) - je mozne hned nastavovat jas bile LED
        prave_sviti = 2;                                          // prave sviti bila LED
      }
    if (zkratka == 2)                                             // pri zkraceni smycky pomoci tlacitka [RED]
      {
        obe_off();                                                // se bila zhasne a v pristim kroku se hned rozsviti cervena sada LED, protoze je stisknute cervene tlacitko
      }
          
  }



// ===================================================================
// zhasne obe LED
void obe_off(void)
  {
     OCR0A  = 255;                                                // PWM na cervene LED na sirku impulzu 0% (zhasne cervenou LED)
     OCR0B  = 255;                                                // PWM na bile LED na sirku impulzu 0% (zhasne bilou LED)
     prave_sviti = 0;                                             // prave nesviti zadna LED
     autosleep = cykly_autosleep;                                 // pri zhasnuti obou LED se nastavi odpocet automatickeho uspani asi na 10 minut     
  }



// ===================================================================
// signalizace stavu baterie bliknutim LED
void test_napeti(void)
  {
    while (true)                                                  // nekonecna blikaci smycka se prerusi jen stiskem tlacitka na PB2 behem vyckavacich pauz
      {
        uint8_t bat = getBatteryLevel();                          // stav baterie (1=slaba .... 4=plna)
        
        for(uint8_t i = 0; i < bat; i++)
          {
            if ( posledni_barva == 2 )                            // kdyz byla naposledy pouzita bila barva ...
              {
                white_on(level_white);                            // ... rozsvit bilou LED prednastavenym jasem      
              }
            else                                                  // kdyz byla naposledy pouzita cervena barva ...
              {
                red_on(level_red);                                // ... rozsvit cervenou LED prednastavenym jasem      
              }                
            if (inter_pauza(bat_led_on ,true) == 1)    return;    // doba trvani sviceni LED  (preruseno pomoci [OFF], tlacitko [RED] se ignoruje)
            obe_off();                                            // a pak zhasni obe LED
            if (inter_pauza(bat_led_off,true) == 1)    return;    // chvili pockej (preruseno pomoci [OFF], tlacitko [RED] se ignoruje)
          }
        if (inter_pauza(bat_led_mezera ,true) == 1)    return;    // delsi mezera po kazde blikaci sekvenci (preruseno pomoci [OFF], tlacitko [RED] se ignoruje)
      }
  }
  

// ===================================================================
// prerusitelna pauza (nahrada funkce delay() s moznosti preruseni stiskem tlacitka [OFF] nebo [RED])
//       Vyuziva se k okamzitemu ukonceni podprogramu "test_napeti()" bez nutnosti dobehnuti vyblikavani do konce sekvence, nebo okamzite zruseni postupneho rozsveceni bile LED
// parametr 'ignoruj_red'  umoznuje ignorovat stisk tlacitka [RED] 
//  return 0 ... pauza dobehla do konce
//  return 1 .... pauza byla prerusena tlacitkem [OFF]
//  return 2 .... pauza byla prerusena tlacitkem [RED]
byte inter_pauza(uint32_t interval,bool ignoruj_red)
  {  
    uint32_t start = millis();
    while (millis() < start + interval)
      {
        // predcasne je mozne pauzu prerusit stiskem tlacitka [OFF] nebo [RED]
        if (STISKNUTO(tlacitko_OFF)) return 1;
        if (STISKNUTO(tlacitko_RED) and ignoruj_red == false) return 2;
      }
    return 0;
  }



// ===================================================================
// bliknuti cervene LED na 50% na 0,2sek. (signalizace rucniho uspani a probuzeni procesoru)
void blikred(void)
  {  
    red_on(4);                                                    // rozsvit cervenou LED na 50%
    delay(blikred_led_on);                                        // chvili pockej
    obe_off();                                                    // a pak zhasni obe LED
    delay(blikred_led_off);                                       // chvili pockej
  }




// ===================================================================
// Podprogram pro uspani a opetovne probuzeni ATtiny25
// Opsano ze stranek :
//    http://www.avrfreaks.net/forum/attiny85-wake-pin-interrupt-not-working

void sleep()
  {
    MCUCR &= ~_BV(ISC01); MCUCR &= ~_BV(ISC00);                   // The low level of INT0 generates an interrupt request.
    ADCSRA &= ~_BV(ADEN);                                         // ADC off
    MCUCR |= _BV(SM1); MCUCR &= ~_BV(SM0);                        // Select "Power-down" sleep mode
    
    sleep_enable();                                               // Enable sleep mode
    GIMSK |= _BV(INT0);                                           // Enable External Interrupt INT0
    
    sei();                                                        // Enable all interrupts
    sleep_cpu();                                                  // GO TO SLEEP
  
    cli();                                                        // Disable all interrupts
    GIMSK &= ~_BV(INT0);                                          // Disable External Interrupt INT0
    
    sleep_disable();                                              // Disable sleep mode
    ADCSRA |= _BV(ADEN);                                          // ADC on
    
    sei();                                                        // Enable all interrupts
  }



// ===================================================================
void setup()
  { 
    delay(boot_timer);                                            // po zapnuti napajeni chvili nic nedelej (to je kvuli zakmitavani napajeni pri zasunovani baterii)

    if (STISKNUTO(tlacitko_RED) && STISKNUTO(tlacitko_WHITE) && STISKNUTO(tlacitko_OFF)) // pri vkladani baterii je mozne prepnout plynule rozsveceni bile sady LED 
      {
        slow_white = !slow_white;    
      }

    

                                                                  // nastaveni smeru portu a Pull-Upu
    DDRB  = 0b00000011;                                           // nastaveni smeru signalu na portu B ("1" = vystup ; "0" = vstup)
    PORTB = 0b00011111;                                           // na vsechny tlacitka (PB2, PB3 a PB4) Pull-Upy a oba vystupy (PB0, PB1) zhasnout

                                                                  // nastaveni rezimu PWM
    TCCR0A = 0b11110011;                                          // rezim FAST PWM; vystup se nastavi do '0' pri dosazeni pozadovaneho cisla
    TCCR0B = 0b00000010;                                          // FAST PWM, delic zakladni frekvence na 4.6kHz (pro prescaler='010b')       

    obe_off();                                                    // po zapnuti napajeni se obe LED zhasnou, a tim se spusti casovac automatickeho uspani (na 10 minut)  
  }



// ===================================================================
// hlavni smycka pro test tlacitek
void loop()
  {

    // ===================================================================
    // stisk tlacitka [RED] - obycejne, okamzite rozsviceni cervene LED

    // kdyz nesviti cervena LED a je sepnute pouze tlacitko [RED]
    if (prave_sviti != 1 && STISKNUTO(tlacitko_RED) && UVOLNENO(tlacitko_WHITE) && UVOLNENO(tlacitko_OFF)) 
      {
        obe_off();                                                // Obe LED zhasnout
        red_on(level_red);                                        // Rozsvit cervenou LED pozadovanym jasem
        posledni_barva = 1;                                       // pamet posledni svitici barvy LED - kvuli funkci BLIKAC v testu stavu baterie 

        starttime = millis();                                     // zapamatovani casu rozsviceni kvuli funkci automatickeho zhasnuti pri delsim drzeni cerveneho tlacitka
        pauza_30();                                               // odruseni zakmitu tlacitka [RED]
        while (STISKNUTO(tlacitko_RED))                           // cekani na uvolneni cerveneho tlacitka
          {
            pauza_30();
          }      
        if (millis() > starttime + longpress_red)                 // pokud bylo cervene tlacitko stisknute na delsi dobu, nez 1 sekunda, ...
          {
            obe_off();                                            //  ... tak se po uvolneni cervena LED zase zhasne
          }
      }



    // ===================================================================
    // stisk tlacitka [RED] pri rozsvicene cervene LED, cervenou LED zhasina
    // kdyz cervena LED sviti a je sepnute pouze tlacitko [RED]
    if (prave_sviti == 1 &&  STISKNUTO(tlacitko_RED) && UVOLNENO(tlacitko_WHITE) && UVOLNENO(tlacitko_OFF))
      {
        obe_off();                                                // cervenou LED zhasnout
        pauza_30();                                               // odruseni zakmitu tlacitka [RED]
        while (STISKNUTO(tlacitko_RED))                           // cekani na uvolneni cerveneho tlacitka
          {}
        pauza_30();                                               // odruseni zakmitu tlacitka [RED]
      }



    // ===================================================================
    // stisk tlacitka [WHITE] - obycejne rozsviceni bile LED (casovane - rozsviti se az po 0.7 sek drzeni tlacitka [WHITE])

    // kdyz nesviti bila LED a je sepnute pouze tlacitko [WHITE]
    if (prave_sviti != 2 && (STISKNUTO(tlacitko_WHITE)) && UVOLNENO(tlacitko_RED) && UVOLNENO(tlacitko_OFF))
      {
        pauza_30();                                               // odruseni zakmitu tlacitka [WHITE]

        starttime = millis();                                     // cas zacatku stisku tlacitka [WHITE] do promenne
        while (STISKNUTO(tlacitko_WHITE))                         // dokud je sepnute tlacitko [WHITE]
          {
            if (millis() > (starttime + (longpress_white)))       // kdyz je tlacitko [WHITE] stisknuto asi 0.7 sekundy ...
              {

                 // kdyz je po te 0.7 sekunde stisknute spolecne s tlacitkem [WHITE] i tlacitko [RED] (tlacitko [OFF] je uvolnene)
                 if (STISKNUTO(tlacitko_WHITE) && STISKNUTO(tlacitko_RED) && UVOLNENO(tlacitko_OFF))
                   {
                     test_napeti();                               // ... prejde se do rezimu testovani napajeciho napeti
                   }
                 else                                             // kdyz po 0.7 sekunde neni stisknute tlacitko [RED], znamena to obycejne rozsviceni bile LED
                   {                    
                     if (prave_sviti != 2)                        // Kdyz bila nesviti, rozsvit ji pozvolna
                       {
                         obe_off();                               // ... obe LED zhasnout
                         if (slow_white == true)  white_on();     // postupne rozsviceni na nastavenou uroven
                         else          white_on(level_white);     // okamzite rozsviceni bez pozvolneho nabehu
                       }
                     posledni_barva = 2;                          // pamet posledni svitici barvy LED - kvuli funkci BLIKAC pri mereni napajeciho napeti

                   }
              }
              pauza_30();
           }
      }

    // ===================================================================
    // stisk tlacitka [WHITE] - pri sviticich bilych LED je zhasne

    // kdyz sviti bila LED a je sepnute pouze tlacitko [WHITE]
    if (prave_sviti == 2 && (STISKNUTO(tlacitko_WHITE)) && UVOLNENO(tlacitko_RED) && UVOLNENO(tlacitko_OFF))
      {
        obe_off();
      }



    // ===================================================================
    // stisk tlacitka [OFF] (pri zmacknutem tlacitku [OFF] je spusten rezim "NASTAVENI JASU")

    // nejaka LED sviti a je stisknuto pouze tlacitko OFF
    if (prave_sviti != 0 && STISKNUTO(tlacitko_OFF) && UVOLNENO(tlacitko_RED) && UVOLNENO(tlacitko_WHITE))
      {
        // vstup do rezimu nastaveni jasu
        pauza_30();                                               // odruseni zakmitu tlacitka [OFF]        
        
        byla_zmena = 0;                                           // pomocna promenna pro zjisteni, jestli po uvolneni tlacitka [OFF] zhasnout (= 0) nebo nechat svitit (= 1) LED

        while (STISKNUTO(tlacitko_OFF))                           // dokud je tlacitko [OFF] sepnute (nezavisle na tom, jak jsou na tom ostatni tlacitka)
          {
             if (STISKNUTO(tlacitko_RED))                         // a zaroven je stisknuto tlacitko [RED] (zvysovani jasu)
               {
                 if (prave_sviti == 1)                            // a kdyz prave sviti cervena
                   {
                     if (level_red == 9) level_red = 8;           // test na prekroceni rozsahu pri zvetsovani jasu (nikdy nebude vic, nez 9)
                     level_red ++;                                // zvetseni cerveneho jasu
                     red_on(level_red);                           // fyzicka aktualizace jasu cervene LED
                     byla_zmena = 1;                              // byla provedena zmena jasu, takze po uvolneni tlacitka [OFF] se nebude cervena LED zhasinat
                   }
                 if (prave_sviti == 2)                            // kdyz prave sviti bila
                   {
                     if (level_white == 9) level_white = 8;       // test na prekroceni rozsahu pri zvetsovani jasu (nikdy nebude vic, nez 9)
                     level_white ++;                              // zvetseni bileho jasu
                     white_on(level_white);                       // fyzicka aktualizace jasu bile LED
                     byla_zmena = 1;                              // byla provedena zmena jasu, takze po uvolneni tlacitka [OFF] se nebude bila LED zhasinat                   
                   }
                 
                 delay(change_level_timer);                       // 0.5s pauza mezi jednotlivymi kroky pri zvysovani jasu
               }


             if  (STISKNUTO(tlacitko_WHITE))                      //kdyz je zaroven s tlacitkem [OFF] stisknuto tlacitko [WHITE] (snizovani jasu)
               {
                 if (prave_sviti == 1)                            // a kdyz  prave sviti cervena
                   {
                     if (level_red == 0) level_red = 1  ;         // test na "podlezeni" rozsahu pri snizovani jasu (nikdy nebude min, nez 0)
                     level_red --;                                // snizeni cerveneho jasu
                     red_on(level_red);                           // fyzicka aktualizace jasu cervene LED
                     byla_zmena = 1;                              // byla provedena zmena jasu, takze po uvolneni tlacitka [OFF] se nebude cervena LED zhasinat
                   }
                 if (prave_sviti == 2)                            // a kdyz prave sviti bila
                   {
                     if (level_white == 0) level_white = 1;       // test na "podlezeni" rozsahu pri snizovani jasu (nikdy nebude min, nez 0)
                     level_white --;                              // snizeni bileho jasu
                     white_on(level_white);                       // fyzicka aktualizace jasu bile LED
                     byla_zmena = 1;                              // byla provedena zmena jasu, takze po uvolneni tlacitka [OFF] se nebude bila LED zhasinat                    
                   }
                 
                 delay(change_level_timer);                       // 0.5s pauza mezi jednotlivymi kroky pri snizovani jasu
               }
          }                                                       // uvolneni tlacitka [OFF]


         // zhasnuti LED pri normalnim rezimu (obycejne uvolneni tlacitka [OFF])
         if (byla_zmena == 0)                                     // pokud nebyla provedena zmena jasu, zhasnou se pri uvolneni tlacitka [OFF] obe LED
           {
             obe_off();                                           // zhasne obe LED
           }
         pauza_30();                                              // odruseni zakmitu uvolnovaneho tlacitka [OFF]
      }



    // ===================================================================
    // rezim SLEEP - uzamceni tlacitek pri 2-sekundovem podrzeni tlacitka [OFF] kdyz nesviti zadna LED

      // kdyz nesviti zadna LED a je stisknuto pouze tlacitko [OFF]
      if (prave_sviti == 0 && (STISKNUTO(tlacitko_OFF)) && (UVOLNENO(tlacitko_RED)) && (UVOLNENO(tlacitko_WHITE)))
        {

          obe_off();                                              // obe LED jsou sice uz zhasnute, takze navenek se nic nedeje. Dochazi ale k resetu casovace pro automaticky SLEEP rezim (opetovne nastaveni na 10 minut)
                    
          starttime = millis();                                   // okamzik ihned po probuzeni se zapamatuje
          
          while (STISKNUTO(tlacitko_OFF) && zamek==0)             // dokud je stisknuto tlaciko [OFF] ...
           {
             if (millis() > (starttime + longpress_off))          // test, jestli uz je zamykaci tlacitko drzeno alespon 2 sekundy
               {
                  blikred();                                      //  pokud ano, bliknuti cervene LED na 50% na 0.2sek. = bude se prechazet do SLEEP rezimu
                  blikred();                                      //  pri manualnim uspani je bliknuti dvojite

                  zamek = 1;                                      // pomocna promenna pro signalizaci, ze se bude uspavat  
               }
             
             pauza_30();
           }


          while (STISKNUTO(tlacitko_OFF))                         // cekej, nez se po pozadavku na uspani uvolni tlacitko [OFF]
           {
            pauza_30();
           }

             
          while (zamek == 1)                                      // dokud je platny pozadavek na uspani (dokud nebylo odemceno dlouhym stiskem, tak se po kazdem probuzeni zase ATtiny uspi)
            { 
              sleep();                                            // uspani do doby, nez dojde k sestupne hrane impulzu na INT0 (PB2)
              delay(wakeup_timer);                                // 0.1 sekundy pauza po probuzeni
              
              starttime = millis();                               // cas probuzeni se zapamatuje


              while (STISKNUTO(tlacitko_OFF) && zamek==1)         // dokud je stisknuto jenom tlaciko [OFF] (na stavu ostatnich tlacitek nezalezi)
               {
                 if (millis() > (starttime + longpress_off))      // .. porovnava, jestli uz je probouzeci tlacitko [OFF] drzeno alespon 2 sekundy
                   {
                      blikred();                                  // pokud ano, bliknuti cervene LED na 50% na 0.2sek. = signalizace probuzeni
                      zamek = 0;                                  // zruseni pozadavku na uspani
                   }
                 pauza_30();
               }

              // tady uz bylo vyhodnoceno, ze je probouzeci tlacitko [OFF] stisknuto dostatecne dlouho

              while (STISKNUTO(tlacitko_OFF))                     // cekej, nez se tlacitko [OFF] uvolni (aby nedoslo pri delsim drzeni opet k uzamceni)
               {
                pauza_30();
               }
            }
        }




   // ===================================================================
   //  rezim AUTOSLEEP - automaticky prechod do SLEEP rezimu po 10 minutach zhasnutych LED
   
    if (prave_sviti == 0 && zamek == 0)                           // kdyz nesviti zadna LED, ale celovka je v normalnim rezimu,
      {
        autosleep --;                                             // probiha odpocitavani do automatickeho uspani (1 pruchod = asi 0.03 sekundy) Promenna "autosleep" se nastavuje na 10 minut ve funkci "obe_off()"
        if (autosleep == 0)                                       // pri dopocitani autosleepu do 0 nastav SLEEP rezim
          {
                                                                  // bliknuti cervenou LED pri automatickem prechodu do SLEEP rezimu
            // blikred();                                         // Ukazalo se, ze je to spise obtezujici signalizace, tak byla zrusena

            zamek = 1;                                            // ATtiny se bude uspavat

            while (zamek == 1)                                    // dokud je platny pozadavek na uspani (dokud nebylo odemceno dlouhym stiskem, tak se po kazdem probuzeni zase ATtiny uspi)
              { 
                 sleep();                                         // uspani do doby, nez dojde k sestupne hrane impulzu na INT0                       
                 starttime = millis();                            // cas probuzeni se zapamatuje
    
                 while (STISKNUTO(tlacitko_OFF)  && zamek == 1)   // dokud je stisknuto tlaciko [OFF] (na stavu ostatnich tlacitek nezalezi)
                   {
                     if (millis() > (starttime + longpress_off) && zamek == 1)  // ... testuj, jestli uz je probouzeci tlacitko [OFF] drzeno alespon 2 sekundy
                       {
                          blikred();                              //  pokud ano, bliknuti cervene LED na 50% na 0.2sek. = probuzeni tlacitkem z automatickeho uspani
                          zamek = 0;                              // ... a odblokuj zamek  
                       }
                     pauza_30();
                   }
               }
 

              // tady uz bylo vyhodnoceno, ze je probouzeci tlacitko [OFF] stisknuto dostatecne dlouho

              while (STISKNUTO(tlacitko_OFF))                     // cekej, nez se tlacitko [OFF] uvolni (aby nedoslo pri delsim drzeni opet k uzamceni)
               {
                pauza_30();
               }

          }
      }

    pauza_30();                                                   // pri kazdem pruchodu hlavni smyckou pauza 0.03 sekundy ( DULEZITE pro casovani autosleepu!) 
  }



// ===================================================================
// podprogram pro obslouzeni preruseni - (nic nedela, jen musi byt definovany)
ISR(INT0_vect)
    {

    }
