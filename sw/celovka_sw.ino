//============================================
// Astronomicka celovka s obvodem ATtiny25
//               verze 12
//              (2.4.2016)
//============================================
//   3 tlacitka, prepinani cervenych a bilych LED s nastavitelnou intenzitou obou LED
//   a s moznosti prechodu do rezimu minimalni spotreby.
//
//
//
//  Zapojeni:
//=============
//                                                  ATtiny25
//                                                   +-\_/-+
//              nezapojeno (RESET)           - PB5  1|     |8  Vcc
//           tlacitko_2 (WHITE / snizit jas) - PB3  2|     |7  PB2  - INT0     - tlacitko_3 (OFF / SLEEP)
//             tlacitko_1 (RED / zvysit jas) - PB4  3|     |6  PB1  OC0B (PWM) - bila LED - spinana pres FET
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
// LowFUSE  : 0xE2
// HighFUSE : 0xD7
//
// velikost prelozeneho programu 2046 Bajtu 



// Seznam zmen proti verzi 11 (z 10.11.2015):
//---------------------------------
//  Tlacitko RED (tlacitko_1) ma dve nove funkce:
//       1) pri rozsvicene cervene LED je mozne tuto LED stejnym tlacitkem zase zhasnout - neni tedy treba prehmatavat na tlacitko_3 (OFF)
//       2) pri stisku tohoto tlacitka na dele nez 1 sekundu se pri jeho uvolneni cervena LED automaticky zhasne - tlacitko funguje jako normalni tlacitko
//  Kvuli velikosti programu bylo nutne ve funkci BLIKAC ubrat nejpomalejsi rezim blikani (1 sekunda svetlo, 15 sekund tma)





//=============================================================================================================

#include <avr/interrupt.h>
#include <avr/sleep.h>

volatile byte level_red = 4;           // uroven jasu cervene LED - po zapnuti napajeni se cervena LED nastavi na 50%  (urovne jsou 0 az 9 pro 10% az 100% jasu)
volatile byte level_white = 7;         // uroven jasu bile LED - po zapnuti napajeni se bila LED nastavi na 80%  (urovne jsou 0 az 9 pro 10% az 100% jasu)
volatile byte prave_sviti = 0;         // pomocna promenna pro indikaci prave svitici LED (0= nesviti zadna; 1= sviti cervena; 2= sviti bila)
volatile byte byla_zmena;              // pomocna promenna pro rozliseni, jsetli je uvolneni tlacitka_3 navratem z nastaveni, nebo obycejnym zhasnutim
volatile byte zamek = 0;               // informace o tom, jestli ma pokracovat SLEEP rezim po nahodnem kratkem stisku tlacitka_3, nebo jestli se ma SLEEP ukoncit po dlouhem stisku       
volatile unsigned long autosleep;      // automaticky prechod do rezimu SLEEP po 10 minutach, kdy jsou obe LED zhasnute (hodnota casu se nastavuje v podprogramu "obe_off()" )
volatile unsigned long starttime;      // pomocna promenna pro zjistovani delky sepnuti tlacitka
volatile byte blik_index = 3;          // promenna pro zjisteni rychlosti a stridy blikani bile LED v rezimu BLIKAC
volatile byte posledni_barva = 1;      // promenna pro zapamatovani posledni pouzite barvy LED - kvuli funkci blikace, kdy pak bude blikat stejna LED (1=cervena , 2= bila)

// ===================================================================
// definice rezimu blikani (cas rozsvicene LED a cas zhasnute LED v ms)


//  blik_index (rezim):   0      1      2      3      4      5      6      
int blik_cas_on [] = {    31,    63,   125,   250,   500,  1000 , 1000};  // cas v ms po ktery ma LED v prislusnem rezimu svitit
int blik_cas_off[] = {    94,   188,   375,   750,  1500,  3000 , 7000};  // cas v ms po ktery ma byt LED v prislusnem rezimu zhasnuta


//                                                                                             
//  blik_index                                     vyznam                                           
//---------------------------------------------------------------------------------            
//       0      =      31ms svetlo,  94ms tma, dohromady pedioda 0.125s =      8Hz / 25%   strida    
//       1      =      63ms svetlo, 188ms tma, dohromady pedioda  0.25s =      4Hz / 25%   strida    
//       2      =     125ms svetlo, 375ms tma, dohromady pedioda   0.5s =      2Hz / 25%   strida    
//       3      =     250ms svetlo, 750ms tma, dohromady pedioda     1s =      1Hz / 25%   strida    
//       4      =      0.5s svetlo,  1.5s tma, dohromady pedioda     2s =    0.5Hz / 25%   strida    
//       5      =        1s svetlo,    3s tma, dohromady pedioda     4s =   0.25Hz / 25%   strida    
//       6      =        1s svetlo,    7s tma, dohromady pedioda     8s =  0.125Hz / 12,5% strida  
// =================================================================================



 // zmena jasu LED neni provadena linearne, ale logaritmicky.
 //    (protoze je dobre znatelny rozdil v jasu mezi cisly 253 a 254, ale neni pozorovatelny zadny rozdil v jasech mezi cisly 10 a 11 )
const byte logaritmy[] = {254,253,252,249,243,233,215,181,117,0};   // 0 = maximalni jas LED ; 255 = zhasnute LED



// temhle podprogram je tu jen kvuli uspore mista ve FLASH pameti ATtiny25
// volanim podprogramu "pauza_30()" se proti volani funkce "delay(30)" usetri v tomto programu vice nez 100 bajtu pameti
void pauza_30(void)
  {
    delay(30);
  }



// ===================================================================
// rozsviceni cervene LED (PWM kanal A - pin PB0)
// parametrem je index pole logaritmy[]  ; 0 az 9 odpovida 10% az 100% jasu
void red_on(byte level_red)
  {
    OCR0A  = logaritmy[level_red];   // sirka PWM impulzu
    prave_sviti = 1;                 // prave sviti cervena LED
  }


// ===================================================================
// rozsviceni bile LED (PWM kanal B - pin PB1)
// parametrem je index pole logaritmy[]  ; 0 az 9 odpovida 10% az 100% jasu
void white_on(byte level_white)
  {
    OCR0B  = logaritmy[level_white];   // sirka PWM impulzu
    prave_sviti = 2;                   // prave sviti bila LED
  }


// ===================================================================
// zhasne obe LED
void obe_off(void)
  {
     OCR0A  = 255;          // PWM na cervene LED na sirku impulzu 0% (zhasne cervenou LED)
     OCR0B  = 255;          // PWM na bile LED na sirku impulzu 0% (zhasne bilou LED)
     prave_sviti = 0;       // prave nesviti zadna LED
     autosleep = 19169;     // pri zhasnuti obou LED se nastavi odpocet automatickeho uspani asi na 10 minut (1 cislo = asi 31ms)
  }





// ===================================================================
// bliknuti cervene LED na 50% na 0,2sek. (signalizace zamceni a odemceni tlacitek)
void blikred(void)
  {  
    red_on(4);     // rozsvit cervenou LED na 50%
    delay(200);    // chvili pockej
    obe_off();     // a pak zhasni obe LED
    delay(200);    // chvili pockej
  }




// ===================================================================
//  Rezim blikani LED podle nastavene frekvence s moznosti zmeny frekvence v 7 krocich od 8Hz do 0.125Hz (8x za sekundu az 1x za 8 sekund)
//  pro vyssi frekvence je strida napevno nastavena na 25% pri nizke frekvenci (perioda 8 sekund) je ale maximalni delka svitu 1 sekunda 
void rezimBLIK(void)
  {

    blikred();
    boolean odejdi = false;
    while ((PINB & 0b00011100) != 0b00011100)   // cekej, nez se uvolni vsechny tlacitka
      {
        pauza_30();    
      }



    while (odejdi == false)
      {

        if ( posledni_barva == 2 )    // kdyz byla naposledy pouzita bila barva ...
          {
            white_on(level_white);   // ... rozsviti bilou LED prednastavenym jasem      
          }
        else                         // kdyz byla naposledy pouzita cervena barva ...
          {
            red_on(level_red);       // ... rozsviti cervenou LED prednastavenym jasem      
          }
          
        starttime = millis();    // zapamatuje si cas rozsviceni LED
        while (millis() < starttime +  blik_cas_on[blik_index])  // cekej dokud neubehne pozadaovana doba pro sviceni
          {

            // behem teto doby testuj tlacitka kdyz bude nejake stisknute, okamzite ukonci vyckavaci smycku
            if ((PINB & 0b00011100) != 0b00011100)
              {
                break;
              }  
          }

        obe_off();               // zhasne obe barvy LED
        starttime = millis();    // zapamatuje si cas zhasnuti bile LED
        while (millis() < starttime + blik_cas_off[blik_index])  // cekej dokud neubehne pozadaovana doba pro zhasnutou LED
          {
            // behem teto doby testuj tlacitka kdyz bude nejake stisknute, okamzite ukonci vyckavaci smycku
            if ((PINB & 0b00011100) != 0b00011100)
              {
                odejdi = true; 
                break;
              }  
          }

        if (odejdi == true)  // kdyz byla prerusena vyckavaci pauza u zhasnute LED vyhodnoti se prave stisknute tlacitko
          {
            pauza_30();  // odruseni zakmitu tlacitek
            if ((PINB & 0b00011100) == 0b00001100)        // kdyz bylo stisknuto tlacitko_1 (PB4) nastavi se index v definici rezimu blikani o 1 nize
              {
                if (blik_index == 0 ) blik_index = 1;     // test podlezeni minimalniho indexu v definici blikani
                blik_index -- ;
                odejdi = false;                           // pokud bylo stisknuto tlacitko_1, neni to duvod ukoncit blikani 
              }

            if ((PINB & 0b00011100) == 0b00010100)        // kdyz bylo stisknuto tlacitko_2 (PB3) nastavi se index v definici rezimu blikani o 1 vyse
              {
                if (blik_index == 6 ) blik_index = 5;     // test prekroceni maximalniho indexu v definici blikani
                blik_index ++ ;
                odejdi = false;                           // pokud bylo stisknuto tlacitko_2, neni to duvod ukoncit blikani 
              }

            blikred();                                   // stisk tlacitka je signalizovan bliknutim cervene LED
            
            while ((PINB & 0b00011100) != 0b00011100)    //cekani na uvolneni vsech tlacitek
              {
                pauza_30();  
              }


            // pokud bylo stisknuto tlacitko_3 a nasledne bylo uvolneno, zustava nastavena znacka "odejdi" na TRUE, takze se blikani ukonci          
          
          }

          
      
      }
      
  }


// ===================================================================
// Podprogram pro uspani a opetovne probuzeni ATtiny25
// Opsano ze stranek :
//    http://www.avrfreaks.net/forum/attiny85-wake-pin-interrupt-not-working

void sleep()
  {
    MCUCR &= ~_BV(ISC01); MCUCR &= ~_BV(ISC00); // The low level of INT0 generates an interrupt request.
    ADCSRA &= ~_BV(ADEN); // ADC off
    MCUCR |= _BV(SM1); MCUCR &= ~_BV(SM0); // Select "Power-down" sleep mode
    
    sleep_enable(); // Enable sleep mode
    GIMSK |= _BV(INT0); // Enable External Interrupt INT0
    
    sei(); // Enable all interrupts
    sleep_cpu(); // GO TO SLEEP
  
    cli(); // Disable all interrupts
    GIMSK &= ~_BV(INT0); // Disable External Interrupt INT0
    
    sleep_disable(); // Disable sleep mode
    ADCSRA |= _BV(ADEN); // ADC on
    
    sei(); // Enable all interrupts
  }



// ===================================================================
void setup()
  { 
    delay(500);         // po zapnuti napajeni chvili nic nedelej (to je kvuli zakmitavani napajeni pri zasunovani baterii)

                         // nastaveni smeru portu a Pull-Upu
    DDRB  = 0b00000011;  // nastaveni smeru signalu na portu B ("1" = vystup ; "0" = vstup)
    PORTB = 0b00011111;  // na vsechny tlacitka (PB2, PB3 a PB4) Pull-Upy a oba vystupy (PB0, PB1) zhasnout

                         // nastaveni rezimu PWM
    TCCR0A = 0b11110011; // rezim FAST PWM; vystup se nastavi do '0' pri dosazeni pozadovaneho cisla
    TCCR0B = 0b00000010; // FAST PWM, delic zakladni frekvence na 4.6kHz (pro prescaler='010b')       

    obe_off();           // po zapnuti napajeni se obe LED zhasnou, a tim se spusti casovac automatickeho uspani na 10 minut
  }



// ===================================================================
// hlavni smycka pro test tlacitek
void loop()
  {

    // ===================================================================
    // stisk tlacitka_1 - obycejne, okamzite rozsviceni cervene LED
    
    if (prave_sviti != 1 && (PINB & 0b00011100) == 0b00001100)   // kdyz nesviti cervena LED a je sepnute pouze tlacitko_1 (PB4='0'; PB2 a PB3 = '1')
      {
        obe_off();                // Obe LED zhasnout
        red_on(level_red);        // Rozsvit cervenou LED pozadovanym jasem
        posledni_barva = 1;       // pamet posledni svitici barvy LED - kvuli funkci BLIKAC 

        starttime = millis();                            // zapamatovani casu rozsviceni kvuli funkci automatickeho zhasnuti pri delsim drzeni cerveneho tlacitka
        pauza_30();  // odruseni zakmitu tlacitka_1
        while ((PINB & 0b00011100) == 0b00001100)        // cekani na uvolneni cerveneho tlacitka
          {
            pauza_30();
          }      
        if (millis() > starttime + 1000)          // pokud bylo cervene tlacitko stisknute na delsi dobu, nez 1 sekunda, ...
          {
              obe_off();                          //  ... tak se po uvolneni cervena LED zase zhasne
          }
      }



    // ===================================================================
    // stisk tlacitka_1 pri rozsvicene cervene LED, cervenou LED zhasina
    
    if (prave_sviti == 1 && (PINB & 0b00011100) == 0b00001100 )   // kdyz cervena LED sviti a je sepnute pouze tlacitko_1 (PB4='0'; PB2 a PB3 = '1')
      {
        obe_off();                                       // cervenou LED zhasnout
        
        pauza_30();  // odruseni zakmitu tlacitka_1
        while ((PINB & 0b00011100) == 0b00001100)        // cekani na uvolneni cerveneho tlacitka (tlacitka_1)
          {}
        pauza_30(); 
      }




    // ===================================================================
    // stisk tlacitka_2 - obycejne rozsviceni bile LED (casovane - rozsviti se az po 0.7 sek drzeni tlacitka_2)
    
    if (prave_sviti != 2 && (PINB & 0b00011100) == 0b00010100)   // kdyz nesviti bila LED a je sepnute pouze tlacitko_2 (PB3='0'; PB2 a PB4 = '1')
      {
        pauza_30();  // odruseni zakmitu tlacitka_2

        starttime = millis();          //cas zacatku stisku tlacitka_2 do promenne
        while ((PINB & 0b00001100) == 0b00000100)   // dokud je sepnute tlacitko_2 (a ted uz nezalezi na stisku tlacitka_1 (PB4)) 
          {
            if (millis() > (starttime + (700)))  // kdyz je tlacitko_1 stisknuto asi 0.7 sekundy ...
              {

                 if ((PINB & 0b00010000) == 0b00000000)  // kdyz je po te 0.7 sekunde stisknute i tlacitko_1...
                   {
                     rezimBLIK();   // ... prejde se do rezimu blikani bile LED 
                   }
                 else               // kdyz po 0.7 sekunde neni stisknute tlacitko_1, znamena to obycejne rozsviceni bile LED
                   {                    
                     obe_off();                   // ... obe LED zhasnout
                     white_on(level_white);       // Rozsvit bilou LED pozadovanym jasem
                     posledni_barva = 2;              // pamet posledni svitici barvy LED - kvuli funkci BLIKAC 

                   }
              }
              pauza_30();
           }
      }






    // ===================================================================
    // stisk tlacitka_3 (pri zmacknutem tlacitku_3 je spusten rezim "NASTAVENI JASU")
    
    if ((PINB & 0b00011100) == 0b00011000 && prave_sviti != 0)          // je sepnute pouze tlacitko_3 (PB2='0'; PB3 a PB4 = '1'), a zaroven sviti nejaka LED
      {
        // vstup do rezimu nastaveni jasu
        pauza_30();  // odruseni zakmitu tlacitka_3        
        
        byla_zmena = 0;   // pomocna promenna pro zjisteni, jestli po uvolneni tlacitka_3 zhasnout (= 0) nebo nechat svitit (= 1) LED

        while ((PINB & 0b00000100) == 0b00000000)  // dokud je tlacitko_3 (PB2) sepnute (nezavisle na tom, jak jsou na tom ostatni tlacitka)
          {
             if  ((PINB & 0b00011100) == 0b00001000)  // a zaroven je stisknuto tlacitko_1 (PB2 + PB4) (zvysovani jasu)
               {
                 if (prave_sviti == 1)   // a kdyz prave sviti cervena
                   {
                     if (level_red == 9) level_red = 8;      // test na prekroceni rozsahu pri zvetsovani jasu (nikdy nebude vic, nez 9)
                     level_red ++;                           // zvetseni cerveneho jasu
                     red_on(level_red);                      // fyzicka aktualizace jasu cervene LED
                     byla_zmena = 1;                         // byla provedena zmena jasu, takze po uvolneni tlacitka_3 se nebude cervena LED zhasinat
                   }
                 if (prave_sviti == 2)   // kdyz prave sviti bila
                   {
                     if (level_white == 9) level_white = 8;  // test na prekroceni rozsahu pri zvetsovani jasu (nikdy nebude vic, nez 9)
                     level_white ++;                         // zvetseni bileho jasu
                     white_on(level_white);                  // fyzicka aktualizace jasu bile LED
                     byla_zmena = 1;                         // byla provedena zmena jasu, takze po uvolneni tlacitka_3 se nebude bila LED zhasinat                   
                   }
                 
                 delay(500);   // 0.5s pauza mezi jednotlivymi kroky pri zvysovani jasu
               }


             if  ((PINB & 0b00011100) == 0b00010000)  //kdyz je zaroven s tlacitkem_3 stisknuto tlacitko_2 (PB2 + PB3) (snizovani jasu)
               {
                 if (prave_sviti == 1)    // a kdyz  prave sviti cervena
                   {
                     if (level_red == 0) level_red = 1  ;    // test na "podlezeni" rozsahu pri snizovani jasu (nikdy nebude min, nez 0)
                     level_red --;                           // snizeni cerveneho jasu
                     red_on(level_red);                      // fyzicka aktualizace jasu cervene LED
                     byla_zmena = 1;                         // byla provedena zmena jasu, takze po uvolneni tlacitka_3 se nebude cervena LED zhasinat
                   }
                 if (prave_sviti == 2)    // a kdyz  prave sviti bila
                   {
                     if (level_white == 0) level_white = 1;  // test na "podlezeni" rozsahu pri snizovani jasu (nikdy nebude min, nez 0)
                     level_white --;                         // snizeni bileho jasu
                     white_on(level_white);                  // fyzicka aktualizace jasu bile LED
                     byla_zmena = 1;                         // byla provedena zmena jasu, takze po uvolneni tlacitka_3 se nebude bila LED zhasinat                    
                   }
                 
                 delay(500);   // 0.5s pauza mezi jednotlivymi kroky pri snizovani jasu
               }

           
          } // uvolneni tlacitka_3


         // zhasnuti LED pri normalnim rezimu (obycejne uvolneni tlacitka_3)
         if (byla_zmena == 0)     // pokud nebyla provedena zmena jasu, zhasnou se pri uvolneni tlacitka_3 obe LED
           {
             obe_off();            // zhasne obe LED
           }
         pauza_30();         // odruseni zakmitu uvolnovaneho tlacitka_3
      }



    // ===================================================================
    // rezim SLEEP - uzamceni tlacitek pri 2-sekundovem podrzeni tlacitka_3 kdyz nesviti zadna LED
    
      if (prave_sviti == 0 && ((PINB & 0b00011100) == 0b00011000))  // kdyz nesviti zadna LED a je stisknuto pouze tlacitko_3
        {

          obe_off();   // obe LED jsou sice uz zhasnute, takze navenek se nic nedeje. Dochazi ale k resetu casovace pro automaticky SLEEP rezim (opetovne nastaveni na 10 minut)
                    
          starttime = millis();  // okamzik ihned po probuzeni se zapamatuje
          
          while ((PINB & 0b00011100) == 0b00011000 && zamek==0)    // dokud je stisknuto tlaciko_3 ...
           {
             if (millis() > (starttime + 2000))  // test, jestli uz je zamykaci tlacitko drzeno alespon 2 sekundy
               {
                  blikred();    //  pokud ano, bliknuti cervene LED na 50% na 0.2sek. = bude se prechazet do SLEEP rezimu
                  blikred();    //  pri manualnim uspani je bliknuti dvojite

                  zamek = 1;    // pomocna promenna pro signalizaci, ze se bude uspavat  
               }
             
             pauza_30();
           }


          while ((PINB & 0b00011100) == 0b00011000)    // cekej, nez se po pozadavku na uspani uvolni tlacitko_3
           {
            pauza_30();
           }

             
          while (zamek == 1)        // dokud je platny pozadavek na uspani (dokud nebylo odemceno dlouhym stiskem, tak se po kazdem probuzeni zase ATtiny uspi)
            { 
              
              sleep();              // uspani do doby, nez dojde k sestupne hrane impulzu na INT0 (PB2)
              delay(100);
              
              starttime = millis();  // cas probuzeni se zapamatuje
              
              while ((PINB & 0b00011100) == 0b00011000 && zamek==1)    // dokud je stisknuto jenom tlaciko_3 ...
               {
                 if (millis() > (starttime + 2000))  // .. porovnava, jestli uz je probouzeci tlacitko_3 drzeno alespon 2 sekundy
                   {
                      blikred();    // pokud ano, bliknuti cervene LED na 50% na 0.2sek. = signalizace probuzeni
                      zamek = 0;    // zruseni pozadavku na uspani
                   }
                 
                 pauza_30();
               }

              // tady uz bylo vyhodnoceno, ze je probouzeci tlacitko_3 stisknuto dostatecne dlouho

              while ((PINB & 0b00011100) == 0b00011000)    // cekej, nez se tlacitko_3 uvolni (aby nedoslo pri delsim drzeni opet k uzamceni)
               {
                pauza_30();
               }
  
            }

        }


   // ===================================================================
   //  rezim AUTOSLEEP - automaticky prechod do SLEEP rezimu po 10 minutach zhasnutych LED
   
    if (prave_sviti == 0 && zamek == 0)  // kdyz nesviti zadna LED, ale celovka je v normalnim rezimu,
      {
        autosleep --;         // probiha odpocitavani do automatickeho uspani (1 pruchod = asi 0.03 sekundy) Promenna "autosleep" se nastavuje na 10 minut ve funkci "obe_off()"
        if (autosleep == 0)   // pri dopocitani autosleepu do 0 nastav SLEEP rezim
          {
                             // bliknuti cervenou LED pri automatickem prechodu do SLEEP rezimu
            // blikred();    // Ukazalo se, ze je to spise obtezujici signalizace, tak byla zrusena

            zamek = 1;       // ATtiny se bude uspavat


            while (zamek == 1)        // dokud je platny pozadavek na uspani (dokud nebylo odemceno dlouhym stiskem, tak se po kazdem probuzeni zase ATtiny uspi)
              { 

    
                 sleep();     // uspani do doby, nez dojde k sestupne hrane impulzu na INT0
                              
                 starttime = millis();  // cas probuzeni se zapamatuje
    
                 while ((PINB & 0b00011100) == 0b00011000  && zamek == 1)    // dokud je stisknuto tlaciko_3 ...
                   {
                     if (millis() > (starttime + 2000) && zamek == 1)  // ... testuj, jestli uz je probouzeci tlacitko_3 drzeno alespon 2 sekundy
                       {
                          blikred();    //  pokud ano, bliknuti cervene LED na 50% na 0.2sek. = probuzeni tlacitkem z automatickeho uspani
                          zamek = 0;    // ... a odblokuj zamek  
                       }
                     pauza_30();
                   }
               }
 
              // tady uz bylo vyhodnoceno, ze je probouzeci tlacitko_3 stisknuto dostatecne dlouho

              while ((PINB & 0b00011100) == 0b00011000)    // cekej, nez se tlacitko_3 uvolni (aby nedoslo pri delsim drzeni opet k uzamceni)
               {
                pauza_30();
               }

          }
      }

    pauza_30();   // pri kazdem pruchodu hlavni smyckou pauza 0.03 sekundy ( DULEZITE pro casovani autosleepu!) 
  }
 
// podprogram pro obslouzeni preruseni - (nic nedela, jen musi byt definovany)
ISR(INT0_vect)
    {

    }

