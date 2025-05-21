#include <Arduino.h>
/*Limiti minimi e massimi di frequenza e duty cycle*/
#define MIN_FREQ                    (250)         /* Minimum frequency (in mHz) */
#define MAX_FREQ                    (100000)      /* Maximum frequency (in mHz) */
#define MIN_DUTY                    (10)          /* Minimum duty cycle (in %) */
#define MAX_DUTY                    (90)          /* Maximum duty cycle (in %) */

/*Nanosecondi in un secondo (1 secondo = 10^9 nanosecondi)*/
#define NSEC_IN_SEC                 (1000000000)  /* Nanseconds in one second */

/*Tolleranze*/
#define TOLERANCE_FREQUENCY         (5)           /* Tolerance on frequency (per thousand) */
#define TOLERANCE_DUTY              (1)           /* Tolerance on duty cycle (percent) */

#define HUNDRED                     (100)         /* One hundred */
#define THOUSAND                    (1000)        /* One thousand */

/*PIN usati come ingresso e uscita*/
#define INPUT_PIN                   (23)          /* PIN used as input */
#define OUTPUT_PIN                  (53)          /* PIN used as output */


/*Defiizione degli stati della FSM come enumerato di valori*/
enum fsm {
  UNCOUPLED = 0,              /*UNCOUPLED: ancora non ho riconosciuto nemmeno un periodo valido*/
  COUPLING,                   /*COUPLING: ho riconosciuto un singolo periodo valido, sono in attesa del secondo*/
  COUPLED                     /*COUPLED: ho riconosciuto almeno due periodi validi, la frequenza è riconosciuta correttamente*/
};



/*Elenco di variabili statiche (comuni a tutte le funzioni)*/
static uint32_t frequency;    /*Frequenza da riconoscere (mHz) senza tolleranza*/
static uint32_t dutyCycle;    /*Duty cycle da riconoscere (%) senza tolleranza*/
static uint32_t periodMin;    /*Periodo minimo (us) da riconscere T_min*/
static uint32_t periodMax;    /*Periodo massimo (us) da riconscere T_max*/
static uint32_t tOnMin;       /*Larghezza d'impulso minima (us) da riconscere TON_min*/
static uint32_t tOnMax;       /*Larghezza d'impulso massima (us) da riconscere TON_max*/


/*Elenco delle funzioni statiche (accessibili solo da questo file .ino)*/
static void printFrequencyRange(bool bInvalid);
static void printDutyCycleRange(bool bInvalid);
static void printConfig(void);
static void configure(void);


/*------------------------------------------------------------------*/
/*Funzione setup                                                    */
/*Configura Arduino Due                                             */
/*Viene chiamata automaticamente da Arduino                         */
/*Configura la porta seriale, aspetta i parametri di configurazione */
/*da seriale, salva la configurazione, configura i due PIN di input */
/*e output, chiude la porta seriale                                 */
/*NOTA: questa funzione non va modificata                           */
/*------------------------------------------------------------------*/
void setup() {
  String tmp;
  unsigned long value;
  int bValid;
  
  /*Initialize serial and wait for port to open*/
  SerialUSB.begin(115200);
  SerialUSB.setTimeout(60000);
  while (!SerialUSB) {
    ; /*wait for serial port to connect. Needed for native USB*/
  }
  
  SerialUSB.println("Configuring Frequency Detector...");
  printFrequencyRange(false);
  bValid = 0;
  do {
    tmp = SerialUSB.readStringUntil('\n');
    if (tmp != NULL) {
      value = strtoul(tmp.c_str(), NULL, 10);
      if (value >= MIN_FREQ && value <= MAX_FREQ) {
        frequency = (uint32_t)value;
        bValid = 1;
      }
      else {
        printFrequencyRange(true);
      }
    }
  } while (bValid == 0);
  printDutyCycleRange(false);
  bValid = 0;
  do {
    tmp = SerialUSB.readStringUntil('\n');
    if (tmp != NULL) {
      value = strtoul(tmp.c_str(), NULL, 10);
      if (value >= MIN_DUTY && value <= MAX_DUTY) {
        dutyCycle = (uint32_t)value;
        bValid = 1;
      }
      else {
        printDutyCycleRange(true);
      }
    }
  } while (bValid == 0);
  
  configure();
  printConfig();
  //SerialUSB.end();


  pmc_set_writeprotect(false);               // Disattiva la protezione sui registri PMC
  pmc_enable_periph_clk(ID_TC1);             // Abilita il clock al TC0 Channel 1 (TC1 corrisponde a TC0_CH1)

  // Configura TC0, Channel 1 (TC1) in modalità waveform
  TC_Configure(TC0, 1,
               TC_CMR_WAVE |                 // Modalità Waveform
               TC_CMR_WAVSEL_UP_RC |         // Conta fino a RC
               TC_CMR_TCCLKS_TIMER_CLOCK4);  // Clock: MCK/128 = 656250Hz

  TC_SetRC(TC0, 1, 656250);                  // Periodo 1Hz
  TC_SetRA(TC0, 1, 328125);                  // 50% duty cycle

  // Imposta il comportamento dell'uscita TIOA1 (HIGH a RA, LOW a RC)
  TC0->TC_CHANNEL[1].TC_CMR |= TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR;

  TC_Start(TC0, 1);                          // Avvia il timer

  // Configura pin 53 (PB14) per funzione periferica B (TIOA1)
  PIOB->PIO_PDR |= PIO_PB14;                 // Disabilita PIO su PB14
  PIOB->PIO_ABSR |= PIO_PB14;                // Seleziona periferica B per PB14

}


/*------------------------------------------------------------------*/
/*Funzione loop                                                     */
/*Esegue il main loop di Arduino Due                                */
/*Viene chiamata automaticamente da Arduino                         */
/*Implementa la FSM per riconoscere la frequenza configurata        */
/*NOTA: questa funzione deve essere scritta da voi :)               */
/*------------------------------------------------------------------*/

void loop() {
  /*NOTA: per minimizzare il jitter del riconoscitore di frequenza, questa funzione non dovrebbe  */
  /*mai ritornare ad Arduino ma realizzare un ciclo infinito                                      */

  /*Acquisisco il tempo corrente (in us) e lo stato corrente dell'ingresso                        */
  /*A seconda dello stato della FSM effettuo una delle seguenti operazioni:                       */


  unsigned long lastRiseTime = 0;  // variabile per il tempo dell'ultimo rising edge
  unsigned long lastFallTime = 0;  // non l'ho usata 
  unsigned long actualTime = 0;

  bool actualPinState;  //inizializzo il pin corrente
  bool oldPinState = LOW;
  bool tOn;  //true = valido, false = invalido

  fsm state = UNCOUPLED;  // inizializzo lo stato della Macchina a Stati

  while (1) {
    actualTime = micros();                    // acquisisco il tempo corrente in microsecondi
    actualPinState = digitalRead(INPUT_PIN);  // controllo lo stato del pin di ingresso del pwm

    /*UNCOUPLED:  Se lo stato corrente dell'ingresso non è cambiato rispetto al precedente non      */
    /*            devo fare nulla                                                                   */
    /*            Se lo stato corrente è cambiato devo valutare se ho avuto un rising edge o un     */
    /*            falling edge. Sul falling edge devo controllare se il tempo dall'ultimo rising    */
    /*            edge (la larghezza d'impulso) è compreso fra TON_min e TON_max, se è vero allora  */
    /*            ho avuto un TON valido, altrimenti ho un TON invalido. Se ho un rising edge       */
    /*            allora controllo se il tempo dall'ultimo rising edge (il periodo) è compreso fra  */
    /*            T_min e T_max, se è vero e il TON precedente era valido allora vado nello stato   */
    /*            COUPLING, altrimenti ho avuto un TON invalido. Infine, se ho avuto un rising      */
    /*            edge, aggiorno il tempo dell'ultimo rising edge                                   */


    if (state == UNCOUPLED){
      //Se lo stato corrente è cambiato
      if (actualPinState != oldPinState){


        //se ho falling edge: old = HIGH & actual=LOW
        if (actualPinState == LOW){
          //tempo dall'ultimo rising edge (la larghezza d'impulso) è compreso fra TON_min e TON_max
          if ((actualTime - lastRiseTime) >= tOnMin && (actualTime - lastRiseTime) <= tOnMax){
            tOn = true;
          } else{
            tOn = false;
          }
        }


        //se ho rising edge: old=LOW & actual=HIGH
        else if (actualPinState == HIGH){
          //tempo dall'ultimo rising edge (il periodo) è compreso fra T_min e T_max
          if ((actualTime - lastRiseTime) >= periodMin && (actualTime - lastRiseTime) <= periodMax)){
            //e ton precedente valido
            if (tOn = true) {
              state = COUPLING;
            }
          }
          else {
            tOn = false;
          }
          //sul rising edge aggiorno il tempo dell'ultimo rising edge
          lastRiseTime = actualTime;
        }

        //aggiorno lo stato del pin
        oldPinState = actualPinState; 
      }
    }

    /*COUPLING:   Se lo stato corrente dell'ingresso non è cambiato rispetto al precedente devo     */
    /*            controllare di non aver superato il valore massimo di TON (se lo stato corrente è */
    /*            alto) oppure il valore massimo di T (se è basso). Se ho passato uno dei due       */
    /*            valori massimi ammessi allora dichiaro l'ultimo TON invalido e torno nello stato  */
    /*            UNCOUPLED                                                                         */
    /*            Se lo stato corrente è cambiato devo valutare se ho avuto un rising edge o un     */
    /*            falling edge. Sul falling edge devo controllare se il tempo dall'ultimo rising    */
    /*            edge (la larghezza d'impulso) è compreso fra TON_min e TON_max, se non è vero     */
    /*            allora ho avuto un TON invalido e torno nello stato UNCOUPLED. Se ho un rising    */
    /*            edge allora controllo se il tempo dall'ultimo rising edge (il periodo) è compreso */
    /*            fra T_min e T_max, se è vero allora vado nello stato COUPLED e accendo l'uscita,  */
    /*            altrimenti ho avuto un TON invalido e torno nello stato UNCOUPLED. Infine, se ho  */
    /*            avuto un rising edge, aggiorno il tempo dell'ultimo rising edge                   */

    if (state == COUPLING){
      //lo stato corrente dell'ingresso non è cambiato rispetto al precedente
      if (actualState == oldPinState){


        //stato corrente alto
        if (actualState == HIGH){
          //se supero valore massimo di tOn
          if (actualTime - lastRiseTime >= t0nMax){  //non ho usato lastFallTime
            tOn = false;
            state = UNCOUPLED;
          }
        }


        //stato corrente basso
        else if (actualState == LOW){
          //se supero valore massimo di T
          if (actualTime - lastRiseTime >= periodMax){
            tOn = false;
            state = UNCOUPLED;
          }
        }
      }


      //lo stato è cambiato
      else{


        //falling edge
        if (actualState == LOW){
          //tempo dall'ultimo rising edge (la larghezza d'impulso) NON è compreso fra TON_min e TON_max
          if ((actualTime - lastRiseTime) <= tOnMin || (actualTime - lastRiseTime) >= tOnMax){
            tOn = false;
            state = UNCOUPLED;
          }
        }

        //rising edge
        if (actualState == HIGH){
          //il tempo dall'ultimo rising edge (il periodo) è compreso fra T_min e T_max
          if ((actualTime - lastRiseTime) >= periodMin && (actualTime - lastRiseTime) <= periodMax)){
            state = COUPLED; // passo nello stato COUPLED
            digitalWrite(OUTPUT_PIN, HIGH); // accendo l'uscita
          }
          //altrimenti ho avuto un TON invalido
          else{
            tOn = false;
            state = UNCOUPLED;
          }

          //sul rising edge aggiorno il tempo dell'ultimo rising edge
          lastRiseTime = actualTime;
        }

        //aggiorno lo stato del pin
        oldPinState = actualPinState;
      }
    }

    /*COUPLED:    Se lo stato corrente dell'ingresso non è cambiato rispetto al precedente devo     */
    /*            controllare di non aver superato il valore massimo di TON (se lo stato corrente è */
    /*            alto) oppure il valore massimo di T (se è basso). Se ho passato uno dei due       */
    /*            valori massimi ammessi allora dichiaro l'ultimo TON invalido e torno nello stato  */
    /*            UNCOUPLED e spengo l'uscita                                                       */
    /*            Se lo stato corrente è cambiato devo valutare se ho avuto un rising edge o un     */
    /*            falling edge. Sul falling edge devo controllare se il tempo dall'ultimo rising    */
    /*            edge (la larghezza d'impulso) è compreso fra TON_min e TON_max, se non è vero     */
    /*            allora ho avuto un TON invalido, torno nello stato UNCOUPLED e spengo l'uscita.   */
    /*            Se ho un rising edge allora controllo se il tempo dall'ultimo rising edge (il     */
    /*            periodo) è  compreso fra T_min e T_max, se non è vero ho avuto un TON invalido,   */
    /*            torno nello stato UNCOUPLED e spengo l'uscita. Infine, se ho avuto un rising      */
    /*            edge, aggiorno il tempo dell'ultimo rising edge                                   */

    if (state == COUPLED){
      //Se lo stato corrente dell'ingresso non è cambiato rispetto al precedente
      if (actualPinState == oldPinState){


        //se lo stato corrente è alto
        if (actualState == HIGH){
          //se supero valore massimo di tOn
          if (actualTime - lastRiseTime >= t0nMax){  //non ho usato lastFallTime
            tOn = false;
            state = UNCOUPLED;
            digitalWrite(OUTPUT_PIN, LOW) //spengo uscita
          }
        }


        //se stato corrente basso
        else if (actualState == LOW){
          //se supero valore massimo di T
          if (actualTime - lastRiseTime >= periodMax){
            tOn = false;
            state = UNCOUPLED;
            digitalWrite(OUTPUT_PIN, LOW) //spengo uscita
          }
        }
      }


      //se lo stato è cambiato
      else{


        //falling edge
        if (actualState == LOW){
          //tempo dall'ultimo rising edge (la larghezza d'impulso) NON è compreso fra TON_min e TON_max
          if ((actualTime - lastRiseTime) <= tOnMin || (actualTime - lastRiseTime) >= tOnMax){
            tOn = false;
            state = UNCOUPLED;
            digitalWrite(OUTPUT_PIN, LOW) //spengo uscita
          }
        }


        //rising edge
        if (actualState == HIGH){
          //tempo dall'ultimo rising edge (la larghezza d'impulso) NON è compreso fra TON_min e TON_max
          if ((actualTime - lastRiseTime) <= tOnMin || (actualTime - lastRiseTime) >= tOnMax){
            tOn = false;
            state = UNCOUPLED;
            digitalWrite(OUTPUT_PIN, LOW) //spengo uscita
          }

          //sul rising edge aggiorno il tempo dell'ultimo rising edge
          lastRiseTime = actualTime;
        }

        //aggiorno lo stato del pin
        oldPinState = actualPinState;
      }
    }
  }
}

/*------------------------------------------------------------------*/
/*Funzione printFrequencyRange                                      */
/*Stampa il range ammesso per la prequenza                          */
/*NOTA: questa funzione non va modificata                           */
/*------------------------------------------------------------------*/
static void printFrequencyRange(bool bInvalid) {
  SerialUSB.print("  ");
  if (bInvalid != false) {
    SerialUSB.print("Invalid value. ");
  }
  SerialUSB.print("Provide frequency (mHz) [");
  SerialUSB.print(MIN_FREQ, DEC);
  SerialUSB.print("-");
  SerialUSB.print(MAX_FREQ, DEC);
  SerialUSB.println("]");
}


/*------------------------------------------------------------------*/
/*Funzione printDutyCycleRange                                      */
/*Stampa il range ammesso per il duty cycle                         */
/*NOTA: questa funzione non va modificata                           */
/*------------------------------------------------------------------*/
static void printDutyCycleRange(bool bInvalid) {
  SerialUSB.print("  ");
  if (bInvalid != false) {
    SerialUSB.print("Invalid value. ");
  }
  SerialUSB.print("Provide duty cycle (%) [");
  SerialUSB.print(MIN_DUTY, DEC);
  SerialUSB.print("-");
  SerialUSB.print(MAX_DUTY, DEC);
  SerialUSB.println("]");
}


/*------------------------------------------------------------------*/
/*Funzione printConfig                                              */
/*Stampa la configurazione ricevuta (frequenza e duty cycle)        */
/*NOTA: questa funzione non va modificata                           */
/*------------------------------------------------------------------*/
static void printConfig(void) {
  SerialUSB.println();
  SerialUSB.println("Frequency Detector configuration");
  SerialUSB.print("  Period: [");
  SerialUSB.print(periodMin, DEC);
  SerialUSB.print(" - ");
  SerialUSB.print(periodMax, DEC);
  SerialUSB.println("] us");
  SerialUSB.print("  T_ON: [");
  SerialUSB.print(tOnMin, DEC);
  SerialUSB.print(" - ");
  SerialUSB.print(tOnMax, DEC);
  SerialUSB.println("] us");
  SerialUSB.print("  Input PIN: D");
  SerialUSB.println(INPUT_PIN, DEC);
  SerialUSB.print("  Output PIN: D");
  SerialUSB.println(OUTPUT_PIN, DEC);
}


/*------------------------------------------------------------------*/
/*Funzione configure                                                */
/*Calcola i valori di periodo minimo e massimo (T_min e T_max) in   */
/*microsecondi (us) e i valori di larghezz d'impulso minima e       */
/*massima (TON_min e TON_max) in microsecondi (us)                  */
/*Configura i due PIN INPUT_PIN e OUTPUT_PIN come ingresso e uscita */
/*NOTA: questa funzione deve essere scritta da voi :)               */
/*------------------------------------------------------------------*/
static void configure(void) {
  uint32_t freq;
  
  /*Calcoliamo il periodo minimo e massimo*/
  /*T_min = 1 / f_max*/
  /*T_max = 1 / f_min*/
  /*NOTA: la frequenza è in mHz (quindi andrebbe divisa per 1000), il periodo lo vogliamo in microsecondi (quindi lo vorremmo moltiplicare per 10^6)*/
  /*Per minimizzare l'errore numerico possiamo fare:*/
  /*    T(us) = 10^9(ns) / f(mHz)     */
  freq = (((THOUSAND + TOLERANCE_FREQUENCY) * frequency) + (THOUSAND - 1)) / THOUSAND;
  periodMin = (NSEC_IN_SEC) / freq;
  freq = ((THOUSAND - TOLERANCE_FREQUENCY) * frequency) / THOUSAND;
  periodMax = (NSEC_IN_SEC + (freq - 1)) / freq;

  /*Calcoliamo la larghezza d'impulso minima e massima*/
  /*TON_min = T_min * d_min / 100*/
  /*TON_max = T_max * d_max / 100*/
  tOnMin = ((dutyCycle - TOLERANCE_DUTY) * periodMin) / HUNDRED;
  tOnMax = (((dutyCycle + TOLERANCE_DUTY) * periodMax) + (HUNDRED - 1)) / HUNDRED;

  /*Configuriamo il piedino INPUT_PIN come ingresso e il piedino OUTPUT_PIN come uscita*/
  pinMode(INPUT_PIN, INPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}
