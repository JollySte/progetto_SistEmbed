#include <HC_SR04.h>
#include <PID_v1.h>
#include <TaskScheduler.h>


#define PWMMOTORE 16
#define IN1 18               //motore 1
#define IN2 17               //motore 2
#define TRIGGER 26
#define ECHO 25
#define FOTORES 35
#define VELOCITAIDLE 230
#define DISTANZAMIN 3
#define DISTANZAMAX 30
#define MAXFERMO 190         //si assume che il nastro inizi a muoversi poco oltre questo valore di pwm 
#define SOGLIALUCE 600       //la fotoresistenza assume valori superiori alla soglia quando un foro ci passa sopra
#define CHECKFERMO 4000     //soglia di tempo in ms per controllare che il nastro non sia fermo

/*
 * collegamenti al motor driver L293D: pin PWMMOTORE collegato al piedino "enable" che attiva i piedini I/O sul lato sinistro (in1, in2, out1, out2)
 * pin in1 e in2, collegati rispettivamente agli omonimi piedini del driver, fanno uscire l'alimentazione per i motori sui piedini out1 e out2
 * pwm su enable per regolare la velocità dei motori
 */

int velocitaMotore = MAXFERMO;   
int luce = 0;
long step1 = 0, step2 = 0;
double rpm = 0;
boolean aggiornato = false;
boolean rilevato = false, arrivato = false;
int tStart = 0, tStop = 0, tempoArrivo = 0;      //timestamps per tempo raggiungimento del target da parte dell'oggetto
int oldDist = 0;                                
long t1 = 0, t2 = 0;                           
double velocitaOggetto = 0;                     

Scheduler scheduler;

//costruttore libreria per sensore a ultrasuoni
HC_SR04<ECHO> uSensor(TRIGGER);

//dati di configurazione canale pwm
const int pwmChannel = 0;
const int freq = 30000;
const int resolution = 8;

//variabili di configurazione controller PID
double setpoint, input, output;
double Kp = 3, Ki = 2, Kd = 4;
PID PIDcontroller(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);



//calcola il numero di secondi per effettuare 1 giro del nastro tramite la differenza degli ultimi 2 tempi campionati (al momento 1/4 giro, essendoci 4 fori sul nastro per mandare luce alla fotoresistenza)
void calcolaRpmNastro(){
  double rps = 0;
  double tempo = 0;
  if((millis()-step2) < CHECKFERMO){   //se il tempo passato dall'ultimo superamento di soglia supera i 4 secondi si assume che il nastro sia fermo
    tempo = ((step2-step1)*4)/1000;  //tempo in secondi per 1 giro del nastro  
    if(tempo > 0)
      rps = 1/tempo;
    rpm = rps*60;
  }else{
   rpm = 0;
  }
}

//raggiunge la velocità idle...
void raggiungiIdle(){
   while(velocitaMotore < VELOCITAIDLE){   //...aumentandola gradualmente per non danneggiare l'aggancio motore-nastro
      velocitaMotore++; 
      ledcWrite(pwmChannel, velocitaMotore);  
    }
    while(velocitaMotore > VELOCITAIDLE){   //...o diminuendola se il PID l'ha aumentata oltre quel livello
      velocitaMotore--; 
      ledcWrite(pwmChannel, velocitaMotore);  
    }
}

//legge il valore della fotoresistenza e tiene aggiornati i timestamp degli ultimi 2 superamenti della soglia
void leggiLuce(){
  luce = analogRead(FOTORES);               
   if(luce >= SOGLIALUCE && !aggiornato){
      step1 = step2;
      step2 = millis();
      aggiornato = true;
   }
   
   if(luce < SOGLIALUCE)
      aggiornato = false;
}


void regolaVelocitaMotori(){
   //se non rileva un oggetto nel range del nastro, imposta i motori alla velocità idle
  if(input > DISTANZAMAX){   
    rilevato = false;
    arrivato = false;
    velocitaOggetto = 0;
    raggiungiIdle();

  //se ha raggiunto la destinazione, ferma il nastro e calcola il tempo di arrivo
  }else if(input <= DISTANZAMIN) {
    
    velocitaMotore = MAXFERMO; 
    if(rilevato && !arrivato){
      tStop = millis();
      tempoArrivo = (tStop - tStart)/1000;
      arrivato = true;
    }
  }

  //se invece rileva un oggetto, la velocità dei motori è affidata al controller PID
  else{
    if(!rilevato){        //salva il timestamp del momento in cui rileva un oggetto (con boolean di controllo per evitare campionamenti indesiderati) - per calcolo tempo di arrivo a destinazione
      tStart = millis();
      rilevato = true;
    }
    
    //calcola la velocità dell'oggetto in avvicinamento      
    int distOggetto;    
    if(oldDist > 0){
      distOggetto = (oldDist - input);
    }
    oldDist = input;      
    velocitaOggetto = abs(distOggetto/10.0);
  

    //regola pwm motori
    PIDcontroller.Compute();
    velocitaMotore = output;
    ledcWrite(pwmChannel, velocitaMotore);  
    
  }
}


void mostraInfo(){
  Serial.println("MISURAZIONI SISTEMA:");
  Serial.print("dist: ");
  Serial.print(input);
  Serial.print("--pwm: ");
  Serial.print(velocitaMotore);
  if(arrivato){
    Serial.print("--tempoArrivo: ");
    Serial.print(tempoArrivo);
    Serial.println(" s");
  }else{
    Serial.println("");
  }
  Serial.println("  ALTRI PARAMETRI:");
  Serial.print("  Luce: ");
  Serial.print(luce);
  Serial.print("--rpm: ");
  Serial.print(rpm); 
  Serial.print("--speed: ");
  Serial.print(velocitaOggetto);
  Serial.println(" cm/s");
}

//definizione dei task per lo scheduler

Task sensoreLuce(50*TASK_MILLISECOND, TASK_FOREVER, leggiLuce);
Task pidCalc(100*TASK_MILLISECOND, TASK_FOREVER, regolaVelocitaMotori);
Task velocitaNastro(300*TASK_MILLISECOND, TASK_FOREVER, calcolaRpmNastro);
Task stampaInfo(500*TASK_MILLISECOND, TASK_FOREVER, mostraInfo);


void setup(){

  pinMode(PWMMOTORE, OUTPUT); //pin "en1" del ponte h, alimentato in pwm
  pinMode(IN1, OUTPUT);       //pin "in1" del ponte h, attiva il motore 1
  pinMode(IN2, OUTPUT);       //pin "in2" del ponte h, attiva il motore 2
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(FOTORES, INPUT);
  Serial.begin(9600);

  //inizializzazione vari task
  scheduler.init();

  scheduler.addTask(sensoreLuce);
  sensoreLuce.enable();
  
  scheduler.addTask(pidCalc);
  pidCalc.enable();

  scheduler.addTask(velocitaNastro);
  velocitaNastro.enable();

  scheduler.addTask(stampaInfo);
  stampaInfo.enable();

  //inizializza il sensore a ultrasuoni in modalità asincrona (a interrupt)
  uSensor.beginAsync();  
  uSensor.startAsync(100000);

  setpoint = DISTANZAMIN;
  PIDcontroller.SetMode(AUTOMATIC);

  //funzioni di configurazione pwm specifiche per esp32
  ledcSetup(pwmChannel, freq, resolution);  
  ledcAttachPin(PWMMOTORE, pwmChannel);

}


void loop() {

  //rotazione motore 1 in senso antiorario e motore 2 in senso orario
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);

  //quando scatta l'interrupt salva il valore letto dal sensore, e fa partire un altro echo
  if (uSensor.isFinished()) {
    input = uSensor.getDist_cm();
    uSensor.startAsync(100000);
  }

  scheduler.execute();
  
}
