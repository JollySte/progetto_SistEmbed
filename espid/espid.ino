#include <HC_SR04.h>
#include <PID_v1.h>

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

/*
 * collegamenti al motor driver L293D: pin PWMMOTORE collegato al piedino "enable" che attiva i piedini I/O sul lato sinistro (in1, in2, out1, out2)
 * pin in1 e in2, collegati rispettivamente agli omonimi piedini del driver, fanno uscire l'alimentazione per i motori sui piedini out1 e out2
 * pwm su enable per regolare la velocità dei motori
 */

int velocitaMotore = MAXFERMO;   
int luce = 0;
long step1 = 0, step2 = 0;
double tPeriodo = 0;
boolean fermo = true;
boolean rilevato = false, arrivato = false;
int tStart = 0, tStop = 0, tempoArrivo = 0;   //timestamps per tempo raggiungimento del target da parte dell'oggetto
int oldDist = 0;
int velocitaOggetto = 0;


//costruttore libreria per sensore a ultrasuoni
HC_SR04<ECHO> uSensor(TRIGGER);

//dati di configurazione canale pwm
const int pwmChannel = 0;
const int freq = 30000;
const int resolution = 8;

//variabili di configurazione controller PID
double setpoint, input, output;
double Kp = 2, Ki = 0.5, Kd = 3;
PID PIDcontroller(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);


//calcola il numero di secondi per effettuare 1 giro del nastro tramite la differenza degli ultimi 2 tempi campionati (al momento 1/4 giro, essendoci 4 fori sul nastro per mandare luce alla fotoresistenza)
double velocitaNastro(){
  double period = 0;
  step2 = millis();
  if(step1 > 0){
    period = ((step2-step1)*4)/1000;  
  }
  step1 = step2;
  return period;
}

//raggiunge la velocità idle...
void raggiungiIdle(){
   while(velocitaMotore < VELOCITAIDLE){   //...aumentandola gradualmente per non danneggiare l'aggancio motore-nastro
      velocitaMotore++; 
      ledcWrite(pwmChannel, velocitaMotore);  
      delay(10);
    }
    while(velocitaMotore > VELOCITAIDLE){   //...o diminuendola se il PID l'ha aumentata oltre quel livello
      velocitaMotore--; 
      ledcWrite(pwmChannel, velocitaMotore);  
      delay(10);
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
    Serial.println(tempoArrivo);
  }else{
    Serial.println("");
  }
  Serial.println("ALTRI PARAMETRI:");
  Serial.print("Luce: ");
  Serial.print(luce);
  Serial.print("--tGiro: ");
  Serial.print(tPeriodo); 
  Serial.print("--speed: ");
  Serial.print(velocitaOggetto);
  Serial.println(" cm/s");
}

void setup(){

  pinMode(PWMMOTORE, OUTPUT); //pin "en1" del ponte h, alimentato in pwm
  pinMode(IN1, OUTPUT);       //pin "in1" del ponte h, attiva il motore 1
  pinMode(IN2, OUTPUT);       //pin "in2" del ponte h, attiva il motore 2
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(FOTORES, INPUT);
  Serial.begin(9600);

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

  //velocità (in cm/s) dell'oggetto sul nastro misurata tramite differenza delle ultime 2 distanze 
  if(oldDist > 0){
    velocitaOggetto = (oldDist - input) * 5;   //l'intervallo di tempo considerato è 200 ms
  }
  oldDist = input;
  
  //se non rileva un oggetto nel range del nastro, imposta i motori alla velocità idle
  if(input > DISTANZAMAX){
    
    rilevato = false;
    arrivato = false;
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
    if(!rilevato){        //salva il timestamp del momento in cui rileva un oggetto (con boolean di controllo per evitare campionamenti indesiderati)
      tStart = millis();
      rilevato = true;
    }
    PIDcontroller.Compute();
    velocitaMotore = output;
    ledcWrite(pwmChannel, velocitaMotore);  
    
  }

  if(velocitaMotore <= MAXFERMO){
    fermo = true;
  }else{
    fermo = false;
  }

  luce = analogRead(FOTORES);
  if(!fermo){
   if(luce > SOGLIALUCE){
    tPeriodo = velocitaNastro();
   }
  }else{
    tPeriodo = 0;
  }

  mostraInfo();
  delay(369);

  
}
