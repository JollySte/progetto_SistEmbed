#include <PID_v1.h>

#define PWMMOTORE 16
#define IN1 18               //motore 1
#define IN2 17               //motore 2
#define TRIGGER 26
#define ECHO 25
#define VELOCITAIDLE 230
#define VELOCITASUONO 0.034  //espressa in cm/us
#define DISTANZAMIN 3
#define DISTANZAMAX 30
#define MAXFERMO 190         //si assume che il nastro inizi a muoversi poco oltre questo valore di pwm 

/*
 * collegamenti al motor driver L293D: pin PWMMOTORE collegato al piedino "enable" che attiva i piedini I/O sul lato sinistro (in1, in2, out1, out2)
 * pin in1 e in2, collegati rispettivamente agli omonimi piedini del driver, fanno uscire l'alimentazione per i motori sui piedini out1 e out2
 * pwm su enable per regolare la velocità dei motori
 */

int velocitaMotore = MAXFERMO;   


//dati di configurazione canale pwm
const int pwmChannel = 0;
const int freq = 30000;
const int resolution = 8;

//variabili di configurazione controller PID
double setpoint, input, output;
double Kp = 2, Ki = 0.5, Kd = 3;
PID PIDcontroller(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);


double calcolaDistanza(){
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  long durata = pulseIn(ECHO, HIGH);
  long distanza = durata * VELOCITASUONO / 2;
  return distanza;
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


void setup(){

  pinMode(PWMMOTORE, OUTPUT); //pin "en1" del ponte h, alimentato in pwm
  pinMode(IN1, OUTPUT);       //pin "in1" del ponte h, attiva il motore 1
  pinMode(IN2, OUTPUT);       //pin "in2" del ponte h, attiva il motore 2
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(9600);

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
  
  input = calcolaDistanza();
  
  //se non rileva un oggetto nel range del nastro, imposta i motori alla velocità idle
  if(input > DISTANZAMAX){
    
    raggiungiIdle();

  //se ha raggiunto la destinazione, ferma il nastro
  }else if(input <= DISTANZAMIN) {
    
    velocitaMotore = MAXFERMO; 
  }

  //se invece rileva un oggetto, la velocità dei motori è affidata al controller PID
  else{
   
    PIDcontroller.Compute();
    velocitaMotore = output;
    ledcWrite(pwmChannel, velocitaMotore);  
    
  }

  Serial.print("dist: ");
  Serial.print(input);
  Serial.print("--pwm: ");
  Serial.println(velocitaMotore);
  delay(200);

  
}
