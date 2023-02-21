#include <PID_v1.h>

#define PWMMOTORE 16
#define IN1 18
#define IN2 17
#define TRIGGER 26
#define ECHO 25
#define VELOCITAIDLE 230
#define VELOCITASUONO 0.034  //espressa in cm/us
#define DISTANZAMIN 3
#define DISTANZAMAX 30

/*
 * collegamenti al motor driver L293D: pin PWMMOTORE collegato al piedino "enable" che attiva i piedini I/O sul lato sinistro (in1, in2, out1, out2)
 * pin in1 e in2, collegati rispettivamente agli omonimi piedini del driver, determinano il senso di rotazione
 * pwm su enable per regolare la velocità del motore
 */

int velocitaMotore = 130;   //il motore inizia a girare con una pwm di circa 150


//dati di configurazione canale pwm
const int pwmChannel = 0;
const int freq = 30000;
const int resolution = 8;

//variabili di configurazione controller PID
double setpoint, input, output;
double Kp = 1, Ki = 2, Kd = 3;
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

void setup(){

  pinMode(PWMMOTORE, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);   //superfluo visto che non serve cambiare senso di rotazione
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(9600);

  PIDcontroller.SetMode(AUTOMATIC);

  //funzioni di configurazione pin per pwm specifiche per esp32
  ledcSetup(pwmChannel, freq, resolution);  
  ledcAttachPin(PWMMOTORE, pwmChannel);

  setpoint = DISTANZAMIN;
}


void loop() {

  //rotazione motore in senso antiorario
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  input = calcolaDistanza();
  
  //se non rileva un oggetto nel range del nastro, imposta il motore alla velocità idle...
  if(input > DISTANZAMAX){
    while(velocitaMotore < VELOCITAIDLE){   //...aumentandola gradualmente per non danneggiare l'aggancio motore-nastro
      velocitaMotore++; 
      ledcWrite(pwmChannel, velocitaMotore);  
      delay(10);
    }
    while(velocitaMotore > VELOCITAIDLE){   //...o diminuendola se il PID l'ha aumentata oltre il livello idle 
      velocitaMotore--; 
      ledcWrite(pwmChannel, velocitaMotore);  
      delay(10);
    }

  //se invece rileva un oggetto, la velocità del motore è affidata al controller PID
  }else{
   
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
