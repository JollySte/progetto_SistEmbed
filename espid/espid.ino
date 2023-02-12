#define PWMMOTORE 16
#define VELOCITAMAX 222

/*
 * collegamenti al motor driver L293D: pin PWMMOTORE collegato al piedino "enable" che attiva i piedini I/O sul lato sinistro (in1, in2, out1, out2)
 * pin in1 e in2, collegati rispettivamente agli omonimi piedini del driver, determinano il senso di rotazione
 * pwm su enable per regolare la velocità del motore
 */

const int in1 = 18;
const int in2 = 17;
int velocitaMotore = VELOCITAMAX;
boolean avvio = true;

//dati di configurazione canale pwm
const int pwmChannel = 0;
const int freq = 30000;
const int resolution = 8;

void setup(){

  pinMode(PWMMOTORE, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  //funzioni di configurazione pin per pwm specifiche per esp32
  ledcSetup(pwmChannel, freq, resolution);  
  ledcAttachPin(PWMMOTORE, pwmChannel);

  
}


void loop() {

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  //aumenta gradualmente la velocità per superare l'inerzia del nastro fermo senza danneggiare l'aggancio al motore 
  if(avvio){
    for(int i = 170; i<velocitaMotore; i++){
      ledcWrite(pwmChannel, i);   
      delay(100);
    }
    avvio = false;
  }
  ledcWrite(pwmChannel, velocitaMotore);   
  delay(500);
  
}
