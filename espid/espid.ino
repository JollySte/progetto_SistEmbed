#define PWMMOTORE 16
#define VELOCITAMAX 255

int velocitaMotore = VELOCITAMAX;

//dati di configurazione canale pwm
const int pwmChannel = 0;
const int freq = 10000;
const int resolution = 8;

void setup(){

  pinMode(PWMMOTORE, OUTPUT);

  //funzioni di configurazione pin per pwm specifiche per esp32
  ledcSetup(pwmChannel, freq, resolution);  
  ledcAttachPin(PWMMOTORE, pwmChannel);

  
}


void loop() {
  
  ledcWrite(pwmChannel, velocitaMotore);   

  delay(500);


}
