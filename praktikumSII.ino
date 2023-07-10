//deklarasi pin
uint8_t check     = 2;  //PD2
uint8_t echo      = 3;  //PD3
uint8_t trig      = 4;  //PD4
uint8_t alarm_led = A0; //PC0
uint8_t run_led   = A1; //PC1
uint8_t buzzer    = A5; //PC2
uint8_t ll        = A3; //PC3
uint8_t hh        = A4; //PC4
//data
volatile bool failsafe = 0;
bool ll_stat;
bool hh_stat;
double jarak, durasi;
float kaldist;
float averaged_filter;
float volume;
int runled_state = LOW;
unsigned long previousMillis = 0;
const int delay_start = 500;
const long interval = 200;
const float uurv    = 5; //hhh
const float urv     = 7; //hh
const float lrv   = 31; //ll
const float llrv    = 45; //lll
const float zero  = 7.5;

void setup() {
  Serial.begin(9600); 
  DDRC = 0xFF;
  DDRD = 0b11110111;
  PORTD = 0b00001000; //input pullup pin check
  attachInterrupt(digitalPinToInterrupt(check), cable_disconnect, RISING);  
}

void cable_disconnect() {
  digitalWrite(alarm_led, HIGH);
  digitalWrite(run_led, LOW);
  digitalWrite(buzzer, HIGH);
  failsafe = 1;
}

float kalman(float U){
  static const double R = 35;
  static const double H = 1.00;
  static double Q = 10;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P*H/(H*P*H+R);
  U_hat += + K*(U-H*U_hat);
  P = (1-K*H)*P+Q;
  return U_hat;
}

float moving_average(float KL){
  uint8_t index = 0;
  uint8_t window_size = 5;
  float sum = 0;
  int readings[window_size];
  float averaged = 0;
  sum = sum - readings[index];
  readings[index] = KL;
  sum = sum + KL;
  index = index + 1;
  if(index >= window_size){
    index = 0;
  }
  averaged = sum / window_size;
  return averaged;
}

void usonic_transmit() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
}

void logika(){
  if(millis() >= delay_start){
    if(averaged_filter <= urv){
      PORTC = 0b00010000; //0 0 0 hh ll buz run alrm
      hh_stat = 1;
      ll_stat = 0;
      if(kaldist < uurv){
        PORTC = 0b00010001; //0 0 0 hh ll buz run alrm
      }
    }
    else if(averaged_filter >= lrv){
      PORTC = 0b00001000; //0 0 0 hh ll buz run alrm
      hh_stat = 0;
      ll_stat = 1;
      if(kaldist > llrv){
        PORTC = 0b00001001; //0 0 0 hh ll buz run alrm
      }
    }
    else{
      PORTC = 0x00;
      hh_stat = 0;
      ll_stat = 0;
    }
  }
}

void debug(){
  Serial.println(failsafe);
  Serial.print("jarak (RAW): ");
  Serial.println(jarak);
  Serial.print("Jarak (KALMAN): ");
  Serial.println(kaldist);
  Serial.print("Jarak (AVRG) : ");
  Serial.println(averaged_filter);
}

void send_data(){
  Serial.print(jarak);
  Serial.print(";");
  Serial.print(kaldist);
  Serial.print(";");
  Serial.print(hh_stat);
  Serial.print(";");
  Serial.println(ll_stat);
}

void loop() {
  PORTC = 0x00;
  switch(failsafe){
    case 0:
      usonic_transmit();
      durasi = pulseIn(echo, HIGH);
      jarak = durasi*0.034/2;
      jarak = jarak - zero;
      kaldist = kalman(jarak);
      averaged_filter = moving_average(kaldist);
      logika();
      //led run blink
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        if (runled_state == LOW) {
          runled_state = HIGH;
        } else {
          runled_state = LOW;
        }
        digitalWrite(run_led, runled_state);
      }
      break;
    case 1:
      PORTC = 0b00000101; //0 0 0 hh ll buz run alrm
      break;
  }
//  debug();  
  send_data();
  delay(50);
  
}
