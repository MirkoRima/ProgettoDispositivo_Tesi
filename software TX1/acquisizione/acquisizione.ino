// I DELAY DI ARDUINO VANNO SETTATI PER SINCRONIZZARE PYTHON

int BUTTON = 2  ;           // pin di input dove è collegato il pulsante  
int  val = 0; 
int  val2 = 0; // si userà val per conservare lo stato del pin di input del bottone2 
int ledPin =  13;      // the number of the LED pin  
int BUTTON2 = 7  ; 

boolean stato = false;

int triggerPort = 9;
int echoPort = 10;

void setup() {  
 
  pinMode(BUTTON, INPUT);
  pinMode(BUTTON2, INPUT); 
  digitalWrite(BUTTON, LOW);// imposta il pin digitale come input  
  digitalWrite(BUTTON2, HIGH);// imposta il pin digitale come input 
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  
  //impostazione dei pin
  pinMode(triggerPort, OUTPUT);
  pinMode(echoPort, INPUT);
  
  Serial.begin(9600);
  //Serial.println("INIZIO");
}  
  
void loop() { 
 
 //cercare di sincronizzare JETSON-ARDUINO, se Jetson lavora Arduino va in pausa
  boolean processa = true;
  
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'S') {
        while(processa){
            if (Serial.available()) {
              char c = Serial.read();
              if (c == 'G') {
                processa = false;
             }
           }
        }
    }  
       
  }
 
  
  val = digitalRead(BUTTON);  // legge il valore dell'input del primo botton e lo conserva  
  val2 = digitalRead(BUTTON2);  // legge il valore dell'input 2 bottone e lo conserva 
  // controlla che l'input sia LOW (pulsante premuto)  
  if (val == HIGH) {
    stato = not(stato);
    Serial.println("STOP");
  }
  if (stato) { 
   /* Serial.println ();
    Serial.println ();
    Serial.println("acceso"); */ //se si vuole vedere a monitor se è acceso
    while (distanza() <= 60){
      Serial.println("ON");
      // LED on:
      digitalWrite(ledPin, HIGH);
      delay(500); //ritarda la visualizzazione del sensore (se si vuole visualizzare immediatamente togliere delay()) 
    }
   Serial.println("OFF"); 
    // LED off:
    digitalWrite(ledPin, LOW);
    delay(500);
       
  }  
  else {  
    /*Serial.println ();
    Serial.println ();
    Serial.println("spento"); */ // rimuovere i commenti se si vuole vedere a monitor se è spento
    delay(500);  
  }  
  // delay(1000);

  if (val2 == LOW) {
    Serial.println("SCATTA");
    delay(500);
  }
       
}  //graffa del "LOOP"

long distanza() // programmazione sensore sfr-05
{
  //porta bassa l'uscita del trigger
  digitalWrite( triggerPort, LOW);
  delayMicroseconds(2);
  //invia un impulso di 10microsec su trigger
  digitalWrite(triggerPort, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPort,LOW);
  // prende in ingresso il segnale nella porta echo e tramite un calcolo trova distanza (r)
  long duration = pulseIn( echoPort, HIGH); 
  long r = 0.034 * duration / 2;
  return r;   
}

