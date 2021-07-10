#include <Servo.h>

#define debug 0

#define initial_delay 10000
#define margin 140 // margine valori tra stessa scheda
#define outer_margin 40 // margine valori tra 2 schede

#define B1_P1_analog A0
#define B1_P2_analog A1
#define B1_P1_digital 2
#define B1_P2_digital 3
#define B1_alive 4

#define B2_P1_analog A2
#define B2_P2_analog A3
#define B2_P1_digital 5
#define B2_P2_digital 8
#define B2_alive 7

#define motor_pin 6
#define servo_pin 9
#define ABORT_Pin 10 // NOT used YET
#define test_pin 11 // NOT used YET

#define BAUD_RATE 9600

void abort(bool value){ // 1 == alive, 0 == abort;
  digitalWrite(ABORT_Pin,value);
}

void motor(bool value){
  if (value){
    analogWrite(motor_pin,20);  
  }else{
    analogWrite(motor_pin,0);  
  }
}


Servo myservo; 
int pos = 0;  
float value = 0.0;
int i=0;
int soglia = 30;

int analog_result = 0;
bool digital_result = 0;

int A1B1 = 1, A2B1 = 2, A1B2 = 3, A2B2 = 4; // last analog boards reading , INIT WITH DIFFERENT VALUES
bool D1B1 = 0, D2B1 = 1, D1B2 = 0, D2B2 = 1;
bool alive1=0,alive2=0;

void setup() {
  
  Serial.begin(BAUD_RATE);
  Serial.println("Starting Setup...");

//  pinMode(test_pin, OUTPUT); 
  pinMode(motor_pin, OUTPUT);  
  pinMode(ABORT_Pin, OUTPUT);
  pinMode(B1_alive,INPUT_PULLUP); // TODO: DOBBIAMO VERIFICARE se pullup adatto
  pinMode(B2_alive,INPUT_PULLUP);
  pinMode(B1_P1_digital,INPUT_PULLUP);
  pinMode(B1_P2_digital,INPUT_PULLUP);
  pinMode(B2_P1_digital,INPUT_PULLUP);
  pinMode(B2_P2_digital,INPUT_PULLUP);  
  abort(0);
  
  myservo.attach(servo_pin); 

  motor(false);  
  myservo.write(90);

  alive1 = !digitalRead(B1_alive); 
  alive2 = !digitalRead(B2_alive); 

  while(alive1==false){
    Serial.println("waiting for board alive");
    alive1 = !digitalRead(B1_alive); 
    alive2 = !digitalRead(B2_alive); 
    //delay(10);
  }


  //delay(1000);
    
  // Serial.print("servo at position: ");
  // Serial.println(myservo.read());  
  // Serial.print("Waiting ");
  // Serial.print(initial_delay/1000);
  // Serial.println(" seconds for board setup ");
  

  // Serial.println("end setup");

}




int READ_board_values(){ // read values from boards and return alive status

  /* ERROR TABLE
    0 -> All boards are alive
    1 -> Board 1 alive
    2 -> Board 2 alive
    3 -> No board alive
  */
  
  if (debug){
    Serial.println("  [!] Reading from  Boards");
  }
    
  A1B1 = analogRead(B1_P1_analog);
  A2B1 = analogRead(B1_P2_analog);
  D1B1 = digitalRead(B1_P1_digital);
  D2B1 = digitalRead(B1_P2_digital);
  
  alive1 = !digitalRead(B1_alive); 
  
  // Serial.println("reading from  B2");
  A1B2 = analogRead(B2_P1_analog);
  A2B2 = analogRead(B2_P2_analog);
  D1B2 = digitalRead(B2_P1_digital);
  D2B2 = digitalRead(B2_P2_digital);
  
  //alive2 = 0;
  alive2 = !digitalRead(B2_alive);// <=============================================== RESET for board 2

  if( alive1 == HIGH && alive2 == HIGH  ){
     
    if (debug){
      Serial.print("Both Boards alive");
    }
    return 0;

  }else if(alive1 == HIGH && alive2 == LOW){

    Serial.print("[WB2]");
    
    if (debug){
      Serial.print("only 1st Boards alive");
    }
    return 1;

  }else if(alive1 == LOW && alive2 == HIGH){

     Serial.print("[WB1]");

    
    if (debug){
      Serial.print("only 2nd Boards alive");
    }
    return 2;
  }else if(alive1 == LOW && alive2 == LOW){
    if (debug){
      Serial.print("NO Board alive");
    }
    return 3;
  }
  
}



int compare_values(int alive_status){  // compare values from different boards

  /* (ALIVE error table)
    0 -> All boards are alive
    1 -> Board 1  alive
    2 -> Board 2  alive
    3 -> No board alive
  */

  
  /* COMPARE ERROR TABLE
    0 -> All values are ok
    1 -> outer margin not respected ( boards value mismatched )
    2 -> margin not respected ( IN - boards value mismatched )
  */

  if (debug){
    Serial.println("  [!] Comparing values from  Boards");
  }

  if (alive_status == 0){ // all alive
    if( abs(A1B1 - A1B2) < margin && abs(A1B2 - A2B2) < margin  && D1B1^D1B2==0  )
    {
      //if( abs(A1B1 - A1B2) < outer_margin){ 
      if( ( abs(A1B1 - A2B1) - abs(A1B2 - A2B2)) < outer_margin){   // TODO: NOT TESTED YET
        return 0;
      }else{
        return 1;        
      }
    }else{
      return 2;
    }
  }

  if (alive_status == 1){ // board 2 not alive, check board 1 ONLY
    if( abs(A1B1 - A2B1) < margin  && D1B2^D2B2==0  ){
        return 0;      
      }
    else{
      return 2;
    }
  }

  if (alive_status == 2){ // board 1 not alive, check board 2 ONLY
    if( abs(A1B2 - A2B2) < margin  && D1B1^D2B1==0  ){
        return 0;      
      }
    else{
      return 2;
    }
  }
  
}

void loop() {

  // Serial.println("\n\n");

  int alive_status = READ_board_values();

  //alive_status = 1; // OVERRIDE reading from board 1 only   <<< ====================||
  
  int compare_status = compare_values(alive_status);
  bool skip = 1;
  
  if(alive_status == 0 && compare_status == 0){ //both boards value ok
    
    // Serial.println("all Values are ok, Calculating output value ");
    analog_result = (A1B1 + A2B1 + A1B2 + A2B2)/4 ; 
    digital_result = D1B1;
    
  }else if(alive_status == 1 && compare_status == 0){

    analog_result = (A1B1 + A2B1)/2 ;
    digital_result = D1B1;

  }else if(alive_status == 2 && compare_status == 0){

    analog_result = (A1B2 + A2B2)/2 ;
    digital_result = D1B2;
    
  }
    else {

     Serial.print("[E!] A_S = ");
     Serial.print(alive_status);

     Serial.print(" C_S = ");
     Serial.print(compare_status);
     Serial.print(" ");
     
    skip = 0;
  
  }
    
  value = ((float)analog_result )/12.0; // 12 xk 1024/90
  
  if(!D1B1){
      value = 90.0 + value ;
      
    }else{
      value = 90.0 - value  ;
      
    }


  if (true){

    // -------- B1 ---------
    Serial.print("D1B1: ");
    Serial.print(D1B1);
    Serial.print(" , D2B1: ");
    Serial.print(D2B1);
       
    Serial.print(" , A1B1: ");
    Serial.print(A1B1);
    Serial.print(" , A2B1: ");
    Serial.print(A2B1);
    Serial.print(" , alive1: ");
    Serial.print(alive1);

    // -------- B2 ---------
    Serial.print(" , | D1B2: ");
    Serial.print(D1B2);
    Serial.print(" , D2B2: ");
    Serial.print(D2B2);
    
    Serial.print(" , A1B2: ");
    Serial.print(A1B2);
    Serial.print(" , A2B2: ");
    Serial.print(A2B2);
    Serial.print(" , alive2: ");
    Serial.print(alive2);
    Serial.print(" , Servo: ");
    Serial.print(value);
    Serial.println("");
  }



  if (skip){
       
    myservo.write(value);

    motor(true);
    
   
  }else{
    motor(false);

  }
    
  delay(10);

}
