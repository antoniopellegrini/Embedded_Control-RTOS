#include <Wire.h>
#include <string.h>
#define reply_len 64
#define global_delay 20
#define loop_delay 100
#define board_1_ID 2 // master
#define board_2_ID 3 //slave

int cmd = 0;


char reply_str[reply_len];
char reply_str_2[reply_len];

char reply;
//STATE 4 COASTING
int stato = 0;


void Send(int command){
  Serial.print("\nSending data ");
  
  Serial.println(command);
  Wire.beginTransmission(2); // transmit to device 
  Wire.write( command );              // sends one byte
  Wire.endTransmission();    // stop transmitting  
}

void Send_2( int board, int command){
  Serial.print("\nSending board ");
  Serial.print(board);  
  Serial.print(" : ");

  Serial.println(command);
  Wire.beginTransmission(board); // transmit to device 
  Wire.write( command );              // sends one byte
  Wire.endTransmission();    // stop transmitting  
}

void request(){
  //Serial.println("\nRequesting data:");

  Wire.requestFrom(2, reply_len);    // request 32 bytes from slave device #2
  //delay(10);
  int i = 0;
  Serial.print("Received: ");
  while (Wire.available()>0) { // slave may send less than requested

    

    reply_str[i] = Wire.read();

    //reply = Wire.read();      // receive a byte as character
    i=i+1;
    //Serial.print(reply);         // print the character
  }
  Serial.print(reply_str);
  Serial.println(".");
}

void request_2(int board){
  //Serial.println("\nRequesting data:");

  Wire.requestFrom(board, reply_len);    // request reply_len bytes from slave device #board
  //delay(10);
  int i = 0;
  Serial.print("Board: ");
  Serial.print(board);
  Serial.print(" : ");

  for(int i = 0; i < reply_len; i++ ){
      reply_str[i] = ' ';
  }


  while (Wire.available()>0) { // slave may send less than requested

    reply_str[i] = Wire.read();

    //reply = Wire.read();      // receive a byte as character
    i=i+1;
    //Serial.print(reply);         // print the character
  }
  Serial.print(reply_str);
  Serial.println(".");
}

void setup() {
  Serial.begin(9600);
  Wire.begin();  
  Serial.println("Setup end...");
}

void loop() {
  //Serial.print(".");

  //    delay(100);


  if (Serial.available() > 0){


    cmd = Serial.parseInt();
    
    switch(stato){
      case 0:
          
          Serial.print("\nstato:");
          Serial.println(stato);
          if (cmd == 0){ 
            request_2(board_1_ID);
            request_2(board_2_ID);  
  
            Serial.println(strcmp(reply_str, "[MPU] Wait for command"));
              if (strcmp(reply_str, "[MPU] Wait for command") == 0) {  // same strings    
                  stato = 1;
                  Serial.println("\n Insert command to send\n1: Recalibrate\n2: Start countdown");
                  break;
                }
            }
          break;

       case 1:

          Serial.println("/nInsert command \n1: Recalibrate\n2: Start countdown");
          if (cmd == 1){
            Serial.println("\n->Recalibrate");
            //Send(1);
            Send_2(board_1_ID,1);  //recalibrate
            Send_2(board_2_ID,1);  //recalibrate

            //'[MPU] Calibration done'

            stato = 0;
			
            break;
            }
          else if(cmd==2){
            Serial.println("\n->Start Countdown");

            //Send(2);
            Send_2(board_1_ID,2);  //countdown
            Send_2(board_2_ID,2);  //countdown


            stato = 3;
            //request();

            while(true){
              //request();
              request_2(board_1_ID);
              request_2(board_2_ID);

              delay(loop_delay);
            }   

            break;

            }
          break;

        case 3:
          
          while(true){
            request();
            }       
          
          break;
    }
    Serial.print("\nstato attuale :");
    Serial.println(stato);

    //delay(80);  
  }
  delay(global_delay);
}
