//#include <HardwareSerial.h>
#include <Roomba.h>


#define RXD2 16
#define TXD2 17
#define WAKEPIN 4
#define ButtonPin 33

Roomba roomba=Roomba();   // Forced this to use Serial2 and 115200 Baud
int iLeftVelocity = 0;
int iRightVelocity = 0;
bool Roomba_Started = false;
byte status[10];
byte encoder[4];
long LeftEncoder = 0;
long RightEncoder = 0;
int cmdStop = 173;
int cmdReset = 7;



// Arduino setup function. Runs in CPU 1
void setup() {

  pinMode(ButtonPin, INPUT_PULLUP);

  //Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial2.begin(115200);

  pinMode(WAKEPIN, OUTPUT);
  digitalWrite(WAKEPIN, HIGH);
  Serial.begin(57600);
  //WakeUpRoomba();
}

// Arduino loop function. Runs in CPU 1
void loop() {



  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

 // if (digitalRead(ButtonPin) == LOW) 
 //   ButtonPressed();
      
  GetSensors();
  
  ReadSerial();


     // Need to check sensors to prevent collisions etc.
  if (iLeftVelocity != 0  || iRightVelocity != 0)
    roomba.driveDirect(iLeftVelocity, iRightVelocity);
  else
    roomba.driveDirect(0 ,0);

  delay(5);
}

void WakeUpRoomba()
{
      Serial.println("Waking Roomba");
      digitalWrite(WAKEPIN, LOW);
      delay(100);
      digitalWrite(WAKEPIN,HIGH);

      roomba.start();
      delay(50);
      roomba.fullMode();
      Serial.println("FullMode");
      roomba.driveDirect(0, 0);
     roomba.digitLedsRaw( 9,  9,  9,  9);

}

void PowerDown()
{
    Serial.println("Power Down");
    Serial2.write(cmdStop);
    roomba.power();     // Power down.
  //  roomba.digitLedsRaw(uint8_t 0, uint8_t 0, uint8_t 0, uint8_t 0)
}

void ButtonPressed()
{
      delay(150);   // debounce button press
      // Once connected, wake up Roomba
      if (!Roomba_Started)
        {

          WakeUpRoomba();
          Roomba_Started = true;

        } else {
            if (Roomba_Started)
            {
              PowerDown();
              Roomba_Started  = false;
            }
        }

}
void GetSensors()
{
    for (int i=0; i<2; i++)
      roomba.getSensors(i+43, &encoder[i*2], 2);

    
}

void ReadSerial()
{
  String cmdString;
  // Read commands from Serial port, and execute.
  if (Serial.available()) {
    cmdString = Serial.readStringUntil(0x0d);   // read all until newline.
    if (cmdString.length() > 0)
      ProcessCmd(cmdString);
  }
}

void ProcessCmd(String cmd)
{
    char c = cmd[0];

    switch (c)
    {
        case 's':
          iLeftVelocity = 0;
          iRightVelocity = 0;
          roomba.driveDirect(0, 0);
          break;
        case 'p':
          roomba.power();
          break;
        case 'm':
          DriveCmd(cmd.substring(1));
          break;
        
        case 'e':
          ReadEncoders();
          break;
        case 'b':
          ButtonPressed();
          break;
        case 'r': // reset
          reset();
          break;
    }
}

void reset()
{
    Serial2.write(cmdReset);
    while (Serial2.available()) {
        Serial.write(Serial2.read());
    }
}

void ReadEncoders()
{
    int l = (encoder[0] * 256) + encoder[1];
    int r = (encoder[2] * 256) + encoder[3];

    Serial.printf("%d %d\n", l, r, 0x0d);

}

void DriveCmd(String cmdStr)
{
 
    char input[20];
    char separator[]=" ";
    char *val;
    int i = 0;

     
    // first trim() cmdStr to remove any leading spaces
    cmdStr.trim();
    // then move data into input[] char array.
    for ( i=0; i<cmdStr.length(); i++)
      input[i] = cmdStr[i];

    input[i]=0x0;
    input[i+1]=0x0;

      // now split input into left and right buffers
    val = strtok(input, separator);

    iLeftVelocity = atoi(val);

    val = strtok(NULL, separator);
    
    iRightVelocity = atoi(val);



}

