//#include <SoftwareSerial.h>

// Define SoftwareSerial pins for communication with Sabertooth
//SoftwareSerial sabertoothSerial(10, 11); // RX, TX

#define laptop Serial  
#define sabertoothSerial Serial2

// Define variables needed to recive user input for motor speed
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
byte dataByte = 0;

// Define Interrupt Pins for Encoder Reading
#define ENC_A 2
#define ENC_B 3
#define ENC_C 18
#define ENC_D 19

// Variables for encoder counting
unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
volatile int counter = 0; 
unsigned long _lastIncReadTime2 = micros();
unsigned long _lastDecReadTime2 = micros();
volatile int counter2 = 0;
int _pauseLength = 25000;
int _fastIncrement = 1; // used to be 10

// Variables for RPM reading
unsigned long previousMillis = 0;  
unsigned long interval = 1000;        
float CPR = 500;
unsigned long seconds = interval/1000;
float prev_pulses_counted = 0;
float  pulses_counted = 0;
float RPM = 0;
float prev_pulses_counted2 = 0;
float pulses_counted2 = 0;
float RPM2 = 0;



void setup() {
  laptop.begin(9600); // hardware serial communication (for laptop)
  sabertoothSerial.begin(9600); // software serial communication (for sabertooth)

  // Set encoder pins and attach interrupts
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, CHANGE); // run function read_encoder whenever there's a change on the ENC_A/B interrupt pins
  attachInterrupt(digitalPinToInterrupt(ENC_B), readEncoder, CHANGE);

  pinMode(ENC_C, INPUT_PULLUP);
  pinMode(ENC_D, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_C), readEncoder2, CHANGE); // same as above for ENC_C/D pins
  attachInterrupt(digitalPinToInterrupt(ENC_D), readEncoder2, CHANGE);

  laptop.println(" ");
  laptop.println(" ");
  laptop.println("<Arduino is ready: enter a motor speed>");
}



void loop() {
  checkForInput();
  showNewNumber();
  checkTime();

  static int lastCounter = 0;
}




void checkForInput() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (laptop.available() > 0) {
        rc = laptop.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        } else {
            receivedChars[ndx] = '\0'; // terminate the string 
            ndx = 0;
            newData = true;
        }
    }
}

void showNewNumber() {
  // Print what the user inputted as a byte
    if (newData == true) {
        dataByte = (byte)atoi(receivedChars);
        laptop.println("User inputted as byte: " + String(dataByte));
        //Serial.println(counter);
        runMotors();
        newData = false;
    }
}

void runMotors() {
  // Send the motor control signal to Sabertooth using SoftwareSerial
  sabertoothSerial.write(dataByte);
}

void readEncoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = 1;
    if((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastIncReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = -1;
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
}

void readEncoder2() {
    static uint8_t old_CD = 3;  
    static int8_t encval2 = 0;   
    static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; 

    old_CD <<= 2;  

    if (digitalRead(ENC_C)) old_CD |= 0x02; 
    if (digitalRead(ENC_D)) old_CD |= 0x01; 
  
    encval2 += enc_states[( old_CD & 0x0F )];

    if (encval2 > 3) {        
        int changevalue = 1;
        if ((micros() - _lastIncReadTime2) < _pauseLength) {
            changevalue = _fastIncrement * changevalue; 
        }
        _lastIncReadTime2 = micros();
        counter2 += changevalue;              
        encval2 = 0;
    }
    else if (encval2 < -3) {        
        int changevalue = -1;
        if ((micros() - _lastDecReadTime2) < _pauseLength) {
            changevalue = _fastIncrement * changevalue; 
        }
        _lastDecReadTime2 = micros();
        counter2 += changevalue;              
        encval2 = 0;
    }
}


void printEncoderCount() {
  unsigned long currentMillis = millis();

  // Check if 1000 milliseconds (1 second) have passed since last print
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the current time
    laptop.print("Encoder count: ");
    laptop.println(counter); // Print the encoder count
  }
}

void checkTime() {
  unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        
        // Calc RPM for both encoders
        pulses_counted = counter - prev_pulses_counted;
        pulses_counted2 = counter2 - prev_pulses_counted2;
        calculateRPM();
        prev_pulses_counted = counter;
        prev_pulses_counted2 = counter2;
    }
}

void calculateRPM() {
  RPM = (pulses_counted/CPR)/seconds * 60;
  RPM2 = (pulses_counted2 / CPR) / seconds * 60;

  laptop.println("---------------------");
  laptop.println("RPM: " + String(RPM));
  laptop.println("RPM2: " + String(RPM2));
  laptop.println("---------------------");
}

