    #include <Wire.h>
    #include <Adafruit_Sensor.h>
    #include <Adafruit_BNO055.h>
    #include <utility/imumaths.h>
    #include <Servo.h>
    #include <IRremote.h>

    #define MIN_SPEED 0
    #define MAX_SPEED 200
    #define A_S 8.0 //"Abort Speed" is the acceleration required to abort flight
      
    Adafruit_BNO055 bno = Adafruit_BNO055(55);
    Servo ESC1, ESC2, ESC3, ESC4; //four motors, with the battery cable facing towards
                                  //you, they go counter-clockwise in order, starting from the front right as
                                  // ESC1, ESC2, ESC3, ESC4
                                  
    IRrecv receiver1(2);          //IR Receiver on pin 2
    IRrecv receiver2(3);          //IR Receiver on pin 3
    decode_results IRoutput1;     //Not sure what that datatype is, but it holds the output from IR1
    decode_results IRoutput2;     //Same, but for IR2

    /* Arms the servo motors through the ESC
     * Wait for two seconds, then send a very
     * small speed for 9 seconds
     */
    void arm(){
      delay(2000);
      for(int i = 0; i < 90; i++){
        ESC1.writeMicroseconds(1260);
        ESC2.writeMicroseconds(1260);
        ESC3.writeMicroseconds(1260);
        ESC4.writeMicroseconds(1260);
        delay(100);
      }
    }

    /* Sets the speed of a single motor
     * motor num can be between 1-4
     * 1 being ESC1, and so on
     */
    void setSpeedSingle(int speed, int motorNum){
      if(motorNum >= 1 && motorNum <= 4){
        if(speed <= MAX_SPEED){
          int angle = map(speed, 0, 200, 1275, 1750);
          switch(motorNum){
            case 1:
              ESC1.writeMicroseconds(angle);
              break;
            case 2:
              ESC2.writeMicroseconds(angle);
              break;
            case 3:
              ESC3.writeMicroseconds(angle);
              break;
            case 4:
              ESC4.writeMicroseconds(angle);
              break;
          }
        }
      }
    }

    /*
     * Sets the speed of all motors to same value
     * NOTE: IMPROVEMENT NEEDED, USE writeMicroseconds() INSTEAD OF write()
     */
    void setSpeed(int speed){
      if(speed <= 100){
        int angle = map(speed, 0, 300, 1275, 1750);
        ESC1.writeMicroseconds(angle);
        ESC2.writeMicroseconds(angle);
        ESC3.writeMicroseconds(angle);
        ESC4.writeMicroseconds(angle);
        Serial.println(angle);
      }
    }

    /*
     * used for terminating flight
     */
    void abort(void){
      Serial.println("ABORT");
      ESC1.writeMicroseconds(1100);
      ESC2.writeMicroseconds(1100);
      ESC3.writeMicroseconds(1100);
      ESC4.writeMicroseconds(1100);
      while(1){};
    }
    

    /*
     * called once at startup, enables the motors 
     * IR receivers, and gyroscope, then arms motors
     */
    void setup(void) 
    {
      Serial.begin(9600);
      Serial.setTimeout(10);

      ESC1.attach(5);
      ESC2.attach(6);
      ESC3.attach(9);
      ESC4.attach(10);   

      receiver1.enableIRIn();
      receiver2.enableIRIn();
   
      /* Initialise the sensor */
      if(!bno.begin())
      {
        Serial.print("BNO055 detected ... Check wiring");
        while(1);
      } else {
        Serial.println("Connected to BNO055!");
      }
      arm();    
      bno.setExtCrystalUse(true);
    }


    float xRot = 0;
    float yRot = 0;
    float zRot = 0;

    short setCalibrate = 0;
    float calibrateX = 0.0; //calibration for rotation
    float calibrateY = 0.0;
    float calibrateZ = 0.0;

    imu::Vector<3> acc;
    imu::Vector<3> calibrateAcc; //calibration for acceleration

    short m1Speed = MIN_SPEED + 19;
    short m2Speed = MIN_SPEED + 9;
    short m3Speed = MIN_SPEED + 9;
    short m4Speed = MIN_SPEED + 19;

   
    bool balance;
   
    void move(){

      balance = true;

      if(acc.x() > 1){
        m1Speed++;
        m4Speed++;
        m2Speed--;
        m3Speed--;
      } else if(acc.x() < -1){
        m1Speed--;
        m4Speed--;
        m2Speed++;
        m3Speed++;
      }

      if(acc.y() > 1){
        m1Speed++;
        m2Speed++;
        m3Speed--;
        m4Speed--;
      } else if(acc.y() < -1){
        m1Speed++;
        m2Speed++;
        m3Speed--;
        m4Speed--;
      }
            
      if (yRot < -1){
        m2Speed ++;
        m3Speed ++;
        if(yRot > -3){
          m1Speed--;
          m4Speed--;
        } else {
          m1Speed + (yRot*.5);
          m4Speed + (yRot*.5);
        }
        
        balance = false;
      }
      else if (yRot > 1){
        m1Speed ++;
        m4Speed ++;
        if(yRot < 3){
          m2Speed --;
          m3Speed --;
        } else {
          m2Speed - (yRot*.5);
          m3Speed - (yRot*.5);
        }
          
        balance = false;  
      }

      if (zRot > 1){
        m3Speed ++;
        m4Speed ++;
        if(zRot < 3){
          m1Speed --;
          m2Speed --;
        } else {
          m1Speed - (zRot*.5);
          m2Speed - (zRot*.5);
        }
        balance = false;
      }
      else if (zRot < -1){
        m1Speed ++;
        m2Speed ++;
        if(zRot > -3){
          m3Speed --;
          m4Speed --;
        } else {
          m3Speed + (zRot*.333);
          m4Speed + (zRot*.333);
        }
        balance = false;
      }

      if (balance == true){
        m1Speed ++;
        m2Speed ++;
        m3Speed ++;
        m4Speed ++;
      }
        
      setSpeedSingle(m1Speed, 1);
      setSpeedSingle(m2Speed, 2);
      setSpeedSingle(m3Speed, 3);
      setSpeedSingle(m4Speed, 4);
            
    /*
      if(goingUp == 1){
        m1Speed += 1;
        m2Speed += 1;
        m3Speed += 1;
        m4Speed += 1;
        if(m1Speed > MIN_SPEED + 20){
          goingUp = 0;
        }
      } else {
        m1Speed -= 1;
        m2Speed -= 1;
        m3Speed -= 1;
        m4Speed -= 1;
        if(m1Speed <= MIN_SPEED){
          goingUp = 1;
        }
      }

      setSpeedSingle(m1Speed, 1);
      setSpeedSingle(m2Speed, 2);
      setSpeedSingle(m3Speed, 3);
      setSpeedSingle(m4Speed, 4);
      delay(500);
      */
      delay(200);
    }
    
    void loop(void) 
    {


      static float calibrateXAcc = 0.0;
      static float calibrateYAcc = 0.0;
      static float calibrateZAcc = 0.0;

      //If an IR signal is received, then abort the flight
      //10839 38221 2686 1918 are "off" signals from remote control (saving this for future reference)
      if(receiver1.decode(&IRoutput1)){
        unsigned int value = IRoutput1.value;
        Serial.print(value);
        abort();
        receiver1.resume();
      }
      if(receiver2.decode(&IRoutput2)){
        unsigned int value = IRoutput2.value;
        Serial.print(value);
        abort();
        receiver2.resume();
      }

      /* Get a new sensor event */ 
      sensors_event_t event; 
      bno.getEvent(&event);

      xRot = event.orientation.x - calibrateX;
      yRot = event.orientation.y - calibrateY;
      zRot = event.orientation.z - calibrateZ;

      acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

      Serial.print("X: ");
      Serial.print(acc.x(), 4);
      Serial.print("\tY: ");
      Serial.print(acc.y(), 4);
      Serial.print("\tZ: ");
      Serial.print(acc.z(), 4);
      Serial.print("\trotX: ");
      Serial.print(xRot, 4);
      Serial.print("\trotY: ");
      Serial.print(yRot, 4);
      Serial.print("\trotZ: ");
      Serial.print(zRot, 4);
      Serial.println("");
      
      if(setCalibrate == 6){
        calibrateX = event.orientation.x;
        calibrateY = event.orientation.y;
        calibrateZ = event.orientation.z;
        
        setCalibrate += 1;
      } else if(setCalibrate < 6) {
        setCalibrate += 1;
      } else {
        if(acc.x() >= A_S || acc.x() <= -A_S || acc.y() >= A_S || acc.y() <= -A_S || acc.z() >= A_S || acc.z() <= -A_S){
          abort();
          Serial.println("abort");
        }
        //programming loop goes here
        move();
      }

      delay(100);
    }
