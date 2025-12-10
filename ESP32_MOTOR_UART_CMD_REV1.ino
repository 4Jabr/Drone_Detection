#include <Arduino.h>

// ====== Modify here if needed ====================================================================================
#define UART1_TX_PIN   17        // UART1 TX -> Motor controller RX
#define UART1_RX_PIN   18        // UART1 RX -> Motor controller TX
#define UART1_BAUD     115200

// Timer variables
unsigned long lastTimeDetected=0; // last time a drone was detected
unsigned long lastMotorUpdate=0; // last time motor was updated (for position tracking)

// Speed variables
uint16_t lastSpeedX = 0;
uint16_t lastSpeedY = 0;
uint8_t lastDirectionX = 0; // 1=Right/CW, 0=Left/CCW
uint8_t lastDirectionY = 0; // 0=Up/CW, 1=Down/CCW
// *** max speed and acceleration - toggle these ***
// units are revolutions per minute?
static const int32_t maxSpeedX = 5;
static const int32_t maxSpeedY = 5;
static const uint8_t acceleration = 100;

// Error margins
const float smallErrorMargin = 0.01; // *** toggle based on smoothness of motors ***
const unsigned long lostTimeMax = 200; // *** toggle based on processing speed *** (large number for my slow computer 4fps)

// *** error gain constants - toggle these ***
const float KX = 33;
const float KY = 33; // KX*(720/1280)

bool droneLost; // for lost drone handling

// *** Position Tracking (software limiting motor position) ***
// Only need to limit Y direction for now
double estimatedPosY = 0.0;
const double pulsesPerDegree = 3200.0/360.0; // 3200 pulses / 360 degrees
const long minDegreeY = 0;
const long maxDegreeY = 85;
long minPulsesY;
long maxPulsesY;

// ===========================================================================================================================

#define ABS(x)    ((x) > 0 ? (x) : -(x))

// Offset and limits (unit: pulses) ?NOT NEEDED ANYMORE?
static const int32_t OFFSET_PULSES = 0;   // Offset = 0 for both motors
static const int32_t LIMIT_MIN     = 0;   // Lower limit = 0
static const int32_t LIMIT_MAX     = 3200;// Upper limit = 3200 (per requirement)
static const int32_t LIMIT_MAX2    = 800; // Upper limit = 800 (per requirement)

HardwareSerial MotorSerial(1);

typedef enum {
  S_VER=0,S_RL=1,S_PID=2,S_VBUS=3,S_CPHA=5,S_ENCL=7,S_TPOS=8,S_VEL=9,
  S_CPOS=10,S_PERR=11,S_FLAG=13,S_Conf=14,S_State=15,S_ORG=16,
} SysParams_t;

// ---------- Forward declarations ----------
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);
void Emm_V5_Reset_Clog_Pro(uint8_t addr);
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_Synchronous_motion(uint8_t addr);
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
void Emm_V5_Origin_Interrupt(uint8_t addr);
void Emm_V5_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount);

//******************* MY FUNCTION - PARSE DATA **********************
bool parseInput(String& line, long& detect, float& x, float& y, float& conf){
  // find first comma
  int firstComma = line.indexOf(',');
  // if comma doesn't exist, data is bad
  if (firstComma==-1){
    return false;
  }
  // find second comma
  int secondComma = line.indexOf(',', firstComma+1);
  // if second comma doesn't exist, data is bad
  if (secondComma==-1){
    return false;
  }
  // find third comma
  int thirdComma = line.indexOf(',', secondComma+1);
  // if third comma doesn't exist, data is bad
  if (thirdComma==-1){
    return false;
  }

  //split string at commas
  String stringDetect = line.substring(0, firstComma);
  String stringX = line.substring(firstComma+1, secondComma);
  String stringY = line.substring(secondComma+1, thirdComma);
  String stringConfidence = line.substring(thirdComma+1);

  // convert strings to floats
  detect = atol(stringDetect.c_str());
  x = atof(stringX.c_str());
  y = atof(stringY.c_str());
  conf = atof(stringConfidence.c_str());

  // algorithm successful --> return true
  return true;
}

//******************* MY FUNCTION - UPDATE POSITION *****************
void updatePositionTracker(){
  unsigned long now = micros();
  float deltaTime = (now-lastMotorUpdate)/1000000.0;
  lastMotorUpdate = now;

  if (lastSpeedY == 0) return;

  double currentRPM = (double)lastSpeedY;
  double pulsesPerSecond = (currentRPM *3200.0) / 60.0;

  double pulsesMoved = pulsesPerSecond * deltaTime;

  // Update Y position (0=UP, 1=DOWN)
  if (lastDirectionY == 0){
    estimatedPosY = estimatedPosY + pulsesMoved;
  }
  else{
    estimatedPosY = estimatedPosY - pulsesMoved;
  }
}

//******************* MY FUNCTION - SAFETY CHECK *****************
// Checks limits and stops motor IMMEDIATELY if hit.
// Returns true if limit was hit.
bool checkAndEnforceLimits() {
  bool limitHit = false;
  
  // Only check if we are currently moving
  if (lastSpeedY > 0) {
    if (lastDirectionY == 0 && estimatedPosY >= maxPulsesY){
      limitHit = true;
      Serial.println("***Y-Axis Max Reached (Safety Stop)***");
    }
    else if (lastDirectionY == 1 && estimatedPosY <= minPulsesY){
      limitHit = true;
      Serial.println("***Y-Axis Min Reached (Safety Stop)***");
    }

    if (limitHit) {
      lastSpeedY = 0;
      // Send IMMEDIATE stop command
      Emm_V5_Vel_Control(2, lastDirectionY, 0, 255, 0); 
    }
  }
  return limitHit;
}

// ---------- Read one line from USB Serial ----------
bool readLineFromUSB(String &outLine) {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;     // Ignore carriage return
    if (c == '\n') {             // End of line
      outLine = buf;
      buf = "";
      return true;
    }
    buf += c;
    if (buf.length() > 64) buf.remove(0, buf.length() - 64); // Simple overflow prevention
  }
  return false;
}

// ---------- Parse two signed decimal integers "a,b" ----------
bool parseTwoInt64(const String& line, long long &v1, long long &v2) {
  int comma = line.indexOf(',');
  if (comma < 0) return false;
  String s1 = line.substring(0, comma);
  String s2 = line.substring(comma + 1);
  s1.trim(); s2.trim();
  if (s1.length()==0 || s2.length()==0) return false;

  auto isInt = [](const String& s)->bool{
    for (size_t i=0;i<s.length();++i){
      char c=s[i];
      if (!(c=='-' || (c>='0' && c<='9'))) return false;
      if (i>0 && c=='-') return false;
    }
    return true;
  };
  if (!isInt(s1) || !isInt(s2)) return false;

  v1 = atoll(s1.c_str());
  v2 = atoll(s2.c_str());
  return true;
}

//**************************************************************************
//********************* MAIN SETUP *****************************************
//**************************************************************************
void setup() {
  // USB serial used for entering two target pulse values, format: n1,n2
  Serial.begin(115200);

  // UART1 for communication with the motor controllers
  MotorSerial.begin(UART1_BAUD, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);

  // Wait for closed-loop initialization
  delay(2000);
  Serial.println(F("AI Drone Tracker - Speed Control Mode"));
  Serial.println(F("Ready for Data in Format: detect,x,y,confidence"));

  // Set motor position to zero
  Emm_V5_Reset_CurPos_To_Zero(1); // X Axis
  delay(50);
  Emm_V5_Reset_CurPos_To_Zero(2); // Y Axis
  delay(50);

  // Calculate pulse limits based on degree limits
  minPulsesY = minDegreeY * pulsesPerDegree;
  maxPulsesY = maxDegreeY * pulsesPerDegree;

  // start program as if we just detected a drone
  lastTimeDetected = millis();
  droneLost = false;
  lastMotorUpdate = micros(); // act as if motors just moved (drone was detected)
}

//**************************************************************************
//******************** MAIN LOOP *******************************************
//**************************************************************************
void loop() {
  // Update position
  updatePositionTracker();

  // Check motor position for safety
  checkAndEnforceLimits();

  String line;
  if (readLineFromUSB(line)) {
    line.trim();
    if (line.length() == 0) return;

    // define parameters for position and confidence
    long detect;
    float x;
    float y;
    float conf;

    // call my function to parse data
    if (parseInput(line, detect, x, y, conf)){
      // print parsed data to terminal
      Serial.print("Detect:");
      Serial.print(detect);
      Serial.print(", x=");
      Serial.print(x, 4);
      Serial.print(", y=");
      Serial.print(y, 4);
      Serial.print(", Confidence=");
      Serial.println(conf, 4);

      // if drone is detected
      if (detect==1){
        droneLost=false;
        lastTimeDetected = millis(); // reset last time detected

        // CALCULATE SPEED FOR MOTOR X
        // if drone is close to centered, set speed to zero
        if(ABS(x)<smallErrorMargin){
          lastSpeedX = 0;
        }
        // otherwise, calculate speed using error constant
        else{
          lastSpeedX = (uint16_t)(ABS(x)*KX);
          // set direction
          if(x>0){
            lastDirectionX = 1; // 1 --> Right
          }
          else{
            lastDirectionX = 0; // 0 --> Left
          }
          // safety to not exceed max speed
          if (lastSpeedX > maxSpeedX){
            lastSpeedX = maxSpeedX;
          }
        }

        // CALCULATE SPEED FOR MOTOR Y
        // ***[INSERT SAFETY HERE WITH LIMIT SWITCH OR OTHER]***
        // if drone is close to centered, set speed to zero
        if(ABS(y)<smallErrorMargin){
          lastSpeedY = 0;
        }
        // otherwise, calculate speed using error constant
        // (uint16_t) is a type conversion for the set velo function
        else{
          lastSpeedY = (uint16_t)(ABS(y)*KY);
          // set direction
          if(y>0){
            lastDirectionY = 0; // 0 --> Up
          }
          else{
            lastDirectionY = 1; // 1 --> Down
          }
          // safety to not exceed max speed
          if (lastSpeedY > maxSpeedY){
            lastSpeedY = maxSpeedY;
          }
          // POSITION ESTIMATION AND LIMIT
          if (lastDirectionY == 0 && estimatedPosY >= maxPulsesY){
            lastSpeedY = 0;
          }
          else if (lastDirectionY == 1 && estimatedPosY <= minPulsesY){
            lastSpeedY = 0;
          }
        }

        // MOVE MOTORS (using built in function)
        // 255 for acceleration (max for uint8_t), toggle this if needed
        Emm_V5_Vel_Control(1, lastDirectionX, lastSpeedX, acceleration, 0);
        delay(5);
        Emm_V5_Vel_Control(2, lastDirectionY, lastSpeedY, acceleration, 0);
        //Print speed and position
        Serial.print("Drone detected. Moving at speed: (");
        Serial.print(lastSpeedX);
        Serial.print(", ");
        Serial.print(lastSpeedY);
        Serial.println(")");
        Serial.print("Position Y: ");
        Serial.println(estimatedPosY);
      }
      else{
        // if the drone is not actually lost (it is just jitter) 
        if (!droneLost){
          // POSITION ESTIMATION AND LIMIT
          if (lastDirectionY == 0 && estimatedPosY >= maxPulsesY){
            lastSpeedY = 0;
          }
          else if (lastDirectionY == 1 && estimatedPosY <= minPulsesY){
            lastSpeedY = 0;
          }
          // MOVE MOTORS (using built in function)
          // 255 for acceleration (max for uint8_t), toggle this if needed
          Emm_V5_Vel_Control(1, lastDirectionX, lastSpeedX, acceleration, 0);
          delay(5);
          Emm_V5_Vel_Control(2, lastDirectionY, lastSpeedY, acceleration, 0);
          Serial.print("No drone detected. Motors coasting at speed: (");
          Serial.print(lastSpeedX);
          Serial.print(", ");
          Serial.print(lastSpeedY);
          Serial.println(")");
          Serial.print("Position Y: ");
          Serial.println(estimatedPosY);
        }
      }
      Serial.println("-");
    // otherwise, the parse failed
    }
    else{
      Serial.println("parse failed");
    }
  }
  // HANDLE TIME TRACKING FOR LOST DRONE
  // checks every loop
  // stop motors if drone hasn't been detected for a certain time
  if(millis()-lastTimeDetected > lostTimeMax){
    if (!droneLost){
      // drone is lost
      droneLost = true;
      // set speeds to zero
      lastSpeedX = 0;
      lastSpeedY = 0;
      // stop motors
      Emm_V5_Vel_Control(1, 0, 0, acceleration, 0);
      delay(5);
      Emm_V5_Vel_Control(2, 0, 0, acceleration, 0);
      Serial.println("Drone lost. Motors stopped");
      Serial.println("-");
    }
  }
  // delay(50); // DEBUGGING PURPOSES ONLY
}



/* ===================== Protocol packaging (all via MotorSerial) ===================== */

void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr){
  uint8_t cmd[4]={addr,0x0A,0x6D,0x6B}; MotorSerial.write(cmd,4);
}
void Emm_V5_Reset_Clog_Pro(uint8_t addr){
  uint8_t cmd[4]={addr,0x0E,0x52,0x6B}; MotorSerial.write(cmd,4);
}
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s){
  uint8_t cmd[16]={0}; uint8_t i=0; cmd[i++]=addr;
  switch(s){
    case S_VER:cmd[i++]=0x1F;break; case S_RL:cmd[i++]=0x20;break; case S_PID:cmd[i++]=0x21;break;
    case S_VBUS:cmd[i++]=0x24;break; case S_CPHA:cmd[i++]=0x27;break; case S_ENCL:cmd[i++]=0x31;break;
    case S_TPOS:cmd[i++]=0x33;break; case S_VEL:cmd[i++]=0x35;break; case S_CPOS:cmd[i++]=0x36;break;
    case S_PERR:cmd[i++]=0x37;break; case S_FLAG:cmd[i++]=0x3A;break; case S_ORG:cmd[i++]=0x3B;break;
    case S_Conf:cmd[i++]=0x42;cmd[i++]=0x6C;break; case S_State:cmd[i++]=0x43;cmd[i++]=0x7A;break;
    default:break;
  }
  cmd[i++]=0x6B; MotorSerial.write(cmd,i);
}
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode){
  uint8_t cmd[6]={addr,0x46,0x69,(uint8_t)svF,ctrl_mode,0x6B}; MotorSerial.write(cmd,6);
}
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF){
  uint8_t cmd[6]={addr,0xF3,0xAB,(uint8_t)state,(uint8_t)snF,0x6B}; MotorSerial.write(cmd,6);
}
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF){
  uint8_t cmd[8]={addr,0xF6,dir,(uint8_t)(vel>>8),(uint8_t)vel,acc,(uint8_t)snF,0x6B}; MotorSerial.write(cmd,8);
}
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF){
  uint8_t cmd[13]={
    addr,0xFD,dir,(uint8_t)(vel>>8),(uint8_t)vel,acc,
    (uint8_t)(clk>>24),(uint8_t)(clk>>16),(uint8_t)(clk>>8),(uint8_t)clk,
    (uint8_t)raF,(uint8_t)snF,0x6B
  };
  MotorSerial.write(cmd,13);
}
void Emm_V5_Stop_Now(uint8_t addr, bool snF){
  uint8_t cmd[5]={addr,0xFE,0x98,(uint8_t)snF,0x6B}; MotorSerial.write(cmd,5);
}
void Emm_V5_Synchronous_motion(uint8_t addr){
  uint8_t cmd[4]={addr,0xFF,0x66,0x6B}; MotorSerial.write(cmd,4);
}
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF){
  uint8_t cmd[5]={addr,0x93,0x88,(uint8_t)svF,0x6B}; MotorSerial.write(cmd,5);
}
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF){
  uint8_t cmd[20]={0};
  cmd[0]=addr; cmd[1]=0x4C; cmd[2]=0xAE; cmd[3]=(uint8_t)svF;
  cmd[4]=o_mode; cmd[5]=o_dir;
  cmd[6]=(uint8_t)(o_vel>>8); cmd[7]=(uint8_t)o_vel;
  cmd[8]=(uint8_t)(o_tm>>24); cmd[9]=(uint8_t)(o_tm>>16); cmd[10]=(uint8_t)(o_tm>>8); cmd[11]=(uint8_t)o_tm;
  cmd[12]=(uint8_t)(sl_vel>>8); cmd[13]=(uint8_t)sl_vel;
  cmd[14]=(uint8_t)(sl_ma>>8);  cmd[15]=(uint8_t)sl_ma;
  cmd[16]=(uint8_t)(sl_ms>>8);  cmd[17]=(uint8_t)sl_ms;
  cmd[18]=(uint8_t)potF;        cmd[19]=0x6B;
  MotorSerial.write(cmd,20);
}
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF){
  uint8_t cmd[5]={addr,0x9A,o_mode,(uint8_t)snF,0x6B}; MotorSerial.write(cmd,5);
}
void Emm_V5_Origin_Interrupt(uint8_t addr){
  uint8_t cmd[4]={addr,0x9C,0x48,0x6B}; MotorSerial.write(cmd,4);
}
void Emm_V5_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount){
  int i=0; unsigned long last=millis();
  while (1){
    if (MotorSerial.available()){
      if (i < 128){
        rxCmd[i++]=MotorSerial.read();
        last=millis();
      }
    } else {
      if (millis()-last > 100){ *rxCount=i; break; }
    }
  }
}
