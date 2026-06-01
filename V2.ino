#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <TMCStepper.h>   
#include "FastAccelStepper.h" // Install "FastAccelStepper"

// --- Pins (ESP32-S3 Configuration) ---
const int STEP_PIN = 1;  
const int DIR_PIN  = 2;  
const int EN_PIN   = 4;  
const int inputPin = 5;  // PWM or Analog Input

// TMC UART Pins (Single Wire on GPIO 17)
const int RX_PIN = 17;
const int TX_PIN = 17;

// --- Driver & Hardware Stepper Engine Setup ---
#define SERIAL_PORT Serial1 
#define R_SENSE 0.11f       
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, 0b00);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// --- System Parameters ---
int tmcCurrent = 800;      // mA
int tmcMicrosteps = 16;
bool tmcStealth = true;

float maxSpeedVal = 15000.0;  // Steps per second (Hz)
float accelVal = 30000.0;     // Steps per second^2
float tolerance = 0.5;         // Deadzone in degrees
float maxDeg = 180.0;
float gearRatio = 1.0;

float targetDeg = 0.0;
float currentDeg = 0.0;
int controlMode = 0;           // 0: Web, 1: PWM, 2: Analog
float filterAlpha = 0.15; 

// --- PID Controller Variables ---
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;
float integralError = 0.0;
float lastError = 0.0;
unsigned long lastPIDTime = 0;

// --- Encoder Variables ---
int lastRaw = 0;
long totalRaw = 0;
long homeOffset = 0;
volatile unsigned long pulseStart = 0;
volatile int pulseWidth = 0;

Preferences prefs; 
AsyncWebServer server(80);

// --- Interrupt for External PWM ---
void IRAM_ATTR handlePWM() {
  if (digitalRead(inputPin) == HIGH) pulseStart = micros();
  else pulseWidth = micros() - pulseStart;
}

// --- Read AS5600 Encoder ---
void updateEncoder() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0E); 
  if (Wire.endTransmission() != 0) return;

  Wire.requestFrom(0x36, 2);
  if (Wire.available() >= 2) {
    int raw = (Wire.read() << 8) | Wire.read();
    int diff = raw - lastRaw;
    
    if (diff > 2048) diff -= 4096;
    if (diff < -2048) diff += 4096;
    
    totalRaw += diff;
    lastRaw = raw;

    currentDeg = ((totalRaw - homeOffset) * (360.0 / 4096.0)) * gearRatio;
  }
}

// --- Core 1 RTOS Loop (Dedicated Dynamic Position Tracking) ---
void controlLoopTask(void * pvParameters) {
  lastPIDTime = micros();
  
  for(;;) {
    updateEncoder();

    // 1. Process Input Signals (Web UI bypasses limits)
    float rawInput = targetDeg; 
    if (controlMode == 1 && pulseWidth > 800 && pulseWidth < 2200) {
      rawInput = map(pulseWidth, 1000, 2000, 0, maxDeg);
    } else if (controlMode == 2) {
      int analogVal = 0;
      for(int i=0; i<4; i++) analogVal += analogRead(inputPin); 
      rawInput = map(analogVal / 4, 0, 4095, 0, maxDeg);
    }

    if (controlMode != 0) {
      targetDeg = (filterAlpha * rawInput) + ((1.0 - filterAlpha) * targetDeg);
    }

    // 2. Closed-Loop PID Mathematical Mapping
    unsigned long now = micros();
    float dt = (now - lastPIDTime) / 1000000.0;
    if (dt <= 0.0) dt = 0.004; // Fallback to 4ms if time is too tight
    lastPIDTime = now;

    float error = targetDeg - currentDeg;

    // Check deadzone tolerance
    if (abs(error) < tolerance) {
      stepper->stopMove(); 
      integralError = 0.0; // Anti-windup when resting in tolerance band
    } else {
      // Calculate Integral with basic Anti-Windup Limits
      integralError += error * dt;
      if (integralError > 50.0) integralError = 50.0;
      if (integralError < -50.0) integralError = -50.0;

      // Calculate Derivative
      float derivative = (error - lastError) / dt;

      // Compute PID Output (Correction in Degrees)
      float pidOutputDeg = (Kp * error) + (Ki * integralError) + (Kd * derivative);

      // Convert PID output into stepper pulses
      long pidSteps = (pidOutputDeg * 200.0 * tmcMicrosteps * gearRatio) / 360.0;
      
      // Dynamic Target Injection
      long dynamicTarget = stepper->getCurrentPosition() + pidSteps;
      stepper->moveTo(dynamicTarget);
    }
    
    lastError = error;

    // Note: No stepper.run() needed! Hardware timers handle it automatically.
    vTaskDelay(4 / portTICK_PERIOD_MS); // ~250Hz loop rate
  }
}

// --- HTML Dashboard Visuals ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>Smart Servo Controller</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: sans-serif; text-align: center; background: #1a1a1a; color: white; padding: 20px; }
  .card { background: #2d2d2d; padding: 20px; border-radius: 15px; display: inline-block; width: 450px; border: 1px solid #444; max-width: 100%; box-sizing: border-box; }
  .btn { padding: 12px; margin: 5px; cursor: pointer; border: none; border-radius: 8px; font-weight: bold; width: 45%; }
  .on { background: #2ecc71; color: white; }
  .save { background: #3498db; color: white; width: 93%; }
  input, select { width: 85%; padding: 10px; margin: 10px 0; border-radius: 5px; background: #444; color: white; border: none; }
  .row { display: flex; justify-content: space-around; align-items: center; margin: 5px 0; }
  .row input, .row select { width: 75px; text-align: center; }
  .stats { display: flex; justify-content: space-between; background: #111; padding: 10px; border-radius: 8px; margin-bottom: 15px; }
  .stat-box { width: 45%; }
  .stat-val { font-size: 1.6em; font-weight: bold; }
  h4 { margin-bottom: 5px; color: #bbb; }
</style></head>
<body>
  <div class="card">
    <h2>Smart Stepper</h2>
    <div class="stats">
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Target&deg;</div><div class="stat-val" id="tgt">0.0</div></div>
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Current&deg;</div><div class="stat-val" id="pos" style="color:#2ecc71;">0.0</div></div>
    </div>
    <hr>
    <h4>Control & Limits</h4>
    <select id="mode">
      <option value="0">Web Dashboard</option>
      <option value="1">External PWM</option>
      <option value="2">Analog Signal</option>
    </select>
    <div class="row"> Ratio: <input type="text" id="ratio"> MaxLimits: <input type="text" id="mDeg"> </div>
    
    <hr>
    <h4>PID Controller Tuning</h4>
    <div class="row"> 
      Kp:<input type="number" id="kp" step="0.1"> 
      Ki:<input type="number" id="ki" step="0.01"> 
      Kd:<input type="number" id="kd" step="0.01"> 
    </div>

    <hr>
    <h4>TMC UART Setup</h4>
    <div class="row"> 
      mA:<input type="number" id="tmcC" step="50"> 
      M-Step:<select id="tmcM"><option value="4">4</option><option value="8">8</option><option value="16">16</option><option value="32">32</option><option value="64">64</option></select>
      Chop:<select id="tmcS"><option value="1">Stealth</option><option value="0">Spread</option></select>
    </div>
    <hr>
    <h4>Hardware Performance Tuning</h4>
    <div class="row"> Max Speed (Hz):<input type="text" id="sp"> Max Accel:<input type="text" id="ac"> </div>
    <div class="row"> Tol&deg;:<input type="text" id="t" style="width:120px;"> </div>
    
    <button class="btn save" onclick="saveSettings()">APPLY & SAVE</button>
    <hr>
    <button class="btn on" onclick="fetch('/sethome')">SET ZERO</button>
    <input type="number" id="moveVal" placeholder="Degrees" style="width: 120px; margin-left:10px;">
    <button class="btn on" style="width:93%; background:#9b59b6; margin-top:10px;" onclick="move()">WEB MOVE</button>
  </div>
<script>
  function move() { fetch('/move?val=' + document.getElementById('moveVal').value); }
  function saveSettings() {
    const p = {
      sp: document.getElementById('sp').value, ac: document.getElementById('ac').value, t: document.getElementById('t').value,
      m: document.getElementById('mode').value, max: document.getElementById('mDeg').value, r: document.getElementById('ratio').value,
      tmcC: document.getElementById('tmcC').value, tmcM: document.getElementById('tmcM').value, tmcS: document.getElementById('tmcS').value,
      kp: document.getElementById('kp').value, ki: document.getElementById('ki').value, kd: document.getElementById('kd').value
    };
    fetch(`/save?` + new URLSearchParams(p)).then(() => alert("Hardware & PID Profiles Updated!"));
  }
  fetch('/getparams').then(r => r.json()).then(data => {
    document.getElementById('sp').value = data.sp; document.getElementById('ac').value = data.ac; document.getElementById('t').value = data.t;
    document.getElementById('mode').value = data.m; document.getElementById('mDeg').value = data.max; document.getElementById('ratio').value = data.r;
    document.getElementById('tmcC').value = data.tmcC; document.getElementById('tmcM').value = data.tmcM; document.getElementById('tmcS').value = data.tmcS;
    document.getElementById('kp').value = data.kp; document.getElementById('ki').value = data.ki; document.getElementById('kd').value = data.kd;
  });
  setInterval(() => { 
    fetch('/status').then(r => r.json()).then(data => { 
      document.getElementById('pos').innerText = data.pos; 
      document.getElementById('tgt').innerText = data.tgt;
    }); 
  }, 250);
</script></body></html>)rawliteral";

void setup() {
  Serial.begin(115200);
  
  // Single-Wire Half Duplex Setup via Hardware Serial 1
  SERIAL_PORT.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); 
  Serial1.setMode(UART_MODE_RS485_HALF_DUPLEX);

  Wire.begin(); 
  Wire.setClock(400000); 
  
  // Initialize FastAccelStepper Engine
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(EN_PIN, false); // True = Active LOW enable pin
    stepper->setAutoEnable(false);       // Keep driver continually engaged for holding torque
  }

  // Load Parameters from Flash Memory
  prefs.begin("servo-data", false);
  homeOffset = prefs.getLong("offset", 0);
  maxSpeedVal = prefs.getFloat("sp", 15000.0);
  accelVal = prefs.getFloat("ac", 30000.0);
  tolerance = prefs.getFloat("tol", 0.5);
  maxDeg = prefs.getFloat("maxDeg", 180.0);
  gearRatio = prefs.getFloat("ratio", 1.0);
  controlMode = prefs.getInt("mode", 0);
  tmcCurrent = prefs.getInt("tmcC", 800);
  tmcMicrosteps = prefs.getInt("tmcM", 16);
  tmcStealth = prefs.getInt("tmcS", 1);
  
  Kp = prefs.getFloat("kp", 1.0);
  Ki = prefs.getFloat("ki", 0.0);
  Kd = prefs.getFloat("kd", 0.0);

  // Configure TMC Chip Hardware Configuration via Single-Wire UART
  driver.begin();
  driver.toff(5);                 
  driver.rms_current(tmcCurrent); 
  driver.microsteps(tmcMicrosteps); 
  driver.en_spreadCycle(!tmcStealth);

  // Inject Initial Speed/Acceleration to Hardware Timer
  if (stepper) {
    stepper->setSpeedInHz(maxSpeedVal);
    stepper->setAcceleration(accelVal);
  }

  // Initial Read on startup
  Wire.beginTransmission(0x36); Wire.write(0x0E); Wire.endTransmission();
  Wire.requestFrom(0x36, 2);
  if (Wire.available() >= 2) {
    lastRaw = (Wire.read() << 8) | Wire.read();
    totalRaw = lastRaw; 
  }
  updateEncoder();
  targetDeg = currentDeg;

  if (controlMode == 1) attachInterrupt(digitalPinToInterrupt(inputPin), handlePWM, CHANGE);

  // Fire up Core 1 for closed-loop monitoring
  xTaskCreatePinnedToCore(controlLoopTask, "StepperLoop", 4096, NULL, 1, NULL, 1);

  WiFi.softAP("ClosedLoop_Stepper", "");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *f){ f->send_P(200, "text/html", index_html); });
  
  server.on("/save", [](AsyncWebServerRequest *f){
    maxSpeedVal = f->getParam("sp")->value().toFloat();
    accelVal = f->getParam("ac")->value().toFloat();
    tolerance = f->getParam("t")->value().toFloat();
    maxDeg = f->getParam("max")->value().toFloat();
    gearRatio = f->getParam("r")->value().toFloat();
    
    int newMode = f->getParam("m")->value().toInt();
    if (newMode == 1 && controlMode != 1) attachInterrupt(digitalPinToInterrupt(inputPin), handlePWM, CHANGE);
    else if (newMode != 1 && controlMode == 1) detachInterrupt(digitalPinToInterrupt(inputPin));
    controlMode = newMode;
    
    tmcCurrent = f->getParam("tmcC")->value().toInt();
    tmcMicrosteps = f->getParam("tmcM")->value().toInt();
    tmcStealth = f->getParam("tmcS")->value().toInt() == 1;

    Kp = f->getParam("kp")->value().toFloat();
    Ki = f->getParam("ki")->value().toFloat();
    Kd = f->getParam("kd")->value().toFloat();
    
    // Reset integral memory on param change to prevent jumpy behavior
    integralError = 0.0; 

    // Apply Real-Time Changes to TMC Hardware Registers
    driver.rms_current(tmcCurrent);
    driver.microsteps(tmcMicrosteps);
    driver.en_spreadCycle(!tmcStealth);
    
    // Dynamically update Hardware pulse generator constraints
    if (stepper) {
      stepper->setSpeedInHz(maxSpeedVal);
      stepper->setAcceleration(accelVal);
    }

    prefs.putFloat("sp", maxSpeedVal); prefs.putFloat("ac", accelVal); prefs.putFloat("tol", tolerance);
    prefs.putInt("mode", controlMode); prefs.putFloat("maxDeg", maxDeg); prefs.putFloat("ratio", gearRatio);
    prefs.putInt("tmcC", tmcCurrent); prefs.putInt("tmcM", tmcMicrosteps); prefs.putInt("tmcS", tmcStealth ? 1 : 0);
    prefs.putFloat("kp", Kp); prefs.putFloat("ki", Ki); prefs.putFloat("kd", Kd);
    
    f->send(200);
  });

  server.on("/move", [](AsyncWebServerRequest *f){ targetDeg = f->getParam("val")->value().toFloat(); f->send(200); });
  
  server.on("/getparams", [](AsyncWebServerRequest *f){
    String json = "{\"sp\":"+String(maxSpeedVal)+",\"ac\":"+String(accelVal)+",\"t\":"+String(tolerance)+
                  ",\"m\":"+String(controlMode)+",\"max\":"+String(maxDeg)+",\"r\":"+String(gearRatio)+
                  ",\"tmcC\":"+String(tmcCurrent)+",\"tmcM\":"+String(tmcMicrosteps)+",\"tmcS\":"+String(tmcStealth ? 1 : 0)+
                  ",\"kp\":"+String(Kp)+",\"ki\":"+String(Ki)+",\"kd\":"+String(Kd)+"}";
    f->send(200, "application/json", json);
  });
  
  server.on("/status", [](AsyncWebServerRequest *f){ 
    String json = "{\"pos\":\"" + String(currentDeg, 1) + "\",\"tgt\":\"" + String(targetDeg, 1) + "\"}";
    f->send(200, "application/json", json);
  });
  
  server.on("/sethome", [](AsyncWebServerRequest *f){ 
    homeOffset = totalRaw; 
    targetDeg = 0; currentDeg = 0;
    integralError = 0.0;
    lastError = 0.0;
    if (stepper) stepper->setCurrentPosition(0);
    prefs.putLong("offset", homeOffset); 
    f->send(200); 
  });

  server.begin();
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS); 
}