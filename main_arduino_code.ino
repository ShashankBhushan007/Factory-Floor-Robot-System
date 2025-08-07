#include <SoftwareSerial.h>
#include <util/atomic.h>

// ----------------------------------------------------------------------
// PIN DEFINITIONS
// ----------------------------------------------------------------------
const int enca0 = 2;  // INT0
const int encb0 = 4;
const int enca1 = 3;  // INT1
const int encb1 = 7;

const int pwm0 = 5;
const int in1_0 = 8;
const int in2_0 = 9;

const int pwm1 = 10;
const int in1_1 = 11;
const int in2_1 = 12;

// ----------------------------------------------------------------------
// GLOBALS
// ----------------------------------------------------------------------
volatile long posi0 = 0;
volatile long posi1 = 0;

const int MAX_QUEUE_SIZE = 100;
String commandQueue[MAX_QUEUE_SIZE];
int queueStart = 0, queueEnd = 0;

bool commandActive = false;
bool reachedTarget = false;
unsigned long tStart = 0;
const unsigned long CMD_DURATION = 2000UL;

bool sequenceTimerStarted = false;
unsigned long sequenceStartTime = 0;
unsigned long elapsedSequenceTime = 0;

String currentCommand = "";
long target0 = 0;
long target1 = 0;

long prevMicros = 0;
float deltaT;

bool stopRequested = false;

// ----------------------------------------------------------------------
class SimplePID {
private:
  float kp, kd, ki, umax;
  float eprev, eintegral;
public:
  SimplePID()
    : kp(1), kd(0), ki(0), umax(255), eprev(0), eintegral(0) {}
  void setParams(float _kp, float _kd, float _ki, float _umax) {
    kp = _kp;
    kd = _kd;
    ki = _ki;
    umax = _umax;
  }
  void evalu(int value, int targ, float dT, int &pwr, int &dir) {
    int e = targ - value;
    float dedt = (e - eprev) / dT;
    eintegral += e * dT;
    float u = kp * e + kd * dedt + ki * eintegral;
    pwr = min((int)fabs(u), (int)umax);
    dir = (u < 0) ? -1 : 1;
    eprev = e;
  }
};
SimplePID pid0, pid1;

SoftwareSerial ArduinoUno(6, 13);  // RX, TX

bool isQueueEmpty() {
  return queueStart == queueEnd;
}
bool isQueueFull() {
  return ((queueEnd + 1) % MAX_QUEUE_SIZE) == queueStart;
}

void enqueue(const String &cmd) {
  if (!isQueueFull()) {
    commandQueue[queueEnd] = cmd;
    queueEnd = (queueEnd + 1) % MAX_QUEUE_SIZE;
  }
}

String dequeue() {
  if (isQueueEmpty()) return "";
  String s = commandQueue[queueStart];
  queueStart = (queueStart + 1) % MAX_QUEUE_SIZE;
  return s;
}

void clearQueue() {
  queueStart = queueEnd = 0;
}

void setMotor(int dir, int pwmVal, int pwmPin, int inA, int inB) {
  analogWrite(pwmPin, pwmVal);
  if (dir > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else if (dir < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  }
}

void readEncoder0() {
  if (digitalRead(encb0)) posi0++;
  else posi0--;
}
void readEncoder1() {
  if (digitalRead(encb1)) posi1++;
  else posi1--;
}

void processInputString(String &s) {
  int idx;
  while ((idx = s.indexOf('|')) != -1) {
    String tok = s.substring(0, idx);
    if (tok == "STOP") {
      stopRequested = true;
    } else if (tok.length()) {
      enqueue(tok);
    }
    s = s.substring(idx + 1);
  }

  if (s.length()) {
    if (s == "STOP") {
      stopRequested = true;
      sequenceTimerStarted = false;
    } else {
      enqueue(s);
    }
  }
}

void processNextCommand() {
  currentCommand = dequeue();
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posi0 = posi1 = 0;
  }
  reachedTarget = false;
  commandActive = true;
  tStart = millis();

  if (currentCommand.startsWith("11")) {
    target0 = 180;
    target1 = -180;
  } else if (currentCommand == "01") {
    target0 = 95;
    target1 = 95;
  } else if (currentCommand == "10") {
    target0 = -95;
    target1 = -95;
  } else {
    target0 = target1 = 0;
  }
  ArduinoUno.print(">> CMD: ");
  ArduinoUno.println(currentCommand);
}

void setup() {
  Serial.begin(9600);
  ArduinoUno.begin(4800);

  pinMode(enca0, INPUT_PULLUP);
  pinMode(encb0, INPUT_PULLUP);
  pinMode(enca1, INPUT_PULLUP);
  pinMode(encb1, INPUT_PULLUP);
  pinMode(pwm0, OUTPUT);
  pinMode(in1_0, OUTPUT);
  pinMode(in2_0, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(in1_1, OUTPUT);
  pinMode(in2_1, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(enca0), readEncoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(enca1), readEncoder1, RISING);

  pid0.setParams(2, 0, 0, 130);
  pid1.setParams(2, 0, 0, 130);
  prevMicros = micros();

  ArduinoUno.println("Ready.");
}
int commandsDone=0;
void loop() {
  static String inBuffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processInputString(inBuffer);
      inBuffer = "";

      if (!sequenceTimerStarted) {
        sequenceStartTime = millis();
        sequenceTimerStarted = true;
        ArduinoUno.println("** Sequence timer started **");
      }
    } else {
      inBuffer += c;
    }
  }

  if (sequenceTimerStarted) {
    elapsedSequenceTime = millis() - sequenceStartTime;
  }

  // If STOP (00) was received and command has just finished
  if (stopRequested && !commandActive) {
    clearQueue();  // remove all subsequent commands

    int executedCommands = elapsedSequenceTime / 2000 + (elapsedSequenceTime % 2000 != 0);
    Serial.println(String(commandsDone));  // send to NodeMCU
    commandsDone=0;
    ArduinoUno.print(">> STOP received. Commands executed: ");
    ArduinoUno.println(executedCommands);

    stopRequested = false;
    return;  // prevent running any more commands
  }

  if (!commandActive && !isQueueEmpty() && !stopRequested) {
    processNextCommand();

  }

  if (commandActive) {
    unsigned long now = millis();
    unsigned long m = micros();
    deltaT = (m - prevMicros) / 1e6;
    prevMicros = m;

    if (!reachedTarget && (currentCommand == "11" || currentCommand == "01" || currentCommand == "10")) {
      int p0, d0, p1, d1;
      if (currentCommand == "11") {
        pid0.evalu(posi0, target0, deltaT, p0, d0);
        pid1.evalu(posi1, target1, deltaT, p1, d1);
      } else {
        pid0.evalu(posi0, target0, deltaT, p0, d0);
        pid1.evalu(posi1, target0, deltaT, p1, d1);
      }
      setMotor(d0, p0, pwm0, in1_0, in2_0);
      setMotor(d1, p1, pwm1, in1_1, in2_1);

      bool ok = (abs(posi0 - target0) < 20 && abs(posi1 - (currentCommand == "11" ? target1 : target0)) < 20);
      if (ok) {
        setMotor(0, 0, pwm0, in1_0, in2_0);
        setMotor(0, 0, pwm1, in1_1, in2_1);
        reachedTarget = true;
        commandsDone++;
        ArduinoUno.println("  – reached early, holding motors off…");
      }
    }
    if (!reachedTarget && !(currentCommand == "11" || currentCommand == "01" || currentCommand == "10")) {
      setMotor(0, 0, pwm0, in1_0, in2_0);
      setMotor(0, 0, pwm1, in1_1, in2_1);
      reachedTarget = true;
      
    }
    if (now - tStart >= CMD_DURATION) {
      commandActive = false;
      ArduinoUno.println("<< CMD complete");
      delay(50);
    }
  }

  delay(10);
}
