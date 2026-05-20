// ===== 感測器設定 =====
// 從左到右：最左、左中、中、右中、最右
const int sensorPins[5] = {32, 33, 34, 35, 36};

// 如果測試後發現黑線輸出 LOW，設 true；黑線輸出 HIGH，設 false
const bool BLACK_IS_LOW = true;

// ===== L298N 馬達腳位 =====
const int ENA = 25;  // 左馬達 PWM
const int IN1 = 26;
const int IN2 = 27;

const int ENB = 13;  // 右馬達 PWM
const int IN3 = 14;
const int IN4 = 23;

// ===== 速度設定，0~255 =====
int baseSpeed = 210;
int smallTurn = 170;
int bigTurn = 130;
int searchSpeed = 180;

int lastDirection = 0; 
// -1 表示上次偏左，1 表示上次偏右，0 表示直走

void setup() {
  Serial.begin(115200);
  Serial.println("hi");

  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, 30000, 8);
  ledcAttach(ENB, 30000, 8);

  Serial.println("Test setMotor");
  setMotor(200, 200);
  delay(3000);

  stopMotors();
}

bool isBlack(int pin) {
  int v = digitalRead(pin);
  if (BLACK_IS_LOW) {
    return v == LOW;
  } else {
    return v == HIGH;
  }
}

void setMotor(int leftSpeed, int rightSpeed) {
  // leftSpeed / rightSpeed: -255 ~ 255
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // 左馬達
  if (leftSpeed >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(ENA, leftSpeed);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, -leftSpeed);
  }

  // 右馬達
  if (rightSpeed >= 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(ENB, rightSpeed);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB, -rightSpeed);
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

void loop() {
  bool L2 = isBlack(sensorPins[0]);
  bool L1 = isBlack(sensorPins[1]);
  bool C  = isBlack(sensorPins[2]);
  bool R1 = isBlack(sensorPins[3]);
  bool R2 = isBlack(sensorPins[4]);

  Serial.print("Sensors: ");
  Serial.print(L2); Serial.print(" ");
  Serial.print(L1); Serial.print(" ");
  Serial.print(C);  Serial.print(" ");
  Serial.print(R1); Serial.print(" ");
  Serial.print(R2); Serial.print("  => ");

  if (C) {
    Serial.println("Forward");
    setMotor(baseSpeed, baseSpeed);
    lastDirection = 0;
  }
  else if (L1) {
    Serial.println("Small Left");
    setMotor(smallTurn, baseSpeed);
    lastDirection = -1;
  }
  else if (L2) {
    Serial.println("Big Left");
    setMotor(bigTurn, baseSpeed);
    lastDirection = -1;
  }
  else if (R1) {
    Serial.println("Small Right");
    setMotor(baseSpeed, smallTurn);
    lastDirection = 1;
  }
  else if (R2) {
    Serial.println("Big Right");
    setMotor(baseSpeed, bigTurn);
    lastDirection = 1;
  }
  else {
    Serial.println("Search Line");

    if (lastDirection < 0) {
      setMotor(-searchSpeed, searchSpeed);
    } else if (lastDirection > 0) {
      setMotor(searchSpeed, -searchSpeed);
    } else {
      setMotor(searchSpeed, searchSpeed);
    }
  }

  delay(50);
}
