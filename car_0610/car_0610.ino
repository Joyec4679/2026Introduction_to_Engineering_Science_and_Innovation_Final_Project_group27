// =========================================================================
// 專案：模組化狀態機循跡車 (ESP32 - 新版 3.0+ API 專用)
// 架構：同學修改版腳位 -> 輸入層 -> 大腦層(狀態機) -> 輸出層(硬體抽象)
// =========================================================================

// ==========================================
// 0. 腳位與參數定義 (同學修改版)
// ==========================================
// RGB LED 引腳
const int LED_R = 17;
const int LED_G = 23; // 已移至 P23
const int LED_B = 18;

// 感測器引腳：L2, L1, C, R1, R2
// 為了程式可讀性與對齊，我們將陣列解開對應到名稱
const int SENSOR_L2 = 16; // 陣列中的 16 (OUT1 最左)
const int SENSOR_L1 = 4;  // 陣列中的 4  (OUT2)
const int SENSOR_M  = 21; // 陣列中的 21 (OUT3 正中)
const int SENSOR_R1 = 22; // 陣列中的 22 (OUT4)
const int SENSOR_R2 = 34; // 陣列中的 34 (OUT5 最右，安全腳)

// TB6612FNG 馬達驅動引腳
const int MOTOR_PWMA = 32;
const int MOTOR_AIN1 = 25;
const int MOTOR_AIN2 = 33;
const int MOTOR_PWMB = 19; // 已移至 P19
const int MOTOR_BIN1 = 27;
const int MOTOR_BIN2 = 14;
const int MOTOR_STBY = 26;

// ESP32 PWM 參數設定
const int PWM_FREQ = 5000;      // 5kHz
const int PWM_RESOLUTION = 8;   // 速度範圍 0 ~ 255

// 速度與 PID 核心參數 (可根據現場賽道調整)
const int BASE_SPEED = 160;     // 直線基礎速度
const int TURN_SPEED = 180;     // 直角彎盲轉速度
float Kp = 45.0;                // P 參數
float Kd = 25.0;                // D 參數
float Ki = 0.0;                 // I 參數

// 全域控制變數
int last_error = 0;             
int last_valid_error = 0; 
// 在第 0 區的 last_valid_error 下方加上這兩行
unsigned long lost_start_time = 0; // 紀錄開始迷路的時間
bool is_lost_counting = false;     // 是否正在迷路計時中      

// ==========================================
// 1. 定義系統狀態 (System States)
// ==========================================
enum CarState {
  STATE_NORMAL_PID,  // 藍燈：常規 PID 循跡（直線與弧線）
  STATE_LEFT_90,     // 紅燈：觸發左直角彎
  STATE_RIGHT_90,    // 綠燈：觸發右直角彎
  STATE_LOST         // 黃燈：衝出賽道迷路
};

CarState current_state = STATE_NORMAL_PID; 

// ==========================================
// 函式前方宣告 (Forward Declarations - 解決定義順序問題)
// ==========================================
void setLED(int r, int g, int b);
void robotForward(int leftSpeed, int rightSpeed);
void robotSpinLeft(int speed);
void robotSpinRight(int speed);
void robotBrake();
CarState updateState();
int calculate_error();
void run_PID_tracking();
void handle_left_90();
void handle_right_90();
void handle_lost_track();

// ==========================================
// 2. 初始化設定 (Setup)
// ==========================================
void setup() {
  Serial.begin(115200);

  // 初始化輸入層 (感測器)
  pinMode(SENSOR_L2, INPUT);
  pinMode(SENSOR_L1, INPUT);
  pinMode(SENSOR_M,  INPUT);
  pinMode(SENSOR_R1, INPUT);
  pinMode(SENSOR_R2, INPUT);

  // 初始化輸出層 (馬達)
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  pinMode(MOTOR_BIN1, OUTPUT);
  pinMode(MOTOR_BIN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH); // 啟動馬達驅動晶片

  // ESP32 新版 (3.0+) PWM 配置
  ledcAttach(MOTOR_PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_PWMB, PWM_FREQ, PWM_RESOLUTION);

  // 初始化指示燈
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  
  // 開機動畫：白燈亮 1 秒，代表系統正常
  setLED(1, 1, 1); delay(1000); setLED(0, 0, 0);
}

// ==========================================
// 3. 主程式大腦層 (Control Layer - 狀態機)
// ==========================================
void loop() {
  // 【一、輸入層】更新目前的系統狀態
  current_state = updateState();

  // 【二、大腦與輸出層】依據不同狀態分流執行
  switch (current_state) {
    
    case STATE_NORMAL_PID:
      setLED(0, 0, 1);       // 藍燈
      run_PID_tracking();    // 執行 PID 弧線/直線循跡
      break;

    case STATE_LEFT_90:
      setLED(1, 0, 0);       // 紅燈
      handle_left_90();      // 執行左直角處理邏輯
      break;

    case STATE_RIGHT_90:
      setLED(0, 1, 0);       // 綠燈
      handle_right_90();     // 執行右直角處理邏輯
      break;

    case STATE_LOST:
      setLED(1, 1, 0);       // 黃燈 (紅+綠)
      handle_lost_track();   // 執行衝出賽道搜線邏輯
      break;
  }
}

/// ==========================================
// 4. 軟體輸入層函式 (Input Layer)
// ==========================================
// ==========================================
// 4. 軟體輸入層函式 (Input Layer)
// ==========================================

/**
 * 計算環境誤差值 (Error)
 */
int calculate_error() {
  // 【關鍵修正】加上驚嘆號 (!)，將電子訊號的 11111(白底) 轉換為大腦好理解的 00000
  int l2 = !digitalRead(SENSOR_L2);
  int l1 = !digitalRead(SENSOR_L1);
  int m  = !digitalRead(SENSOR_M);
  int r1 = !digitalRead(SENSOR_R1);
  int r2 = !digitalRead(SENSOR_R2);

  // 權重計算法轉換誤差 (此時 1 代表黑線，0 代表白底)
       if (l2 == 1 && l1 == 0 && m == 0 && r1 == 0 && r2 == 0) return -4;
  else if (l2 == 1 && l1 == 1 && m == 0 && r1 == 0 && r2 == 0) return -3;
  else if (l2 == 0 && l1 == 1 && m == 0 && r1 == 0 && r2 == 0) return -2;
  else if (l2 == 0 && l1 == 1 && m == 1 && r1 == 0 && r2 == 0) return -1;
  else if (l2 == 0 && l1 == 0 && m == 1 && r1 == 0 && r2 == 0) return  0; // 正中央
  else if (l2 == 0 && l1 == 0 && m == 1 && r1 == 1 && r2 == 0) return  1;
  else if (l2 == 0 && l1 == 0 && m == 0 && r1 == 1 && r2 == 0) return  2;
  else if (l2 == 0 && l1 == 0 && m == 0 && r1 == 1 && r2 == 1) return  3;
  else if (l2 == 0 && l1 == 0 && m == 0 && r1 == 0 && r2 == 1) return  4;
  
  return last_error; 
}

/**
 * 狀態識別器 
 */
CarState updateState() {
  // 同樣加上驚嘆號 (!)，統一輸入層邏輯
  int l2 = !digitalRead(SENSOR_L2);
  int l1 = !digitalRead(SENSOR_L1);
  int m  = !digitalRead(SENSOR_M);
  int r1 = !digitalRead(SENSOR_R1);
  int r2 = !digitalRead(SENSOR_R2);

  // 1. 判斷是否迷路 (這下子全白底真的會變成 00000 了！)
  if (l2 == 0 && l1 == 0 && m == 0 && r1 == 0 && r2 == 0) {
    return STATE_LOST;
  }
  // 2. 判斷左直角彎特判 
  else if (l2 == 1 && r2 == 0) {
    return STATE_LEFT_90;
  }
  // 3. 判斷右直角彎特判 
  else if (r2 == 1 && l2 == 0) {
    return STATE_RIGHT_90;
  }
  // 4. 其餘情況皆屬於常規循跡路段
  else {
    return STATE_NORMAL_PID;
  }
}
// ==========================================
// 5. 大腦演算法處理函式 (Control Algorithms)
// ==========================================

void run_PID_tracking() {
  // 在 void run_PID_tracking() 的大括號一進來加上這行
  is_lost_counting = false; // 只要回到常規循跡，就重置迷路計時器 
  int error = calculate_error();
  // 如果誤差有效（不為0），就紀錄為最後有效誤差，留給迷路狀態使用
  if (error != 0) {
    last_valid_error = error;
  }

  // PID 核心公式
  int P = error;
  int D = error - last_error;
  int output = (Kp * P) + (Kd * D);
  last_error = error;

  // 根據 PID 輸出修正左右輪速度
  int left_speed  = BASE_SPEED + output;
  int right_speed = BASE_SPEED - output;

  robotForward(left_speed, right_speed);
}

void handle_left_90() {
  // 1. 煞車消除慣性
  robotBrake();
  delay(50); // 停頓 0.05 秒穩定車身

  // 2. 盲轉跨過當前黑線
  robotSpinLeft(TURN_SPEED);
  delay(80); 

  // 3. 尋找新黑線 (白底是 HIGH，所以要在 HIGH 的時候持續尋找！)
  while (digitalRead(SENSOR_M) == HIGH) {
    robotSpinLeft(TURN_SPEED);
    delay(1); 
  }

  // 4. 找到黑線，急煞回正
  robotBrake();
  delay(30);
  last_error = 0; 
}

void handle_right_90() {
  robotBrake();
  delay(50); // 煞車消除慣性

  robotSpinRight(TURN_SPEED);
  delay(80); 

  // 白底是 HIGH，持續右轉直到感測器讀到 LOW (黑線)
  while (digitalRead(SENSOR_M) == HIGH) {
    robotSpinRight(TURN_SPEED);
    delay(1);
  }

  robotBrake();
  delay(30);
  last_error = 0;
}

void handle_lost_track() {
  // 1. 如果是剛進入迷路狀態，開始計時
  if (!is_lost_counting) {
    lost_start_time = millis(); // 抓取當前的系統時間（毫秒）
    is_lost_counting = true;
  }

  // 2. 計算已經迷路了多久
  unsigned long duration = millis() - lost_start_time;

  // 3. 如果迷路超過 5000 毫秒（5秒），強制煞車，亮紅燈，退出程式
  if (duration > 5000) {
    robotBrake();
    setLED(1, 0, 0); // 亮紅燈代表超時受罰停機
    while(1);        // 鎖死在這裡，直到人工重置開關
  }

  // 4. 在 5 秒內，繼續執行原本的原地搜線邏輯
  if (last_valid_error < 0) {
    robotSpinLeft(BASE_SPEED);
  } else {
    robotSpinRight(BASE_SPEED);
  }
}

void robotForward(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // 還原為正確的前進方向 (A與B皆為 HIGH/LOW)
  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, LOW);
  ledcWrite(MOTOR_PWMA, leftSpeed); 

  digitalWrite(MOTOR_BIN1, HIGH);
  digitalWrite(MOTOR_BIN2, LOW);
  ledcWrite(MOTOR_PWMB, rightSpeed); 
}

void robotSpinLeft(int speed) {
  speed = constrain(speed, 0, 255);

  // 左轉：左輪後退(LOW/HIGH)，右輪前進(HIGH/LOW)
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, HIGH);
  ledcWrite(MOTOR_PWMA, speed);

  digitalWrite(MOTOR_BIN1, HIGH);
  digitalWrite(MOTOR_BIN2, LOW);
  ledcWrite(MOTOR_PWMB, speed);
}

void robotSpinRight(int speed) {
  speed = constrain(speed, 0, 255);

  // 右轉：左輪前進(HIGH/LOW)，右輪後退(LOW/HIGH)
  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, LOW);
  ledcWrite(MOTOR_PWMA, speed);

  digitalWrite(MOTOR_BIN1, LOW);
  digitalWrite(MOTOR_BIN2, HIGH);
  ledcWrite(MOTOR_PWMB, speed);
}

void robotBrake() {
  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, HIGH);
  digitalWrite(MOTOR_BIN1, HIGH);
  digitalWrite(MOTOR_BIN2, HIGH);
  ledcWrite(MOTOR_PWMA, 0);
  ledcWrite(MOTOR_PWMB, 0);
}

void setLED(int r, int g, int b) {
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
}