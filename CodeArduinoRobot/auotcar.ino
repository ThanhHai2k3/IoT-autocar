#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <queue>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <nvs_flash.h>

// Thông tin WiFi
const char* ssid = "duke"; // Thay bằng tên mạng WiFi của bạn
const char* password = "123456789";      // Thay bằng mật khẩu WiFi (để trống nếu không có)

// Kích thước màn hình OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Định nghĩa chân kết nối cho cảm biến và động cơ
const int trigPin = 5;  // Chân Trig của cảm biến HC-SR04
const int echoPin = 18; // Chân Echo của cảm biến HC-SR04
const int servoPin = 13; // Chân điều khiển servo
#define IN1 12 // Chân điều khiển động cơ 1 (L298N)
#define IN2 14 // Chân điều khiển động cơ 1 (L298N)
#define IN3 27 // Chân điều khiển động cơ 2 (L298N)
#define IN4 26 // Chân điều khiển động cơ 2 (L298N)

// Hằng số thời gian và các giá trị cố định
const int TURN_DURATION = 525; // Thời gian quay (ms)
const int MOVE_DURATION = 500; // Thời gian di chuyển thẳng (ms)
const float TARGET_DISTANCE = 20.0; // Khoảng cách mục tiêu để phát hiện chướng ngại vật (cm)
const long displayInterval = 1000; // Khoảng thời gian cập nhật hiển thị (ms)
const long wifiCheckInterval = 5000; // Khoảng thời gian kiểm tra WiFi (ms)
const int MAX_DISTANCE_ATTEMPTS = 5; // Số lần thử đo khoảng cách tối đa
const int MAX_QUEUE_SIZE = 100; // Kích thước tối đa của hàng đợi BFS
const int WIFI_MAX_ATTEMPTS = 20; // Số lần thử kết nối WiFi tối đa
const int MAX_CONSECUTIVE_TURNS = 10; // Số lần quay liên tiếp tối đa trước khi dừng
const int SERVO_INIT_ATTEMPTS = 3; // Số lần thử khởi tạo servo tối đa

// Bản đồ và hàng đợi
#define GRID_SIZE 10 // Kích thước lưới (10x10)
int grid[GRID_SIZE][GRID_SIZE] = {0}; // Lưới lưu trạng thái ô (0: chưa thăm, 1: đã thăm, 2: chướng ngại vật, 3: robot)
float wifiMap[GRID_SIZE][GRID_SIZE] = {0}; // Lưu RSSI WiFi tại mỗi ô
int robotX = 0, robotY = 0; // Tọa độ hiện tại của robot
int robotHeading = 0; // Hướng của robot (0: Bắc, 90: Đông, 180: Nam, 270: Tây)
int consecutiveTurns = 0; // Đếm số lần quay liên tiếp (tránh quay vòng)

// Theo dõi RSSI WiFi cao nhất
float maxWifiSpeed = -100; // RSSI WiFi cao nhất (dBm, khởi tạo thấp để cập nhật)
int maxWifiSpeedX = -1; // Tọa độ X của ô có RSSI cao nhất
int maxWifiSpeedY = -1; // Tọa độ Y của ô có RSSI cao nhất

// Cấu trúc và hàng đợi cho thuật toán BFS
struct Position {
  int x, y, heading; // Vị trí (x, y) và hướng của robot
};
std::queue<Position> posQueue; // Hàng đợi lưu các vị trí để quay lại (dùng trong BFS)

// Khởi tạo web server trên cổng 80
AsyncWebServer server(80);

// Khởi tạo các đối tượng và biến toàn cục
Servo servo; // Đối tượng điều khiển servo
float distanceLeft = 0, distanceRight = 0, distanceFront = 0; // Khoảng cách bên trái, phải, phía trước
String direction = "F"; // Hướng di chuyển hiện tại (F: Forward, L: Left, R: Right)
float wifiSpeed = 0; // RSSI WiFi hiện tại (dBm)
unsigned long lastWiFiDisplayTime = 0; // Thời gian hiển thị RSSI cuối cùng
unsigned long lastDistanceDisplayTime = 0; // Thời gian hiển thị khoảng cách cuối cùng
unsigned long lastWiFiCheckTime = 0; // Thời gian kiểm tra WiFi cuối cùng
bool isFinished = false; // Trạng thái hoàn thành khám phá
bool wifiConnected = false; // Trạng thái kết nối WiFi
bool isDisplayInitialized = false; // Trạng thái khởi tạo màn hình OLED
bool resetFlag = true; // Cờ báo robot vừa được reset

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting OLED, Servo, HC-SR04, L298N, and WiFi test...");

  resetFlag = true; // Đặt cờ reset khi khởi động

  // Xóa bộ nhớ NVS
  esp_err_t ret = nvs_flash_erase();
  if (ret == ESP_OK) {
    Serial.println("NVS erased successfully.");
  } else {
    Serial.println("Failed to erase NVS: " + String(ret));
  }
  ret = nvs_flash_init();
  if (ret == ESP_OK) {
    Serial.println("NVS initialized successfully.");
  } else {
    Serial.println("Failed to initialize NVS: " + String(ret));
  }

  // Khởi tạo giao tiếp I2C cho OLED
  Wire.begin(22, 23);
  Serial.println("I2C initialized.");

  // Khởi tạo màn hình OLED
  isDisplayInitialized = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!isDisplayInitialized) {
    Serial.println("OLED initialization failed, continuing without OLED.");
  } else {
    Serial.println("OLED initialized successfully.");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Initializing...");
    display.display();
  }

  // Khởi tạo servo và cảm biến HC-SR04
  Serial.println("Initializing Servo and HC-SR04...");
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  bool servoAttached = servo.attach(servoPin);
  if (!servoAttached) {
    Serial.println("Servo initialization failed.");
    for (int i = 0; i < SERVO_INIT_ATTEMPTS; i++) {
      delay(500);
      if (servo.attach(servoPin)) {
        servoAttached = true;
        Serial.println("Servo initialized on attempt " + String(i + 1));
        break;
      }
      Serial.println("Servo initialization failed. Attempt " + String(i + 1));
    }
  }
  if (servoAttached) {
    servo.write(90); // Đặt servo về vị trí giữa
    delay(1000);
  }

  // Khởi tạo chân điều khiển động cơ L298N
  Serial.println("Initializing L298N pins...");
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors(); // Dừng động cơ ban đầu

  // Kết nối WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < WIFI_MAX_ATTEMPTS) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed, continuing without WiFi.");
    wifiConnected = false;
    if (isDisplayInitialized) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Connect Wifi FAILED");
      display.display();
    }
  } else {
    Serial.println("\nWiFi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;

    // Thiết lập endpoint "/data" để gửi dữ liệu robot
    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
      DynamicJsonDocument doc(2048);
      doc["robotX"] = robotX;
      doc["robotY"] = robotY;
      doc["robotHeading"] = robotHeading;
      doc["isFinished"] = isFinished;
      doc["maxWifiSpeed"] = maxWifiSpeed;
      doc["maxWifiSpeedX"] = maxWifiSpeedX;
      doc["maxWifiSpeedY"] = maxWifiSpeedY;
      doc["wifiSpeed"] = wifiSpeed;
      doc["distanceLeft"] = distanceLeft;
      doc["distanceRight"] = distanceRight;
      doc["distanceFront"] = distanceFront;
      doc["direction"] = direction;
      doc["resetFlag"] = resetFlag; // Thêm resetFlag vào JSON

      JsonArray gridArray = doc.createNestedArray("grid");
      for (int i = 0; i < GRID_SIZE; i++) {
        JsonArray row = gridArray.createNestedArray();
        for (int j = 0; j < GRID_SIZE; j++) {
          row.add(grid[i][j]);
        }
      }

      JsonArray wifiMapArray = doc.createNestedArray("wifiMap");
      for (int i = 0; i < GRID_SIZE; i++) {
        JsonArray row = wifiMapArray.createNestedArray();
        for (int j = 0; j < GRID_SIZE; j++) {
          row.add(wifiMap[i][j]);
        }
      }

      String output;
      serializeJson(doc, output);
      request->send(200, "application/json", output);
      Serial.println("Sent data to client.");

      resetFlag = false; // Đặt lại resetFlag sau khi gửi lần đầu
    });

    // Thiết lập endpoint "/command" để nhận lệnh từ ứng dụng Android
    server.on("/command", HTTP_POST, [](AsyncWebServerRequest *request){
      if (request->hasParam("action", true)) {
        String action = request->getParam("action", true)->value();
        Serial.print("Received command: ");
        Serial.println(action);
        if (action == "stop") {
          stopMotors();
          isFinished = true;
          Serial.println("Robot stopped via command. isFinished set to true.");
          request->send(200, "text/plain", "Robot stopped");
          if (isDisplayInitialized) {
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println("Stopped by User");
            display.display();
          }
        } else if (action == "forward") {
          if (!isFinished) {
            moveForward();
            updatePosition("F");
            consecutiveTurns = 0;
            Serial.println("Robot moved forward.");
            request->send(200, "text/plain", "Robot moved forward");
          } else {
            request->send(400, "text/plain", "Robot is stopped");
          }
        } else if (action == "left") {
          if (!isFinished) {
            turnLeft();
            updatePosition("L");
            consecutiveTurns++;
            Serial.println("Robot turned left.");
            request->send(200, "text/plain", "Robot turned left");
          } else {
            request->send(400, "text/plain", "Robot is stopped");
          }
        } else if (action == "right") {
          if (!isFinished) {
            turnRight();
            updatePosition("R");
            consecutiveTurns++;
            Serial.println("Robot turned right.");
            request->send(200, "text/plain", "Robot moved right");
          } else {
            request->send(400, "text/plain", "Robot is stopped");
          }
        } else {
          Serial.println("Invalid action received.");
          request->send(400, "text/plain", "Invalid action");
        }
      } else {
        Serial.println("Missing action parameter in command request.");
        request->send(400, "text/plain", "Missing action parameter");
      }
    });

    server.begin();
    Serial.println("Web server started at http://" + WiFi.localIP().toString() + "/data");
  }

  // Khởi tạo bản đồ và vị trí ban đầu của robot
  grid[robotX][robotY] = 3; // Robot tại vị trí ban đầu
  posQueue.push({robotX, robotY, robotHeading});
  Serial.println("Setup complete. Robot starts at (0,0).");
}

void resetMap() {
  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      grid[i][j] = 0;
      wifiMap[i][j] = -100; // Reset wifiMap
    }
  }
  robotX = 0;
  robotY = 0;
  robotHeading = 0;
  while (!posQueue.empty()) posQueue.pop();
  posQueue.push({robotX, robotY, robotHeading});
  grid[robotX][robotY] = 3;
  direction = "F";
  maxWifiSpeed = -100;
  maxWifiSpeedX = -1;
  maxWifiSpeedY = -1;
  consecutiveTurns = 0;
  resetFlag = true; // Đặt lại resetFlag khi bản đồ được reset
  Serial.println("Map and robot state reset.");
}

void exportData() {
  DynamicJsonDocument doc(2048);
  doc["robotX"] = robotX;
  doc["robotY"] = robotY;
  doc["robotHeading"] = robotHeading;
  doc["isFinished"] = isFinished;
  doc["maxWifiSpeed"] = maxWifiSpeed;
  doc["maxWifiSpeedX"] = maxWifiSpeedX;
  doc["maxWifiSpeedY"] = maxWifiSpeedY;
  doc["wifiSpeed"] = wifiSpeed;
  doc["distanceLeft"] = distanceLeft;
  doc["distanceRight"] = distanceRight;
  doc["distanceFront"] = distanceFront;
  doc["direction"] = direction;

  JsonArray gridArray = doc.createNestedArray("grid");
  for (int i = 0; i < GRID_SIZE; i++) {
    JsonArray row = gridArray.createNestedArray();
    for (int j = 0; j < GRID_SIZE; j++) {
      row.add(grid[i][j]);
    }
  }

  JsonArray wifiMapArray = doc.createNestedArray("wifiMap");
  for (int i = 0; i < GRID_SIZE; i++) {
    JsonArray row = wifiMapArray.createNestedArray();
    for (int j = 0; j < GRID_SIZE; j++) {
      row.add(wifiMap[i][j]);
    }
  }

  String output;
  serializeJson(doc, output);
  Serial.println("Exported Data: " + output);
}

bool hasUnvisitedCells() {
  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      if (grid[i][j] == 0) {
        return true;
      }
    }
  }
  return false;
}

bool hasValidPath() {
  bool frontValid = (distanceFront > TARGET_DISTANCE);
  bool rightValid = (distanceRight > TARGET_DISTANCE);
  bool leftValid = (distanceLeft > TARGET_DISTANCE);

  int frontX = robotX, frontY = robotY;
  int rightX = robotX, rightY = robotY;
  int leftX = robotX, leftY = robotY;

  if (robotHeading == 0) {
    frontY--; rightX++; leftX--;
  } else if (robotHeading == 90) {
    frontX++; rightY++; leftY--;
  } else if (robotHeading == 180) {
    frontY++; rightX--; leftX++;
  } else if (robotHeading == 270) {
    frontX--; rightY--; leftY++;
  }

  bool frontFree = frontValid && frontX >= 0 && frontX < GRID_SIZE && frontY >= 0 && frontY < GRID_SIZE && (grid[frontX][frontY] == 0 || grid[frontX][frontY] == 1);
  bool rightFree = rightValid && rightX >= 0 && rightX < GRID_SIZE && rightY >= 0 && rightY < GRID_SIZE && (grid[rightX][rightY] == 0 || grid[rightX][rightY] == 1);
  bool leftFree = leftValid && leftX >= 0 && leftX < GRID_SIZE && leftY >= 0 && leftY < GRID_SIZE && (grid[leftX][leftY] == 0 || grid[leftX][leftY] == 1);

  Serial.print("hasValidPath: frontFree=");
  Serial.print(frontFree);
  Serial.print(" (");
  Serial.print(frontX);
  Serial.print(",");
  Serial.print(frontY);
  Serial.print("), rightFree=");
  Serial.print(rightFree);
  Serial.print(" (");
  Serial.print(rightX);
  Serial.print(",");
  Serial.print(rightY);
  Serial.print("), leftFree=");
  Serial.print(leftFree);
  Serial.print(" (");
  Serial.print(leftX);
  Serial.print(",");
  Serial.print(leftY);
  Serial.println(")");

  return frontFree || rightFree || leftFree;
}

bool findNearestVisitedWithUnvisited(int &targetX, int &targetY) {
  bool visited[GRID_SIZE][GRID_SIZE] = {false};
  std::queue<std::pair<int, int>> q;
  q.push({robotX, robotY});
  visited[robotX][robotY] = true;

  int directions[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}}; // Bắc, Đông, Nam, Tây

  while (!q.empty()) {
    int x = q.front().first;
    int y = q.front().second;
    q.pop();

    if (grid[x][y] == 1 && hasUnvisitedNeighbor(x, y)) {
      targetX = x;
      targetY = y;
      Serial.print("Found nearest visited with unvisited at (");
      Serial.print(targetX);
      Serial.print(", ");
      Serial.print(targetY);
      Serial.println(")");
      return true;
    }

    for (int i = 0; i < 4; i++) {
      int nx = x + directions[i][0];
      int ny = y + directions[i][1];
      if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE && !visited[nx][ny] && (grid[nx][ny] == 1 || grid[nx][ny] == 0)) {
        q.push({nx, ny});
        visited[nx][ny] = true;
      }
    }
  }
  Serial.println("No visited cells with unvisited neighbors found.");
  return false;
}

bool findNearestVisited(int &targetX, int &targetY) {
  bool visited[GRID_SIZE][GRID_SIZE] = {false};
  std::queue<std::pair<int, int>> q;
  q.push({robotX, robotY});
  visited[robotX][robotY] = true;

  int directions[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}}; // Bắc, Đông, Nam, Tây

  while (!q.empty()) {
    int x = q.front().first;
    int y = q.front().second;
    q.pop();

    if (grid[x][y] == 1 && (x != robotX || y != robotY)) {
      targetX = x;
      targetY = y;
      Serial.print("Found nearest visited cell at (");
      Serial.print(targetX);
      Serial.print(", ");
      Serial.print(targetY);
      Serial.println(")");
      return true;
    }

    for (int i = 0; i < 4; i++) {
      int nx = x + directions[i][0];
      int ny = y + directions[i][1];
      if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE && !visited[nx][ny] && (grid[nx][ny] == 1 || grid[nx][ny] == 0)) {
        q.push({nx, ny});
        visited[nx][ny] = true;
      }
    }
  }
  Serial.println("No other visited cells found.");
  return false;
}

void loop() {
  if (isFinished) {
    stopMotors();
    Serial.println("Program stopped after completion.");
    exportData();
    if (isDisplayInitialized) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Program Stopped");
      display.display();
    }
    for (;;) {
      delay(1000);
    }
  }

  unsigned long currentTime = millis();

  if (wifiConnected && currentTime - lastWiFiCheckTime >= 10000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, attempting reconnect...");
      wifiConnected = false;
      WiFi.reconnect();
      unsigned long startTime = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
        delay(500);
      }
      if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.println("WiFi reconnected. IP: " + WiFi.localIP().toString());
      }
    }
    lastWiFiCheckTime = currentTime;
  }

  if (currentTime - lastWiFiDisplayTime >= wifiCheckInterval) {
    wifiSpeed = measureWiFiSpeed();
    if (robotX >= 0 && robotX < GRID_SIZE && robotY >= 0 && robotY < GRID_SIZE) {
      wifiMap[robotX][robotY] = wifiSpeed;
      if (wifiSpeed > maxWifiSpeed) {
        maxWifiSpeed = wifiSpeed;
        maxWifiSpeedX = robotX;
        maxWifiSpeedY = robotY;
      }
    }
    if (isDisplayInitialized) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("WiFi RSSI:");
      display.setCursor(0, 8);
      display.print("RSSI: ");
      display.print(wifiSpeed);
      display.println(" dBm");
      display.display();
    }
    Serial.println("Displaying WiFi RSSI on OLED");
    lastWiFiDisplayTime = currentTime;
  }

  Serial.println("Scanning distances...");
  if (!servo.attached()) {
    bool reattached = false;
    for (int i = 0; i < SERVO_INIT_ATTEMPTS; i++) {
      if (servo.attach(servoPin)) {
        reattached = true;
        Serial.println("Servo re-attached on attempt " + String(i + 1));
        break;
      }
      Serial.println("Servo re-attach failed. Attempt " + String(i + 1));
      delay(500);
    }
    if (!reattached) {
      Serial.println("Servo re-attach failed, continuing anyway.");
    }
  }
  int angles[] = {0, 90, 180};
  for (int i = 0; i < 3; i++) {
    int pos = angles[i];
    if (servo.attached()) {
      servo.write(pos);
      delay(500);
    }

    float distance_cm = measureDistance();
    if (pos == 0) distanceRight = distance_cm;
    else if (pos == 90) distanceFront = distance_cm;
    else distanceLeft = distance_cm;

    Serial.print("Angle: ");
    Serial.print(pos);
    Serial.print(" Distance: ");
    Serial.print(distance_cm);
    Serial.println(" cm");
    delay(300);
  }
  if (servo.attached()) {
    servo.detach();
  }

  updateGrid();

  if (!hasUnvisitedCells() || (!hasValidPath() && posQueue.empty() && !findNearestVisitedWithUnvisited(robotX, robotY))) {
    Serial.println("All cells explored or no path to unvisited cells. Exploration complete!");
    if (isDisplayInitialized) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Exploration Complete");
      display.display();
    }
    isFinished = true;
    exportData();
    resetMap();
    return;
  }

  String bestDirection = decideDirection();
  Serial.print("Decided direction: ");
  Serial.println(bestDirection);
  direction = bestDirection;

  if (currentTime - lastDistanceDisplayTime >= displayInterval) {
    if (isDisplayInitialized) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Robot Running...");
      display.setCursor(0, 8);
      display.print("L:");
      display.print(distanceLeft);
      display.print(" R:");
      display.println(distanceRight);
      display.setCursor(0, 16);
      display.print("F:");
      display.print(distanceFront);
      display.print(" Dir:");
      display.println(direction);
      display.display();
    }
    Serial.println("Displaying distances and direction on OLED");
    lastDistanceDisplayTime = currentTime;
  }

  if (bestDirection == "F") {
    Serial.println("Moving forward...");
    moveForward();
    updatePosition("F");
    consecutiveTurns = 0;
  } else if (bestDirection == "L") {
    Serial.println("Turning left...");
    turnLeft();
    updatePosition("L");
    consecutiveTurns++;
  } else if (bestDirection == "R") {
    Serial.println("Turning right...");
    turnRight();
    updatePosition("R");
    consecutiveTurns++;
  } else {
    Serial.println("No valid direction, turning right to scan again...");
    turnRight();
    updatePosition("R");
    consecutiveTurns++;
  }

  if (consecutiveTurns >= MAX_CONSECUTIVE_TURNS) {
    Serial.println("Max consecutive turns reached. Stopping exploration.");
    isFinished = true;
  }

  delay(1000);
}

float measureWiFiSpeed() {
  if (!wifiConnected) {
    Serial.println("WiFi not available, skipping RSSI measurement.");
    return -100;
  }

  Serial.println("Checking WiFi status...");
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, reconnecting...");
    WiFi.reconnect();
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\nWiFi reconnection failed.");
      wifiConnected = false;
      return -100;
    }
    Serial.println("\nWiFi reconnected.");
  }

  long rssi = WiFi.RSSI();
  Serial.print("WiFi RSSI: ");
  Serial.print(rssi);
  Serial.println(" dBm");
  return (float)rssi;
}

float measureDistance() {
  long duration = 0;
  float distance_cm = 0;
  for (int i = 0; i < MAX_DISTANCE_ATTEMPTS; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH, 30000);
    distance_cm = duration * 0.034 / 2;
    Serial.print("Attempt ");
    Serial.print(i + 1);
    Serial.print(": Duration = ");
    Serial.print(duration);
    Serial.print(" us, Distance = ");
    Serial.print(distance_cm);
    Serial.println(" cm");
    if (duration != 0 && distance_cm >= 0) {
      break;
    }
    delay(50);
  }
  if (duration == 0 || distance_cm < 0) {
    Serial.println("HC-SR04: No valid echo received!");
    return 510;
  }
  return distance_cm;
}

void updateGrid() {
  int frontX = robotX, frontY = robotY;
  int leftX = robotX, leftY = robotY;
  int rightX = robotX, rightY = robotY;

  if (robotHeading == 0) {
    frontY--; rightX++; leftX--;
  } else if (robotHeading == 90) {
    frontX++; rightY++; leftY--;
  } else if (robotHeading == 180) {
    frontY++; rightX--; leftX++;
  } else if (robotHeading == 270) {
    frontX--; rightY--; leftY++;
  }

  Serial.print("updateGrid: frontX=");
  Serial.print(frontX);
  Serial.print(", frontY=");
  Serial.print(frontY);
  Serial.print(", distanceFront=");
  Serial.print(distanceFront);
  Serial.println();
  Serial.print("updateGrid: rightX=");
  Serial.print(rightX);
  Serial.print(", rightY=");
  Serial.print(rightY);
  Serial.print(", distanceRight=");
  Serial.print(distanceRight);
  Serial.println();
  Serial.print("updateGrid: leftX=");
  Serial.print(leftX);
  Serial.print(", leftY=");
  Serial.print(leftY);
  Serial.print(", distanceLeft=");
  Serial.print(distanceLeft);
  Serial.println();

  // Chỉ đánh dấu ô là chướng ngại (2) nếu ô hiện tại là chưa thăm (0)
  if (distanceFront >= 0 && distanceFront <= TARGET_DISTANCE && frontX >= 0 && frontX < GRID_SIZE && frontY >= 0 && frontY < GRID_SIZE && grid[frontX][frontY] == 0) {
    grid[frontX][frontY] = 2;
    Serial.print("Marked front (");
    Serial.print(frontX);
    Serial.print(",");
    Serial.print(frontY);
    Serial.println(") as obstacle.");
  }
  if (distanceRight >= 0 && distanceRight <= TARGET_DISTANCE && rightX >= 0 && rightX < GRID_SIZE && rightY >= 0 && rightY < GRID_SIZE && grid[rightX][rightY] == 0) {
    grid[rightX][rightY] = 2;
    Serial.print("Marked right (");
    Serial.print(rightX);
    Serial.print(",");
    Serial.print(rightY);
    Serial.println(") as obstacle.");
  }
  if (distanceLeft >= 0 && distanceLeft <= TARGET_DISTANCE && leftX >= 0 && leftX < GRID_SIZE && leftY >= 0 && leftY < GRID_SIZE && grid[leftX][leftY] == 0) {
    grid[leftX][leftY] = 2;
    Serial.print("Marked left (");
    Serial.print(leftX);
    Serial.print(",");
    Serial.print(leftY);
    Serial.println(") as obstacle.");
  }
}

bool hasUnvisitedNeighbor(int x, int y) {
  int directions[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}}; // Bắc, Đông, Nam, Tây
  for (int i = 0; i < 4; i++) {
    int nx = x + directions[i][0];
    int ny = y + directions[i][1];
    if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE && grid[nx][ny] == 0) {
      return true;
    }
  }
  return false;
}

bool hasNearbyValidCell() {
  int directions[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}}; // Bắc, Đông, Nam, Tây
  for (int i = 0; i < 4; i++) {
    int nx = robotX + directions[i][0];
    int ny = robotY + directions[i][1];
    if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE && (grid[nx][ny] == 0 || grid[nx][ny] == 1)) {
      return true;
    }
  }
  return false;
}

String decideDirection() {
  bool frontValid = (distanceFront > TARGET_DISTANCE);
  bool rightValid = (distanceRight > TARGET_DISTANCE);
  bool leftValid = (distanceLeft > TARGET_DISTANCE);

  int frontX = robotX, frontY = robotY;
  int rightX = robotX, rightY = robotY;
  int leftX = robotX, leftY = robotY;

  if (robotHeading == 0) {
    frontY--; rightX++; leftX--;
  } else if (robotHeading == 90) {
    frontX++; rightY++; leftY--;
  } else if (robotHeading == 180) {
    frontY++; rightX--; leftX++;
  } else if (robotHeading == 270) {
    frontX--; rightY--; leftY++;
  }

  bool frontFreeUnvisited = frontValid && frontX >= 0 && frontX < GRID_SIZE && frontY >= 0 && frontY < GRID_SIZE && grid[frontX][frontY] == 0;
  bool rightFreeUnvisited = rightValid && rightX >= 0 && rightX < GRID_SIZE && rightY >= 0 && rightY < GRID_SIZE && grid[rightX][rightY] == 0;
  bool leftFreeUnvisited = leftValid && leftX >= 0 && leftX < GRID_SIZE && leftY >= 0 && leftY < GRID_SIZE && grid[leftX][leftY] == 0;

  if (frontFreeUnvisited) {
    posQueue.push({robotX, robotY, robotHeading});
    consecutiveTurns = 0; // Đặt lại khi di chuyển hợp lệ
    return "F";
  } else if (rightFreeUnvisited) {
    posQueue.push({robotX, robotY, robotHeading});
    return "R";
  } else if (leftFreeUnvisited) {
    posQueue.push({robotX, robotY, robotHeading});
    return "L";
  }

  // Kiểm tra ô đã thăm có lân cận chưa thăm
  bool frontFreeVisited = frontValid && frontX >= 0 && frontX < GRID_SIZE && frontY >= 0 && frontY < GRID_SIZE && grid[frontX][frontY] == 1 && hasUnvisitedNeighbor(frontX, frontY);
  bool rightFreeVisited = rightValid && rightX >= 0 && rightX < GRID_SIZE && rightY >= 0 && rightY < GRID_SIZE && grid[rightX][rightY] == 1 && hasUnvisitedNeighbor(rightX, rightY);
  bool leftFreeVisited = leftValid && leftX >= 0 && leftX < GRID_SIZE && leftY >= 0 && leftY < GRID_SIZE && grid[leftX][leftY] == 1 && hasUnvisitedNeighbor(leftX, leftY);

  if (frontFreeVisited) {
    posQueue.push({robotX, robotY, robotHeading});
    consecutiveTurns = 0; // Đặt lại khi di chuyển hợp lệ
    return "F";
  } else if (rightFreeVisited) {
    posQueue.push({robotX, robotY, robotHeading});
    return "R";
  } else if (leftFreeVisited) {
    posQueue.push({robotX, robotY, robotHeading});
    return "L";
  }

  // Kiểm tra ô đã thăm bất kỳ
  bool frontFree = frontValid && frontX >= 0 && frontX < GRID_SIZE && frontY >= 0 && frontY < GRID_SIZE && grid[frontX][frontY] == 1;
  bool rightFree = rightValid && rightX >= 0 && rightX < GRID_SIZE && rightY >= 0 && rightY < GRID_SIZE && grid[rightX][rightY] == 1;
  bool leftFree = leftValid && leftX >= 0 && leftX < GRID_SIZE && leftY >= 0 && leftY < GRID_SIZE && grid[leftX][leftY] == 1;

  if (frontFree) {
    posQueue.push({robotX, robotY, robotHeading});
    consecutiveTurns = 0; // Đặt lại khi di chuyển hợp lệ
    return "F";
  } else if (rightFree) {
    posQueue.push({robotX, robotY, robotHeading});
    return "R";
  } else if (leftFree) {
    posQueue.push({robotX, robotY, robotHeading});
    return "L";
  }

  // Quay lại nếu không có đường đi gần
  if (!hasNearbyValidCell() && !posQueue.empty()) {
    Position target = posQueue.front();
    String action = calculateBacktrackAction(target);
    if (action == "F") consecutiveTurns = 0; // Đặt lại khi di chuyển hợp lệ
    return action;
  }

  // Tìm ô đã thăm gần nhất hoặc ô có lân cận chưa thăm
  int targetX, targetY;
  if (findNearestVisitedWithUnvisited(targetX, targetY)) {
    if (robotHeading == 0 && targetY < robotY) return "F";
    if (robotHeading == 90 && targetX > robotX) return "F";
    if (robotHeading == 180 && targetY > robotY) return "F";
    if (robotHeading == 270 && targetX < robotX) return "F";
    if (robotHeading == 0 && targetX > robotX) return "R";
    if (robotHeading == 90 && targetY > robotY) return "R";
    if (robotHeading == 180 && targetX < robotX) return "R";
    if (robotHeading == 270 && targetY < robotY) return "R";
    if (robotHeading == 0 && targetX < robotX) return "L";
    if (robotHeading == 90 && targetY < robotY) return "L";
    if (robotHeading == 180 && targetX > robotX) return "L";
    if (robotHeading == 270 && targetY > robotY) return "L";
  }

  if (findNearestVisited(targetX, targetY)) {
    if (robotHeading == 0 && targetY < robotY) return "F";
    if (robotHeading == 90 && targetX > robotX) return "F";
    if (robotHeading == 180 && targetY > robotY) return "F";
    if (robotHeading == 270 && targetX < robotX) return "F";
    if (robotHeading == 0 && targetX > robotX) return "R";
    if (robotHeading == 90 && targetY > robotY) return "R";
    if (robotHeading == 180 && targetX < robotX) return "R";
    if (robotHeading == 270 && targetY < robotY) return "R";
    if (robotHeading == 0 && targetX < robotX) return "L";
    if (robotHeading == 90 && targetY < robotY) return "L";
    if (robotHeading == 180 && targetX > robotX) return "L";
    if (robotHeading == 270 && targetY > robotY) return "L";
  }

  return "R"; // Quay phải để quét lại nếu không có lựa chọn nào khác
}

String calculateBacktrackAction(Position target) {
  // Tính toán hướng cần quay để đối mặt với vị trí mục tiêu
  int dx = target.x - robotX;
  int dy = target.y - robotY;

  // Xác định hướng mục tiêu dựa trên vị trí tương đối
  int targetHeading = robotHeading;
  if (dx == 1 && dy == 0) targetHeading = 90; // Đông
  else if (dx == -1 && dy == 0) targetHeading = 270; // Tây
  else if (dx == 0 && dy == 1) targetHeading = 180; // Nam
  else if (dx == 0 && dy == -1) targetHeading = 0; // Bắc

  // Tính số lần quay để đạt hướng mục tiêu
  int turns = (targetHeading - robotHeading + 360) % 360 / 90;
  if (turns == 0) {
    // Đã đúng hướng, di chuyển tới và cập nhật posQueue
    posQueue.pop(); // Xóa vị trí mục tiêu khỏi hàng đợi
    return "F";
  } else if (turns == 1) {
    return "R"; // Quay phải
  } else if (turns == 3) {
    return "L"; // Quay trái
  } else {
    // Cần quay 180 độ, chọn quay phải hai lần (hoặc trái)
    return "R";
  }
}

void updatePosition(String dir) {
  int newX = robotX, newY = robotY;
  int oldX = robotX, oldY = robotY; // Lưu vị trí cũ

  if (dir == "F") {
    if (robotHeading == 0) newY--; // Bắc
    else if (robotHeading == 90) newX++; // Đông
    else if (robotHeading == 180) newY++; // Nam
    else if (robotHeading == 270) newX--; // Tây
  } else if (dir == "L") {
    robotHeading = (robotHeading + 270) % 360; // Quay trái
  } else if (dir == "R") {
    robotHeading = (robotHeading + 90) % 360; // Quay phải
  }

  if (dir == "F" && newX >= 0 && newX < GRID_SIZE && newY >= 0 && newY < GRID_SIZE && grid[newX][newY] != 2) {
    robotX = newX;
    robotY = newY;
    // Cập nhật lưới: ô cũ thành 1, ô mới thành 3
    if (oldX != newX || oldY != newY) {
      if (grid[oldX][oldY] != 2) grid[oldX][oldY] = 1;
      if (grid[robotX][robotY] != 2) grid[robotX][robotY] = 3;
    }
  } else if (dir == "F") {
    Serial.println("Invalid move forward! Staying in place.");
    // Không di chuyển, giữ nguyên vị trí
  }

  Serial.print("Updated position: (");
  Serial.print(robotX);
  Serial.print(", ");
  Serial.print(robotY);
  Serial.print("), Heading: ");
  Serial.println(robotHeading);
}

void moveForward() {
  Serial.println("Moving forward for " + String(MOVE_DURATION) + " ms...");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(MOVE_DURATION);
  stopMotors();
  Serial.println("Move forward complete.");
}

void turnLeft() {
  Serial.println("Turning left for " + String(TURN_DURATION) + " ms...");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(TURN_DURATION);
  stopMotors();
  Serial.println("Turn left complete.");
}

void turnRight() {
  Serial.println("Turning right for " + String(TURN_DURATION) + " ms...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(TURN_DURATION);
  stopMotors();
  Serial.println("Turn right complete.");
}

void stopMotors() {
  Serial.println("stopMotors: All pins LOW");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}