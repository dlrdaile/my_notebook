# esp32开发笔记

> API相关链接：https://docs.espressif.com/projects/arduino-esp32/en/latest/api/rainmaker.html

## 中断相关

> 18行的`IRAM_ATTR`用于说明该函数运行于ram中，否则默认在flash中，运行速度较慢

```c++
/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#define timeSeconds 10

// Set GPIOs for LED and PIR Motion Sensor
const int led = 26;
const int motionSensor = 27;

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;

// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR detectsMovement() {
  Serial.println("MOTION DETECTED!!!");
  digitalWrite(led, HIGH);
  startTimer = true;
  lastTrigger = millis();
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  
  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(motionSensor, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  // Set LED to LOW
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
}

void loop() {
  // Current time
  now = millis();
  // Turn off the LED after the number of seconds defined in the timeSeconds variable
  if(startTimer && (now - lastTrigger > (timeSeconds*1000))) {
    Serial.println("Motion stopped...");
    digitalWrite(led, LOW);
    startTimer = false;
  }
}
```

## Flash相关

* Flash可以和EEPROM进行自定义参数的存储和读取，不过需要注意的是，其具有**擦写**的上限，大概是`100,000`到`1,000,000`次
* 在esp32-arduino中，我们使用EEPROM库进行Flash的读写
* 我们可以使用最多高达`512bytes`的flash内存，这意味着我们有512个内存地址，每个地址都可以读写一个字节的数据<img src="/home/dllr/Desktop/notebook/esp32笔记/esp32开发笔记.assets/59_1661413099_hd" alt="img" style="zoom:50%;" />
* 
