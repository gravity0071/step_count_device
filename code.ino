/*
   红色按钮：pin12（显示信息）
   mpu6050:SCL--A5
           SDA--A4
   OLED: SCL--A5
         SDA--A4
   心率: SCL--A5
         SDA--A4
   蜂鸣器: pin3（PWM波）

   蓝牙：tx--11
        rx--10
   触摸传感器 DC--5
   LED灯泡 -- pin13
*/

#include <Wire.h>    //Include Wire Library for I2C
#include <Adafruit_GFX.h> //Include Adafruit GFX Library
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include "MAX30105.h" //MAX3010x library
#include "heartRate.h" //Heart rate calculating algorithm

#define FallDownHeartBeats 50  //测试时不能保证心跳超过阈值，所以暂定摔倒时判断心跳的阈值为50

long accelX, accelY, accelZ;      // 定义为全局变量，可直接在函数内部使用

/*这五个数存储加速度感应器检测到的数据，进而进行姿态判断*/
float gForceX, gForceY, gForceZ, gForce, gAfterGauss;

/*changeStpes:当加速度最大值发生变化时，值变为1，为判断是否走路的一个条件。
   k:从监测到第一步开始进行数，如果两步之间的k过大或过小，说明没有走路。
   changeSteps：配合k进行走路判断*/
int steps = 0, changeSteps, k = 20, isWalking = 0;
/*
   wasFall: 判断上一次是不是摔倒，0没摔倒，1摔倒
   fallDown：当wasFall等于1后，进行倒数，倒数3s内没有取消，进行报警
*/
int wasFall = 0, fallDown = 100;


/*显示屏参数*/
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(OLED_RESET); //Declaring the display name (display)
int measure = 0; //进行初始信息显示的判断参数

/*
  frequ：通过isWalking改变刷新率
  movingStatus: 0为静止，1为走路，2为跑步
  movingStatusPre：之前状态，只有相邻两次状态相同时，才输出运动状态
*/
int frequ, movingStatus, movingStatusNow, movingStatusPre;


float maxA, maxAOrigin, minA;

/* gauss存储最开始检测到的数据，进行高斯滤波滤除高频后存储到g数列中进行姿态判断*/
float gauss[3] = {0, 0, 0};
float g[7] = {0, 0, 0, 0, 0, 0, 0};


/*蓝牙参数初始化*/
SoftwareSerial BT (11, 10);


/*
   心率元件参数以及元器件初始化
*/
MAX30105 particleSensor;


byte rates[4] = {80, 80, 80, 80}; //Array of heart rates
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;



/*
  初始化元器件
*/
void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Start the OLED display
  display.display();
  delay(3000);

  pinMode(12, INPUT); //按钮用来刷新显示屏
  pinMode(5, INPUT); //定义金属触摸传感器为输出接口


  pinMode(13, OUTPUT); //定义LED 为输出接口

  BT.begin(9600);
  BT.println("send 1 to access the accelerate, steps and moving status.");


  Serial.begin(500000);
  Wire.begin();
  setupMPU();
  setupMAX30152();
}

/*
   进行MAX30152元器件的初始化
*/
void setupMAX30152() {
  particleSensor.begin(Wire, I2C_SPEED_STANDARD); //Use default I2C port, 100kHz speed
  particleSensor.setup(); //Configure sensor with default settings
}

/*
   进行三轴加速度mpu6050的初始化，利用I2C通讯方法。将mpu元器件设置为记录范围为+/- 8g
*/
void setupMPU() {
  // REGISTER 0x6B/REGISTER 107:Power Management 1
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet Sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B/107 - Power Management (Sec. 4.30)
  Wire.write(0b00000000); //Setting SLEEP register to 0, using the internal 8 Mhz oscillator
  Wire.endTransmission();

  // REGISTER 0x1C/REGISTER 28:ACCELEROMETER CONFIGURATION
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00010000); //Setting the accel to +/- 8g
  Wire.endTransmission();
}







/********************************************************************************************************************/
void loop() {
  blueTooth(); //检测是否有新来的蓝牙消息
  if (measure == 0) {
    displayMeasuring(); // 第一次开始时进行初始信息显示
    measure = 1;
  }
  if (digitalRead(5) == HIGH) { //当检测到触摸后才运行，LED灯会随刷新率进行闪烁

    generateDataProcess();//进行陀螺仪以及心率数据采集及处理

    testSerial(); //通过串口监视器进行数据的检测，以进行测试
    delay(frequ); //通过frequ控制刷新率
    digitalWrite(13, HIGH);//LED灯表示刷新率的变化
    delay(1);
    digitalWrite(13, LOW);
  }
  else {
    digitalWrite(13, LOW); //当没有触摸时，LED灯灭掉
  }
}


/*
   利用三个函数进行陀螺仪数据获取以及处理
   初步检测到摔倒后进行倒计时处理
   心率采集以及处理
*/
void generateDataProcess() {
  if (wasFall == 0) {
    normalRecord();
  }
  else if (wasFall == 1) { //检测到加速度超过4g时,通过之后的加速度以及心率进行判断是否摔倒，判断为摔倒后修改wasFall为2，下一loop进行按钮取消操作，否则进行报警
    abnormalRecord();
  }
  else if (wasFall == 2) {
    fallReaction();
  }
}

void normalRecord() {
  recordAccelRegisters();
  processData(); //将数据传入数列中等待处理，以及判断加速度是否到达摔倒的阈值
  movingState();//判断运动姿态
  heartBeats(); //采集心跳
  flashRate();//调整刷新率

  if (digitalRead(12) == HIGH) {
    displayOled();     //按下按钮时显示实时信息
  }
}

void abnormalRecord() {
  int f = 0;
  for (int x = 0; x < 200; x++) { //检测五秒，观察是否符合摔倒的心率以及加速度值
    recordAccelRegisters();
    processData(); //将数据传入数列中等待处理，以及判断加速度是否到达摔倒的阈值
    movingState();//判断运动姿态
    heartBeats(); //采集心跳
    if (g[2] > 1.6 && x > 20) { //当检测到加速度阈值超过后，如果人体发生移动，更改flag为未摔倒
      wasFall = 0;
    }
    if (beatsPerMinute > FallDownHeartBeats) {
      f++;
    }
    delay(10);
    displayWhetherFall(); //测试时显示心率以及陀螺仪数据

  }
  if (wasFall == 1 && f >= 2) {
    Serial.println("falling");
    tone(3, 1000); //蜂鸣器响100ms
    delay(100);
    noTone(3);
    displayFall(); //进行跌倒后的程序
    wasFall = 2;
  }
  else {
    wasFall = 0;
  }
}

void fallReaction() {
  frequ = 10;
  fallDown--;
  if (digitalRead(12) == HIGH) {
    displayMeasuring();
    wasFall = 0; //如果按按钮取消后，更新wasFall参数进行取消跌倒
    fallDown = 120;
  }
  if (fallDown == 0) {
    //进行摔倒后的报警以及通过蓝牙发送信息给手机
    fallDownEvent();
    displayMeasuring();
  }
}

void heartBeats() {
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running

  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 150 && beatsPerMinute > 50)
    {
      rates[0] = rates[1];
      rates[1] = rates[2];
      rates[2] = rates[3];
      rates[3] = (byte)beatsPerMinute; //Store this reading in the array

      //Take average of readings
      beatAvg = 0;

      beatAvg = (rates[0] + rates[1] + rates[2] + rates[3]) / 4;
    }
  }
}


/*
   每次loop会检测蓝牙是否接收到新的信息，
   如果有新的信息，
   接受到的信息为1：上传实时加速度、步数以及运动状态到手机
*/
void blueTooth() {
  if (Serial.available()) { //如果接收到蓝牙信息
    String val = Serial.readString();
    Serial.println(val);
    BT.println(val);
  }

  while (BT.available()) { //当接受到蓝牙信息后进行的操作
    String sc = BT.readString();
    Serial.println(sc);

    if (sc == "1") {
      digitalWrite(13, HIGH);
      BT.print("accelerate: ");
      BT.print(g[3]);
      BT.print("(g)  steps: ");
      BT.print(steps);
      if (movingStatus == 0) {
        BT.print("  static");
      }
      else if (movingStatus == 1) {
        BT.print("  walking");
      }
      else if (movingStatus == 2) {
        BT.print("  running");
      }
      BT.print("  heart beats: ");
      BT.println(beatAvg);
    }
  }
}


/*
   跌倒后没有取消后会发生的事件：蜂鸣器报警，发送蓝牙到手机
*/
void fallDownEvent() {

  BT.println("falling!!!!");
  displayAfterFall();

  while (1) {
    tone(3, 1000); //And tone the buzzer
    delay(100);
    tone(3, 500); 
    delay(100);
    if (digitalRead(12) == HIGH)
      break;
  }
noTone(3);

  wasFall = 0;
  fallDown = 100;
}


/*
   此函数用来判断身体运动状态，通过是否运动、相邻加速度差值判断是否为静止、走路以及跑步状态
   用到的参数：
   isWalking: 0为静止状态，1为运动状态
   movingStatus: 记录运动姿态的参数，0为静止，1为走路，2为走路
   movingStatusNow: 当前loop中识别出的运动姿势
   movingStatusPre: 前一loop中识别出的运动姿态
   当当前运动姿态和前一轮的姿态相同时，才会进行运动姿态参数movingStatus改变
*/
void movingState() {
  if (isWalking == 0 || k > 50) { //通过isWalking参数和峰值共同判断是否运动
    movingStatus = 0;
  }
  else if (maxA - minA < 0.24) {
    movingStatus = 0;
  }
  else {
    if (maxA - minA > 0.25 && maxA - minA < 1.2)
      movingStatusNow = 1;
    else if (maxA - minA > 1, 2 && maxA - minA < 4)
      movingStatusNow = 2;
    if (movingStatusNow == movingStatusPre)
      movingStatus = movingStatusNow;
    movingStatusPre = movingStatusNow;
  }
}


/*
   通过识别人体身体的运动方式来进行获得三轴加速度数值，从而节省能源
   frequ：每次刷新时间间隔，单位为毫秒
   isWalking：0为人体为静止状态，1为人体开始运动状态
*/
void flashRate() {
  if (isWalking == 0) {
    frequ = 40;//涮心率变为12hz
  }
  else if (isWalking == 1) {
    frequ = 10; //当检测到走路或者跑动时，刷新率变为40hz
  }
}



/*
   对mpu进行指令，读取其存储加速度数据的寄存器中的数据
   并通过mpu传回的数据进行读取以及调用processAccelData()函数进行进一步的处理
*/
void recordAccelRegisters() {
  // REGISTER 0x3B~0x40/REGISTER 59~64
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Accel Registers (3B - 40)

  // 使用了左移<<和位运算|。Wire.read()一次读取1bytes，并在下一次调用时自动读取下一个地址的数据
  while (Wire.available() < 6); // Waiting for all the 6 bytes data to be sent from the slave machine （必须等待所有数据存储到缓冲区后才能读取）
  accelX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX （自动存储为定义的long型值）
  accelY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}


/*
   利用recordAccelRegisters()函数中得到的数值进行进一步的运算，得到三轴合成后的加速度
   gForce：三轴合成后的总加速度
*/
void processAccelData() {
  gForceX = accelX / 4096.0;     //float = long / float
  gForceY = accelY / 4096.0;
  gForceZ = accelZ / 4096.0;
  gForce = sqrt(gForceX * gForceX + gForceY * gForceY + gForceZ * gForceZ); //三轴加速度计算总重力加速度
}


/*
   将得到的loop中的数值进行高斯滤波存储到新的数列中
   当检测到的瞬时加速度大于4g时，判定为摔倒
   gauss[]：取相邻三个加速度得到的数值进行高斯变换，滤除高频噪声
   g[]：将滤波后的数据存储到g中，进行下一步的数据处理
*/
void processData() {
  { //进行高斯滤波的三个数据
    gauss[0] = gauss[1];
    gauss[1] = gauss[2];
    gauss[2] = gForce;
  }

  { //将新数据传入g数列中，进而进行平滑滤波
    g[0] = g[1];
    g[1] = g[2];
    g[2] = g[3];
    g[3] = g[4];
    g[4] = g[5];
    g[5] = g[6];
    g[6] = (gauss[0] + 2 * gauss[1] + gauss[2]) / 4; //高斯滤波，将之后需要处理的数据传入g数列中
  }
  if (gForce < 4) {
    countSteps();
  }
  else if (gForce > 4) {                  //检测到摔倒
    wasFall = 1; //设置wasFall到1
  }
}


/*
   此函数进行当前数据的检测以及判断身体是否为运动状态
   k：存储检测到上一步后计数了几个loop
   当人体经过1.5毫秒后没有进行运动后自动将isWalking参数设置为0，即人体不在运动状态，从而进行刷新率的调整
*/
void countSteps() {
  detectCrest();//检测波峰，将波峰值存入maxA中
  detectValley();//检测波谷，将波峰值存入minA中

  k++; //经过一个loop后，k值加一
  if (k > 60) {
    isWalking = 0;  // 当1.5秒没有走路时，把是否走路的flag标记为0
    maxA = 0;
    minA = 0;//计算一步后之前的最大最小值清零
  }

  if (isWalking == 0) {
    k = 200;  //静止时，给k赋值一个不会阻碍进入steppsC()函数的值
    stepsC();
  }
  else if (isWalking == 1) { //当为走路时
    stepsC();
  }
}


/*
   利用当一个数据比前两个以及后两个数据都大时，检测为波峰
   当波峰数值与前一个数值有区别后，修改changeSteps使stepsC()函数进行记步运算
   maxA: 检测到的波峰值
   maxAOrigin：存储上一波峰值，当当前波峰与上一个波峰值不同时，进行计数处理
*/
void detectCrest() {     //检测出一次波峰

  if (g[3] - g[2] > 0.0008 && g[4] - g[3] < 0.0008 && g[3] > 1) {
    if (g[1] - g[0] >= 0.0008 && g[2] - g[1] >= 0.0008 && g[4] - g[5] >= 0.008 && g[5] - g[6] >= 0.0008) {
      maxAOrigin = maxA;
      maxA = g[3];
      if (maxAOrigin != maxA) {
        changeSteps = 1; //当波峰值变化时，才会进行新的步数计算
      }
    }
  }
}

/*
   利用当一个数据比前两个以及后两个数据都小时，检测为波谷
*/
void detectValley() {  //检测出一次波谷
  if (g[3] - g[2] < 0.0008 && g[4] - g[3] > 0.0008 && g[3] < 1) {
    if (g[1] - g[0] <= 0.0008 && g[2] - g[1] <= 0.0008 && g[4] - g[5] <= 0.0008 && g[5] - g[6] <= 0.0008) {
      minA = g[3];
    }
  }
}


/*
   利用波峰与波谷之间的差值进行步数判断
   只有连续两次或以上次数判断为有效步数后，才会进行步数的累积
   当两步之间间隔时间过长或过短后，不会判断为有效步数
*/
void stepsC() { //利用波峰波谷是否超过阈值进行步数计算
  if (changeSteps == 1) {
    if (k > 6 && k < 38) { //如果两步之间间隔时间不在0.2到1.2秒之间，视为无效步伐
      if (maxA - minA > 0.25) { //经过多次测试，最后得到当大于0.25这个阈值时，可以过滤掉大部分人体的大幅动作，只有当走路时才会进行记步
        if (maxA != 0 && minA != 0) {
          steps++;
          changeSteps = 0;  //将flag改回去
          isWalking = 1; //说明在走路，k开始每次从0开始加一
          k = 0;
        }
      }
    }
    else if (k == 200) {     //防止身体动一下就会计入步数，通过此判断，只有连续的满足步数姿态判断才会计入步数
      changeSteps = 0;  //将flag改回去
      isWalking = 1; //说明在走路，k开始每次从0开始加一
      k = 0;
    }
  }
}



/*
   展示初始信息：we are measuring you status and steps
*/
void displayMeasuring() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(13, 5);
  display.println("we are measuring");
  display.setCursor(5, 15);
  display.println("your status and");
  display.setCursor(5, 25);
  display.println("steps.");
  display.display();
}

/*
   当按钮按下时，展示步数以及当前的运动状态
*/
void displayOled() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(5, 3);
  display.print("steps:");
  display.println(steps);
  display.setCursor(5, 13);
  display.print("heart beat:");
  if (beatAvg == 0) {
    display.println("measuring");
  }
  else {
    display.println(beatAvg);
  }
  display.setCursor(5, 23);
  display.print("status:");
  if (movingStatus == 0) {
    display.println("static");
  }
  else if (movingStatus == 1) {
    display.println("walking");
  }
  else if (movingStatus == 2) {
    display.println("running");
  }
  display.display();
  frequ = 10;
}

void displayFall() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(13, 5);
  display.println("did you fall?");
  display.setCursor(5, 15);
  display.println("press button to cancel");
  display.display();
}

void displayAfterFall() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(20, 10);
  display.println("FALL!!");
  display.display();
  delay(2000);
}

void testSerial() {
  Serial.print(g[3]);//Serial.print("       ");
  Serial.print("    max:");
  Serial.print(maxA);
  Serial.print("    min");
  Serial.print(minA);
  Serial.print("     相减");
  Serial.print(maxA - minA);
  Serial.print("    k:");
  Serial.print(k);
  Serial.print("    isWalking:");
  Serial.print(isWalking);
  Serial.print("    steps:");
  Serial.print(steps);
  Serial.print("    status:");
  Serial.print(movingStatus);
  Serial.print("    freq:");
  Serial.print(frequ);
  Serial.print("    heart:");
  Serial.println(beatAvg);
}

void displayWhetherFall() {
  Serial.print("g[2]:");
  Serial.print(g[2]);
  Serial.print("    beats");
  Serial.print(beatsPerMinute);
  Serial.print("   wasFall:");
  Serial.println(wasFall);
} 
