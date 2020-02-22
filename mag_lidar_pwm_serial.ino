/*
  Controlling a servo position using a potentiometer (variable resistor)
  by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

  modified on 8 Nov 2013
  by Scott Fitzgerald
  http://www.arduino.cc/en/Tutorial/Knob
*/
//#include "header.h"
#include "structs_functions.h"
#include <Servo.h>

Servo MAG_LEFT;
Servo MAG_RIGHT;



void setup() {
  // Setup Serial Communication
  Serial.begin(115200); // UART to USB, ROS Interface
#ifdef DEBUG_MODE
  Serial.println("Setup Start");
  Serial.println("");
#endif

  // Setup pins' input/output
  pinMode(PC13, OUTPUT); // heartbeat
//  pinMode(PA0, INPUT);   // TSW_1
  pinMode(PB6, INPUT);   // TSW_1
  pinMode(PA1, INPUT);   // TSW_2
  pinMode(PA2, INPUT);   // TSW_3
  pinMode(PA3, INPUT);   // TSW_4
  pinMode(PA4, OUTPUT);  // LED_MAG_LEFT
  pinMode(PA5, OUTPUT);  // LED_MAG_RIGHT
  pinMode(PA6, OUTPUT);  // LED_TSW_1
  pinMode(PA7, OUTPUT);  // LED_TSW_2
  pinMode(PB0, OUTPUT);  // LED_TSW_3
  pinMode(PB1, OUTPUT);  // LED_TSW_4
  MAG_LEFT.attach(PB8);  // MAG_LEFT
  MAG_RIGHT.attach(PB9); // MAG_RIGHT
  pinMode(PA8, INPUT);   // Lidar A (Timer 1 channel 1)
//  pinMode(PB6, INPUT);   // Lidar B (Timer 4 channel 1)
  pinMode(PA0, INPUT);   // Lidar B (Timer 2 channel 1)
  delay(2);

  // Setup the timer 1 for Lidar A
  Timer1.pause(); // stop the timers before configuring them
  Timer1.setPrescaleFactor(72); // 1 microsecond resolution
  Timer1.setInputCaptureMode(TIMER_CH1, TIMER_IC_INPUT_DEFAULT); // setup timer 1 channel 1 capture on rising edge, use default input TI1
  Timer1.setInputCaptureMode(TIMER_CH2, TIMER_IC_INPUT_SWITCH); // setup timer 1 channel 2 capture on falling edge, use switched input TI1
  Timer1.setPolarity(TIMER_CH2, 1); // trigger on falling edge
  Timer1.setSlaveFlags( TIMER_SMCR_TS_TI1FP1 | TIMER_SMCR_SMS_RESET ); // counter setup as slave triggered by TI1 in reset mode

  Timer1.refresh();
  Timer1.getCompare(TIMER_CH1); // clear capture flag
  Timer1.getCompare(TIMER_CH2); // clear capture flag
  Timer1.resume(); // let timer 1 run

  while ( !Timer1.getInputCaptureFlag(TIMER_CH1) ); // discard first reading, wait for first period end
  Timer1.getCompare(TIMER_CH1); // clear capture flag

  // Setup the timer 2 for Lidar B
  Timer2.pause(); // stop the timers before configuring them
  Timer2.setPrescaleFactor(72); // 1 microsecond resolution
  Timer2.setInputCaptureMode(TIMER_CH1, TIMER_IC_INPUT_DEFAULT); // setup timer 2 channel 1 capture on rising edge, use default input TI1
  Timer2.setInputCaptureMode(TIMER_CH2, TIMER_IC_INPUT_SWITCH); // setup timer 2 channel 2 capture on falling edge, use switched input TI1
  Timer2.setPolarity(TIMER_CH2, 1); // trigger on falling edge
  Timer2.setSlaveFlags( TIMER_SMCR_TS_TI1FP1 | TIMER_SMCR_SMS_RESET ); // counter setup as slave triggered by TI1 in reset mode

  Timer2.refresh();
  Timer2.getCompare(TIMER_CH1); // clear capture flag
  Timer2.getCompare(TIMER_CH2); // clear capture flag
  Timer2.resume(); // let timer 2 run

  while ( !Timer2.getInputCaptureFlag(TIMER_CH1) ); // discard first reading, wait for first period end
  Timer2.getCompare(TIMER_CH1); // clear capture flag
  delay(2);

  // Setup the time
  cur_time = millis();
  prev_time = cur_time;
  prev_time_20hz = cur_time;

  // Indicates the setup is completed
  for(int i = 0; i < 10; i++) { // Let us know the setup is completed
    Toggle_OnOff_LED(13, 0);
    delay(200);
  }
}



void loop() {
  // put your main code here, to run repeatedly:
  cur_time = millis();
  if (cur_time - prev_time > 500)
  {
    Toggle_OnOff_LED(13, 0);
    prev_time = cur_time;
  }
  if (cur_time - prev_time_20hz > 50)
  {
    flag_ROS_TX_Status = 1;
    prev_time_20hz = cur_time;
  }

  //
  // Read Tactile Switches & Control LEDs
  //
//  status_TSW_1 = digitalRead(PA0);
  status_TSW_1 = digitalRead(PB6);
  status_TSW_2 = digitalRead(PA1);
  status_TSW_3 = digitalRead(PA2);
  status_TSW_4 = digitalRead(PA3);

  Toggle_OnOff_LED(6, status_TSW_1);
  Toggle_OnOff_LED(7, status_TSW_2);
  Toggle_OnOff_LED(0, status_TSW_3);
  Toggle_OnOff_LED(1, status_TSW_4);
  
  status_TSW_temp = status_TSW_1 || status_TSW_2 || status_TSW_3 || status_TSW_4;

  if(status_TSW_temp != status_TSW_temp_prev) {
    time_TSW = millis();
    status_TSW_temp_prev = status_TSW_temp;
  }
  if (status_TSW_temp == 1) {
    cur_FlagA = 1;
  }
  else if(status_TSW_temp == 0) {
    status_TSW = 0;
  }
  if ((status_TSW_temp == 1) && (cur_time - time_TSW > 3000)) {
    status_TSW = 1;
  }
  if ((status_TSW == 0) && (cur_time - time_TSW > 3000)) {
    cur_FlagA = 0;
  }
  
  
  //
  // ROS Interface
  //
  if(flag_ROS_TX_Status == 1)
    ROS_TX(1); // [1] TX_Status
  
  ROS_RX();



  //
  // Magnet Control & Control LEDs
  //

//  // for test
//  if (cur_time < 10000) {
//    cur_FlagA = 1;
//  }
//  else {
//    cur_FlagA = 0;
//  }
//
//  cur_FlagA = 0;
  
  if(cur_FlagA != prev_FlagA) {
    time_MAG = millis();
    prev_FlagA = cur_FlagA;
  }
  if(cur_FlagA == 0) { // Magnet Off
    if(cur_time - time_MAG < 50) {
      MAG_LEFT.write(5);
      MAG_RIGHT.write(5);
    }
    else if(cur_time - time_MAG > 200) {
      MAG_LEFT.write(90);
      MAG_RIGHT.write(90);
      status_MAG_LEFT = 0;
      status_MAG_RIGHT = 0;
    }
  }
  else if(cur_FlagA == 1) { // Magnet On
    if(cur_time - time_MAG < 50) {
      MAG_LEFT.write(175);
      MAG_RIGHT.write(175);
    }
    else if(cur_time - time_MAG > 200) {
      MAG_LEFT.write(90);
      MAG_RIGHT.write(90);
      status_MAG_LEFT = 1;
      status_MAG_RIGHT = 1;
    }
  }

  Toggle_OnOff_LED(4, status_MAG_LEFT);
  Toggle_OnOff_LED(5, status_MAG_RIGHT);

  status_MAG = status_MAG_LEFT && status_MAG_RIGHT;

  // Lidar Control
  if (Timer1.getInputCaptureFlag(TIMER_CH2)) // high pulse end
    LidarA_Alt_mm = Timer1.getCompare(TIMER_CH2);
  if (Timer2.getInputCaptureFlag(TIMER_CH2)) // high pulse end
    LidarB_Alt_mm = Timer2.getCompare(TIMER_CH2);
    
  LidarMax_Alt_mm = max(LidarA_Alt_mm, LidarB_Alt_mm);
  LidarMax_Smooth_Alt_mm = A_Lidar_LPF * LidarMax_Alt_mm + (1.0 - A_Lidar_LPF) * Prev_LidarMax_Alt_mm;
  Prev_LidarMax_Alt_mm = LidarMax_Alt_mm;

//  // for Debug
//  Serial.print("A: "); Serial.print(LidarA_Alt_mm, DEC);
//  Serial.print("\tB: "); Serial.print(LidarB_Alt_mm, DEC);
//  Serial.print("\tMax: "); Serial.print(LidarMax_Alt_mm, DEC);
//  Serial.print("\tMaxSmooth: "); Serial.println(LidarMax_Smooth_Alt_mm, DEC);
}
