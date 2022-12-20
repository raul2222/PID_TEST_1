#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <ArduinoJson.h>
#include <geometry_msgs/msg/twist.h>
#include <SoftwareSerial.h>
#include <geometry_msgs/msg/point.h>

#include <HardwareSerial.h>

#define RXD1 9
#define TXD1 10
#define RXD2 16
#define TXD2 17
// Right and Left
HardwareSerial SERIAL_RIGHT(2);
HardwareSerial SERIAL_LEFT(1);
#define MYPORT_TX 12
#define MYPORT_RX 13
uint32_t enc_r=0;
uint32_t enc_l=0;
SoftwareSerial myPort;


rcl_publisher_t publisher;
geometry_msgs__msg__Twist msg_encoder;
geometry_msgs__msg__Twist msg;
rcl_subscription_t subscriber;
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;
#define LED_PIN 2

//#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
//#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

void error_loop(){
  int i = 0;
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    if(i++ > 50 ) ESP.restart();
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
              
         myPort.println("llega,");
      if(SERIAL_RIGHT.available() > 0){
         String str = SERIAL_RIGHT.readStringUntil('\n');

         enc_r = str.toFloat(); // to pub
         msg_encoder.linear.x=enc_r;
         RCSOFTCHECK(rcl_publish(&publisher, &msg_encoder, NULL));
         myPort.print(enc_r);
         myPort.println(",");
      }


    
    //msg_heartbeat.data++; //my_pid_wheel->getEncoder();
    //msg_encoder.x=enc_r; // my_pid_wheel_2.getEncoder();
    //msg_heartbeat.data=99;
    //rcl_publish(&publisher, &msg_encoder, NULL);
    
  }
}



//twist message cb

float LINEAR_X = 0;
float ANGULAR_Z = 0;

void task_enc_r(void* _this){
  while(1){
      if(SERIAL_RIGHT.available() > 0){
         String str = SERIAL_RIGHT.readStringUntil('\n');
         StaticJsonDocument<30> docr;
         DeserializationError error = deserializeJson(docr, str);
         if(enc_r != docr["e_r"]) {
            //rcl_publish(&publisher, &msg_encoder, NULL);
         }
         //msg_encoder.x=enc_r;
         //rcl_publish(&publisher, &msg_encoder, NULL);
         enc_r = docr["e_r"]; // to pub
         myPort.print(enc_r);
         myPort.println(",");
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
  }
      
}



void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    // if velocity in x direction is 0 turn off LED, if 1 turn on LED
    digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);
    float radius = 0.044;
    float b = 0.32;
    float  linear = msg->linear.x;
    float angular = msg->angular.z;
    float  leftwheel =  (linear - angular * b / 2.0) / radius;  // rad /s
    float rightwheel = (linear + angular * b / 2.0) / radius;
    StaticJsonDocument<30> docl;
    StaticJsonDocument<30> docr;
    docl["l"] = leftwheel;
    docr["r"] = rightwheel;
  
    serializeJson(docr, SERIAL_RIGHT);
    SERIAL_RIGHT.println();
  
    serializeJson(docl, SERIAL_LEFT);
    SERIAL_LEFT.println();
    LINEAR_X = msg->linear.x;
    ANGULAR_Z = msg->angular.z;
    
    myPort.print(LINEAR_X);
    myPort.print(",");
    myPort.println(ANGULAR_Z);
    myPort.print(",");
    myPort.println(leftwheel);
    myPort.print(",");
    myPort.println(rightwheel);
}



void setup() {
    
    Serial.begin(115200); 
    set_microros_transports();
    myPort.begin(57600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
    if (!myPort) { // If the object did not initialize, then its configuration is invalid
      Serial.println("Invalid SoftwareSerial pin configuration, check config"); 
      while (1) { // Don't continue with invalid configuration
        delay (1000);
      }
    } 
      
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  
    SERIAL_RIGHT.begin(57600, SERIAL_8N1, RXD2, TXD2); 
    SERIAL_LEFT.begin(57600, SERIAL_8N1, RXD1, TXD1); 
    delay(20);
    
    StaticJsonDocument<30> docl;
    StaticJsonDocument<30> docr;
    docl["l"] = 0;
    docr["r"] = 0;
  
    serializeJson(docr, SERIAL_RIGHT);
    SERIAL_RIGHT.println();
  
    serializeJson(docl, SERIAL_LEFT);
    SERIAL_LEFT.println();
  
  /*
    if(xTaskCreatePinnedToCore( task_enc_r , "task_enc_r", 2048, NULL, 1, NULL,0) != pdPASS){
      exit(-1);
    }*/
    
    state = WAITING_AGENT;
}



bool create_entities(){
  
 
    allocator = rcl_get_default_allocator();
  
     //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  
      // create subscriber
      RCCHECK(rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/nav_vel"));
  
      RCCHECK(rclc_publisher_init_best_effort(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "pid_wheels_enc"));
  
        const unsigned int timer_timeout = 2.05;
      RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
  
  
    // create executor
    RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
      RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
    return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  //rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  //rcl_timer_fini(&timer);
  //rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void loop() {
  //delay(100);
  
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
            rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100));
            rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}
