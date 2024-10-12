#include <ros.h>
//#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
// #include <geometry_msgs/Twist.h>
#include <AccelStepper.h>

#include "PinChangeInterrupt.h"


// befor d_1 == 2 , step1 = 3 , d2 = 4 , s2 =5 
#define dir_1  2
#define step_1  3

#define dir_2 4
#define step_2  5

#define listen_puls1 9
#define listen_puls2 8
#define pushButton  A5 


#define  R  0.0425 
#define  L  0.221

#define IRSensor A4

unsigned int temp = 0 ;
// unsigned int counter = 0 ;
unsigned int prev_temp =0 ;

volatile  int32_t encoder_right_read = 0 ;
volatile  int32_t encoder_left_read = 0 ;
 
// volatile  uint16_t encoder_right_read_pre = 0 ;
// volatile  uint16_t encoder_left_read_pre = 0 ;


int left_steps = 0 ;
int right_steps = 0 ;  

ros::NodeHandle node ;

//std_msgs::String str ;


// geometry_msgs::Twist cmd_vel ;
std_msgs::Int16 right_vel_msg;
std_msgs::Int16 left_vel_msg;


// std_msgs::Int16 pulses_command ;
std_msgs::Int32 encoder_right_msg;
std_msgs::Int32  encoder_left_msg;

std_msgs::Bool empty_msg ;

std_msgs::Bool limitswitch_msg ;
std_msgs::Bool IR_msg ;

AccelStepper stepper_right(1 , step_1,dir_1);
AccelStepper stepper_left(1 , step_2,dir_2);


//ros::Publisher pulses_read_pub("/pps", &pulse_per_seconde);

// ros::Publisher send_data("/data", &pulses_command );
ros::Publisher encoder_right("/encoder_right", &encoder_right_msg);
ros::Publisher encoder_left("/encoder_left", &encoder_left_msg);

ros::Publisher limitswitch_pub("/limitswitch", &limitswitch_msg);
ros::Publisher IR_pub("/IR", &IR_msg );


// void subscriberCallback(const geometry_msgs::Twist& cmd_vel) {

//      float vel_r = 0 ; 
//      float vel_l = 0 ;
     

//     vel_r =  (2 * cmd_vel.linear.x + cmd_vel.angular.z * L)/(2 * R) ;
//     vel_l =  (2 * cmd_vel.linear.x - cmd_vel.angular.z * L)/(2 * R) ;

//     left_steps  =  vel_l * (360 / 1.8);
//     right_steps =  - vel_r * (360 / 1.8);
//     // pulses_command.data = -right_steps;
//     // send_data.publish (&pulses_command);

// }

void reset_encoderCallback(const std_msgs::Bool& empty_msg) {
  encoder_left_read = 0 ;
  encoder_right_read = 0;

}

void right_vel_callback(const std_msgs::Int16& right_vel_msg) {
   right_steps =  right_vel_msg.data ;

}

void left_vel_callback(const std_msgs::Int16& left_vel_msg) {
   left_steps =  left_vel_msg.data ;

}


// ros::Subscriber<geometry_msgs::Twist> speed_control_sub("/cmd_vel", &subscriberCallback);
ros::Subscriber<std_msgs::Int16> right_vel_sub("/right_vel", &right_vel_callback);
ros::Subscriber<std_msgs::Int16> left_vel_sub ("/left_vel" , &left_vel_callback);

ros::Subscriber<std_msgs::Bool> reset_encoder_sub("/reset_encoder", &reset_encoderCallback);



void setup() {

 



  pinMode(dir_1  , OUTPUT) ;
  pinMode(step_1 , OUTPUT) ;
  pinMode(IRSensor, INPUT);

  pinMode(dir_2  , OUTPUT) ;
  pinMode(step_2, OUTPUT) ;

  pinMode(listen_puls1, INPUT_PULLUP);
  pinMode(listen_puls2, INPUT_PULLUP);
  pinMode(pushButton, INPUT_PULLUP);

  //  attachInterrupt(digitalPinToInterrupt(listen_puls1),count_pulses1_right, FALLING); 
//   attachInterrupt(digitalPinToInterrupt(listen_puls2),count_pulses2_left, FALLING); 

  attachPCINT(digitalPinToPCINT(listen_puls1), count_pulses1_right, CHANGE);
  attachPCINT(digitalPinToPCINT(listen_puls2), count_pulses2_left, CHANGE);
  

  node.getHardware()->setBaud(57600);
  

  node.initNode();
 // node.advertise(pulses_read_pub);
  node.subscribe(reset_encoder_sub);
  node.subscribe(right_vel_sub);
  node.subscribe(left_vel_sub);
  node.advertise(limitswitch_pub);
  node.advertise(IR_pub);

  node.advertise(encoder_right);
  node.advertise(encoder_left);


  stepper_right.setMaxSpeed(1000);
  stepper_left.setMaxSpeed(1000);
  // stepper_right.setSpeed(200);	
  // stepper_left.setSpeed(-200);	


}

void loop() {
   temp = millis();


   stepper_right.setSpeed(right_steps) ;	
   stepper_left.setSpeed(left_steps)   ;	

   stepper_right.runSpeed();	
   stepper_left.runSpeed() ;	




  if ( (temp - prev_temp)  > 20 )
  { 
    
     
     // encoder 
     encoder_right_msg.data  =  encoder_right_read ;//- encoder_right_read_pre  ;
     encoder_left_msg.data  =  encoder_left_read ; // - encoder_left_read_pre;
     encoder_right.publish(&encoder_right_msg) ;
     encoder_left.publish(&encoder_left_msg) ;

     //encoder_right_read_pre = encoder_right_read  ; 
     //encoder_left_read_pre = encoder_left_read  ; 

     
      // limitswitch
      if (digitalRead(pushButton) == 0 ){
        limitswitch_msg.data = true;

        limitswitch_pub.publish(&limitswitch_msg);

      }

      // IR 

      if (digitalRead(IRSensor) == 1 ){
        IR_msg.data = true;

      }
      else {
        IR_msg.data = false ;
      }

      IR_pub.publish(&IR_msg);




       prev_temp = temp ;

      
   
        
  }


   node.spinOnce();


}
// void count_pulses () {



// }


void count_pulses1_right (){

     if (digitalRead(listen_puls1) == LOW) {
      if (right_steps <= 0 ){
          encoder_right_read++ ; 

     }

      else {
          encoder_right_read-- ; 
      }
    
  }  

}





void count_pulses2_left () {


  if (digitalRead(listen_puls2) == LOW) {
      if (left_steps >= 0){
        encoder_left_read++ ; 
      }
      else {
        encoder_left_read-- ; 
      
      }  
    }
}

// void count_pulses1_right () {

//     if (digitalRead(listen_puls1) == LOW) {
//     //Pin D2 triggered the ISR on a Falling pulse
//      encoder_right_read++ ; 
//     //Set LED 1 to state of D2_state boolean
    
//   } 

//       if (digitalRead(listen_puls2) == LOW) {
//     //Pin D2 triggered the ISR on a Falling pulse
//     encoder_left_read++ ; 
//     //Set LED 1 to state of D2_state boolean
    
//   }
  



// }



