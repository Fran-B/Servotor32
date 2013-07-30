#include "Servotor32.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "Arduino.h"

#include "Servotor32_SPI.h"
#include "Servotor32_TimerOne.h"

//#define DEBUG_PROC_CHAR

/// Minimum allowed position value
#define MIN_POS 500
/// Maximum allowed position value
#define MAX_POS 2500
/// Center position value
#define CENTER_POS 1500
/// Position value for killing control on a servo
#define KILL_POS -1

/// Servo motor attached to sonar sensor
#define PING_SERVO 31

/// Default number of pings for multi-ping
#define DEFAULT_MULTI_PING_CNT 5
/// Default number of measurement locations in 180 degree arc-ping scan
#define DEFAULT_SCAN_PING_CNT 9
/// Maximum number of measurement locations in 180 degree arc-ping scan
#define MAX_SCAN_PING_CNT (((MAX_POS - MIN_POS) / 10) + 1)
/// Maximum number of pings for multi-ping
#define MAX_MULTI_PING_CNT 64

Servotor32::Servotor32()
{
    // nothing needed
}

//stores information about the servos and groups
signed short servo_positions[SERVOS]; // where the servos are currently (supposed to be) at
signed char servos_sorted[GROUPS][SERVOS_PER_GROUP]; // index in servo_timings to where the servo ends
signed char servos_active_in_group[GROUPS]; // the number of servos in a group currently active
uint8_t active_servos_hex[GROUPS];

// all updates to shift registers in order of their updates
signed short servo_timings[MAX_TIMINGS]; // the timing where the change occurs
uint8_t  shift_output[MAX_TIMINGS];  // the output of the shift register
uint8_t  shift_latch[MAX_TIMINGS];   // the shift register latch used

// keeps track of whether its safe or not to update the servos
uint8_t update_reg_flag = 0;

// variables for the callback
uint16_t timer;
uint8_t  counter = 0;
uint8_t  pwm_active = 1;

uint16_t group_offsets[4] = {0,251,502,753};
uint8_t group_latches[4] = {5,6,7,4};
uint8_t pin_2_num[8] = {0x08,0x04,0x02,0x01, 0x80,0x40,0x20,0x10};

void Servotor32::begin(){
  //setup pin modes
  DDRF |= 0xF0;  // sets pins F7 to F4 as outputs
  DDRB = 0xFF;  // sets pins B0 to B7 as outputs

  //setup PC serial port
  Serial.begin(9600);
  // reconfigure bluetooth module to 9600 baud id needed
  Serial1.begin(115200);     // Changed from 9600 baud
  Serial1.print("AT+BAUD4"); // Tell the module to change the baud rate to 9600
  delay(1100); // Wait a notch over 1 second to make sure the setting "sticks"
  Serial1.begin(9600);     // Changed from 9600 baud

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  Timer1.initialize(10);
  Timer1.attachInterrupt(callback);

  for(byte i=0; i<SERVOS; i++){
    servo_positions[i] = -1;
  }
  for(byte i=0; i<GROUPS; i++){
    for(byte j=0; j<SERVOS_PER_GROUP; j++){
      servos_sorted[i][j] = -1;
    }
  }

  for(uint8_t i=0; i<MAX_TIMINGS; i++){
    servo_timings[i] = 0;
    shift_output[i] = 0xFF;
    shift_latch[i] = 0xFF;
  }

  TIMSK0 &= ~(_BV(TOIE0)); // disables the arduino delay function, but also
                           // all but eliminates servo jitter
  TIMSK2 &= ~(_BV(TOIE2)); // disable the arduino tone  function, but also
                           // also helps eliminate some jitter
  TIMSK3 &= ~(_BV(TOIE3)); // for good measure
  TIMSK4 &= ~(_BV(TOIE4)); // for good measure
}

long unsigned int us_counter = 0;
long unsigned int startTime = 0;
long unsigned int currentTime = 0;
long unsigned int last_update = 0;

long unsigned int Servotor32::micros_new() const {
  return us_counter;
}

long unsigned int Servotor32::millis_new() const {
  return us_counter/1000;
}

void Servotor32::delay_ms(long unsigned int delay_time) const {
  startTime = millis_new();
  currentTime = millis_new() - startTime;
  while(currentTime < delay_time){
    delayMicroseconds(10);
    currentTime = millis_new() - startTime;
  }
}

void Servotor32::delay_us(long unsigned int delay_time) const {
  startTime = micros_new();
  currentTime = micros_new() - startTime;
  while(currentTime < delay_time){
    delayMicroseconds(10);
    currentTime = micros_new() - startTime;
  }
}

void Servotor32::callback(){
  cli();
  if(timer < 1100){ // keep it from updating servos mid-array change by some weird coincidence
    if(timer == servo_timings[counter]){ // if the time has arrived to update a shift reg
      SPDR = shift_output[counter]; // push the byte to be loaded to the SPI register
      while(!(SPSR & (1<<SPIF))); //wait till the register completes
      PORTF &= ~(shift_latch[counter]); // clock the shift register latch pin low, setting the register
      PORTF |= shift_latch[counter];  // clock the shift register latch pin high, ready to be set low next time
      counter++;
    }
  }

  timer++;
  us_counter += 10;
  if(timer == 1100){ // all servo timing completed
    update_reg_flag = 1; // allow updates to the timing arrays
  }
  if(timer == 1900){ // getting close to servo start-up again,
    update_reg_flag = 0; // don't allow any new timing array updates
  }
  if(timer == 2000){
    timer=0;
    counter=0;
  }
  sei();
}

void Servotor32::delete_from_sorted_array(byte servo, byte group, signed short pos){
  for(byte i=0; i<servos_active_in_group[group]; i++){ // go through all the servos
    if(servos_sorted[group][i] == servo){ // find its place
       for(signed char j=i; j<servos_active_in_group[group]-1; j++){//move all servos in front of it back by one
         servos_sorted[group][j] = servos_sorted[group][j+1];
       }
       servos_sorted[group][servos_active_in_group[group]-1] = -1; //insert a -1 at the end of the move
       break; //break out of previous for loop, now that the job is done
    }
  }
  active_servos_hex[group] &= ~pin_2_num[servo-group*SERVOS_PER_GROUP];
  servos_active_in_group[group] -= 1;// decrease the number of active servos in the group by 1
}

void Servotor32::add_to_sorted_array(byte servo, byte group, signed short pos){
  for(byte i=0; i<=servos_active_in_group[group]; i++){ // find the servo
     if(servos_sorted[group][i] == -1){ // if no servos yet entered, set as first
       servos_sorted[group][i] = servo; //insert the servo in its sorted place
       break; //stop the for loop, as the job is done
     }
     else{
       if(servo_positions[servos_sorted[group][i]] > pos){ // if this servo should go before this one
         for(signed char j=servos_active_in_group[group]-1; j>=i; j--){// move all others forward one
           servos_sorted[group][j+1] = servos_sorted[group][j];
         }
         servos_sorted[group][i] = servo; //insert the servo in its sorted place
         break;
       }
     }
  }
  active_servos_hex[group] |= pin_2_num[servo-group*SERVOS_PER_GROUP];
  servos_active_in_group[group] += 1;
}

void Servotor32::update_registers_fast(byte servo, signed short pos){
  byte group = servo/8;
  while(update_reg_flag == 0){ // wait for the servos to stop pulsing before updating the timing arrays
    delayMicroseconds(10);
  }
  // ----- put the servo into, or take it out of its sorted array ------

  if(pos > 0){ // if the sevo isn't a kill command, then its an add/change
    if(servo_positions[servo] == -1){// if the servo is inactive
      // insert the servo into the array sorted
      add_to_sorted_array(servo,group,pos);
    }
    else{
      // updating the servo. First delete its existing entry, then insert it

      delete_from_sorted_array(servo,group,pos);
      add_to_sorted_array(servo,group,pos);
    }
  }
  else{ // servo is a kill command
    if(servo_positions[servo] != -1){ // make sure its even on first
      delete_from_sorted_array(servo,group,pos);
    }
  }

  servo_positions[servo] = pos;

  // ----- create timing idicies from servo/group data -------

  // clear the timing arrays for fresh start
  for(uint8_t i=0; i<MAX_TIMINGS; i++){
    servo_timings[i] = 0;
    shift_output[i] = 0xFF;
    shift_latch[i] = 0xFF;
  }

  uint8_t counter_index=0;
  uint8_t current_timing=0;
  uint8_t current_shift_output=0;

  for(byte group=0; group<GROUPS; group++){ //go through each group
    if(servos_active_in_group[group] > 0){ // skip it if the group is active, otherwise:
      servo_timings[counter_index] = group_offsets[group];
      shift_output[counter_index] = active_servos_hex[group];
      shift_latch[counter_index] = (1<<group_latches[group]);
      counter_index +=1;


      //create additional timings
      for(byte i=0; i<servos_active_in_group[group]; i++){ //create the timings for each servo after that, using the previous output
        if(servo_positions[servos_sorted[group][i]] == servo_positions[servos_sorted[group][i-1]]){ // if this servo's time is the same as the last's
          if(i != 0){
            counter_index -= 1; //reverse the index count-up
          }
          else{
            current_shift_output = shift_output[counter_index-1];
            servo_timings[counter_index] = servo_positions[servos_sorted[group][i]]+ group_offsets[group];
            shift_latch[counter_index] = (1<<group_latches[group]);
          }
        }
        else{
          current_shift_output = shift_output[counter_index-1];
          servo_timings[counter_index] = servo_positions[servos_sorted[group][i]]+ group_offsets[group];
          shift_latch[counter_index] = (1<<group_latches[group]);
        }

        //subtract the current servo from the shift register output
        current_shift_output &= ~pin_2_num[servos_sorted[group][i]-group*SERVOS_PER_GROUP];
        shift_output[counter_index] = current_shift_output;
        counter_index +=1;
      }
    }
  }

}


void Servotor32::printStatus(Stream *serial) const {
  serial->println("--------------------- Registers ----------------------");

  serial->println("Servo Data:");
  serial->println("Servo\tPos\tTimeEnd\t");
  for(byte i=0; i<SERVOS; i++){
    serial->print(i);
    serial->print("\t");
    serial->print(servo_positions[i]);
    serial->println("");
  }
  serial->println("");

  serial->println("Sorted Groups");
  for(byte i=0; i<GROUPS; i++){
    serial->print("Group: ");
    serial->println(i);
    for(byte j=0; j<SERVOS_PER_GROUP; j++){
      serial->print("Servo: ");
      serial->print(servos_sorted[i][j]);
      serial->print("\t");
      serial->println(servo_positions[servos_sorted[i][j]]);

    }
  }

  serial->println("Group Data:");
  serial->println("#\tActive\tHex");
  for(byte i=0; i<GROUPS; i++){
    serial->print(i);
    serial->print("\t");
    serial->print(servos_active_in_group[i]);
    serial->print("\t");
    serial->println(active_servos_hex[i],HEX);
  }
  serial->println("");

  serial->println("Timings:");
  serial->println("Pos\tTiming\tOutput\tLatch");
  for(uint8_t i=0; i<MAX_TIMINGS; i++){ // clear existing registers, so they can be cleanly written
    serial->print(i);
    serial->print(":\t");
    serial->print(servo_timings[i]);
    serial->print(",\t");
    serial->print(shift_output[i],HEX);
    serial->print(",\t");
    serial->println(shift_latch[i],HEX);
  }
  serial->println("----------------------------------------------------");
}

// modify the state of a servo
void Servotor32::changeServo(byte servo, short pos){
  if(pos == 0){
    pos = -1;
  }
  if(pos == -1){
    update_registers_fast(servo, pos);
  }
  else{
    update_registers_fast(servo, pos/10);
  }
}

short inServo = -1;
short inPos = -1;
// // // //
boolean accumulating = false;
boolean gotDigits = false;
unsigned long accumulator = ~0L;

void startAccumulating() {
    accumulating = true;
    gotDigits = false;
    accumulator = 0;
}
void stopAccumulating() {
    accumulating = false;
}
void accumulateDigit(char digit) {
    if (accumulating) {
        gotDigits = true;
        accumulator = (accumulator * 10) + (digit - '0');
    }
}
boolean isAccumulatorInRange(const unsigned long min, const unsigned long max) {
    return (min <= accumulator) && (max >= accumulator);
}

boolean haveValidServo = false;
boolean haveValidPos = false;
boolean acquiringPos = false;
//short servoNum = -1;
//short servoPos = -1;
short multiPingCnt = DEFAULT_MULTI_PING_CNT;
short arcScanPingCnt = DEFAULT_SCAN_PING_CNT;

/**
 * @brief Acquires a new servo number.
 *
 * If not told to acquire a value, or no digits were supplied, then there is
 * no new number to acquire.  When no new number is detected, the previous
 * servo number is used.
 *
 * When the supplied number is not valid, an error condition is set up to be
 * reported by the caller.
 */
void acquireServo() {
    if (accumulating) { // told to acquire

        stopAccumulating();

        if (gotDigits) { // digits were supplied

            haveValidServo = isAccumulatorInRange(0, 31);

            if (haveValidServo) {
                inServo = (short)accumulator;
            }
            else {
                inServo = -1;
            }
        }
    }
}

/**
 * @brief Acquires a new servo position.
 *
 * If not told to acquire a value, or no digits were supplied, then there is
 * no new posiitopn to acquire.  When no new positiom is detected, the
 * previous position is used.
 *
 * When the supplied position is not valid, an error condition is set up to
 * be reported by the caller.
 */
void acquirePos() {
    if (accumulating)
    {
        stopAccumulating();
        haveValidPos = isAccumulatorInRange(500, 2500);

        if (haveValidPos) {
            inPos = (short)accumulator;
        }
        else {
            inPos = -1;
        }
    }
}

// // // //

void Servotor32::process(Stream *serial) {
  if(serial->available()) { // port has input available
    char inChar = (char)serial->read();
    switch(inChar){
      case '#':
        startAccumulating();
        acquiringPos = false;
        break;
      case 'D':
        printStatus(serial);
        break;
      case 'P':
        acquireServo();
        startAccumulating();
        acquiringPos = true;
        break;
      case ';':
      case '\r':
      case '\n':
        // do nothing when not in "acquiring position" state.
        if (acquiringPos) {
            acquiringPos = false;
            acquirePos();
            if (!haveValidServo) {
                serial->println("Error: Bad Servo");
            }
            else if (!haveValidPos) {
                serial->println("Error: Bad Position");
            }
            else {
#ifdef DEBUG_PROC_CHAR
                serial->print("changeServo(");
                serial->print(inServo);
                serial->print(", ");
                serial->print(inPos);
                serial->println(")");
#endif
                changeServo(inServo,inPos);
                haveValidPos = false;
            }
        }
        break;
      case 'V':
        serial->println("SERVOTOR32_v2.0a");
        break;
      case 'C':
        for(int i=0; i<32; i++){
          changeServo(i, 1500);
        }
        serial->println("All Centered");
        break;
      case 'K':
        for(int i=0; i<32; i++){
          changeServo(i,-1);
        }
        serial->println("All Turned Off");
        break;
      case 'L':
        acquireServo();
        if (haveValidServo) {
#ifdef DEBUG_PROC_CHAR
            serial->print("changeServo(");
            serial->print(inServo);
            serial->println(", -1)");
#endif
            changeServo(inServo, -1);
            serial->println("Servo Turned Off");
        }
        else {
            serial->println("Error: Bad Servo.");
        }
        break;

    case 'm':
        if (accumulating) // told to acquire
        {
            stopAccumulating();

            if (gotDigits) // digits were supplied
            {
                if (1 >= accumulator)
                {
                    multiPingCnt = 1; // use min valid value
                }
                else if (MAX_MULTI_PING_CNT <= accumulator)
                {
                    multiPingCnt = MAX_MULTI_PING_CNT; // use max valid value
                }
                else
                {
                    multiPingCnt = (short)accumulator;
                }
            }
            else
            {
                // no digits supplied, return to default
                multiPingCnt = DEFAULT_MULTI_PING_CNT;
            }
        }
        serial->print("Multi-ping count = ");
        serial->println(multiPingCnt);
        break;
    case 's':
        if (accumulating) // told to acquire
        {
            stopAccumulating();

            if (gotDigits) // digits were supplied
            {
                if (2 >= accumulator)
                {
                    arcScanPingCnt = 2; // use min valid value
                }
                else if (MAX_SCAN_PING_CNT <= accumulator)
                {
                    arcScanPingCnt = MAX_SCAN_PING_CNT; // use max valid value
                }
                else
                {
                    arcScanPingCnt = (short)accumulator;
                }
            }
            else
            {
                // no digits supplied, return to default
                arcScanPingCnt = DEFAULT_SCAN_PING_CNT;
            }
        }
        serial->print("Scan-ping count = ");
        serial->println(arcScanPingCnt);
        break;
    case 'M':
        {
            float d = multiPing(multiPingCnt);
            serial->println(d);
        }
        break;
    case 'S':
        arcPing(serial);
        break;

    case 't':
        serial->println(millis_new());
        break;

    default:
        if ((inChar >= '0') && (inChar <= '9')) {
            accumulateDigit(inChar);
        }
        break;
    }
  }
}

#define MAX_TIME 1000000

float Servotor32::ping(){
  //PB0 for Trigger (17)
  //PB7 for Echo (11)

  pinMode(17,OUTPUT);
  pinMode(11,INPUT);

  long duration;
  float cm;
  digitalWrite(17, LOW);
  delayMicroseconds(2);
  digitalWrite(17, HIGH);
  delayMicroseconds(10);
  digitalWrite(17, LOW);


  uint8_t bit = digitalPinToBitMask(11);
  uint8_t port = digitalPinToPort(11);
  uint8_t stateMask = (HIGH ? bit : 0);

  unsigned long startCount = 0;
  unsigned long endCount = 0;
  unsigned long width = 0; // keep initialization out of time critical area

  // convert the timeout from microseconds to a number of times through
  // the initial loop; it takes 16 clock cycles per iteration.
  unsigned long numloops = 0;
  unsigned long maxloops = 500;

  // wait for any previous pulse to end
  while ((*portInputRegister(port) & bit) == stateMask)
    if (numloops++ == maxloops)
      return 0;

  // wait for the pulse to start
  while ((*portInputRegister(port) & bit) != stateMask)
    if (numloops++ == maxloops)
      return 0;

  startCount = micros_new();
  // wait for the pulse to stop
  while ((*portInputRegister(port) & bit) == stateMask) {
    if (numloops++ == maxloops)
      return 0;
    delayMicroseconds(10); //loop 'jams' without this
    if((micros_new() - startCount) > 58000 ){ // 58000 = 1000CM
      return 0;
      break;
    }
  }
  duration = micros_new() - startCount;
  //--------- end pulsein
  cm = (float)duration / 29.0 / 2.0;
  return cm;
}

float Servotor32::multiPing(unsigned short attempts=DEFAULT_MULTI_PING_CNT){
    float distances [attempts];

    for (int i=0; i<attempts; i++) {
        distances[i] = ping();
    }

    int i, j;

    if (1 < attempts) {
        float temp;

        // sort them in order
        for (i = (attempts - 1); i > 0; i--)
        {
            for (j = 1; j <= i; j++)
            {
                if (distances[j-1] > distances[j])
                {
                    temp = distances[j-1];
                    distances[j-1] = distances[j];
                    distances[j] = temp;
                }
            }
        }
        // choose the middle entry
        i = (int)ceil((float)attempts/2.0);
    }
    else {
        i = 0;
    }
    return distances[i];
}

// measure distances over a 180 degree arc and report them to the given serial port.
void Servotor32::arcPing(Stream* serial) {

    float step = ((float)(MAX_POS - MIN_POS)) / (arcScanPingCnt - 1);
    /*
     * It takes about 500 mSec for full swing travel, so we start with a compromise of 300.
     * 
     * At each step we only need enough delay for that fraction of full travel.
     * We add one as a round-up and to guard against the integer division returning zero.
     */
    long unsigned int mSecDelay = 300;
    long unsigned int stepDelay = ((500 / arcScanPingCnt) + 1);

    for (int i = 0; i < arcScanPingCnt; i++)
    {
        int pos = ((int)(step * i)) + MIN_POS;
        changeServo(PING_SERVO, pos);

        delay_ms(mSecDelay);

        mSecDelay = stepDelay;

        float d = multiPing(multiPingCnt);
        serial->println(d);
    }
    changeServo(PING_SERVO, CENTER_POS);
    delay_ms(400);
    changeServo(PING_SERVO, KILL_POS);
}
