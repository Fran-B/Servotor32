#ifndef Servotor32_h
#define Servotor32_h

#include "Servotor32_SPI.h"
#include "Servotor32_TimerOne.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "Arduino.h"

#define STATUS_LED 7

#define SERVOS 32
#define MAX_TIMINGS 36

#define GROUPS 4
#define SERVOS_PER_GROUP 8

/**
 * @brief Servotor32 is the main Servo-Motor control class.
 *
 * This is a "Singleton" class in that only one should exist.
 *
 * The "begin" method should be called only once when it is time to
 * initialize the system.
 *
 ***************************************************************************
 */
class Servotor32 {
public:
    /**
     * @brief Default constructor creates a Servotor32.
     *
     ***********************************************************************
     */
    Servotor32();

    /**
     * @brief Initializes the Servo-motor system.
     *
     * This method should be called only once when the system is ready to be
     * started.
     *
     ***********************************************************************
     */
    void begin();

    /**
     * @defgroup Time Time replacement methods
     * @{
     * These methods provide replacement methods for the standard time
     * methods that are broken by the way Servotor32 configures the hardware.
     */
    /**
     * @brief Replacement for the standard "micros" method.
     *
     * @return "unsigned int" number of micro-seconds since the sketch started.
     *
     ***********************************************************************
     */
    long unsigned int micros_new() const;

    /**
     * @brief Replacement for the standard "millis" method.
     *
     * @return "unsigned int" number of milli-seconds since the sketch started.
     *
     ***********************************************************************
     */
    long unsigned int millis_new() const;

    /**
     * @brief Replacement for the standard "delay" method.
     *
     * @param delay_time "long unsigned int" number of milli-seconds to
     *      delay.
     *
     ***********************************************************************
     */
    void delay_ms(long unsigned int delay_time) const;

    /**
     * @brief Replacement for the standard "delayMicroseconds" method.
     *
     * @param delay_time "long unsigned int" number of micro-seconds to
     *      delay.
     *
     ***********************************************************************
     */
    void delay_us(long unsigned int delay_time) const;
    /** @} */ // end Time group

    /**
     * @brief Changes the state of a Servo-motor.
     *
     * @param servo "byte"identifying the servo to be changed.
     * @param pos "short" containing the new position for the servo.
     *
     ***********************************************************************
     */
    void changeServo(byte servo, short pos);
  
    /**
     * @brief Prints the state of all internal servo control registers to
     *      a serial port.
     *
     * @param serial Pointer to Stream to print status to.
     *
     ***********************************************************************
     */
    void printStatus(Stream* serial) const;
  
    /**
     * @brief Checks serial stream for available input and processes a
     *      single character in a command string thereby providing for
     *      external control.
     *
     * This implements the following commands:
     *
     *  * Display the firmware version number with 'V'
     *
     *  * Set the multiPing() attempt count with '#[count]m'
     *    + Display current miltiPing() attempt count with 'm'
     *
     *  * Set the arcPing() point count with '#[count]s'
     *    + Display current arcPing() point count with 's'
     *
     *  * Make a multiPing() measurement with 'M'
     *
     *  * Make an arcPing() with 'S'
     *
     *  * Display current milli-seconds since boot with 't'
     *
     *  * Print icky debug details with 'D'
     *
     *  * Center all servos with 'C'
     *
     *  * Kill all servos with 'K'
     *
     *  * Kill a single servo with '#[servo number]L'
     *
     *  * Move a single servo with '#[servo Number]P[Servo Position]'. Must
     *    be followed by ';', carrage-return or line-feed.  Both/all is okay.
     *
     *  Servo number must be in the range 0..31
     *
     *  Servo position value must be in the range of 500..2500
     *  based on the following:
     *
     *  *  500 = 90 degrees Left
     *  * 1500 = 0 degrees, center  
     *  * 2500 = 90 degrees Right  
     *
     * @param serial Pointer to Stream to check for input.
     *
     ***********************************************************************
     */
    void process(Stream* serial);

    /**
     * @defgroup Sonar Sonar access methods
     * @{
     * These methods provide access to the sonar sensor.
     */
    /**
     * @brief Obtain a single sonar reading.
     *
     * Zero is returned when something goes wrong with the reading.
     *
     * @return "float" containing the distance measured in centimeters.
     *
     ***********************************************************************
     */
    float ping();

    /**
     * @brief Obtain a median sonar reading.
     *
     * This method will take a given number of readings and return the
     * middle distance measured.
     *
     * @param attempts "short" number of sonar read attempts to make.
     *      Default=5.
     *
     * @return "float" containing the average distance measured in centimeters.
     *
     ***********************************************************************
     */
    float multiPing(unsigned short attempts);

    /**
     * @brief Obtain a median sonar reading at a number of points along a
     * 180 degree arc.
     *
     * This method will calls {@link multiPing(unsigned short) multiPing()}
     * with the currently set number of "attempts" at a number of points
     * along a 180 degree arc, including both ends.  The number of points is
     * also configurable and the default number is 9.
     *
     * Each multiPing() result is pirinted to the give serial Stream.
     *
     * @param serial Pointer to Stream to print results out to.
     *
     ***********************************************************************
     */
    void arcPing(Stream* serial);

    /** @} */ // end Sonar group

private:
    void update_registers_fast(byte, signed short);
    void delete_from_sorted_array(byte, byte, signed short);
    void add_to_sorted_array(byte, byte, signed short);

    /**
     * @brief Method set up to be called every 10 uSec by the Arduino runtime.
     *
     * This method handles time and servo updates.
     *
     ***********************************************************************
     */
    static void callback();
};

#endif
