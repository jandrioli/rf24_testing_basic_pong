/*
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *           WARNING
 * 
 * 
 * This sketch is using " strange " pipes. At least the PING I know is working with the 
 * old ATMEGA328P that I have programmed a long time ago to use pipes "3Node", "4Node".
 * Like I said the PING sketch works, but this here doesn't seem to be compatible so 
 * a re-write of this pong sketch might be necessary before testing with new chips. 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 

JCAM Radio slave sleeper program. 
This is based on https://gist.github.com/bryanthompson/ef4ecf24ad36410f077b for radio
This is based on [Martin Nawrath] for watchdog

This project is for arduinos that use batteries as power source. These devices will go into sleep mode for configurable
amounts of time, and during that time they appear to be powered off. 
The Radio transmission part of this software is copy-paste from Bryan's code. It is slightly modified only to be 
compatible with the shutdown/sleep/watchdog functionality.
In other words, we use the Martin's code wrapping Bryan's code, taking care to properly initialize and turn up on the
radio once we are in the active loop. 

In very basic setups this can mean that your arduino will run for about 3 months with only a few AA batteries.
*/

#include <SPI.h>
#include <RF24.h>
#include "printf.h"

//includes for watchdog/nightingale/sleep features
#include <avr/sleep.h> 
#include <avr/wdt.h>

#define LED 13
#define TMP A0
#define RF_CS 9
#define RF_CSN 10
#define BUZ 8

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

int nint;
boolean ledState = false;
byte del;
int cnt;
byte state=0;
int light=0;

volatile boolean f_wdt=1;

RF24 radio(RF_CS, RF_CSN);
const uint64_t pipes[2] = { 0xe7e7e7e7e7LL, 0xc2c2c2c2c2LL };

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(TMP, INPUT);
  pinMode(BUZ, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(3000);
  Serial.begin(115200);
  printf_begin();
  radio.begin();
  radio.openWritingPipe(pipes[1]);    // note that our pipes are the same above, but that
  radio.openReadingPipe(1, pipes[0]); // they are flipped between rx and tx sides.
  radio.startListening();
  radio.printDetails();
  digitalWrite(LED, LOW);
  
  // CPU Sleep Modes 
  // SM2 SM1 SM0 Sleep Mode
  // 0    0  0 Idle
  // 0    0  1 ADC Noise Reduction
  // 0    1  0 Power-down
  // 0    1  1 Power-save
  // 1    0  0 Reserved
  // 1    0  1 Reserved
  // 1    1  0 Standby(1)

  /*cbi( SMCR,SE );      // sleep enable, power down mode
  cbi( SMCR,SM0 );     // power down mode
  sbi( SMCR,SM1 );     // power down mode
  cbi( SMCR,SM2 );     // power down mode

  setup_watchdog(9);*/
}
 
void loop() {
  
  //if (f_wdt==1)   // wait for timed out watchdog / flag is set when a watchdog timeout occurs
  {
    f_wdt=0;       // reset flag

    int temp = 0.0;
    temp = 66;//analogRead(TMP) * 00.48828125; // if no thermometer is connected, what do we read?
  
    if (radio.available()) {
      uint8_t rx_data[32];  // we'll receive a 32 byte packet
    
      bool done = false;
      while (!done) {
        done = radio.read( &rx_data, sizeof(rx_data) );
        printf("Got payload @ %lu...\r\n", millis());
        printf("First Value: %i...\r\n", rx_data[0] );
      
        //TODO: add code that checks for precise data in the rvd package
        //TODO: acceptable commands: wake up, tx log, sleep nightingale
      }
    
      // first value was probably 254 or 666 or 66 depending on Joao's code evolution... replace that value with current temperature
      rx_data[0] = 666;
      radio.stopListening();
      radio.write( &rx_data, sizeof(rx_data) );
      radio.startListening(); 
    }
    digitalWrite(LED, !digitalRead(LED));
    /*whistle();
    system_sleep();*/
  }
}
 
//****************************************************************  
// put some whiste sound on piezo
void whistle() {
  for (int ii = 0; ii<= 20; ii++) {  
    for (del=0; del <=254; del++) {
      digitalWrite(BUZ,0);
      delayMicroseconds((ii*5)+(del * 2));
      digitalWrite(BUZ,1);
      delayMicroseconds((ii*5)+(del * 2));
    }
    PORTB ^= 32;  // toggle pinLed
  }  
}

//****************************************************************  
// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep() 
{
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System sleeps here
  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
}

//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) 
{
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}
//****************************************************************  
// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}
 


