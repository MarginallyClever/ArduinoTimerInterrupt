//------------------------------------------------------------------------------
// Timer Interrupt Demo - Supports most Arduino-based shields
// dan@marginallycelver.com 2013-11-03
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/ArduinoTimerInterrupt for more information.
// Special thanks to Reddit users UserNotAvailable & OverVolt for peerless programming :)


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define VERBOSE (1)
#define LEDPIN      (13)
#define BAUD        (57600)
#define CLOCK_FREQ  (16000000L)
#define MAX_COUNTER (65536L)


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
/**
 * Clock interrupt method.
 */
ISR(TIMER1_COMPA_vect) {
  digitalWrite( LEDPIN, digitalRead(LEDPIN)^1 );
}


/**
 * Set the clock 1 timer frequency.
 * @input desired_freq_hz the desired frequency
 */
void timer_set_frequency(long desired_freq_hz) {
  // Source: http://letsmakerobots.com/node/28278
  // Different clock sources can be selected for each timer independently. 
  // To calculate the timer frequency (for example 2Hz using timer1) you will need:
  
  //  CPU frequency 16Mhz for Arduino
  //  maximum timer counter value (256 for 8bit, 65536 for 16bit timer)
  int prescaler_index=-1;
  int prescalers[] = {1,8,64,256,1024};
  long counter_value;
  do {
    ++prescaler_index;
    //  Divide CPU frequency through the choosen prescaler (16000000 / 256 = 62500)
    counter_value = CLOCK_FREQ / prescalers[prescaler_index];
    //  Divide result through the desired frequency (62500 / 2Hz = 31250)
    counter_value /= desired_freq_hz;
    //  Verify counter_value < maximum timer. if fail, choose bigger prescaler.
  } while(counter_value > MAX_COUNTER && prescaler_index<4);
  
  if( prescaler_index>=5 ) {
    Serial.println(F("Timer could not be set: Desired frequency out of bounds."));
    return;
  }

//#ifdef VERBOSE
  Serial.print(F("counter_value  ="));  Serial.print(counter_value);
  Serial.print(F(" prescaler_index="));  Serial.print(prescaler_index);
  Serial.print(F(" = "));  Serial.print(((prescaler_index&0x1)   ));
  Serial.print(F("/"));  Serial.print(((prescaler_index&0x2)>>1));
  Serial.print(F("/"));  Serial.println(((prescaler_index&0x4)>>2));
//#endif

  // disable global interrupts
  noInterrupts();
  
  // set entire TCCR1A register to 0
  TCCR1A = 0;
  // set entire TCCR1B register to 0
  TCCR1B = 0;
  // set the overflow clock to 0
  TCNT1  = 0;
  // set compare match register to desired timer count
  OCR1A = counter_value;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10, CS11, and CS12 bits for prescaler
  TCCR1B |= ( (( prescaler_index&0x1 )   ) << CS10);
  TCCR1B |= ( (( prescaler_index&0x2 )>>1) << CS11);
  TCCR1B |= ( (( prescaler_index&0x4 )>>2) << CS12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  interrupts();  // enable global interrupts
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms
  pinMode(LEDPIN, OUTPUT);
}


/**
 * Runs after setup() and repeats forever
 */
void loop() {
  Serial.println(F("1hz"));  timer_set_frequency(1);  delay(2000);
  Serial.println(F("2hz"));  timer_set_frequency(2);  delay(2000);
  Serial.println(F("4hz"));  timer_set_frequency(4);  delay(2000);
  Serial.println(F("8hz"));  timer_set_frequency(8);  delay(2000);
  Serial.println(F("16hz"));  timer_set_frequency(16);  delay(2000);
  Serial.println(F("32hz"));  timer_set_frequency(32);  delay(2000);
  Serial.println(F("64hz"));  timer_set_frequency(64);  delay(2000);
  Serial.println(F("128hz"));  timer_set_frequency(128);  delay(2000);
  Serial.println(F("256hz"));  timer_set_frequency(256);  delay(2000);
  Serial.println(F("512hz"));  timer_set_frequency(512);  delay(2000);
  Serial.println(F("1000hz"));  timer_set_frequency(1000);  delay(2000);
  Serial.println(F("2000hz"));  timer_set_frequency(2000);  delay(2000);
  Serial.println(F("4000hz"));  timer_set_frequency(4000);  delay(2000);
  Serial.println(F("10000hz"));  timer_set_frequency(10000);  delay(2000);
}


/**
* This file is part of Arduino Timer Interrupt.
*
* Arduino Timer Interrupt is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Arduino Timer Interrupt is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Arduino Timer Interrupt. If not, see <http://www.gnu.org/licenses/>.
*/
