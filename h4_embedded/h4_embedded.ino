//////////////////////////////////////////////////////////////////
// Author: RSP @ ESL (Embedded System Lab), KMUTNB
// Date: 16-Jul-2013
// Target Board: Arduino Uno (ATmega328P, 5V, 16MHz)
// Arduino IDE: version 1.0.5
// HC-SR04 Ultrasonic module (using VCC=5V)

#define ECHO_PIN  3  // Echo Pin (Input) -- from the ECHO pin of HC-SR04
#define TRIG_PIN  5  // Trigger Pin (Output) -- to the TRIG pin of HC-SR04
#include <avr/io.h>

void T1_init( );
void INT0_init();

void setup() {
  T1_init( );
  INT0_init();
  pinMode( TRIG_PIN, OUTPUT );
  pinMode( ECHO_PIN, INPUT );  
  digitalWrite( TRIG_PIN, LOW ); // output LOW to the TRIG pin
  Serial.begin( 115200 ); // initialize serial, use baudrate=115200
}



volatile uint32_t count = 0;
volatile uint8_t state1 = 0, state2 = 0;

void T1_init() {
  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = 0;
  ICR1 = 0;
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0; // disable all interrupts
  TIFR1 = (1<<ICF1) | (1<<OCF1B) | (1<<OCF1A) | (1<<TOV1); // clear all flags
  // Use Timer/Count1 in normal mode (no output compare, no PWM)
  TCCR1B |= (1<<CS11); // prescaler=8
  TIMSK1 = (1<<TOIE1); // Timer1 Overflow Interrupt Enable
}

void INT0_init() {
  cli(); // disable global interrupt
  EICRA |= (1 << ISC01); // falling edge
  EIMSK |= (1 << INT0); // enable INT0 interrupt
  sei(); // enable global interrupt
}

ISR(TIMER1_OVF_vect) { // ISR for Timer/Counter1 Overflow Interrupt
  if(count == 99){
    count = 0;
  } else 
    count++; // increment counter
}

ISR(INT0_vect) { // ISR for INT0
  state1 ^= 1; // toggle state1
  digitalWrite( LED1_PIN, state1 ); // update LED1 status
}

void loop() {
  unsigned long duration_usec;
  unsigned long distance_mm;
  // v = 340 m/s = (340 * 100)/10^6 cm/usec = 34/1000 cm/usec
  // 2*d = v*t => d = v*t/2 = (17*t)/1000 cm = (17*t)/100 mm.  
  while (1) {
    duration_usec = ping();
    distance_mm = (17*duration_usec)/100; 
    if ( distance_mm > 4000 ) { // out of range (beyond 4 meters)
       // Serial.println( "Out of range!" );
       continue;
    }
    Serial.print( "Distance: " );
    Serial.print( distance_mm / 10 );
    Serial.print( '.' );
    Serial.print( distance_mm % 10 );
    Serial.println( " cm" );
    break;
  }
  delay(300);
}

unsigned long ping() {
  // send a pulse (at least 10 usec long) to the TRIG pin
  digitalWrite( TRIG_PIN, HIGH ); 
  delayMicroseconds( 12 ); 
  digitalWrite( TRIG_PIN, LOW ); 
  // measure the ECHO pulse width (in microseconds)
  unsigned long duration_usec = pulseIn( ECHO_PIN, HIGH );
  return duration_usec;
}
/////////////////////////////////////////////////////////////////////





