
#define ECHO_PIN  2  // Echo Pin (Input) -- from the ECHO pin of HC-SR04
#define TRIG_PIN  5  // Trigger Pin (Output) -- to the TRIG pin of HC-SR04
#include <avr/io.h>

void T1_init( );

void setup() {
  T1_init( );
  PCICR |= (1 << PCIE2); // enable Pin Change Interrupt 2
  PCMSK2 |= (1 << PCINT18); // enable PCINT19 interrupt
  pinMode( TRIG_PIN, OUTPUT );
  pinMode( ECHO_PIN, INPUT );
  digitalWrite( TRIG_PIN, LOW ); // output LOW to the TRIG pin
  Serial.begin( 115200 ); // initialize serial, use baudrate=115200
}

volatile uint32_t count = 0;
volatile uint8_t state = 1, finish_state = 0;
volatile uint32_t value2 = 0;

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

ISR(TIMER1_OVF_vect) { // ISR for Timer/Counter1 Overflow Interrupt
    count++; // increment counter
}

ISR(PCINT2_vect) { // ISR for INT0
  switch (state){
    case 1:
      if(digitalRead( ECHO_PIN) == 1){
        count = 0;
        TCNT1 = 0;
        state = 2;
        //Serial.println( "isr case 1"  + String(EICRA));
      }
      break;
    case 2:
      if(digitalRead( ECHO_PIN) == 0){
        state = 1;
        value2 = (count * 65536 + TCNT1)*5/10;
        finish_state = 1;
        //Serial.println("value2 = " + String(count));
      }
      break;
      //null
  }
}

void loop() {
  unsigned long duration_usec;
  unsigned long distance_mm;
  // v = 340 m/s = (340 * 100)/10^6 cm/usec = 34/1000 cm/usec
  // 2*d = v*t => d = v*t/2 = (17*t)/1000 cm = (17*t)/100 mm.  
  while (1) {
    duration_usec = pulse();
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

unsigned long pulse() {
  digitalWrite( TRIG_PIN, HIGH ); 
  delayMicroseconds( 12 ); 
  digitalWrite( TRIG_PIN, LOW ); 
  while(finish_state != 1){
    //null
  }
  finish_state = 0;
  Serial.println( "method pulse = " +  String(value2));
  return (value2);
}


/////////////////////////////////////////////////////////////////////





