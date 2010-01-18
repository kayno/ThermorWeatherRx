/*
 * "Thermor" DG950R Weather Station receiver v 0.2
 *
 * Receives data from "Thermor" DG950R Weather Station receiver, via
 * a 433Mhz RF receiver connected to pin 8 of the arduino, and outputs
 * to serial.
 *
 * Based on the Practical Arduino Weather Station Receiver project 
 * (http://www.practicalarduino.com/projects/weather-station-receiver). 
 * For more info: 
 * http://kayno.net/2010/01/15/arduino-weather-station-receiver-shield/
 *
 * TODO:
 * - determine if there is a checksum byte/bits, or if the sending of the data 4 times is itself a checksum
 * - handle rain packets better - bucket tips are are sent incrementally - rain mm is calculated by subtracting
 *   current tip count from previous. should report tip count rather than doing calulation on arduino so that
 *   if the arduino is reset or a packet sent from it is missed, any missed rain value can still be calculated
 * - wind speed is currently determined by a 'fudge factor', which itself was calculated based on the value being
 *   transmitted compared to the value displayed on the weather station's base station. perhaps there is a better way?
 */

#define INPUT_CAPTURE_IS_RISING_EDGE()    ((TCCR1B & _BV(ICES1)) != 0)
#define INPUT_CAPTURE_IS_FALLING_EDGE()   ((TCCR1B & _BV(ICES1)) == 0)
#define SET_INPUT_CAPTURE_RISING_EDGE()   (TCCR1B |=  _BV(ICES1))
#define SET_INPUT_CAPTURE_FALLING_EDGE()  (TCCR1B &= ~_BV(ICES1))

#define WEATHER_RX_LED_ON()         ((PORTD &= ~(1<<PORTD6)))
#define WEATHER_RX_LED_OFF()        ((PORTD |=  (1<<PORTD6)))

#define WEATHER_RESET()             { short_count = packet_bit_pointer = 0; weather_rx_state = RX_STATE_IDLE; current_bit = BIT_ZERO; WEATHER_RX_LED_OFF(); }

#define TIMER_PERIOD_US             4
#define WEATHER_PACKET_BIT_LENGTH   64

// pulse widths. short pulses ~500us, long pulses ~1000us. 50us tolerance
#define SHORT_PULSE_MIN_WIDTH       450/TIMER_PERIOD_US
#define SHORT_PULSE_MAX_WIDTH       550/TIMER_PERIOD_US
#define LONG_PULSE_MIN_WIDTH        950/TIMER_PERIOD_US
#define LONG_PULSE_MAX_WIDTH        1050/TIMER_PERIOD_US

// number of shorts in a row before the stream is treated as valid
#define SHORT_COUNT_SYNC_MIN        100

// states the receiver can be
#define RX_STATE_IDLE               0 // waiting for incoming stream
#define RX_STATE_RECEIVING          1 // receiving valid stream
#define RX_STATE_PACKET_RECEIVED    2 // valid stream received

#define BIT_ZERO                    0
#define BIT_ONE                     1

//byte locations of generic weather data in weather_packet[] array
#define WEATHER_STATION_ID          0
#define WEATHER_PACKET_TYPE         1

//types of packets
#define PACKET_TYPE_SYNC            0
#define PACKET_TYPE_WIND            1
#define PACKET_TYPE_TEMP            2
#define PACKET_TYPE_RAIN            3

//byte locations of temperature data in weather_packet[] array
#define TEMP_WHOLE                  2
#define TEMP_DECIMAL                3
#define TEMP_OFFSET                 41

//byte locations of wind data in weather_packet[] array
#define WIND_DIRECTION              5
#define WIND_SPEED                  2
#define WIND_SPEED_OVERFLOW         3
#define WIND_SPEED_FACTOR           0.117

//byte locations of rain data in weather_packet[] array
#define RAIN_TIP_COUNT              2
#define RAIN_TIP_COUNT_OVERFLOW     3
#define RAIN_MM_PER_TIP             0.5


//#define DEBUG

// Type aliases for brevity in the actual code
typedef unsigned int       uint; //16bit
typedef signed int         sint; //16bit


uint captured_time;
uint previous_captured_time;
uint captured_period;
uint current_bit;
uint packet_bit_pointer;
uint short_count;
uint weather_rx_state;

boolean previous_period_was_short = false;

//byte array used to store incoming weather data
byte weather_packet[(WEATHER_PACKET_BIT_LENGTH/8)];

const char wind_directions[16][4] = 
{
  "SSE", "SW", "S", "SSW",
  "SE", "WSW", "ESE", "W", 
  "NE", "NNW", "NNE", "N", 
  "ENE", "NW", "E", "WNW"
};
  
long int current_rain_tip_count = 0;
long int previous_rain_tip_count = 0;

/* Overflow interrupt vector */
ISR(TIMER1_OVF_vect){                 // here if no input pulse detected
}

/* ICR interrupt vector */
ISR(TIMER1_CAPT_vect){
  // Immediately grab the current capture time in case it triggers again and
  // overwrites ICR1 with an unexpected new value
  captured_time = ICR1;

  //immediately grab the current capture polarity and reverse it to catch all the subsequent high and low periods coming in
  if(INPUT_CAPTURE_IS_RISING_EDGE()) {
    SET_INPUT_CAPTURE_FALLING_EDGE();      //previous period was low and just transitioned high   
  } else {
    SET_INPUT_CAPTURE_RISING_EDGE();       //previous period was high and transitioned low    
  }

  // calculate the current period just measured, to accompany the polarity now stored
  captured_period = (captured_time - previous_captured_time);

  // Analyse the incoming data stream. If idle, we need to detect the start of an incoming weather packet.
  // Incoming packet starts with several short pulses (over 100 short pulses) before a long pulse to signify 
  // the start of the data.
  
  if(weather_rx_state == RX_STATE_IDLE) {
    
    if(((captured_period >= SHORT_PULSE_MIN_WIDTH) && (captured_period <= SHORT_PULSE_MAX_WIDTH))) { 
      // short pulse, continue counting short pulses
      short_count++;
    } else if(((captured_period >= LONG_PULSE_MIN_WIDTH) && (captured_period <= LONG_PULSE_MAX_WIDTH))) { 
      // long pulse. if there has been enough short pulses beforehand, we have a valid bit stream, else reset and start again
      if(short_count > SHORT_COUNT_SYNC_MIN) {
        weather_rx_state = RX_STATE_RECEIVING;
      } else {
        WEATHER_RESET();
      }
    } else {
      WEATHER_RESET();
    }
  } else if(weather_rx_state == RX_STATE_RECEIVING) {
    // incoming pulses are a valid bit stream, manchester encoded. starting with a zero bit, the next bit will be the same as the 
    // previous bit if there are two short pulses, or the bit will swap if the pulse is long
    if(((captured_period >= SHORT_PULSE_MIN_WIDTH) && (captured_period <= SHORT_PULSE_MAX_WIDTH))) {  
      // short pulse
      if(previous_period_was_short) { 
        // previous bit was short, add the current_bit value to the stream and continue to next incoming bit
        if(current_bit == BIT_ONE) {
          weather_packet[packet_bit_pointer >> 3] |=  (0x80 >> (packet_bit_pointer&0x07));
        } else if (current_bit == BIT_ZERO) {
          weather_packet[packet_bit_pointer >> 3] &= ~(0x80 >> (packet_bit_pointer&0x07));
        }

        packet_bit_pointer++;
        
        previous_period_was_short = false;
      } else {
        // previous bit was long, remember that and continue to next incoming bit
        previous_period_was_short = true;
      }
    } else if(((captured_period >= LONG_PULSE_MIN_WIDTH) && (captured_period <= LONG_PULSE_MAX_WIDTH))) { 
      // long pulse
      // swap the currrent_bit
      current_bit = !current_bit;
      
      // add current_bit value to the stream and continue to next incoming bit
      if(current_bit == BIT_ONE) {
        weather_packet[packet_bit_pointer >> 3] |=  (0x80 >> (packet_bit_pointer&0x07));
      } else if (current_bit == BIT_ZERO) {
        weather_packet[packet_bit_pointer >> 3] &= ~(0x80 >> (packet_bit_pointer&0x07));
      }

      packet_bit_pointer++;
    }
    
    // check to see if a full packet has been received
    if(packet_bit_pointer > WEATHER_PACKET_BIT_LENGTH) {
      // full packet received, switch state to RX_STATE_PACKET_RECEIVED
      weather_rx_state = RX_STATE_PACKET_RECEIVED;
    }
  }
  
  // save the current capture data as previous so it can be used for period calculation again next time around
  previous_captured_time = captured_time;
}

void setup() {
  Serial.begin(9600);
  
  DDRB = 0x2F;   // B00101111
  DDRB  &= ~(1<<DDB0);    // PBO(ICP1) input
  PORTB &= ~(1<<PORTB0);  // ensure pullup resistor is also disabled
  DDRD  |=  B11000000;    // (1<<PORTD6);   //DDRD  |=  (1<<PORTD7); (example of B prefix)

  //---------------------------------------------------------------------------------------------
  //ICNC1: Input Capture Noise Canceler         On, 4 successive equal ICP1 samples required for trigger (4*4uS = 16uS delayed)
  //ICES1: Input Capture Edge Select            1 = rising edge to begin with, input capture will change as required
  //CS12,CS11,CS10   TCNT1 Prescaler set to 0,1,1 see table and notes above
  TCCR1A = B00000000;   //Normal mode of operation, TOP = 0xFFFF, TOV1 Flag Set on MAX
                        //This is supposed to come out of reset as 0x00, but something changed it, I had to zero it again here to make the TOP truly 0xFFFF
  TCCR1B = ( _BV(ICNC1) | _BV(CS11) | _BV(CS10) );
  SET_INPUT_CAPTURE_RISING_EDGE();
  //Timer1 Input Capture Interrupt Enable, Overflow Interrupt Enable  
  TIMSK1 = ( _BV(ICIE1) | _BV(TOIE1) );
  
  WEATHER_RESET();
  
  Serial.println("\"Thermor\" DG950R Weather Station receiver v0.2");
  Serial.println("Ready to receive weather data");
}


/* 
 * main loop waits for valid packet, decodoes it and outputs the weather data contained 
 * in the packet to the serial port.
 *
 * there are 4 known packet types - sync, wind, temp and rain:
 *
 * PACKET_TYPE_SYNC:
 * ssssssss     pppp
 * 10000000 11000000 01000100 00000010 00000000 00000000 00000010 00001110 
 *
 * PACKET_TYPE_TEMP:
 *                   72       6        //72.6 - 41 (temp offset) = 31.6 degrees celcius
 * ssssssss     pppp tttttttt tttttttt
 * 10000000 11010010 01001000 00000110 01010011 01111111 11111111 11111111
 *
 * PACKET_TYPE_WIND:
 *                   wwwwwwww wwwwwwww              wwww
 * ssssssss     pppp ssssssss ssssssss              dddd
 * 10000000 11010001 00000000 00000000 00100011 00001101 10000001 1111111
 *
 * PACKET_TYPE_RAIN:
 * ssssssss     pppp rrrrrrrr rrrrrrrr
 * 10000000 11010011 11001110 10111011 00000000 00100110 01111111 11111111
 *
 * s = station id
 * p = packet type
 * t = outside temp
 * ws = wind speed
 * wd = wind direction
 * r = rain 
 */
void loop() {
  uint station_id;
  uint packet_type;
  
  // weather packet ready to decode
  if(weather_rx_state == RX_STATE_PACKET_RECEIVED) {
    // get station if and packet type
    station_id = weather_packet[WEATHER_STATION_ID];
    packet_type = weather_packet[WEATHER_PACKET_TYPE] & 0x0F; //last 4 bits of the byte
  
#ifdef DEBUG
    Serial.println();
    
    for(int i = 0; i < ((WEATHER_PACKET_BIT_LENGTH/8)); i++) {
      Serial.print(weather_packet[i], BIN);
      Serial.print(" ");
    }
    
    Serial.println();
#endif
    
    // decode the packet, based on the packet type
    switch(packet_type) {
      case PACKET_TYPE_SYNC:
        // sync packet, just report it.
        Serial.print("SYNC,");
        Serial.print(station_id, DEC);
        break;
        
      case PACKET_TYPE_WIND:
        // wind packet
        // check first to make sure the wind anemometer and vane is connected
        // if it is not connected, B11111111 and B11111111 is transmitted
        if(weather_packet[WIND_SPEED] != B11111111 && weather_packet[WIND_SPEED_OVERFLOW] != B11111111) {
          Serial.print("WIND,");
          Serial.print(station_id, DEC);
          Serial.print(",");
          
          // wind direction
          Serial.print(wind_directions[weather_packet[WIND_DIRECTION]]);
          Serial.print(",");
    
          // wind speed
          Serial.print((weather_packet[WIND_SPEED]+(weather_packet[WIND_SPEED_OVERFLOW]*256))*WIND_SPEED_FACTOR);
          break;
        }
        
      case PACKET_TYPE_TEMP:
        // outside temp packet
        // if the the data is B11111111 and B11111111, ignore (happens when wind anemometer and vane is first connected)
        if(weather_packet[TEMP_WHOLE] != B11111111 && weather_packet[TEMP_DECIMAL] != B11111111) {
          Serial.print("TEMP,");
          Serial.print(station_id, DEC);
          Serial.print(",");
          
          // print the whole number value of the temperature
          // temp offset (-41) allows -40 degrees to reported as 1 (a positive number, negating need to deal with negatives)
          Serial.print((weather_packet[TEMP_WHOLE]-TEMP_OFFSET), DEC); //temp before decimal point. 
          Serial.print("."); //decimal point
          
          //print the decimal value of the temperature
          Serial.print(weather_packet[TEMP_DECIMAL], DEC);
          break;
        }
        
      case PACKET_TYPE_RAIN:
        // rain packet
        // if the the data is B11111111 and B11111111, ignore (happens when wind anemometer and vane is first connected)
        if(weather_packet[TEMP_WHOLE] != B11111111 && weather_packet[TEMP_DECIMAL] != B11111111) {
        
          // calculate rain tips
          if(previous_rain_tip_count == 0) {
            current_rain_tip_count = 0;
          } else {
            current_rain_tip_count = weather_packet[RAIN_TIP_COUNT]+(weather_packet[RAIN_TIP_COUNT_OVERFLOW]*256) - previous_rain_tip_count;
          }
          
          // store this so we can calculate number of tips next time round
          previous_rain_tip_count = weather_packet[RAIN_TIP_COUNT]+(weather_packet[RAIN_TIP_COUNT_OVERFLOW]*256);
          
          // print the rain fall, in mm
          Serial.print("RAIN,");
          Serial.print(station_id, DEC);
          Serial.print(",");
          Serial.print(current_rain_tip_count*RAIN_MM_PER_TIP);
          break;
        }
        
      default:
        // unknown packet, output the packet for further investigation
        Serial.print("UNKNOWN,");
        Serial.print(station_id, DEC);
        Serial.print(",");
    
        // print out the bytes for debug purposes
        for(int i = 0; i < ((WEATHER_PACKET_BIT_LENGTH/8)); i++) {
          Serial.print(weather_packet[i], BIN);
          Serial.print(" ");
        }
        
        break;
        
    }
    
    Serial.println();
    
    WEATHER_RESET();
  }
} 


