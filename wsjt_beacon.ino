// Based on Si5351_WSPR_2560.ino by Jason Milldrum NT7S.
//
// Si5351_WSPR
//
// Simple WSPR beacon for Arduino Uno, with the Etherkit Si5351A Breakout
// Board by Jason Milldrum NT7S.
// 
// Original code based on Feld Hell beacon for Arduino by Mark 
// Vandewettering K6HX, adapted for the Si5351A by Robert 
// Liesenfeld AK6L <ak6l@ak6l.org>.  Timer setup
// code by Thomas Knutsen LA3PNA.
//
// Time code adapted from the TimeSerial.ino example from the Time library.

// Hardware Requirements
// ---------------------
// This firmware should be able to run on most Arduino microcontrollers, since it solely relies
// on the millis() function for timing.
//
// Required Libraries
// ------------------
// Etherkit Si5351 (Library Manager)
// Etherkit JTEncode (Library Manager)
// Time (Library Manager)
// Wire (Arduino Standard Library)
//
// License
// -------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#include <si5351.h>
#include <JTEncode.h>
#include <int.h>
#include <TimeLib.h>
#include <TinyGPS.h>
#include <ClickEncoder.h>
#include <Adafruit_SSD1306.h>

constexpr uint8_t TONE_SPACING = 146;                 // ~1.46 Hz
constexpr uint16_t WSPR_INTERVAL = 683;
constexpr uint8_t SYMBOL_COUNT = WSPR_SYMBOL_COUNT;
constexpr uint32_t CORRECTION = 0;                    // Change this for your ref osc

#define GPS_PPS_PIN       2

#define TX_LED_PIN       25
#define SYNC_LED_PIN     13

#define ENC_A_PIN        19
#define ENC_B_PIN        18
#define ENC_BUTTON_PIN   20

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT    64

#define OLED_RESET       -1
#define SCREEN_ADDRESS 0x3D

// Global variables
Si5351 si5351;
JTEncode jtencode;
TinyGPS gps;
ClickEncoder encoder(ENC_A_PIN, ENC_B_PIN, ENC_BUTTON_PIN, 4);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint64_t freq = 10140200UL;                // Change this
char call[13] = "OK8RM";                   // Change this
char loc[7] = "JN79LT";                    // Change this
uint8_t dbm = 30;
uint8_t tx_buffer_1[SYMBOL_COUNT];
uint8_t tx_buffer_2[SYMBOL_COUNT];
uint32_t timer_expire = UINT32_MAX;

// Offset hours from gps time (UTC)
const int offset = 1;   // Central European Time

// Global variables used in ISRs
volatile bool proceed = false;
 
// Loop through the string, transmitting one character at a time.
void encode(uint8_t * buf)
{
  uint8_t i;

  // jtencode.wspr_encode(call, loc, dbm, tx_buffer_1);

  timer_expire = millis() + WSPR_INTERVAL;
    
  // Reset the tone to 0 and turn on the output
  si5351.set_clock_pwr(SI5351_CLK0, 1);
  digitalWrite(TX_LED_PIN, LOW);
    
  // Now do the rest of the message
  for (i = 0; i < SYMBOL_COUNT; i++)
  {
    si5351.set_freq((freq * 100) + (buf[i] * TONE_SPACING), SI5351_CLK0);
    while(millis() < timer_expire);
    timer_expire = millis() + WSPR_INTERVAL;
  }
        
  // Turn off the output
  si5351.set_clock_pwr(SI5351_CLK0, 0);
  digitalWrite(TX_LED_PIN, HIGH);
}

void processSyncMessage()
{
  // Process gps messages
  if (gps.encode(Serial1.read()))
  {
    // When TinyGPS reports new data...
    unsigned long age;
    int Year;
    byte Month, Day, Hour, Minute, Second;

    gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);

    if (age < 500)
    {
      // Set the Time to the latest GPS reading
      setTime(Hour, Minute, Second, Day, Month, Year);
      adjustTime(offset * SECS_PER_HOUR);
    }
  }
}

void ppsInterrupt()
{
  
}

void setup()
{
  // Use the Arduino's on-board LED as a keying indicator.
  pinMode(TX_LED_PIN, OUTPUT);
  pinMode(SYNC_LED_PIN, OUTPUT);
  
  digitalWrite(TX_LED_PIN, HIGH);
  digitalWrite(SYNC_LED_PIN, LOW);

  pinMode(GPS_PPS_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  Serial1.begin(4800);
  Serial.println("Waiting for GPS time ...");

  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), ppsInterrupt, RISING);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println("SSD1306 allocation failed!");
    for(;;);
  }

  // Clear the buffer
  display.clearDisplay();

  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_0PF, 0, CORRECTION);

  // Set CLK0 output
  si5351.set_freq(freq * 100, SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Disable the clock initially

  jtencode.wspr_encode(call, loc, dbm, tx_buffer_1);
  jtencode.wspr_encode("OK8RM/X", "JN79LT", 23, tx_buffer_2);

  // Start at top-left corner
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Hello, world!"));
}
 
void loop()
{
  if (Serial1.available())
  {
    processSyncMessage();
  }
  
  if (timeStatus() == timeSet)
  {
    digitalWrite(SYNC_LED_PIN, HIGH); // LED on if synced
  }
  else
  {
    digitalWrite(SYNC_LED_PIN, LOW);  // LED off if needs refresh
  }

  // Trigger every 10th minute
  // WSPR should start on the 1st second of the minute, but there's a slight delay
  // in this code because it is limited to 1 second resolution.
  // if(timeSet && timeStatus() == timeSet && minute() % 2 == 0 && second() == 0)
  if (timeSet && timeStatus() == timeSet && (minute() % 10 == 0 || minute() % 10 == 4) && second() == 0)
  {
    encode(tx_buffer_1);
    delay(1000);
  }

  if (timeSet && timeStatus() == timeSet && (minute() % 10 == 2 || minute() % 10 == 6) && second() == 0)
  {
    encode(tx_buffer_2);
    delay(1000);
  }
  
  //delay(100);
}
