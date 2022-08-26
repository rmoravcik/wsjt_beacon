// Based on Si5351_WSPR_2560.ino by Jason Milldrum NT7S.

#include <si5351.h>
#include <JTEncode.h>
#include <int.h>
#include <TimeLib.h>
#include <TinyGPS.h>
#include <ClickEncoder.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#include "config.h"

#define DEBUG_TRACES     0

#if DEBUG_TRACES
#define DEBUG(x)    Serial.print(x)
#define DEBUGLN(x)  Serial.println(x)
#else
#define DEBUG(x)
#define DEBUGLN(x)
#endif

#define EEPROM_MODE      0
#define EEPROM_FREQUENCY 1

enum screen {
  SCREEN_STATUS = 0,
  SCREEN_SET_FREQUENCY,
  SCREEN_GPS_STATUS,
  SCREEN_COUNT
};

enum mode {
  MODE_JT9 = 0,
  MODE_JT65,
/*  
  MODE_JT4,
*/
  MODE_WSPR,
/*
  MODE_FSQ_2,
  MODE_FSQ_3,
  MODE_FSQ_4_5,
  MODE_FSQ_6,
*/
  MODE_FT8,
  MODE_COUNT
};

enum band {
  BAND_160M = 0,
  BAND_80M,
  BAND_40M,
  BAND_30M,
  BAND_20M,
  BAND_17M,
  BAND_15M,
  BAND_12M,
  BAND_10M,
  BAND_6M,
  BAND_2M,
  BAND_COUNT
};

struct mode_param {
  const char mode_name[5];
  uint16_t symbol_count;
  uint16_t tone_spacing;
  uint16_t tone_delay;
  uint32_t *freqs;
};

const uint32_t jt9_freqs[BAND_COUNT] = {
    1836600UL,
    3568600UL,
    7038600UL,
   10138700UL,
   14095600UL,
   18104600UL,
   21094600UL,
   24924600UL,
   28124600UL,
   50293000UL,
  144489000UL
};

const uint32_t jt65_freqs[BAND_COUNT] = {
    1836600UL,
    3568600UL,
    7038600UL,
   10138700UL,
   14095600UL,
   18104600UL,
   21094600UL,
   24924600UL,
   28124600UL,
   50293000UL,
  144489000UL
};

const uint32_t wspr_freqs[BAND_COUNT] = {
    1836600UL,
    3568600UL,
    7038600UL,
   10138700UL,
   14095600UL,
   18104600UL,
   21094600UL,
   24924600UL,
   28124600UL,
   50293000UL,
  144489000UL
};

const uint32_t ft8_freqs[BAND_COUNT] = {
    1836600UL,
    3568600UL,
    7038600UL,
   10138700UL,
   14095600UL,
   18104600UL,
   21094600UL,
   24924600UL,
   28124600UL,
   50293000UL,
  144489000UL
};

const struct mode_param mode_params[MODE_COUNT] {
 { "JT9",  JT9_SYMBOL_COUNT,  174, 576, jt9_freqs  },
 { "JT65", JT65_SYMBOL_COUNT, 269, 371, jt65_freqs },
/*
 { "JT4",  JT4_SYMBOL_COUNT,  437, 229, NULL       },
*/
 { "WSPR", WSPR_SYMBOL_COUNT, 146, 683, wspr_freqs },
/*
 { "FSQ2", 0,                 879, 500, NULL       },
 { "FSQ3", 0,                 879, 333, NULL       },
 { "FSQ4", 0,                 879, 222, NULL       },
 { "FSQ5", 0,                 879, 167, NULL       },
*/
 { "FT8",  FT8_SYMBOL_COUNT,  628, 159, ft8_freqs  }
};

#define GPS_PPS_PIN           10
#define SYNC_LED_PIN          12

#define ENC_A_PIN              3
#define ENC_B_PIN              2
#define ENC_BUTTON_PIN         4

#define SCREEN_WIDTH         128
#define SCREEN_HEIGHT         64

#define SSD1306_I2C_ADDRESS 0x3C

// Global variables
Si5351 si5351;
JTEncode jtencode;
TinyGPS gps;
ClickEncoder encoder(ENC_A_PIN, ENC_B_PIN, ENC_BUTTON_PIN, 4);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

uint8_t sel_freq = BAND_20M;
uint8_t cur_mode = MODE_WSPR;
uint8_t cur_screen = SCREEN_STATUS;
bool edit_mode = false;
constexpr uint32_t CORRECTION    = 0;                   // Change this for your ref osc

uint8_t tx_buffer[255];
int16_t encoder_prev_value = 0;

// Loop through the string, transmitting one character at a time.
static void encode(void)
{
  uint8_t i;

  // Reset the tone to the base frequency and turn on the output
  si5351.output_enable(SI5351_CLK0, 1);
    
/*
  // Now transmit the channel symbols
  if ((cur_mode == MODE_FSQ_2) || (cur_mode == MODE_FSQ_3) || (cur_mode == MODE_FSQ_4_5) || (cur_mode == MODE_FSQ_6))
  {
    uint8_t j = 0;

    while(tx_buffer[j++] != 0xff);

    mode_params[cur_mode].symbol_count = j - 1;
  }
*/

  for (i = 0; i < mode_params[cur_mode].symbol_count; i++)
  {
      si5351.set_freq((mode_params[cur_mode].freqs[sel_freq] * 100) + (tx_buffer[i] * mode_params[cur_mode].tone_spacing), SI5351_CLK0);
      delay(mode_params[cur_mode].tone_delay);
  }

  // Turn off the output
  si5351.output_enable(SI5351_CLK0, 0);
}

static void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  switch(cur_mode)
  {
  case MODE_JT9:
    jtencode.jt9_encode(message, tx_buffer);
    break;
  case MODE_JT65:
    jtencode.jt65_encode(message, tx_buffer);
    break;
/*
  case MODE_JT4:
    jtencode.jt4_encode(message, tx_buffer);
    break;
*/
  case MODE_WSPR:
    jtencode.wspr_encode(call, loc, dbm, tx_buffer);
    break;
  case MODE_FT8:
    jtencode.ft8_encode(message, tx_buffer);
    break;
/*
  case MODE_FSQ_2:
  case MODE_FSQ_3:
  case MODE_FSQ_4_5:
  case MODE_FSQ_6:
    jtencode.fsq_dir_encode(call, "n0call", ' ', "hello world", tx_buffer);
    break;
*/
  }
}

static void processSyncMessage()
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

static void display_test(void)
{
  display.drawTriangle(8, 0, 0, 8, 8, 8, SSD1306_WHITE);
  display.fillTriangle(4, 4, 0, 8, 4, 8, SSD1306_WHITE);
  display.drawRect(110, 0, 16, 8, SSD1306_WHITE);
  display.drawRect(126, 2, 2, 4, SSD1306_WHITE);

  // Start at top-left corner
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(48, 0);
  display.println("20:59");
  display.setCursor(0, 24);
  display.setTextSize(2);
  display.println("  14.0936");
  display.setCursor(0, 56);
  display.setTextSize(1);
  display.println("OK8RM");
  display.setCursor(90, 56);
  display.setTextSize(1);
  display.println("JN79LT");
  display.display();
}

static void display_header(const char *text)
{
  display.setCursor(24, 0);
  display.setTextSize(2);
  display.println(text);
  display.display();
}

static void switch_screen(void)
{
  if (!edit_mode)
  {
    int16_t encoder_cur_value = encoder.getValue();
  
    if (encoder_prev_value != encoder_cur_value)
    {
      if (encoder_cur_value > encoder_prev_value)
      {
        cur_screen++;
        if (cur_screen == SCREEN_COUNT)
        {
          cur_screen = SCREEN_STATUS;
        }
      }
      else
      {
        if (cur_screen == SCREEN_STATUS)
        {
          cur_screen = SCREEN_COUNT;
        }
        cur_screen--;
      }
      encoder_prev_value = encoder_cur_value;
    }  
  }
}

static void show_status_screen(void)
{
  display_test();
}

static void show_set_frequency_screen(void)
{
  display.clearDisplay();
  display_header("Frequency");
}

static void show_gps_status_screen(void)
{
  display.clearDisplay();
  display_header("GPS Status");

  DEBUG("Satellites: ");
  DEBUGLN(gps.satellites());
}

void pps_interrupt()
{
  
}

static void read_configuration(void)
{
  cur_mode = EEPROM.read(EEPROM_MODE);
  if (cur_mode > MODE_COUNT)
  {
    cur_mode = MODE_WSPR;
  }

  sel_freq = EEPROM.read(EEPROM_FREQUENCY);
  if (sel_freq > BAND_COUNT)
  {
    sel_freq = BAND_20M;
  } 
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  DEBUGLN("Reading EEPROM configuration...");
  read_configuration();

  DEBUGLN("NEO-6M setup...");
  Serial1.begin(9600);

  DEBUGLN("NEO-6M 1PPS IRQ setup...");
  pinMode(GPS_PPS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), pps_interrupt, RISING);

  DEBUGLN("SSD1306 setup...");
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS))
  {
    DEBUGLN("ERROR: SSD1306 allocation failed!");
    for(;;);
  }

  // Clear the buffer
  display.clearDisplay();
  display.display();

  DEBUGLN("Si5351 setup...");
  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_0PF, 0, CORRECTION);

  // Set CLK0 output
  si5351.set_freq(wspr_freqs[sel_freq] * 100, SI5351_CLK0);
  // Set for max power
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  // Disable the clock initially
  si5351.set_clock_pwr(SI5351_CLK0, 0);

  set_tx_buffer();

  DEBUGLN("Done...");
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
    encode();
    delay(1000);
  }

  switch_screen();

  switch (cur_screen)
  {
    case SCREEN_SET_FREQUENCY:
      show_set_frequency_screen();
      break;
    case SCREEN_GPS_STATUS:
      show_gps_status_screen();
      break;
    default:
      show_status_screen();
      break;
  }
}
