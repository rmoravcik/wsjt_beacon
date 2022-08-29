// Based on Si5351_WSPR_2560.ino by Jason Milldrum NT7S.

#include <si5351.h>
#include <JTEncode.h>
#include <int.h>
#include <TimeLib.h>
#include <TinyGPS.h>
#include <ClickEncoder.h>
#include "ssd1306.h"
#include <EEPROM.h>

#include "config.h"

#define DEBUG_TRACES     1

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
  SCREEN_SET_MODE,
  SCREEN_SET_FREQUENCY,
  SCREEN_GPS_STATUS,
  SCREEN_CALIBRATION,
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
#define CAL_SIGNAL             6

#define ENC_A_PIN              3
#define ENC_B_PIN              2
#define ENC_BUTTON_PIN         4

// Global variables
Si5351 si5351;
JTEncode jtencode;
TinyGPS gps;
ClickEncoder encoder(ENC_A_PIN, ENC_B_PIN, ENC_BUTTON_PIN, 4);

uint8_t sel_freq = BAND_20M;
uint8_t cur_mode = MODE_WSPR;

static uint8_t tx_buffer[255];
uint8_t cur_screen = SCREEN_STATUS;

bool edit_mode = false;

#define CAL_FREQ 100000000UL
static int32_t cal_factor = 0;
static bool cal_factor_valid = false;
volatile bool cal_finished = true;
volatile uint32_t measured_freq = 0;
volatile uint32_t freq_counter = 0;

static void read_config(void)
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

static void write_config(void)
{
  EEPROM.write(EEPROM_MODE, cur_mode);
  EEPROM.write(EEPROM_FREQUENCY, sel_freq);
}

// Loop through the string, transmitting one character at a time.
static void encode(char *tx_buffer)
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

static void set_tx_buffer(char *tx_buffer)
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

static void process_sync_message()
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

static void pps_interrupt()
{
  if (cal_finished == true)
  {
    freq_counter = 0;
    cal_finished = false;
  }
  else
  {
    measured_freq = freq_counter;
    cal_finished = true;
  }
}

ISR(TCB0_INT_vect)
{
  freq_counter += TCB0.CCMP;
}

static void config_timer(void)
{
  EVSYS.CHANNEL4 = EVSYS_GENERATOR_PORT1_PIN4_gc;
  EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL4_gc;

  TCB0.CTRLA = 0;
  TCB0.CTRLB = TCB_CNTMODE_FRQ_gc;
  TCB0.EVCTRL = TCB_CAPTEI_bm | TCB_EDGE_bp;
  TCB0.INTCTRL = TCB_CAPT_bm;
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm | TCB_RUNSTDBY_bm;
}

static void start_calibration(void)
{
  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), pps_interrupt, RISING);

  si5351.output_enable(SI5351_CLK2, 1);

  do {
    delay(100);
  } while (!cal_finished);

  detachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN));

  cal_factor = (CAL_FREQ - (measured_freq * 100)) + cal_factor;
  cal_factor_valid = true;

  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);

  si5351.output_enable(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK2, 0);
}

static void display_header(const char *text)
{
  uint8_t x = (ssd1306_displayWidth() - (6 * strlen(text))) / 2;
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_printFixed(x, 0, text, STYLE_NORMAL);
}

static void display_mode(const char *text)
{
  ssd1306_setFixedFont(ssd1306xled_font8x16);
  ssd1306_printFixed(48, 24, text, STYLE_NORMAL);
}

static void display_frequency(const uint32_t value, const char *unit)
{
  uint8_t freq1 = value / 1000000;
  uint16_t freq2 = (value / 100) % 1000;
  char text[13];

  sprintf(text, "%3d.%04d %3s", freq1, freq2, unit);

  ssd1306_setFixedFont(ssd1306xled_font8x16);
  ssd1306_printFixed(16,  24, text, STYLE_NORMAL);
}

static void switch_screen(void)
{
  static int16_t prev_value = 0;

  if (!edit_mode)
  {
    int16_t cur_value = encoder.getValue();
  
    if (prev_value != cur_value)
    {
      if (cur_value > prev_value)
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
      prev_value = cur_value;
    }  
  }
}

static int8_t check_value_changed(void)
{
  static int16_t prev_value = 0;
  
  if (edit_mode == true)
  {
    if (encoder.getButton() == ClickEncoder::Clicked)
    {
      write_config();
      edit_mode = false;
    }

    int16_t cur_value = encoder.getValue();
    if (cur_value != prev_value)
    {
      if (cur_value > prev_value)
      {
        return 1;
      }
      else
      {
        return -1;
      }
      prev_value = cur_value;
    }
  }
  else
  {
    if (encoder.getButton() == ClickEncoder::Clicked)
    {
      edit_mode = true;
    }
  }  

  return 0;
}

static void show_status_screen(void)
{
  char buf[6];

//  display.drawTriangle(8, 0, 0, 8, 8, 8, SSD1306_WHITE);
//  display.fillTriangle(4, 4, 0, 8, 4, 8, SSD1306_WHITE);
//  display.drawRect(110, 0, 16, 8, SSD1306_WHITE);
//  display.drawRect(126, 2, 2, 4, SSD1306_WHITE);

  ssd1306_setFixedFont(ssd1306xled_font6x8);

  sprintf(buf, "%02d:%02d", hour(), minute());
  ssd1306_printFixed( 48,  0,  buf, STYLE_NORMAL);

  ssd1306_printFixed(  0, 56, call, STYLE_NORMAL);  
  ssd1306_printFixed(104, 56,  loc, STYLE_NORMAL);
}

static void show_set_mode_screen(void)
{
  int8_t value = check_value_changed();

  if (value != 0)
  {
    if (value > 0)
    {
      cur_mode++;
      if (cur_mode > MODE_COUNT)
      {
        cur_mode = 0;
      }
    }
    else
    {
      if (cur_mode == 0)
      {
        cur_mode = MODE_COUNT;
      }
      cur_mode--;
    }
  }

  ssd1306_clearScreen();
  display_header("Mode");
  display_mode(mode_params[cur_mode].mode_name);
}

static void show_set_frequency_screen(void)
{
  int8_t value = check_value_changed();

  if (value != 0)
  {
    if (value > 0)
    {
      sel_freq++;
      if (sel_freq > BAND_COUNT)
      {
        sel_freq = 0;
      }
    }
    else
    {
      if (sel_freq == 0)
      {
        sel_freq = BAND_COUNT;
      }
      sel_freq--;
    }
  }

  ssd1306_clearScreen();
  display_header("Frequency");

  display_frequency(mode_params[cur_mode].freqs[sel_freq], "MHz");
}

static void show_gps_status_screen(void)
{
  long lat, lon;
  unsigned long age;
  char buf[21];
  
  ssd1306_clearScreen();
  display_header("GPS Status");

  ssd1306_setFixedFont(ssd1306xled_font6x8);

  gps.get_position(&lat, &lon, &age);

  sprintf(buf, "Satellites: %2d", gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
  ssd1306_printFixed( 0, 16, buf, STYLE_NORMAL);

  sprintf(buf, "HDOP: %02d", gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  ssd1306_printFixed( 0, 16, buf, STYLE_NORMAL);

  int8_t lat1 = lat / 1000000;
  int8_t lat2 = (lat / 1000) % 1000;
  sprintf(buf, "Latitude  : %02d.%03d", lat1, lat2);
  ssd1306_printFixed( 0, 24, buf, STYLE_NORMAL);

  int8_t lon1 = lon / 1000000;
  int8_t lon2 = (lon / 1000) % 1000;
  sprintf(buf, "Longitude : %02d.%03d", lon1, lon2);
  ssd1306_printFixed( 0, 32, buf, STYLE_NORMAL);

  sprintf(buf, "Fix Age   : %4d", age);
  ssd1306_printFixed( 0, 40, buf, STYLE_NORMAL);

  int Year;
  byte Month, Day, Hour, Minute, Second;

  gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, NULL);
  sprintf(buf, "GPS Time  : %02u.%02u.%04u %02u:%02u:%02u", Day, Month, Year, Hour, Minute, Second);
  ssd1306_printFixed( 0, 48, buf, STYLE_NORMAL);
}

static void show_calibration_screen(void)
{
  check_value_changed();
  
  ssd1306_clearScreen();
  display_header("Calibration");

  ssd1306_setFixedFont(ssd1306xled_font8x16);

  if (edit_mode == true)
  {
    ssd1306_printFixed(36,  24, "Disabled", STYLE_NORMAL);  
 
  }
  else
  {
    ssd1306_printFixed(36,  24, "Enabled", STYLE_NORMAL);
    start_calibration();  
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  DEBUGLN("Reading EEPROM configuration...");
  read_config();

  DEBUGLN("NEO-6M setup...");
  Serial1.begin(9600);

  DEBUGLN("NEO-6M 1PPS IRQ setup...");
  pinMode(GPS_PPS_PIN, INPUT_PULLUP);

  DEBUGLN("SSD1306 setup...");
  ssd1306_128x64_i2c_init();

  // Clear the buffer
  ssd1306_clearScreen();

  DEBUGLN("Si5351 setup...");
  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_0PF, 0, cal_factor);

  // Set CLK0 output
  si5351.set_freq(mode_params[cur_mode].freqs[sel_freq] * 100, SI5351_CLK0);
  // Set for max power
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  // Disable the clock initially
  si5351.set_clock_pwr(SI5351_CLK0, 0);

  // Set CLK2 output
  si5351.set_freq(CAL_FREQ, SI5351_CLK2);
  // Disable the clock initially
  si5351.set_clock_pwr(SI5351_CLK2, 0);

  DEBUGLN("Setting TX buffer...");
  set_tx_buffer(tx_buffer);

  DEBUGLN("Configure TimerB...");
  config_timer();

  DEBUGLN("Done...");
}
 
void loop()
{

  if (Serial1.available())
  {
    process_sync_message();
  }
  
  if (timeStatus() == timeSet)
  {
    if (cal_factor_valid == false)
    {
      start_calibration();
    }
  }

  // Trigger every 10th minute
  // WSPR should start on the 1st second of the minute, but there's a slight delay
  // in this code because it is limited to 1 second resolution.
  // if(timeSet && timeStatus() == timeSet && minute() % 2 == 0 && second() == 0)
  if (timeSet && timeStatus() == timeSet && (minute() % 10 == 0 || minute() % 10 == 4) && second() == 0)
  {
    encode(tx_buffer);
    delay(1000);
  }

  switch_screen();

  switch (cur_screen)
  {
    case SCREEN_SET_MODE:
      show_set_mode_screen();
      break;
    case SCREEN_SET_FREQUENCY:
      show_set_frequency_screen();
      break;
    case SCREEN_GPS_STATUS:
      show_gps_status_screen();
      break;
    case SCREEN_CALIBRATION:
      show_calibration_screen();
      break;
    default:
      show_status_screen();
      break;
  }

  DEBUG("freq_counter:");
  DEBUGLN(freq_counter);
  delay(1000);
}
