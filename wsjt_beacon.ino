// Based on Si5351_WSPR_2560.ino by Jason Milldrum NT7S.

#include <si5351.h>
#include <JTEncode.h>
#include <int.h>
#include <TimeLib.h>
#include <TinyGPS.h>
#include <Encoder.h>
#include "ssd1306.h"
#include <EEPROM.h>
#include <Button.h>

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

#define VERSION_STRING   "v0.9.0"

enum screen {
  SCREEN_STATUS = 0,
  SCREEN_SET_MODE,
  SCREEN_SET_FREQUENCY,
  SCREEN_GPS_STATUS,
  SCREEN_CALIBRATION,
  SCREEN_VERSION,
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
  const uint32_t *freqs;
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
 { "JT9 ", JT9_SYMBOL_COUNT,  174, 576, jt9_freqs  },
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
 { "FT8 ", FT8_SYMBOL_COUNT,  628, 159, ft8_freqs  }
};

#define GPS_PPS_PIN           10
#define CAL_SIGNAL_PIN         6

#define ENC_A_PIN              2
#define ENC_B_PIN              3
#define ENC_BUTTON_PIN         4

// Global variables
Si5351 si5351;
JTEncode jtencode;
TinyGPS gps;
Encoder encoder(ENC_A_PIN, ENC_B_PIN);
Button encoder_button(ENC_BUTTON_PIN);

uint8_t sel_freq = BAND_20M;
uint8_t cur_mode = MODE_WSPR;

static uint8_t tx_buffer[255];
uint8_t cur_screen = SCREEN_COUNT;
bool refresh_screen = false;

bool edit_mode = false;
bool edit_mode_blink_toggle = false;

#define CAL_FREQ 250000000UL
#define CAL_TIME_SECONDS 20UL
volatile uint32_t timer0_ovf_counter = 0;
volatile uint16_t timer0_count = 0;
volatile uint8_t cal_timeout = CAL_TIME_SECONDS;
static int32_t cal_factor = 0;
static bool cal_factor_valid = false;

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
static void encode(uint8_t *tx_buffer)
{
  uint8_t i;

  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_negativeMode();
  ssd1306_printFixed(52, 8, " TX ", STYLE_NORMAL);

  // Reset the tone to the base frequency and turn on the output
  si5351.set_clock_pwr(SI5351_CLK0, 1);
    
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
  si5351.set_clock_pwr(SI5351_CLK0, 0);

  ssd1306_positiveMode();
  ssd1306_printFixed(52, 8, "    ", STYLE_NORMAL);
}

static void set_tx_buffer(uint8_t *tx_buffer)
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
  if (cal_timeout == 0)
  {
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.CTRLESET = TCA_SPLIT_CMD_RESET_gc | 0x03;  // Reset TCA0
    TCA0.SINGLE.EVCTRL = TCA_SINGLE_EVACT_POSEDGE_gc | TCA_SINGLE_CNTEI_bm;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;
    timer0_ovf_counter = 0;
  }
  else if (cal_timeout == CAL_TIME_SECONDS)
  {
    timer0_count = TCA0.SINGLE.CNT;
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.CTRLESET = TCA_SPLIT_CMD_RESET_gc | 0x03;  // Reset TCA0
    TCA0.SINGLE.EVCTRL = 0;
    TCA0.SINGLE.INTCTRL = 0;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
  }
  cal_timeout++;
}

ISR(TCA0_OVF_vect)
{
  timer0_ovf_counter++;
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

static void init_evsys(void)
{
  EVSYS.CHANNEL4 = EVSYS_GENERATOR_PORT1_PIN4_gc;
  EVSYS.USERTCA0 = EVSYS_CHANNEL_CHANNEL4_gc;

}

static void do_calibration(void)
{
  cal_timeout = 0;

  DEBUG("Calibration started");

// REMOVE ME
#if 1
  TCA0.SINGLE.CTRLA = 0;
  TCA0.SINGLE.CTRLESET = TCA_SPLIT_CMD_RESET_gc | 0x03;  // Reset TCA0
  TCA0.SINGLE.EVCTRL = TCA_SINGLE_EVACT_POSEDGE_gc | TCA_SINGLE_CNTEI_bm;
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;
#endif

  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), pps_interrupt, RISING);

  si5351.set_clock_pwr(SI5351_CLK2, 1);

  do {
    delay(1000);
    DEBUG(".");
    DEBUGLN(timer0_ovf_counter);
  } while (cal_timeout < CAL_TIME_SECONDS);
  DEBUGLN("");

  detachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN));

  cal_factor = (CAL_FREQ - (((timer0_ovf_counter * 0x10000) + timer0_count) / CAL_TIME_SECONDS)) + cal_factor;
  if ((cal_factor < 50000) && (cal_factor > -50000))
  {
    DEBUG("Calibration finished. factor=");
    DEBUGLN(cal_factor);
    cal_factor_valid = true;
  }
  else
  {
    DEBUG("Calibration factor=");
    DEBUG(cal_factor);
    DEBUGLN(" invalid!");
    cal_factor = 0;
  }

  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);

  si5351.set_clock_pwr(SI5351_CLK2, 0);
}

static void display_header(const char *text)
{
  uint8_t x = (ssd1306_displayWidth() - (6 * strlen(text))) / 2;
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_printFixed(x, 0, text, STYLE_BOLD);
}

static void display_mode(const char *text)
{
  if (edit_mode == true)
  {
    if (edit_mode_blink_toggle == true)
    {
       ssd1306_negativeMode();
       edit_mode_blink_toggle = false;
    }
    else
    {
       ssd1306_positiveMode();
       edit_mode_blink_toggle = true;
    }
  }

  ssd1306_setFixedFont(ssd1306xled_font8x16);
  ssd1306_printFixed(48, 24, text, STYLE_NORMAL);
}

static void display_frequency(const uint32_t value, const char *unit)
{
  uint16_t freq1 = value / 1000000;
  uint16_t freq2 = (value / 100) % 1000;
  char text[13];

  if (edit_mode == true)
  {
    if (edit_mode_blink_toggle == true)
    {
       ssd1306_negativeMode();
       edit_mode_blink_toggle = false;
    }
    else
    {
       ssd1306_positiveMode();
       edit_mode_blink_toggle = true;
    }
  }

  sprintf(text, "%3d.%04d %3s", freq1, freq2, unit);

  ssd1306_setFixedFont(ssd1306xled_font8x16);
  ssd1306_printFixed(16,  24, text, STYLE_NORMAL);
}

static uint8_t get_next_screen(void)
{
  static int32_t old_position = 0;
  uint8_t next_screen = cur_screen;

  if (next_screen == SCREEN_COUNT)
  {
    return SCREEN_STATUS;
  }

  if (edit_mode == false)
  {
    int32_t new_position = encoder.read();
  
    if (old_position != new_position)
    {
      if (new_position >= (old_position + 4))
      {
        next_screen++;
        if (next_screen == SCREEN_COUNT)
        {
          next_screen = SCREEN_STATUS;
        }
        old_position = new_position;
        refresh_screen = false;
      }
      else if (new_position <= (old_position - 4))
      {
        if (next_screen == SCREEN_STATUS)
        {
          next_screen = SCREEN_COUNT;
        }
        next_screen--;
        old_position = new_position;
        refresh_screen = false;
      }
    }  
  }
  else
  {
    old_position = encoder.read();
  }

  return next_screen;
}

static int8_t get_new_value(void)
{
  static int32_t old_position = 0;
  
  if (edit_mode == true)
  {
    if (encoder_button.pressed())
    {
      DEBUGLN("Leaving edit mode");
      write_config();
      edit_mode = false;
      edit_mode_blink_toggle = false;
    }

    int32_t new_position = encoder.read();
    if (new_position != old_position)
    {
      if (new_position >= (old_position + 4))
      {
        old_position = new_position;
        return 1;
      }
      else if (new_position <= (old_position - 4))
      {
        old_position = new_position;
        return -1;
      }
    }
  }
  else
  {
    if (encoder_button.pressed())
    {
      DEBUGLN("Entering edit mode");
      old_position = encoder.read();
      edit_mode = true;
    }
  }  

  return 0;
}

static void draw_gps_symbol(bool fix)
{
  uint8_t gps_no_signal[8] = { 0x3F, 0x62, 0xC4, 0x88, 0x94, 0xA4, 0xC1, 0x81 };
  uint8_t gps_fix[8] = { 0x3F, 0x62, 0xC4, 0x88, 0x94, 0xAD, 0xC1, 0x87 };

  if (fix == true)
  {
    ssd1306_drawBuffer(0, 0, 8, 8, gps_fix);
  }
  else
  {
    ssd1306_drawBuffer(0, 0, 8, 8, gps_no_signal);
  }
}

static void draw_battery(const uint8_t capacity)
{
  ssd1306_drawRect(110, 0, 125, 8);
  ssd1306_drawRect(126, 2, 127, 6);

  if (capacity > 75)
  {
    ssd1306_fillRect(111, 0, 124, 8);
  }
  else if (capacity > 50)
  {
    ssd1306_fillRect(111, 0, 121, 8);
  }
  else if (capacity > 25)
  {
    ssd1306_fillRect(111, 0, 117, 8);

  }
  else if (capacity > 0)
  {
    ssd1306_fillRect(111, 0, 113, 8);    
  }
}

static void show_status_screen(void)
{
  char buf[6];

  if (refresh_screen == false)
  {
    ssd1306_clearScreen();
    ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_printFixed(  0, 56, call, STYLE_NORMAL);  
    ssd1306_printFixed( 52, 56, mode_params[cur_mode].mode_name, STYLE_NORMAL);
    ssd1306_printFixed(104, 56, loc, STYLE_NORMAL);
    display_frequency(mode_params[cur_mode].freqs[sel_freq], "MHz");
  }

  ssd1306_setFixedFont(ssd1306xled_font6x8);

  sprintf(buf, "%02d:%02d", hour(), minute());
  ssd1306_printFixed( 48,  0,  buf, STYLE_NORMAL);

  draw_gps_symbol(true);
  draw_battery(50);
}

static void show_set_mode_screen(void)
{
  int8_t value = get_new_value();

  if (value != 0)
  {
    if (value > 0)
    {
      cur_mode++;
      if (cur_mode == MODE_COUNT)
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
    refresh_screen = true;
  }

  if (refresh_screen == false)
  {
    ssd1306_clearScreen();
    display_header("Mode");
  }

  display_mode(mode_params[cur_mode].mode_name);
}

static void show_set_frequency_screen(void)
{
  int8_t value = get_new_value();

  if (value != 0)
  {
    if (value > 0)
    {
      sel_freq++;
      if (sel_freq == BAND_COUNT)
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
    refresh_screen = true;
  }

  if (refresh_screen == false)
  {
    ssd1306_clearScreen();
    display_header("Frequency");
  }

  display_frequency(mode_params[cur_mode].freqs[sel_freq], "MHz");
}

static void show_gps_status_screen(void)
{
  int Year;
  byte Month, Day, Hour, Minute, Second;
  long lat, lon;
  unsigned long age;
  char buf[21];

  if (refresh_screen == false)
  {
    ssd1306_clearScreen();
    display_header("GPS Status");
  }

  ssd1306_setFixedFont(ssd1306xled_font6x8);

  gps.get_position(&lat, &lon, &age);

  sprintf(buf, "Sat.: %4d", gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
  ssd1306_printFixed( 0, 16, buf, STYLE_NORMAL);

  int16_t lat1 = lat / 1000000;
  int16_t lat2 = (lat / 1000) % 1000;
  sprintf(buf, "Lat.: %02d.%03d", lat1, lat2);
  ssd1306_printFixed( 0, 24, buf, STYLE_NORMAL);

  int16_t lon1 = lon / 1000000;
  int16_t lon2 = (lon / 1000) % 1000;
  sprintf(buf, "Lon.: %02d.%03d", lon1, lon2);
  ssd1306_printFixed( 0, 32, buf, STYLE_NORMAL);

  sprintf(buf, "Age : %4ld", age);
  ssd1306_printFixed( 0, 40, buf, STYLE_NORMAL);

  gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, NULL);
  sprintf(buf, "Time: %02u:%02u:%02u", Hour, Minute, Second);
  ssd1306_printFixed( 0, 48, buf, STYLE_NORMAL);
  sprintf(buf, "Date: %02u.%02u.%04u", Day, Month, Year);
  ssd1306_printFixed( 0, 56, buf, STYLE_NORMAL);
}

static void show_calibration_screen(void)
{
  static bool waiting_to_disable = false;
  
  get_new_value();
  
  if (refresh_screen == false)
  {
    ssd1306_clearScreen();
    display_header("Calibration");
  }

  ssd1306_setFixedFont(ssd1306xled_font8x16);

  if (edit_mode == false)
  {
    char buf[15];
    sprintf(buf, "Factor:%7ld", cal_factor);
    ssd1306_printFixed(8,  24, buf, STYLE_NORMAL);  
    si5351.set_clock_pwr(SI5351_CLK0, 0);
  }
  else
  {
    if (waiting_to_disable == false)
    {
      ssd1306_printFixed(8,  24, "Calibrating...", STYLE_NORMAL);
      do_calibration();  
      si5351.set_clock_pwr(SI5351_CLK0, 1);
      waiting_to_disable = true;
    }
    ssd1306_printFixed(8,  24, "  Disable TX  ", STYLE_NORMAL);
  }
}

static void show_version_screen(void)
{
  if (refresh_screen == false)
  {
    ssd1306_clearScreen();
    display_header("Version");
    ssd1306_setFixedFont(ssd1306xled_font8x16);
    ssd1306_printFixed(40,  24, VERSION_STRING, STYLE_NORMAL);
  }
}

static void show_screen(void)
{
  uint8_t next_screen = get_next_screen();

  if ((next_screen != cur_screen) || (refresh_screen == true))
  {
    switch (next_screen)
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
      case SCREEN_VERSION:
        show_version_screen();
        break;
      default:
        show_status_screen();
        break;
    }  

    cur_screen = next_screen;
    refresh_screen = false;
  } 
}

void setup()
{
  Serial.begin(115200);

  DEBUGLN("Reading EEPROM configuration...");
  read_config();

  DEBUGLN("NEO-6M setup...");
  Serial1.begin(9600);

  DEBUGLN("NEO-6M 1PPS IRQ setup...");
  pinMode(GPS_PPS_PIN, INPUT_PULLUP);

  DEBUGLN("SSD1306 setup...");
  ssd1306_128x64_i2c_init();

  // Clear the buffer
  DEBUGLN("Clear screen...");
  ssd1306_clearScreen();

  DEBUGLN("Si5351 setup...");
  pinMode(CAL_SIGNAL_PIN, INPUT);
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
  si5351.set_ms_source(SI5351_CLK2, SI5351_PLLB);
  si5351.set_freq(CAL_FREQ, SI5351_CLK2);
  // Set for max power
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
  // Disable the clock initially
  si5351.set_clock_pwr(SI5351_CLK2, 0);

  DEBUGLN("Setting TX buffer...");
  set_tx_buffer(tx_buffer);

  DEBUGLN("Encoder setup...");
  encoder_button.begin();

  DEBUGLN("Initialize EVSYS...");
  init_evsys();

  DEBUGLN("Done...");
}
 
void loop()
{
  static uint32_t last_update = 0;

  if (Serial1.available())
  {
    process_sync_message();
  }
  
  if (timeStatus() == timeSet)
  {
    if (cal_factor_valid == false)
    {
      do_calibration();
    }
  }

  // Trigger every 10th minute
  // WSPR should start on the 1st second of the minute, but there's a slight delay
  // in this code because it is limited to 1 second resolution.
  // if(timeSet && timeStatus() == timeSet && minute() % 2 == 0 && second() == 0)
  if (timeStatus() == timeSet && (minute() % 10 == 0 || minute() % 10 == 4) && second() == 0)
  {
    encode(tx_buffer);
    delay(1000);
  }

  show_screen();

  if ((millis() - last_update) > 1000)
  {
    refresh_screen = true;
    last_update = millis();
  }

  delay(20);
}
