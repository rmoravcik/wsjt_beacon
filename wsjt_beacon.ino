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
#include "priv_types.h"

#define DEBUG_TRACES     1

#if DEBUG_TRACES
#define DEBUG(x)    Serial.print(x)
#define DEBUGLN(x)  Serial.println(x)
#else
#define DEBUG(x)
#define DEBUGLN(x)
#endif

#define EEPROM_MODE       0
#define EEPROM_FREQUENCY  1
#define EEPROM_CAL_FACTOR 2

#define VERSION_STRING   "v1.0.10"

const uint8_t gps_icon[8] = { 0x3F, 0x62, 0xC4, 0x88, 0x94, 0xAD, 0xC1, 0x87 };
const uint8_t battery_icon[17] = { 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
                                   0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0x3C,
                                   0x3C };
const uint8_t battery_bar[2] = { 0xBD, 0xBD };

const struct mode_param mode_params[MODE_COUNT] {
 { "JT9 ", JT9_SYMBOL_COUNT,  174, 576, 0, jt9_freqs  },
 { "JT65", JT65_SYMBOL_COUNT, 269, 371, 0, jt65_freqs },
/*
 { "JT4",  JT4_SYMBOL_COUNT,  437, 229, 0, NULL       },
*/
 { "WSPR", WSPR_SYMBOL_COUNT, 146, 683, 1, wspr_freqs },
/*
 { "FSQ2", 0,                 879, 500, 0, NULL       },
 { "FSQ3", 0,                 879, 333, 0, NULL       },
 { "FSQ4", 0,                 879, 222, 0, NULL       },
 { "FSQ5", 0,                 879, 167, 0, NULL       },
*/
 { "FT8 ", FT8_SYMBOL_COUNT,  628, 159, 0, ft8_freqs  }
};

#define GPS_PPS_PIN           10
#define CAL_SIGNAL_PIN         6

#define ENC_A_PIN              2
#define ENC_B_PIN              3
#define ENC_BUTTON_PIN         4

#define BATTERY_PIN           14

// Global variables
Si5351 si5351;
JTEncode jtencode;
TinyGPS gps;
Encoder encoder(ENC_A_PIN, ENC_B_PIN);
Button encoder_button(ENC_BUTTON_PIN);

char loc[5] = "AA00";

uint8_t sel_freq = BAND_20M;
uint8_t cur_mode = MODE_WSPR;

#define DISPLAY_REFRESH_TIME (1000)
static uint8_t tx_buffer[255];
uint8_t cur_screen = SCREEN_COUNT;
bool refresh_screen = false;

bool edit_mode = false;
bool blink_toggle = false;
bool tx_active = false;
volatile int8_t tx_timeout = 0;

#define CAL_FREQ         (2500000ULL)
#define CAL_TIME_SECONDS (40UL)
volatile uint32_t timer0_ovf_counter = 0;
volatile uint16_t timer0_count = 0;
volatile uint8_t cal_timeout = CAL_TIME_SECONDS;
static int32_t cal_factor = 0;
static bool cal_factor_valid = false;

typedef void (*cal_refresh_cb)(void);

// Overtaken from https://github.com/W3PM/GPS-Display-and-Time-Grid-Square-Synchronization-Source/blob/master/GPS_display_source_v2_4a.ino
void calc_grid_square(float lat, float lon)
{
  int o1, o2; //, o3;
  int a1, a2; //, a3;
  double remainder;

  // longitude
  remainder = lon + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (double)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
//  remainder = remainder - 2.0 * (double)o2;
//  o3 = (int)(12.0 * remainder);

  // latitude
  remainder = lat + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (double)a1 * 10.0;
  a2 = (int)(remainder);
//  remainder = remainder - (double)a2;
//  a3 = (int)(24.0 * remainder);

  loc[0] = (char)o1 + 'A';
  loc[1] = (char)a1 + 'A';
  loc[2] = (char)o2 + '0';
  loc[3] = (char)a2 + '0';
  loc[4] = (char)0;
//  loc[4] = (char)o3 + 'A';
//  loc[5] = (char)a3 + 'A';
//  loc[6] = (char)0;
}

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

  cal_factor  = (uint32_t)EEPROM.read(EEPROM_CAL_FACTOR)     << 24;
  cal_factor |= (uint32_t)EEPROM.read(EEPROM_CAL_FACTOR + 1) << 16;
  cal_factor |= (uint32_t)EEPROM.read(EEPROM_CAL_FACTOR + 2) <<  8;
  cal_factor |= (uint32_t)EEPROM.read(EEPROM_CAL_FACTOR + 3);
  if (cal_factor == -1)
  {
    cal_factor = 0;
  }
}

static void write_config(void)
{
  EEPROM.write(EEPROM_MODE, cur_mode);
  EEPROM.write(EEPROM_FREQUENCY, sel_freq);
  EEPROM.write(EEPROM_CAL_FACTOR,     (cal_factor >> 24) & 0xFF);
  EEPROM.write(EEPROM_CAL_FACTOR + 1, (cal_factor >> 16) & 0xFF);
  EEPROM.write(EEPROM_CAL_FACTOR + 2, (cal_factor >>  8) & 0xFF);
  EEPROM.write(EEPROM_CAL_FACTOR + 3, (cal_factor)       & 0xFF);
}

ISR(TCA0_CMP0_vect)
{
  tx_timeout--;
  TCA0.SINGLE.CNT = 0;
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0EN_bm;
}

// Loop through the string, transmitting one character at a time.
static void encode(uint8_t *tx_buffer, cal_refresh_cb cb)
{
  uint8_t i;

  // Reset the tone to the base frequency and turn on the output
  si5351.set_clock_pwr(SI5351_CLK0, 1);
  si5351.set_freq(mode_params[cur_mode].freqs[sel_freq] * SI5351_FREQ_MULT, SI5351_CLK0);
  tx_active = true;
    
  DEBUGLN("Transmitting TX buffer...");

  display_frequency(mode_params[cur_mode].freqs[sel_freq]);

/*
  // Now transmit the channel symbols
  if ((cur_mode == MODE_FSQ_2) || (cur_mode == MODE_FSQ_3) || (cur_mode == MODE_FSQ_4_5) || (cur_mode == MODE_FSQ_6))
  {
    uint8_t j = 0;

    while(tx_buffer[j++] != 0xff);

    mode_params[cur_mode].symbol_count = j - 1;
  }
*/

  TCA0.SINGLE.CMP0 = mode_params[cur_mode].tone_delay * 25;

  for (i = 0; i < mode_params[cur_mode].symbol_count; i++)
  {
    si5351.set_freq((mode_params[cur_mode].freqs[sel_freq] * SI5351_FREQ_MULT) + (tx_buffer[i] * mode_params[cur_mode].tone_spacing), SI5351_CLK0);

    // Wait for timer expire
    TCA0.SINGLE.CNT = 0;
    tx_timeout = 10;

    if (cb != NULL)
    {
      cb();
    }
    while (tx_timeout > 0) {};
  }

  // Turn off the output
  si5351.set_clock_pwr(SI5351_CLK0, 0);
  tx_active = false;

  DEBUGLN("Transmition finished...");
}

static void set_tx_buffer(uint8_t *tx_buffer)
{
  char message[14];

  DEBUGLN("Setting TX buffer...");

  sprintf(message, "%s %s", call, loc);

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

static void init_tca0(const bool cal_mode)
{
  if (cal_mode == true)
  {
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.CTRLESET = TCA_SPLIT_CMD_RESET_gc | 0x03;  // Reset TCA0
    TCA0.SINGLE.EVCTRL = TCA_SINGLE_EVACT_POSEDGE_gc | TCA_SINGLE_CNTEI_bm;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;
  }
  else
  {
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.CTRLESET = TCA_SPLIT_CMD_RESET_gc | 0x03;  // Reset TCA0
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    TCA0.SINGLE.PER = 0xFFFF;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0EN_bm;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
  }
}

static void pps_interrupt()
{
  if (cal_timeout == 0)
  {
    init_tca0(true);
    timer0_ovf_counter = 0;
  }
  else if (cal_timeout == CAL_TIME_SECONDS)
  {
    timer0_count = TCA0.SINGLE.CNT;
    init_tca0(false);
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

static void calibration(cal_refresh_cb cb)
{
  uint8_t prev_cal_timeout = 0;
  cal_timeout = 0;

  DEBUG("Calibration started");

  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), pps_interrupt, RISING);

  si5351.set_clock_pwr(SI5351_CLK2, 1);

  do {
    if (prev_cal_timeout != cal_timeout)
    {
      DEBUG(".");
      if (cb != NULL)
      {
        (*cb)();
      }
    }
    prev_cal_timeout = cal_timeout;
  } while (cal_timeout < CAL_TIME_SECONDS + 1);
  DEBUGLN("");

  detachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN));

  uint32_t pulse_count = ((timer0_ovf_counter * 0x10000) + timer0_count);
  int32_t pulse_diff = pulse_count - (CAL_FREQ * CAL_TIME_SECONDS);
  cal_factor = pulse_diff;

  DEBUG("measured_freq=");
  DEBUGLN(pulse_count / CAL_TIME_SECONDS);
  DEBUG("cal_factor=");
  DEBUGLN(cal_factor);

  if ((cal_factor < 1000000) && (cal_factor > -1000000))
  {
    cal_factor_valid = true;
  }
  else
  {
    cal_factor = 0;
  }

  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_freq(CAL_FREQ * SI5351_FREQ_MULT, SI5351_CLK2);
  si5351.set_clock_pwr(SI5351_CLK2, 0);
}

static void display_header(const char *text)
{
  uint8_t x = (ssd1306_displayWidth() - (6 * strlen(text))) / 2;
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_printFixed(x, 0, text, STYLE_BOLD);
}

static void display_variable(const uint8_t x, const uint8_t y, const char* text)
{
  if (edit_mode == true)
  {
    if (blink_toggle == true)
    {
       ssd1306_negativeMode();
       blink_toggle = false;
    }
    else
    {
       ssd1306_positiveMode();
       blink_toggle = true;
    }
  }
  else
  {
    if (tx_active == true)
    {
      ssd1306_negativeMode();
    }
    else
    {
      ssd1306_positiveMode();
    }
  }

  ssd1306_setFixedFont(ssd1306xled_font8x16);
  ssd1306_printFixed(x,  y, text, STYLE_NORMAL);
  ssd1306_positiveMode();
}

static void display_mode(const char *text)
{
  display_variable(48, 24, text);
}

static void display_frequency(const uint32_t value)
{
  uint16_t freq1 = value / 1000000;
  uint16_t freq2 = (value / 100) % 1000;
  char text[13];

  sprintf(text, "%3d.%04d MHz", freq1, freq2);
  display_variable(16, 24, text);
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
      blink_toggle = false;
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

static void draw_gps_symbol(void)
{
  unsigned long age = TinyGPS::GPS_INVALID_FIX_TIME;

  gps.get_position(NULL, NULL, &age);

  if (age < 5000)
  {
    ssd1306_drawBuffer(0, 0, 8, 8, gps_icon);
  }
  else
  {
    if (blink_toggle)
    {
      ssd1306_drawBuffer(0, 0, 8, 8, gps_icon);
    }
    else
    {
      ssd1306_clearBlock(0, 0, 8, 8);
    }
    blink_toggle = !blink_toggle;
  }
}

static void draw_battery(void)
{
  static uint8_t last_bars = 0xFF;
  uint8_t cur_bars = 0;

  int16_t raw = analogRead(BATTERY_PIN);

  if (raw >= 930)
  {
    cur_bars = 4;
  }
  else if (raw >= 906)
  {
    cur_bars = 3;
  }
  else if (raw >= 860)
  {
    cur_bars = 2;
  }
  else if (raw >= 815)
  {
    cur_bars = 1;
  }
  else
  {
    cur_bars = 0;
  }

  if (last_bars != cur_bars)
  {
    ssd1306_drawBuffer(110, 0, 17, 8, battery_icon);

    for (uint8_t i = 0; i < cur_bars; i++)
    {
      ssd1306_drawBuffer(112 + (i * 3), 0, 2, 8, battery_bar);
    }

    last_bars = cur_bars;
  }
}

static void draw_clock(void)
{
  static uint8_t last_minute = 0xFF;
  uint8_t cur_minute = minute();
  bool draw = false;

  if (refresh_screen == false)
  {
    draw = true;
  }
  else
  {
    if (last_minute != cur_minute)
    {
      draw = true;
      last_minute = cur_minute;
    }
  }

  if (draw)
  {
    char buf[6];

    ssd1306_setFixedFont(ssd1306xled_font6x8);
    sprintf(buf, "%02d:%02d", hour(), cur_minute);
    ssd1306_printFixed( 48,  0,  buf, STYLE_NORMAL);
  }
}

static void draw_enable_status(const bool enabled)
{
  ssd1306_setFixedFont(ssd1306xled_font8x16);

  if (enabled == false)
  {
    ssd1306_printFixed(40,  24, "Enable ", STYLE_NORMAL);
  }
  else
  {
    ssd1306_printFixed(40,  24, "Disable", STYLE_NORMAL);
  }
}

static void show_transmit_status(void)
{
  refresh_screen = true;
  draw_clock();
}

static void show_status_screen(void)
{
  if (refresh_screen == false)
  {
    ssd1306_clearScreen();
    ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_printFixed(  0, 56, call, STYLE_NORMAL);  
    ssd1306_printFixed( 52, 56, mode_params[cur_mode].mode_name, STYLE_NORMAL);
  }

  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_printFixed(104, 56, loc, STYLE_NORMAL);

  draw_clock();
  draw_gps_symbol();
  draw_battery();

  display_frequency(mode_params[cur_mode].freqs[sel_freq]);
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

  display_frequency(mode_params[cur_mode].freqs[sel_freq]);
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

static void show_calibration_progress(void)
{
  static uint8_t counter = 0;

  ssd1306_setFixedFont(ssd1306xled_font8x16);
  ssd1306_printFixed(8, 24, "Calibrating", STYLE_NORMAL);

  switch (counter)
  {
    case 1:  ssd1306_printFixed(96, 24, ".   ", STYLE_NORMAL); break;
    case 2:  ssd1306_printFixed(96, 24, "..  ", STYLE_NORMAL); break;
    case 3:  ssd1306_printFixed(96, 24, "... ", STYLE_NORMAL); break;
    default: ssd1306_printFixed(96, 24, "    ", STYLE_NORMAL); break;
  }

  counter++;
  if (counter > 3)
  {
    counter = 0;
  }
}

static void show_calibration_screen(void)
{
  get_new_value();
  
  if (refresh_screen == false)
  {
    ssd1306_clearScreen();
    display_header("Calibration");
  }

  ssd1306_setFixedFont(ssd1306xled_font8x16);

  if (edit_mode == false)
  {
    char buf[16];
    sprintf(buf, "Factor:%7ld", cal_factor);
    ssd1306_printFixed(8,  24, buf, STYLE_NORMAL);  
  }
  else
  {
    calibration(show_calibration_progress);
    write_config();
    edit_mode = false;
  }
}

static void show_transmitter_screen(void)
{
  get_new_value();

  if (refresh_screen == false)
  {
    ssd1306_clearScreen();
    display_header("Transmitter");
    draw_enable_status(false);
    tx_active = false;
  }

  if (edit_mode == false)
  {
    if (tx_active == true)
    {
      draw_enable_status(false);
      si5351.set_clock_pwr(SI5351_CLK0, 0);
      tx_active = false;
    }
  }
  else
  {
    if (tx_active == false)
    {
      draw_enable_status(true);
      si5351.set_clock_pwr(SI5351_CLK0, 1);
      si5351.set_freq(mode_params[cur_mode].freqs[sel_freq] * SI5351_FREQ_MULT, SI5351_CLK0);
      tx_active = true;
    }
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
      case SCREEN_TRANSMITTER:
        show_transmitter_screen();
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
  pinMode(GPS_PPS_PIN, INPUT);

  DEBUGLN("SSD1306 setup...");
  ssd1306_128x64_i2c_init();
  ssd1306_clearScreen();

  DEBUGLN("Si5351 setup...");
  pinMode(CAL_SIGNAL_PIN, INPUT);
  si5351.init(SI5351_CRYSTAL_LOAD_10PF, SI5351_XTAL_FREQ, cal_factor);

  // Set CLK0 output
  si5351.set_freq(mode_params[cur_mode].freqs[sel_freq] * SI5351_FREQ_MULT, SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.set_clock_pwr(SI5351_CLK0, 0);

  // Set CLK2 output
  si5351.set_ms_source(SI5351_CLK2, SI5351_PLLB);
  si5351.set_freq(CAL_FREQ * SI5351_FREQ_MULT, SI5351_CLK2);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
  si5351.set_clock_pwr(SI5351_CLK2, 0);

  DEBUGLN("Encoder setup...");
  encoder_button.begin();

  DEBUGLN("Initialize timer...");
  init_evsys();
  init_tca0(false);

  DEBUGLN("ADC setup...");
  analogReference(INTERNAL1V1);
  pinMode(BATTERY_PIN, INPUT);

  DEBUGLN("Done...");
}

void loop()
{
  static uint32_t last_update = 0;
  static timeStatus_t last_time_status = timeNotSet;
  timeStatus_t time_status = timeStatus();

  if (Serial1.available())
  {
    process_sync_message();
  }

  if (time_status != last_time_status)
  {
    switch (time_status)
    {
      case timeSet:
        {
          float lat, lon;

          gps.f_get_position(&lat, &lon);
          calc_grid_square(lat, lon);

          set_tx_buffer(tx_buffer);

          if (cal_factor_valid == false)
          {
            calibration(show_calibration_progress);
          }
        }
        break;

      case timeNeedsSync:
        {
          cal_factor_valid = false;
        }
        break;

      default:
        break;
    }

    last_time_status = time_status;
  }

  if (time_status == timeSet)
  {
    switch (minute())
    {
      case  0:
      case 10:
      case 20:
      case 30:
      case 40:
      case 50:
        {
          if (second() == mode_params[cur_mode].start_time)
          {
            encode(tx_buffer, show_transmit_status);
          }
        }
        break;

      case 15:
      case 45:
        {
          if (second() == 0)
          {
            calibration(show_calibration_progress);
          }
        }
        break;

      default:
        break;
    }
  }

  show_screen();

  if ((millis() - last_update) > DISPLAY_REFRESH_TIME)
  {
    refresh_screen = true;
    last_update = millis();
  }
} 
