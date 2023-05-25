// Based on Si5351_WSPR_2560.ino by Jason Milldrum NT7S.

#include "src/Si5351Arduino/src/si5351.h"
#include <JTEncode.h>
#include <int.h>
#include <DS3231.h>
#include <TinyGPS.h>
#include <Encoder.h>
#include <ssd1306.h>
#include <EEPROM.h>
#include <Button.h>

#include "config.h"
#include "priv_types.h"

#define DEBUG_TRACES     1
// #define DEBUG_GPS_NMEA   1

#if DEBUG_TRACES
#define DEBUG(x)    Serial.print(x)
#define DEBUGLN(x)  Serial.println(x)
#else
#define DEBUG(x)
#define DEBUGLN(x)
#endif

#define EEPROM_MODE         (0)
#define EEPROM_BAND         (1)
#define EEPROM_CAL_FACTOR   (2)
#define EEPROM_OUTPUT_POWER (6)

#define VERSION_STRING   "v1.1.3"

const uint8_t gps_icon[8] = { 0x3F, 0x62, 0xC4, 0x88, 0x94, 0xAD, 0xC1, 0x87 };
const uint8_t battery_icon[17] = { 0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
                                   0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0x3C,
                                   0x3C
                                 };
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
DS3231 ds3231;
Encoder encoder(ENC_A_PIN, ENC_B_PIN);
Button encoder_button(ENC_BUTTON_PIN);
char loc[5] = "UNKN";

uint8_t cur_band = BAND_20M;
uint8_t cur_mode = MODE_WSPR;
uint8_t output_power[BAND_COUNT] = { OUTPUT_POWER_MAX_DBM };

#define TX_CHECK_TIME         (100)
static uint8_t tx_buffer[255];
uint8_t cur_screen = SCREEN_COUNT;
volatile bool refresh_screen = false;

bool edit_mode = false;
bool blink_toggle = false;
bool tx_active = false;
volatile int8_t tx_timeout = 0;
bool gps_enabled = false;

#define CAL_FREQ             (2500000ULL)
#define CAL_TIME_SECONDS     (40UL)
#define CAL_TIMEOUT_SECONDS  (CAL_TIME_SECONDS + 1)
volatile uint32_t timer0_ovf_counter = 0;
volatile uint16_t timer0_count = 0;
volatile uint8_t cal_counter = CAL_TIMEOUT_SECONDS;
volatile uint8_t cal_watchdog_last_counter = 0;
volatile uint8_t cal_watchdog = 0;
static int32_t cal_factor = 0;
static bool cal_factor_valid = false;

typedef void (*cal_refresh_cb)(void);

/* This function returns the DST offset for the current UTC time.
   This is valid for the EU, for other places see
   http://www.webexhibits.org/daylightsaving/i.html

   Results have been checked for 2012-2030 (but should work since
   1996 to 2099) against the following references:
   - http://www.uniquevisitor.it/magazine/ora-legale-italia.php
   - http://www.calendario-365.it/ora-legale-orario-invernale.html
*/
byte dstOffset (byte d, byte m, unsigned int y, byte h) {
  // Day in March that DST starts on, at 1 am
  byte dstOn = (31 - (5 * y / 4 + 4) % 7);

  // Day in October that DST ends  on, at 2 am
  byte dstOff = (31 - (5 * y / 4 + 1) % 7);

  if ((m > 3 && m < 10) ||
      (m == 3 && (d > dstOn || (d == dstOn && h >= 1))) ||
      (m == 10 && (d < dstOff || (d == dstOff && h <= 1))))
    return 1;
  else
    return 0;
}

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
  DEBUGLN("Reading config");

  cur_mode = EEPROM.read(EEPROM_MODE);
  if (cur_mode > MODE_COUNT)
  {
    cur_mode = MODE_WSPR;
  }

  cur_band = EEPROM.read(EEPROM_BAND);
  if (cur_band > BAND_COUNT)
  {
    cur_band = BAND_20M;
  }

  cal_factor  = (uint32_t)EEPROM.read(EEPROM_CAL_FACTOR)     << 24;
  cal_factor |= (uint32_t)EEPROM.read(EEPROM_CAL_FACTOR + 1) << 16;
  cal_factor |= (uint32_t)EEPROM.read(EEPROM_CAL_FACTOR + 2) <<  8;
  cal_factor |= (uint32_t)EEPROM.read(EEPROM_CAL_FACTOR + 3);
  if (cal_factor == -1)
  {
    cal_factor = 0;
  }

  for (uint8_t band = 0; band < BAND_COUNT; band++)
  {
    output_power[band] = EEPROM.read(EEPROM_OUTPUT_POWER + band);
    if (output_power[band] == 0xFF)
    {
      output_power[band] = OUTPUT_POWER_MAX_DBM;
    }
  }
}

static void write_config(void)
{
  DEBUGLN("Writing config");

  EEPROM.write(EEPROM_MODE, cur_mode);
  EEPROM.write(EEPROM_BAND, cur_band);
  EEPROM.write(EEPROM_CAL_FACTOR,     (cal_factor >> 24) & 0xFF);
  EEPROM.write(EEPROM_CAL_FACTOR + 1, (cal_factor >> 16) & 0xFF);
  EEPROM.write(EEPROM_CAL_FACTOR + 2, (cal_factor >>  8) & 0xFF);
  EEPROM.write(EEPROM_CAL_FACTOR + 3, (cal_factor)       & 0xFF);

  for (uint8_t band = 0; band < BAND_COUNT; band++)
  {
    EEPROM.write(EEPROM_OUTPUT_POWER + band, output_power[band]);
  }
}

static const uint8_t ubxSetPMREQ[16] = { 0xb5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4d, 0x3b };
static const uint8_t ubxVer[6] = { 0xb5, 0x62, 0x0a, 0x04, 0x0e, 0x18 };

static void turn_off_gps(void)
{
  if (gps_enabled == true)
  {
    DEBUGLN("Disabling GPS");
    Serial1.write(ubxSetPMREQ, sizeof(ubxSetPMREQ));
    gps_enabled = false;
    delay(1000);
  }
}

static void turn_on_gps(void)
{
  if (gps_enabled == false)
  {
    DEBUGLN("Enabling GPS");
    Serial1.write(ubxVer, sizeof(ubxVer));
    gps_enabled = true;
    delay(1000);
  }
}

static bool is_gps_fixed(void)
{
  unsigned long age = TinyGPS::GPS_INVALID_FIX_TIME;

  gps.get_position(NULL, NULL, &age);

  if (age < 5000)
  {
    return true;
  }

  return false;
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
  si5351.set_freq(mode_params[cur_mode].freqs[cur_band] * SI5351_FREQ_MULT, SI5351_CLK0);
  tx_active = true;

  DEBUGLN("Starting TX");

  display_frequency(mode_params[cur_mode].freqs[cur_band]);

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
    si5351.set_freq((mode_params[cur_mode].freqs[cur_band] * SI5351_FREQ_MULT) + (tx_buffer[i] * mode_params[cur_mode].tone_spacing), SI5351_CLK0);

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

  DEBUGLN("TX finished");
}

static void set_tx_buffer(uint8_t *tx_buffer)
{
  char message[14];

  DEBUGLN("Setting TX buffer");

  sprintf(message, "%s %s", call, loc);

  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  switch (cur_mode)
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
      jtencode.wspr_encode(call, loc, output_power[cur_band], tx_buffer);
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

static void process_gps_sync_message()
{
  if (Serial1.available())
  {
    char c = Serial1.read();

#if DEBUG_GPS_NMEA
    DEBUG(c);
#endif

    // Process gps messages
    if (gps.encode(c))
    {
      // When TinyGPS reports new data...
      unsigned long age;
      int Year;
      byte Month, Day, Hour, Minute, Second;

      gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);

      if (age < 500)
      {
        // Set the Time to the latest GPS reading
        ds3231.setSecond(Second);
        ds3231.setMinute(Minute);
        ds3231.setHour(Hour + offset + dstOffset(Day, Month, Year, Hour));
        ds3231.setDate(Day);
        ds3231.setMonth(Month);
        ds3231.setYear(Year);
      }
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

static void init_pit(void)
{
  // Internal 32.768 kHz from OSCULP32K
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;

  // Periodic Interrupt: enabled
  RTC.PITINTCTRL = RTC_PI_bm;

  // set pit period off at startup
  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;
}

static void disable_pit_irq(void)
{
  RTC.PITINTCTRL = 0;
}

static void enable_pit_irq(void)
{
  RTC.PITINTCTRL = RTC_PI_bm;
}

ISR(RTC_PIT_vect)
{
  if (cal_counter < CAL_TIME_SECONDS)
  {
    DEBUG("Cal. watchdog: prev/cur=");
    DEBUG(cal_watchdog_last_counter);
    DEBUG("/");
    DEBUGLN(cal_counter);

    if (cal_watchdog_last_counter == cal_counter)
    {
      cal_watchdog++;
    }
    else
    {
      cal_watchdog = 0;
    }

    if (cal_watchdog >= 3)
    {
      // Timeout
      DEBUGLN("Lost GPS!");
      cal_counter = CAL_TIMEOUT_SECONDS;
      init_tca0(false);
    }

    cal_watchdog_last_counter = cal_counter;
  }

  refresh_screen = true;

  RTC.PITINTFLAGS = RTC_PI_bm;
}

static void pps_interrupt()
{
  if (cal_counter == 0)
  {
    init_tca0(true);
    timer0_ovf_counter = 0;
  }
  else if (cal_counter == CAL_TIME_SECONDS)
  {
    timer0_count = TCA0.SINGLE.CNT;
    init_tca0(false);
  }
  if (cal_counter < CAL_TIMEOUT_SECONDS)
  {
    cal_counter++;
  }
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
  uint8_t prev_cal_counter = CAL_TIMEOUT_SECONDS;
  cal_watchdog_last_counter = CAL_TIMEOUT_SECONDS;

  DEBUGLN("Cal. started");

  cal_counter = 0;
  cal_watchdog = 0;

  do {
    if (prev_cal_counter != cal_counter)
    {
      if (cb != NULL)
      {
        (*cb)();
      }
    }
    prev_cal_counter = cal_counter;
  } while (cal_counter < CAL_TIMEOUT_SECONDS);

  uint32_t pulse_count = ((timer0_ovf_counter * 0x10000) + timer0_count);
  int32_t pulse_diff = pulse_count - (CAL_FREQ * CAL_TIME_SECONDS);
  int32_t new_cal_factor = cal_factor + (pulse_diff * 10UL);

  DEBUG("Cal. freq.: ");
  DEBUG((float)(pulse_count / CAL_TIME_SECONDS));
  DEBUGLN("Hz");
  DEBUG("Cal. factor: ");
  DEBUGLN(new_cal_factor);

  if ((new_cal_factor < 1000000) && (new_cal_factor > -1000000))
  {
    cal_factor_valid = true;
    cal_factor = new_cal_factor;
    write_config();
  }

  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_freq(CAL_FREQ * SI5351_FREQ_MULT, SI5351_CLK2);

  if (cal_factor_valid == true)
  {
    si5351.set_clock_pwr(SI5351_CLK2, 0);
    turn_off_gps();
  }
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
  char text[16];

  sprintf(text, " %3d.%04d MHz ", freq1, freq2);
  display_variable(8, 24, text);
}

static void display_output_power(const uint32_t value)
{
  char text[16];

  sprintf(text, "%2lu dBm", value);
  display_variable(40, 24, text);
}

static uint8_t get_next_screen(void)
{
  static int32_t old_position = 0;
  uint8_t next_screen = cur_screen;

  if (next_screen == SCREEN_COUNT)
  {
    refresh_screen = false;
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
  if (gps_enabled == false)
  {
    ssd1306_clearBlock(0, 0, 8, 8);
    return;
  }

  if (is_gps_fixed())
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

static void draw_battery(bool force_update)
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

  if ((last_bars != cur_bars) || (force_update))
  {
    ssd1306_drawBuffer(110, 0, 17, 8, battery_icon);

    for (uint8_t i = 0; i < cur_bars; i++)
    {
      ssd1306_drawBuffer(112 + (i * 3), 0, 2, 8, battery_bar);
    }

    last_bars = cur_bars;
  }
}

static void draw_clock(bool force_update)
{
  static uint8_t last_minute = 0xFF;
  bool h12 = false, PM_time = false;
  uint8_t cur_minute = ds3231.getMinute();
  uint8_t cur_hour = ds3231.getHour(h12, PM_time);

  if (last_minute != cur_minute)
  {
    force_update = true;
    last_minute = cur_minute;
  }

  if (force_update)
  {
    char buf[6];

    if ((cur_hour > 24) || (cur_minute > 59))
    {
      sprintf(buf, "--:--");
    }
    else
    {
      sprintf(buf, "%02u:%02u", cur_hour, cur_minute);      
    }

    DEBUG("Time: ");
    DEBUGLN(buf);

    ssd1306_setFixedFont(ssd1306xled_font6x8);
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
  draw_clock(false);
}

static void show_status_screen(bool force_update)
{
  if (force_update)
  {
    ssd1306_clearScreen();
    ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_printFixed(  0, 56, call, STYLE_NORMAL);
    ssd1306_printFixed( 52, 56, mode_params[cur_mode].mode_name, STYLE_NORMAL);
  }

  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_printFixed(104, 56, loc, STYLE_NORMAL);

  draw_clock(force_update);
  draw_gps_symbol();
  draw_battery(force_update);

  display_frequency(mode_params[cur_mode].freqs[cur_band]);
}

static void show_set_mode_screen(bool force_update)
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
    force_update = false;
  }

  if (force_update)
  {
    ssd1306_clearScreen();
    display_header("Mode");
  }

  display_mode(mode_params[cur_mode].mode_name);
}

static void show_set_frequency_screen(bool force_update)
{
  int8_t value = get_new_value();

  if (value != 0)
  {
    if (value > 0)
    {
      cur_band++;
      if (cur_band == BAND_COUNT)
      {
        cur_band = 0;
      }
    }
    else
    {
      if (cur_band == 0)
      {
        cur_band = BAND_COUNT;
      }
      cur_band--;
    }
    force_update = false;
  }

  if (force_update)
  {
    ssd1306_clearScreen();
    display_header("Frequency");
  }

  display_frequency(mode_params[cur_mode].freqs[cur_band]);
}

static void show_set_output_power(bool force_update)
{
  int8_t value = get_new_value();

  if (value != 0)
  {
    if (value > 0)
    {
      if (output_power[cur_band] < OUTPUT_POWER_MAX_DBM)
      {
        output_power[cur_band]++;
      }
    }
    else
    {
      if (output_power[cur_band] > OUTPUT_POWER_MIN_DBM)
      {
        output_power[cur_band]--;
      }
    }
    force_update = false;
  }

  if (force_update)
  {
    ssd1306_clearScreen();
    display_header("Output power");
  }

  display_output_power(output_power[cur_band]);
}

static void show_gps_status_screen(bool force_update)
{
  int Year;
  byte Month, Day, Hour, Minute, Second;
  long lat, lon;
  unsigned long age;
  char buf[21];

  if (force_update)
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

static void show_calbration_status(void)
{
  draw_clock(false);

  ssd1306_negativeMode();
  ssd1306_setFixedFont(ssd1306xled_font8x16);
  ssd1306_printFixed(8, 24, "C", STYLE_NORMAL);
  ssd1306_positiveMode();
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

static void show_gps_acquisition_progress(void)
{
  static uint8_t counter = 0;

  ssd1306_setFixedFont(ssd1306xled_font8x16);
  ssd1306_printFixed(8, 24, "Waiting GPS", STYLE_NORMAL);

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

static void show_calibration_screen(bool force_update)
{
  get_new_value();

  if (force_update)
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
    si5351.set_clock_pwr(SI5351_CLK2, 1);
    turn_on_gps();

    while (is_gps_fixed() == false)
    {
      process_gps_sync_message();

      if (refresh_screen)
      {
        show_gps_acquisition_progress();
        refresh_screen = false;
      }
    }

    calibration(show_calibration_progress);
    edit_mode = false;
  }
}

static void show_transmitter_screen(bool force_update)
{
  get_new_value();

  if (force_update == true)
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
      si5351.set_freq(mode_params[cur_mode].freqs[cur_band] * SI5351_FREQ_MULT, SI5351_CLK0);
      tx_active = true;
    }
  }
}

static void show_version_screen(bool force_update)
{
  if (force_update == true)
  {
    ssd1306_clearScreen();
    display_header("Version");
    ssd1306_setFixedFont(ssd1306xled_font8x16);
    ssd1306_printFixed(40,  24, VERSION_STRING, STYLE_NORMAL);
  }
}

static void show_welcome_screen(void)
{
  ssd1306_clearScreen();
  ssd1306_setFixedFont(ssd1306xled_font8x16);
  ssd1306_printFixed(16,  24, "WSJT Beacon", STYLE_NORMAL); 
}

static void force_switch_to_status_screen(void)
{
  bool force_update = false;

  if (cur_screen != SCREEN_STATUS)
  {
    cur_screen = SCREEN_STATUS;
    force_update = true;
  }

  show_status_screen(force_update);
}

static void show_screen(void)
{
  uint8_t next_screen = get_next_screen();
  bool force_update = false;

  if (next_screen != cur_screen)
  {
    force_update = true;
  }

  if ((next_screen != cur_screen) || refresh_screen)
  {
    switch (next_screen)
    {
      case SCREEN_SET_MODE:
        show_set_mode_screen(force_update);
        break;
      case SCREEN_SET_FREQUENCY:
        show_set_frequency_screen(force_update);
        break;
      case SCREEN_SET_OUTPUT_POWER:
        show_set_output_power(force_update);
        break;
      case SCREEN_GPS_STATUS:
        show_gps_status_screen(force_update);
        break;
      case SCREEN_CALIBRATION:
        show_calibration_screen(force_update);
        break;
      case SCREEN_TRANSMITTER:
        show_transmitter_screen(force_update);
        break;
      case SCREEN_VERSION:
        show_version_screen(force_update);
        break;
      default:
        show_status_screen(force_update);
        break;
    }

    cur_screen = next_screen;
    refresh_screen = false;
  }
}

void setup()
{
  Serial.begin(115200);

  read_config();

  DEBUGLN("Setup:");
  DEBUGLN("- NEO-6M");
  Serial1.begin(9600);
  pinMode(GPS_PPS_PIN, INPUT);

  DEBUGLN("- SSD1306");
  ssd1306_128x64_i2c_init();
  // ssd1306_setContrast(0x01);
  show_welcome_screen();

  DEBUGLN("- SI5351");
  pinMode(CAL_SIGNAL_PIN, INPUT);
  si5351.init(SI5351_CRYSTAL_LOAD_10PF, SI5351_XTAL_FREQ, cal_factor);
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), pps_interrupt, RISING);

  // Set CLK0 output
  si5351.set_freq(mode_params[cur_mode].freqs[cur_band] * SI5351_FREQ_MULT, SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.set_clock_pwr(SI5351_CLK0, 0);

  // Set CLK2 output
  si5351.set_ms_source(SI5351_CLK2, SI5351_PLLB);
  si5351.set_freq(CAL_FREQ * SI5351_FREQ_MULT, SI5351_CLK2);
  //  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);

  DEBUGLN("- Encoder");
  encoder_button.begin();

  DEBUGLN("- Timer");
  init_pit();
  init_evsys();
  init_tca0(false);

  DEBUGLN("- DS3231");
  ds3231.setClockMode(false);

  DEBUGLN("- ADC");
  analogReference(INTERNAL1V1);
  pinMode(BATTERY_PIN, INPUT);

  turn_on_gps();

  DEBUGLN("Done");
}

void loop()
{
  static uint32_t last_update = 0;

  process_gps_sync_message();

  if (is_gps_fixed() && (cal_factor_valid == false))
  {
    float lat, lon;

    gps.f_get_position(&lat, &lon);
    calc_grid_square(lat, lon);

    set_tx_buffer(tx_buffer);

    force_switch_to_status_screen();
    calibration(show_calbration_status);
  }

  if ((cal_factor_valid) && ((millis() - last_update) > TX_CHECK_TIME))
  {
    switch (ds3231.getMinute())
    {
      case  0:
      case 10:
      case 20:
      case 30:
      case 40:
      case 50:
        {
          if (ds3231.getSecond() == mode_params[cur_mode].start_time)
          {
            force_switch_to_status_screen();
            encode(tx_buffer, show_transmit_status);
          }
        }
        break;

      case 13:
      case 43:
        {
          if (ds3231.getSecond() == 0)
          {
            si5351.set_clock_pwr(SI5351_CLK2, 1);
            turn_on_gps();
          }
        }
        break;

      case 15:
      case 45:
        {
          if (ds3231.getSecond() == 0)
          {
            if (is_gps_fixed())
            {
              force_switch_to_status_screen();
              calibration(show_calbration_status);
            }
            else
            {
              si5351.set_clock_pwr(SI5351_CLK2, 0);
              turn_off_gps();
            }
          }
        }
        break;

      default:
        break;
    }

    last_update = millis();
  }

  show_screen();
}
