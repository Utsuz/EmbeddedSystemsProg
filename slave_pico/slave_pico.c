#include "activation_driver.h"
#include "bmp388_driver.h"
#include "hmac_sha256.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "uart_driver.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* ========================================= */
/* ===== 1. CONSTANTS AND DEFINITIONS ===== */
/* ========================================= */

#define UART_PORT_NUM 1
#define UART_TX_PIN 8
#define UART_RX_PIN 9
#define UART_BAUDRATE 115200

// Sensor/Hardware
#define EX_I2C_PORT 0
#define EX_SDA_PIN 4
#define EX_SCL_PIN 5
#define EX_ADDR 0x77
#define BUZZER_PIN 18

// Security
#define HMAC_KEY "INF2004_TEMP_LOG_KEY"
#define HMAC_LEN 32

// Buttons
#define BTN_TIME 20
#define BTN_DUMP 21
#define BTN_TOGGLE 22

// Timing
#define SAMPLE_PERIOD_MS 5000
#define EXCURSION_PERIOD_MS 200
#define BLINK_FAST_MS 200

// Binary Protocol
#define DONE_CHAR 0x06
#define CMD_HELLO 0x11     
#define CMD_HI_ACK 0x12    
#define CMD_GET_TIME 0x30  
#define CMD_TIME_RSP 0x31  
#define ACK_EMPTY_LOG 0x40 
#define CMD_REQ_DATA 0x21
#define CMD_SEND_DATA_HDR 0x20
#define BINARY_HEADER_SIZE 3
#define BINARY_TIME_SIZE   9

typedef enum { STATE_HANDSHAKE, STATE_IDLE, STATE_WAIT_TIME } comm_state_t;

/* ========================================= */
/* ===== 2. GLOBAL STATE & PROTOTYPES ===== */
/* ========================================= */

static absolute_time_t next_sample_time;
static uint64_t initial_unix_time = 0; // The time received from Master
static bool sensor_active = false;
static bool prev_dump = true;
static bool prev_toggle = true;

// Prototypes for new refactored functions
static void init_hardware_and_gpio(void);
static void periodic_sampling_and_log(void);
static void handle_led_and_buzzer_state(void);
static void handle_button_polling(void);
static void handle_uart_state_machine(comm_state_t *state_ptr);
static uint32_t dump_compact_to_uart(void);
static void print_hex(const uint8_t *b, size_t n, const char *prefix);

static inline uint32_t current_period_ms(void) {
  return bmp388_excursion_state() ? EXCURSION_PERIOD_MS : SAMPLE_PERIOD_MS;
}

/* ========================================= */
/* ===== 3. UTILITY AND DRIVER WRAPPERS ===== */
/* ========================================= */

static void print_hex(const uint8_t *b, size_t n, const char *prefix) {
  printf("%s", prefix);
  for (size_t i = 0; i < n; ++i) {
    printf("%02x", b[i]);
  }
  printf("\n");
}

static void buzzer_init(void) {
  gpio_init(BUZZER_PIN);
  gpio_set_dir(BUZZER_PIN, GPIO_OUT);
  gpio_put(BUZZER_PIN, 0);
}

static void buzzer_set(bool on) { gpio_put(BUZZER_PIN, on); }

static uint32_t dump_compact_to_uart(void) {
  const compact_hdr_t *h = xip_hdr_compact();

  if (h->magic != CB_MAGIC || h->length == 0) {
    printf("[COMPACT] No valid blob found (len: %u).\n", (unsigned)h->length);
    uart_putc_raw(uart_get_instance(UART_PORT_NUM), ACK_EMPTY_LOG);
    return 0;
  }

  const uint32_t time_size = sizeof(uint64_t);
  uint32_t compact_length = h->length;
  
  // CALCULATE NEW TOTAL LENGTH (8 bytes time + compact data)
  const uint32_t total_payload_length = compact_length + time_size; 
  
  uint8_t mac[HMAC_LEN];

  const uint8_t *data_ptr = xip_data_compact();
  // NOTE: HMAC is calculated only on the original compact data, NOT including the time.
  hmac_sha256((const uint8_t *)HMAC_KEY, strlen(HMAC_KEY), data_ptr, compact_length, mac);

  print_hex(mac, HMAC_LEN, "[Slave] Calculated HMAC: ");
  printf("[Slave] Data length: %u bytes (Compact: %u).\n", (unsigned)total_payload_length, (unsigned)compact_length);

  // 1. Send the fixed binary header: CMD (1B) + Length (2B)
  uart_putc_raw(uart_get_instance(UART_PORT_NUM), CMD_SEND_DATA_HDR); // 0x20
  
  // Send NEW Total Payload Length (Total Length = Compact Length + 8)
  uart_putc_raw(uart_get_instance(UART_PORT_NUM), (uint8_t)(total_payload_length & 0xFF)); // LSB
  uart_putc_raw(uart_get_instance(UART_PORT_NUM), (uint8_t)(total_payload_length >> 8));  // MSB

  sleep_ms(5); 

  printf("[Slave] Sending %u bytes payload (8B time + %uB data)...\n", 
            (unsigned)total_payload_length, (unsigned)compact_length);

  // --- 2. Send the raw binary data (8B Time + Compact Blob) ---
  
  // 2a. Send the 8-byte Unix time (NEW)
  uint64_t time_val = initial_unix_time;
  // Send Little-Endian (LSB first) for compatibility with PC systems
  for (int i = 0; i < time_size; i++) {
      uart_putc_raw(uart_get_instance(UART_PORT_NUM), (uint8_t)(time_val >> (i * 8)));
  }

  // 2b. Send the compact binary data (UNMODIFIED LOOP)
  for (uint32_t i = 0; i < compact_length; i++) {
    uart_putc_raw(uart_get_instance(UART_PORT_NUM), data_ptr[i]);
  }

  // 3. Send the single DONE character
  uart_putc_raw(uart_get_instance(UART_PORT_NUM), DONE_CHAR);

  return total_payload_length;
}

/* ========================================= */
/* ===== 4. REFACTORED LOOP FUNCTIONS ===== */
/* ========================================= */

static void periodic_sampling_and_log(void) {
  if (sensor_active &&
      absolute_time_diff_us(get_absolute_time(), next_sample_time) <= 0) {
    bmp388_sample_t s;
    if (bmp388_read(&s) == 0) {
      uint32_t now_ms = to_ms_since_boot(get_absolute_time());
      int lr = bmp388_storage_append(now_ms, s.temperature_c);
      printf("T=%.2f C (t=%u)%s\n", s.temperature_c, (unsigned)now_ms,
             bmp388_excursion_state() ? " [FAST]" : "");
      if (lr) {
        printf("Note: ACTIVE region is full; sample not saved.\n");
      }
    }
    next_sample_time =
        delayed_by_ms(get_absolute_time(), current_period_ms());
  }
}

static void handle_led_and_buzzer_state(void) {
  static absolute_time_t next_blink_toggle = {0};
  static uint32_t current_blink_period_ms = 0;
  static bool led_state_on = false;

  if (sensor_active) {
    if (bmp388_excursion_state()) {
      buzzer_set(true);
      current_blink_period_ms = BLINK_FAST_MS;
    } else {
      buzzer_set(false);
      current_blink_period_ms = 0;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
      led_state_on = true;
    }
  } else {
    current_blink_period_ms = 0;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    buzzer_set(false);
    led_state_on = false;
  }

  if (current_blink_period_ms > 0 &&
      absolute_time_diff_us(get_absolute_time(), next_blink_toggle) <= 0) {
    led_state_on = !led_state_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state_on);
    next_blink_toggle =
        delayed_by_ms(get_absolute_time(), current_blink_period_ms);
  }
}

static void handle_button_polling(void) {
  // USB serial 'A' dump
  int c = getchar_timeout_us(0);
  if (c == 'A' || c == 'a') {
    stdio_flush();
    dump_active_compact_bit();
    stdio_flush();
  }

  // GP21: dump compact (to USB stdio)
  bool now_dump = gpio_get(BTN_DUMP);
  if (!now_dump && prev_dump) {
    stdio_flush();
    dump_active_compact_bit();
    stdio_flush();
    sleep_ms(200);
  }
  prev_dump = now_dump;

  // GP22: toggle sensor
  bool now_toggle = gpio_get(BTN_TOGGLE);
  if (!now_toggle && prev_toggle) {
    if (sensor_active) {
      bmp388_sensorStop();
      sensor_active = false;
      printf("[BTN22] Sensor stopped.\n");
    } else {
      bmp388_sensorStart();
      sensor_active = true;
      printf("[BTN22] Sensor started.\n");
    }
  }
  prev_toggle = now_toggle;
}

static void handle_uart_state_machine(comm_state_t *state_ptr) {
  switch (*state_ptr) {
  case STATE_HANDSHAKE:
    // Expect single CMD_HELLO byte
    if (uart_driver_available()) {
        uint8_t cmd = uart_driver_read_byte();
        if (cmd == CMD_HELLO) {
            uart_putc_raw(uart_get_instance(UART_PORT_NUM), CMD_HI_ACK);
            printf("[Slave] Handshake OK\n");
            *state_ptr = STATE_IDLE;
        }
    }
    break;

  case STATE_IDLE:
    // GP20: ask master for time
    if (!gpio_get(BTN_TIME)) {
      printf("[Slave] Button pressed → Requesting time\n");
      // Send binary GET_TIME command
      uart_putc_raw(uart_get_instance(UART_PORT_NUM), CMD_GET_TIME);
      *state_ptr = STATE_WAIT_TIME;
    }
    
    // Handle incoming commands
    if (uart_driver_available()) {
      uint8_t cmd = uart_driver_read_byte();

      if (cmd == CMD_REQ_DATA) {
        printf("[Slave] Received GET_DATA (0x%02X). Stopping sensor, creating backup, and transferring...\n", CMD_REQ_DATA);

        bmp388_sensorStop();
        sensor_active = false;
        printf("[Slave] Sensor stopped.\n");

        bmp388_backup_compact_save();
        printf("[Slave] Compact backup created on flash.\n");

        uint32_t len = dump_compact_to_uart();

        if (len > 0) {
          printf("[Slave] Sent %u bytes of compact data over UART1.\n",
                 (unsigned)len);
        } else {
          printf("[Slave] Log empty, sent ACK_EMPTY.\n");
        }
        
        // Clear any pending commands before returning to idle
        int flushed_count = 0;
        while (uart_driver_available()){
              uart_driver_read_byte();
              flushed_count++;
        }
        if (flushed_count > 0) {
          printf("[Slave] DEBUG: Flushed %d bytes of pending commands from UART.\n", flushed_count);
        }

        sleep_ms(500); // Protective delay

      } else if (cmd == CMD_HELLO) {
        uart_putc_raw(uart_get_instance(UART_PORT_NUM), CMD_HI_ACK);
      } else {
        // Discard stray byte that wasn't a recognized command
        printf("[Slave] WARN: Discarded stray byte 0x%02X in IDLE.\n", cmd);
      }
    }
    break;

  case STATE_WAIT_TIME: {
    uint8_t time_buf[BINARY_TIME_SIZE]; // 1B CMD + 8B Unix Time
    size_t read_count = 0;
    
    // Use reliable timed read for the entire fixed-size binary response.
    // This replaces the old 'while' loop and text parsing.
    read_count = uart_driver_read_timed(time_buf, BINARY_TIME_SIZE, 5000);

    // Check if the full packet was read AND if the command byte is correct.
    if (read_count == BINARY_TIME_SIZE && time_buf[0] == CMD_TIME_RSP) {
        uint64_t base_time = 0;
        
        // Extract 8 bytes of time using memcpy
        memcpy(&base_time, &time_buf[1], sizeof(uint64_t)); 
        // Stores the time
        initial_unix_time = base_time;
        
        printf("[Slave] Received Unix time: %" PRIu64 "\n", base_time);
        time_t t = (time_t)base_time;
        printf("[Slave] Human time: %s", ctime(&t));

        bmp388_sensorStart();
        sensor_active = true;
        printf("[Slave] Sensor started (time synchronized).\n");
        next_sample_time =
            delayed_by_ms(get_absolute_time(), current_period_ms());
        
        *state_ptr = STATE_IDLE;
        
    } else {
        printf("[Slave] TIME wait failed (Read %zu bytes, CMD 0x%02X). Back to IDLE.\n", 
               read_count, time_buf[0]);

        // Clear any lingering garbage before returning to IDLE on failure
        while (uart_driver_available()) {
            uart_driver_read_byte();
        }

        *state_ptr = STATE_IDLE;
    }
  } break;
}
}

/* ========================================= */
/* ===== 5. MAIN EXECUTION ===== */
/* ========================================= */

static void init_hardware_and_gpio(void) {
  stdio_init_all();
  sleep_ms(1200);

  if (cyw43_arch_init()) {
    printf("Wi-Fi init failed, LED control may be unavailable.\n");
  }

  printf("=== Slave Pico (BMP388 logger) ===\n");

  activation_driver_init(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                         UART_BAUDRATE);

  bmp388_storage_init();
  int rc = bmp388_init(EX_I2C_PORT, EX_SDA_PIN, EX_SCL_PIN, EX_ADDR);
  if (rc) {
    printf("BMP388 init failed: %d\n", rc);
    while (1)
      sleep_ms(1000);
  }

  buzzer_init();

  gpio_init(BTN_TIME); gpio_set_dir(BTN_TIME, GPIO_IN); gpio_pull_up(BTN_TIME);
  gpio_init(BTN_DUMP); gpio_set_dir(BTN_DUMP, GPIO_IN); gpio_pull_up(BTN_DUMP);
  gpio_init(BTN_TOGGLE); gpio_set_dir(BTN_TOGGLE, GPIO_IN); gpio_pull_up(BTN_TOGGLE);

  printf("Buttons: GP20=request TIME, GP21=dump compact, GP22=start/stop\n");
  printf("USB serial: press 'A' to dump compact\n");
}

int main(void) {
  init_hardware_and_gpio();

  comm_state_t state = STATE_HANDSHAKE;
  
  // Initialize sample time to now + period
  next_sample_time = make_timeout_time_ms(SAMPLE_PERIOD_MS); 

  while (true) {
    handle_button_polling();
    periodic_sampling_and_log();
    handle_led_and_buzzer_state();
    handle_uart_state_machine(&state);

    sleep_ms(20);
  }
}
