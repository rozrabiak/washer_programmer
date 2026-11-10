/**
 * ============================================================================
 * FULL RECONSTRUCTION OF WASHING MACHINE CONTROLLER FIRMWARE (ATmega644P)
 * Reconstructed from IAR firmware decompilation (flash_intelhex.c / dekomp.txt)
 * 
 * Includes all subsystems:
 *  1. PROGMEM tables: 16 programs, 100 NTC points, 16 faults, spin VM scripts
 *  2. EEPROM manager: 83B calibration (0xCA..0x55+CRC), 5x state redundancy, 10-fault FIFO
 *  3. ADC subsystem (80 samples) + AC waveform classifier (32 windows) for triac and door lock
 *  4. AC motor phase-angle controller: Zero-Cross (INT0), Tacho (ICP1 + median), adaptive PID
 *  5. Virtual Machine (Bytecode Interpreter) for spin profiles and unbalance control
 *  6. Full 16-fault diagnostics + door locking (8 retries) + heater supervision (+2C/30s)
 *  7. Wash state machine (steps 0..28), 5-phase drum reversal and 10 load distribution retries
 *  8. UI handling: resistor ladder buttons (ADC0), dials (ADC4/ADC6), LED matrix and Service Mode
 *  9. UART0 diagnostic telemetry (54 system variables from pointer table)
 * ============================================================================
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

// ============================================================================
// 1. ATMEGA644P MICROCONTROLLER PIN DEFINITIONS (from FUN_code_1e28 and 093d..09ea)
// ============================================================================
namespace Pins {
    // PORTA (0x20..0x22) - Analog inputs ADC0..ADC7
    constexpr uint8_t ADC_BUTTONS_CH     = 0; // PA0 (ADC0) - Button resistor ladder
    constexpr uint8_t ADC_DOOR_TRIAC_CH  = 1; // PA1 (ADC1) - Door lock / triac sensing
    constexpr uint8_t ADC_PUMP_SENSE_CH  = 2; // PA2 (ADC2) - Pump / output circuit sensing
    constexpr uint8_t ADC_PROG_DIAL_CH   = 4; // PA4 (ADC4) - Program selector dial (1..16)
    constexpr uint8_t ADC_NTC_SENSOR_CH  = 5; // PA5 (ADC5) - NTC water temperature sensor
    constexpr uint8_t ADC_SPIN_DIAL_CH   = 6; // PA6 (ADC6) - Options / spin speed dial (0..6)
    constexpr uint8_t ADC_MAINS_VOLT_CH  = 7; // PA7 (ADC7) - 230V AC mains voltage measurement

    // PORTB (0x23..0x25) - Water valves and LED matrix columns
    constexpr uint8_t PB_VALVE_PREWASH   = 0; // PB0 - Prewash solenoid valve (30% PWM)
    constexpr uint8_t PB_VALVE_SOFTENER  = 1; // PB1 - Softener / rinse solenoid valve (30% PWM)
    constexpr uint8_t PB_LED_MASK        = 0xFC; // PB2..PB7 - LED matrix lines

    // PORTC (0x26..0x28) - Pressure switch (hydrostat), Motor triac, Pump, Main valve
    constexpr uint8_t PC_HYDROSTAT_IN    = 0; // PC0 - Digital pressure switch (hydrostat) input
    constexpr uint8_t PC_MOTOR_TRIAC     = 1; // PC1 - Motor triac gate output
    constexpr uint8_t PC_DIAG_TICK_100MS = 2; // PC2 - 100ms loop diagnostic pin
    constexpr uint8_t PC_DIAG_TICK_200MS = 3; // PC3 - 200ms loop diagnostic pin
    constexpr uint8_t PC_DIAG_TICK_FAST  = 4; // PC4 - Fast loop diagnostic pin
    constexpr uint8_t PC_RELAY_AUX2      = 5; // PC5 - Auxiliary relay (for board variant 1)
    constexpr uint8_t PC_DRAIN_PUMP      = 6; // PC6 - Drain pump (50% PWM)
    constexpr uint8_t PC_VALVE_MAIN      = 7; // PC7 - Main wash solenoid valve (30% PWM)

    // PORTD (0x29..0x2B) - UART0, Zero-Crossing (INT0), Tacho (ICP1), Power relays
    constexpr uint8_t PD_UART_RX         = 0; // PD0 - RXD0
    constexpr uint8_t PD_UART_TX         = 1; // PD1 - TXD0
    constexpr uint8_t PD_ZERO_CROSS_INT0 = 2; // PD2 - INT0 (50/60Hz zero-crossing detection)
    constexpr uint8_t PD_RELAY_MOTOR_A   = 3; // PD3 - Motor reversal relay A / Heater
    constexpr uint8_t PD_RELAY_MOTOR_B   = 4; // PD4 - Motor reversal relay B (CW/CCW)
    constexpr uint8_t PD_RELAY_HEATER    = 5; // PD5 - Heater / field tap relay
    constexpr uint8_t PD_TACHO_ICP1      = 6; // PD6 - ICP1 (Tachogenerator pulse input)
    constexpr uint8_t PD_MUX_CTRL        = 7; // PD7 - Multiplexer control line
}

// ============================================================================
// 2. CONSTANT DATA TABLES IN FLASH MEMORY (PROGMEM) - DECODED FROM HEX
// ============================================================================

// Flash 0x0070 (code:0038): ADC channel sampling sequence across 10 steps
const uint8_t ADC_CHANNEL_MAP[10] PROGMEM = {
    6, 5, 0, 4, 1, 2, 7, 0xFF, 0xFF, 0xFF
};

// Flash 0x007A (code:003d): Output PWM modulation mask in a 10-step cycle
const uint8_t OUTPUT_DUTY_MASK[10] PROGMEM = {
    0x30, 0x3F, 0x3F, 0x3F, 0x38, 0x38, 0x30, 0x30, 0x30, 0x30
};

// Flash 0x0084 (code:0042): 100-point NTC sensor linearization lookup table (0..99 °C)
const uint8_t NTC_LOOKUP_TABLE[100] PROGMEM = {
    0x15, 0x16, 0x17, 0x19, 0x1A, 0x1B, 0x1C, 0x1E, 0x1F, 0x21, // 0..9 °C
    0x22, 0x24, 0x25, 0x27, 0x28, 0x2A, 0x2C, 0x2D, 0x2F, 0x30, // 10..19 °C
    0x32, 0x34, 0x36, 0x38, 0x3A, 0x3C, 0x3E, 0x40, 0x43, 0x45, // 20..29 °C (25°C = 0x3C)
    0x47, 0x49, 0x4B, 0x4E, 0x50, 0x52, 0x54, 0x57, 0x59, 0x5C, // 30..39 °C
    0x5E, 0x60, 0x63, 0x65, 0x68, 0x6A, 0x6C, 0x6F, 0x71, 0x74, // 40..49 °C
    0x76, 0x78, 0x7B, 0x7D, 0x80, 0x82, 0x84, 0x87, 0x89, 0x8C, // 50..59 °C (50°C = 0x76)
    0x8E, 0x90, 0x92, 0x95, 0x97, 0x99, 0x9B, 0x9D, 0x9F, 0xA1, // 60..69 °C
    0xA3, 0xA5, 0xA7, 0xA9, 0xAB, 0xAD, 0xAF, 0xB1, 0xB2, 0xB4, // 70..79 °C (75°C = 0xAD)
    0xB6, 0xB8, 0xB9, 0xBB, 0xBC, 0xBE, 0xBF, 0xC1, 0xC2, 0xC4, // 80..89 °C
    0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCF, 0xD0  // 90..99 °C
};

// Flash 0x00E8 (code:0074..007b.1): 16-fault configuration table
// Upper 2 bits (>>6) = target failure mode (0..3), lower 6 bits (& 0x3F) = LED blink pattern
const uint8_t FAULT_CONFIG_TABLE[16] PROGMEM = {
    0x00, // 0: No fault
    0x38, // 1: Triac/door lock circuit signal present while output is off (LED: 111000b)
    0x02, // 2: Temperature failed to rise by 2°C within 30 steps - heater fault (LED: 000010b)
    0x04, // 3: NTC sensor short / open circuit (LED: 000100b)
    0x50, // 4: Water fill timeout - no pressure switch signal (Mode 1, LED: 010000b)
    0xB1, // 5: Program selector dial ADC4 read error (Mode 2, LED: 110001b)
    0x98, // 6: EEPROM checksum error (Mode 2, LED: 011000b)
    0x15, // 7: Configuration / communication fault (LED: 010101b)
    0xF0, // 8: Water overflow / pressure switch fault (Mode 3, LED: 110000b)
    0xC5, // 9: Load distribution failed after 10 attempts - Unbalance (Mode 3, LED: 000101b)
    0xC7, // 10: Water inlet valve circuit fault (Mode 3, LED: 000111b)
    0x11, // 11: Spin substate 0x1C fault (LED: 010001b)
    0xDC, // 12: Motor triac short / breakdown (Mode 3, LED: 011100b)
    0xC8, // 13: Door lock engagement failed after 8 attempts (Mode 3, LED: 001000b)
    0xC3, // 14: No tachogenerator signal / locked motor after 3 attempts (Mode 3, LED: 000011b)
    0xC6  // 15: Water overheating >= 83°C (Mode 3, LED: 000110b)
};

// Flash 0x0D9F (code:06cf.1..070f): Table of 16 wash programs (8 bytes per program)
// Format: { Group, StartStep, EndStep, OptionMask, Reserved, MaxRPM_div10, TargetTemp_C, Flag }
const uint8_t WASH_PROGRAMS_FLASH[16][8] PROGMEM = {
    { 0x07, 0x18, 0x1D, 0x10, 0x00, 0x78, 0x00, 0x00 }, // Prog 1: Rinse + Spin 1200
    { 0x04, 0x03, 0x1D, 0x14, 0x00, 0x3D, 0x1E, 0x00 }, // Prog 2: Quick 30°C (600 RPM)
    { 0x05, 0x03, 0x1D, 0x14, 0x00, 0x78, 0x1E, 0x00 }, // Prog 3: Daily 30°C (1200 RPM)
    { 0x06, 0x03, 0x1D, 0x14, 0x00, 0x78, 0x28, 0x00 }, // Prog 4: Daily 40°C (1200 RPM)
    { 0x00, 0x03, 0x1D, 0x3F, 0x00, 0x78, 0x00, 0x00 }, // Prog 5: Cotton Cold (1200 RPM)
    { 0x00, 0x03, 0x1D, 0x3F, 0x00, 0x78, 0x28, 0x00 }, // Prog 6: Cotton 40°C (1200 RPM)
    { 0x00, 0x03, 0x1D, 0x3F, 0x00, 0x78, 0x3C, 0x00 }, // Prog 7: Cotton 60°C (1200 RPM)
    { 0x00, 0x03, 0x1D, 0x3F, 0x00, 0x78, 0x5A, 0x00 }, // Prog 8: Cotton 90°C (1200 RPM)
    { 0x01, 0x03, 0x1D, 0x3F, 0x00, 0x50, 0x3C, 0x00 }, // Prog 9: Synthetics 60°C (800 RPM)
    { 0x01, 0x03, 0x1D, 0x3F, 0x00, 0x50, 0x28, 0x00 }, // Prog 10: Synthetics 40°C (800 RPM)
    { 0x01, 0x03, 0x1D, 0x3F, 0x00, 0x50, 0x1E, 0x00 }, // Prog 11: Synthetics 30°C (800 RPM)
    { 0x01, 0x03, 0x1D, 0x3F, 0x00, 0x50, 0x00, 0x00 }, // Prog 12: Synthetics Cold (800 RPM)
    { 0x02, 0x03, 0x1D, 0x1E, 0x00, 0x3D, 0x1E, 0x00 }, // Prog 13: Delicates 30°C (600 RPM)
    { 0x03, 0x03, 0x1D, 0x1E, 0x00, 0x3D, 0x00, 0x00 }, // Prog 14: Wool Cold (600 RPM)
    { 0x03, 0x03, 0x1D, 0x1E, 0x00, 0x3D, 0x28, 0x00 }, // Prog 15: Wool 40°C (600 RPM)
    { 0x08, 0x1A, 0x1D, 0x10, 0x00, 0x78, 0x00, 0x00 }  // Prog 16: Spin Only (1200 RPM)
};

// Flash 0x0164 (code:00b2..0205): Spin Virtual Machine Bytecode (7-byte instructions)
// Opcodes: 0x01=Pause, 0x02=MotorMove(Hold, RPM_H, RPM_L, Ramp, Dir, Time), 0x03=UnbalanceCheck, 0x06=Pump, 0x07=End
const uint8_t SPIN_VM_SCRIPT_MAIN[] PROGMEM = {
    // Phase 1: Load distribution (52 RPM CW/CCW)
    0x02, 0x05, 0x00, 0x34, 0x05, 0x00, 0x01,
    0x02, 0x05, 0x00, 0x34, 0x05, 0x01, 0x01,
    0x02, 0x05, 0x00, 0x34, 0x05, 0x00, 0x01,
    0x02, 0x05, 0x00, 0x34, 0x05, 0x01, 0x01,
    // Phase 2: Ramping 65 -> 80 -> 100 RPM and balance check
    0x02, 0x00, 0x00, 0x41, 0x04, 0x00, 0x01,
    0x02, 0x00, 0x00, 0x50, 0x04, 0x00, 0x01,
    0x02, 0x00, 0x00, 0x64, 0x14, 0x00, 0x04,
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Enable drain pump
    0x03, 0x01, 0x4F, 0x03, 0x08, 0x02, 0x00, // Balance check threshold 0x034F
    0x03, 0x01, 0xCC, 0x03, 0x08, 0x04, 0x02, // Balance check threshold 0x03CC
    // Phase 3: Intermediate spin 300 -> 600 -> 800 -> 1000 -> 1200 RPM
    0x02, 0x00, 0x01, 0x2C, 0x01, 0x00, 0x0A, // 300 RPM
    0x02, 0x00, 0x02, 0x58, 0x3C, 0x00, 0x3C, // 600 RPM
    0x02, 0x00, 0x03, 0x20, 0x3C, 0x00, 0x14, // 800 RPM
    0x02, 0x00, 0x03, 0xE8, 0x64, 0x00, 0x14, // 1000 RPM
    0x02, 0x00, 0x04, 0xB0, 0xA0, 0x00, 0x14, // 1200 RPM
    // Phase 4: Drum coasting and end
    0x01, 0xB4, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Voltage thresholds for ADC0 button resistor ladder (from FUN_code_3d6c, addresses 0x072F and 0x0736)
const uint8_t BTN_ADC_LOW[7]  PROGMEM = { 0x10, 0x30, 0x50, 0x70, 0x95, 0xB5, 0xD5 };
const uint8_t BTN_ADC_HIGH[7] PROGMEM = { 0x28, 0x48, 0x68, 0x8E, 0xAE, 0xCE, 0xF5 };

// Thresholds for auxiliary dial ADC6 (from FUN_code_3cf6, address 0x0728)
const uint8_t AUX_DIAL_THRESHOLDS[7] PROGMEM = { 0x1E, 0x3C, 0x5A, 0x78, 0x96, 0xB4, 0xD2 };

// Wash step durations in minutes (copied in FUN_code_3915 to 0x0725)
const uint8_t STEP_DURATION_TABLE[29] PROGMEM = {
    0, 2, 1, 5, 25, 10, 5, 3, 2, 1, 3, 0, 0, 2, 3, 2, 0, 0, 2, 5, 4, 3, 2, 3, 4, 2, 0, 0, 6
};

// ============================================================================
// 3. SYSTEM DATA STRUCTURES (SRAM AND EEPROM)
// ============================================================================

// Single ADC channel statistics (8 bytes, base 0x033A)
struct AdcChannelStats {
    uint16_t avg_8bit;  // +0x00
    uint16_t sum_10bit; // +0x02
    uint16_t max_8bit;  // +0x04
    uint16_t min_8bit;  // +0x06
};

// 32-sample window for AC waveform shape classification (6 bytes: 0x038C and 0x0392)
struct SignalWindow {
    uint16_t sum_avg;
    uint16_t sum_min;
    uint16_t sum_max;
};

// 28-byte current wash cycle state stored in 5 redundant copies in EEPROM (0x0100..0x011B)
struct __attribute__((packed)) CycleStateRecord {
    uint8_t  step_elapsed_time; // 0x0100: Elapsed time in the current step
    uint8_t  system_mode;       // 0x0101: 0=Run, 1=Pause, 2=Ready/Stop, 3=Fault/End, 4=DoorLocking/Start
    uint8_t  error_index;       // 0x0102: Active fault index (0..15)
    uint8_t  config_flags_lo;   // 0x0103: Bits: 0=ChildLock, 2=DrainReq, 3=FillTimeout, 4=RinseHold
    uint8_t  program_group;     // 0x0104: Program group (0=Cotton, 1=Synthetics, 2=Delicates, 3=Wool...)
    uint8_t  user_options;      // 0x0105: User options (bits: 0=Prewash, 1=ExtraRinse, 2=Cold, 3=RinseHold, 4=Eco)
    uint8_t  target_temp_c;     // 0x0106: Target wash temperature (0, 30, 40, 60, 90)
    uint8_t  target_spin_div10; // 0x0107: Target spin speed / 10 (e.g., 120 = 1200 RPM)
    uint8_t  measured_fill_t;   // 0x0108: Measured water refill time
    uint8_t  agitation_cycles;  // 0x0109: Reversal agitation cycle counter
    uint8_t  agitation_sub;     // 0x010A: Agitation reversal substate (0..5)
    uint8_t  cycle_step;        // 0x010B: Current program step (0..28 or 0xFF=End)
    uint8_t  phase_substate;    // 0x010C: Phase substate (0, 1, 2, 3)
    uint8_t  unbalance_stage;   // 0x010D: Load distribution stage (0..3)
    uint8_t  program_offset;    // 0x010E: Program table offset ((prog-1)*8)
    uint8_t  saved_step_info;   // 0x010F: Saved rinse substep info
    uint8_t  fault_mask_lo;     // 0x0110: Fault bitmask low byte
    uint8_t  fault_mask_hi;     // 0x0111: Fault bitmask high byte
    uint16_t sub_timer_sec;     // 0x0112:0113: Subphase timer in seconds
    uint8_t  door_seq_state;    // 0x0114: Door/start sequence state
    uint8_t  selected_prog_id;  // 0x0115: Selected program on the dial (1..16)
    uint8_t  power_fail_flag;   // 0x0116: Power failure flag (7 = resume)
    uint8_t  saved_flags_0399;  // 0x0117: Saved copy of 0x0399 flags
    uint8_t  saved_flags_039a;  // 0x0118: Saved copy of 0x039A flags
    uint8_t  packed_nibbles;    // 0x0119: Packed rinse counters
    uint16_t total_cycles_cnt;  // 0x011A:011B: Total washing machine cycle counter
};

// 83-byte factory configuration record in EEPROM (initialized in FUN_code_3915, 0x011C..0x016E)
struct __attribute__((packed)) FactoryConfigRecord {
    uint8_t tacho_pulses;       // 0x011C: Tachogenerator constant (default 0xDA = 218)
    uint8_t spin_table_1[9];    // 0x011D..0x0125
    uint8_t spin_table_2[9];    // 0x0126..0x012E: {0, 30, 30, 40, 40, 60, 80, 100, 0xFE}
    uint8_t unused_012f;        // 0x012F
    uint8_t opt_led_mask[6];    // 0x0130..0x0135: {0x20, 0x02, 0x04, 0xFE, 0xFE, 0xFE}
    uint8_t max_temp_limit;     // 0x0136: Temperature limit (0x3C = 60°C)
    uint8_t unused_0137[3];     // 0x0137..0x0139
    uint8_t agitate_params[8];  // 0x013A..0x0141: {0x7A, 0x3F, 0x50, 0x2D, 0x40, 0x1F, 0x70, 0x17}
    uint8_t machine_model_type; // 0x0142: Machine model type (0..4, default 1)
    uint8_t hw_board_variant;   // 0x0143: PCB hardware variant (0 or 1)
    uint8_t ui_btn_layout;      // 0x0144: Button layout (0 or 1)
    uint8_t dial_positions_cfg; // 0x0145: Dial configuration (0/1 = 16 pos., 2 = 8 pos.)
    uint8_t reserved_block[14]; // 0x0146..0x0153
    uint8_t angle_min_power;    // 0x0154 -> 0x0471: Max triac delay / min power (0x99 = 153)
    uint8_t angle_max_wash;     // 0x0155 -> 0x046F: Min delay for washing (0x32 = 50)
    uint8_t angle_max_spin;     // 0x0156 -> 0x0470: Min delay for spinning (0x19 = 25)
    uint8_t angle_initial;      // 0x0157 -> 0x0472: Initial triac firing angle (0x7F = 127)
    uint8_t pid_wash_kp;        // 0x0158 -> 0x0417: Wash Kp (100)
    uint8_t pid_wash_ki;        // 0x0159 -> 0x0418: Wash Ki (80)
    uint8_t pid_wash_kd;        // 0x015A -> 0x0419: Wash Kd (128)
    uint8_t accel_limit_wash;   // 0x015B -> 0x0421: Wash acceleration limit (12)
    uint8_t pid_spin_kp;        // 0x015C -> 0x041A: Spin Kp (70)
    uint8_t pid_spin_ki;        // 0x015D -> 0x041B: Spin Ki (53)
    uint8_t pid_spin_kd;        // 0x015E -> 0x041C: Spin Kd (128)
    uint8_t accel_limit_spin;   // 0x015F -> 0x0420: Spin acceleration limit (12)
    uint8_t pid_dist_kp;        // 0x0160 -> 0x041D: Load distribution Kp (110)
    uint8_t pid_dist_ki;        // 0x0161 -> 0x041E: Load distribution Ki (70)
    uint8_t pid_dist_kd;        // 0x0162 -> 0x041F: Load distribution Kd (128)
    uint8_t temp_eco_60;        // 0x0163: Actual temperature for 60°C Eco (0x2F = 47°C)
    uint8_t temp_eco_40;        // 0x0164: Actual temperature for 40°C Eco (0x28 = 40°C)
    uint8_t temp_hysteresis;    // 0x0165: Thermostat hysteresis (1°C)
    uint8_t cold_wash_assist;   // 0x0166: Cold wash heating assist (0/1)
    uint8_t table_variant_flag; // 0x0167: Table variant flag (1)
    uint8_t max_agitate_reps;   // 0x0168: Max agitation cycle repetitions (5)
    uint8_t time_wash_60;       // 0x0169: Base 60°C wash time (0x41 = 65)
    uint8_t time_wash_60_short; // 0x016A: Shortened 60°C wash time (0x14 = 20)
    uint8_t time_wash_40;       // 0x016B: Base 40°C wash time (0x41 = 65)
    uint8_t time_wash_40_short; // 0x016C: Shortened 40°C wash time (0x14 = 20)
    uint8_t max_heater_cycles;  // 0x016D: Max heater activation cycles (8)
    uint8_t auto_start_mode;    // 0x016E: Auto-start mode (0)
};

// ============================================================================
// 4. CONTROLLER GLOBAL VARIABLES (RAM MAPPING)
// ============================================================================
CycleStateRecord    g_cycle;
FactoryConfigRecord g_cfg;

// ADC measurement and window filtering buffers
AdcChannelStats g_adc_channels[8];
uint8_t         g_adc_avg_bytes[8];         // 0x03C1..0x03C8 (low bytes of averages)
uint16_t        g_adc_sample_cnt    = 0;    // 0x037B:037C (0..80)
uint16_t        g_adc_timeout_cnt   = 1000; // 0x02EF:02F0
uint8_t         g_current_adc_ch    = 0;    // 0x037D
uint8_t         g_adc_ready_flags   = 0;    // 0x03BF

SignalWindow    g_win_ch1           = {0, 0, 0}; // 0x038C..0x0391 (Channel 1: DoorLock/Triac)
SignalWindow    g_win_ch2           = {0, 0, 0}; // 0x0392..0x0397 (Channel 2: Pump)
uint8_t         g_win_samples_ch1   = 32;        // 0x02F4
uint8_t         g_win_samples_ch2   = 32;        // 0x02F3
uint8_t         g_prev_out_bit3     = 0;         // 0x02F1
uint8_t         g_prev_out_bit4     = 0;         // 0x02F2
uint8_t         g_signal_state_ch1  = 1;         // 0x0387 (Classification result 0..3)
uint8_t         g_signal_state_ch2  = 1;         // 0x0388 (Classification result 0..3)

// Sensor variables: NTC Temperature, Pressure Switch (Hydrostat), Mains Voltage
uint8_t         g_temperature_c     = 20;   // 0x038B
uint8_t         g_ntc_valid_cnt     = 5;    // 0x0389
uint8_t         g_temp_snapshot     = 0;    // 0x03B8
uint8_t         g_temp_rise_timer   = 30;   // 0x039E
uint8_t         g_computed_target_t = 0;    // 0x04BB
uint8_t         g_mains_voltage_raw = 180;  // 0x037A
uint16_t        g_mains_fault_timer = 0;    // 0x03B1:03B2
uint8_t         g_water_level_state = 0;    // 0x0385 (0=Empty, 2=Full)
uint8_t         g_hydro_debounce    = 0;    // 0x0386
uint8_t         g_hydro_pulse_cnt   = 0;    // 0x0380

// Output variables and system flags
uint8_t         g_outputs_req_mask  = 0;    // 0x037E (Bits: 0=Prewash, 1=Main, 2=Soft, 3=Pump, 4=TriacEn, 5=Heater)
uint8_t         g_outputs_pwm_mask  = 0;    // 0x037F
uint8_t         g_flags_038a        = 4;    // 0x038A
uint8_t         g_status_flags_0399 = 0;    // 0x0399
uint8_t         g_comm_flags_039a   = 0;    // 0x039A
uint8_t         g_comm_flags_039b   = 0;    // 0x039B
uint16_t        g_latched_faults    = 0;    // 0x039C:039D
uint8_t         g_sys_flags_03be    = 0;    // 0x03BE
uint8_t         g_save_req_flags    = 0;    // 0x046B (bit 0=Save state to EEPROM, bit 1=Update time, bit 2=EEPROM fault)
uint8_t         g_wash_ctrl_04bc    = 0;    // 0x04BC

// Door lock and motor supervision counters (from FUN_code_0e18)
uint16_t        g_door_retry_timer  = 0;    // 0x03A1:03A2
uint16_t        g_door_unlock_timer = 0;    // 0x03A3:03A4
uint8_t         g_door_retry_cnt    = 0;    // 0x03A5 (0..8)
uint16_t        g_tacho_retry_timer = 0;    // 0x03A6:03A7
uint16_t        g_tacho_ok_timer    = 0;    // 0x03A8:03A9
uint8_t         g_tacho_retry_cnt   = 0;    // 0x03AA (0..3)
uint8_t         g_fault_slot_03ab   = 0;    // 0x03AB (Overheat)
uint8_t         g_fault_slot_03ad   = 0;    // 0x03AD (Door lock)
uint8_t         g_fault_slot_03ae   = 0;    // 0x03AE (Tacho loss)
uint8_t         g_fault_slot_03af   = 0;    // 0x03AF (Triac short)
uint8_t         g_fault_slot_03b0   = 0;    // 0x03B0 (Overflow)
uint8_t         g_triac_test_t1     = 0;    // 0x03B3
uint8_t         g_triac_test_t2     = 0;    // 0x03B4
uint8_t         g_overheat_t1       = 0;    // 0x03B5
uint8_t         g_overheat_t2       = 0;    // 0x03B6
uint8_t         g_unbalance_cnt     = 0;    // 0x04C2 (0..10)
uint16_t        g_redist_timer      = 0;    // 0x04BD:04BE

// Motor controller and PID regulator (0x03FB .. 0x0447)
uint8_t         g_motor_state       = 0;    // 0x03FB (0=Stop, 1..7=Motion phases)
uint8_t         g_motor_prev_state  = 0;    // 0x03DE
uint8_t         g_motor_flags       = 0;    // 0x03FF (bit 0=PID_Active, bit 1=DirCurr, bit 2=DirReq, bit 3=SpinMode)
uint8_t         g_motor_ctrl_flags  = 0;    // 0x0400 (bit 1=RunReq, bit 3=VM_Active, bit 4=StopDelay, bit 5=ZeroRPM)
uint16_t        g_motor_curr_rpm    = 0;    // 0x0403:0404
uint16_t        g_motor_tacho_med   = 0;    // 0x0405:0406
uint16_t        g_motor_target_rpm  = 0;    // 0x0407:0408
uint16_t        g_motor_ramped_rpm  = 0;    // 0x0409:040A
uint16_t        g_motor_hold_time   = 0;    // 0x040B:040C
uint16_t        g_motor_ramp_time   = 0;    // 0x040D:040E
uint16_t        g_motor_total_time  = 0;    // 0x040F:0410
uint8_t         g_req_rpm_arg       = 0;    // 0x0411
uint8_t         g_req_ramp_arg      = 0;    // 0x0413
uint8_t         g_req_time_arg      = 0;    // 0x0415
uint8_t         g_pid_sample_div    = 5;    // 0x0424
uint8_t         g_pid_sample_cnt    = 0;    // 0x0427
uint8_t         g_soft_start_flag   = 1;    // 0x0426
uint8_t         g_soft_start_step   = 0;    // 0x0428
uint8_t         g_unbal_sample_cnt  = 0;    // 0x0429
uint8_t         g_active_kp         = 100;  // 0x042A
uint8_t         g_active_ki         = 80;   // 0x042B
uint8_t         g_active_kd         = 128;  // 0x042C
int16_t         g_firing_angle_16   = 127;  // 0x042E:042F
uint8_t         g_firing_angle      = 127;  // 0x03FC
uint8_t         g_triac_timer_ticks = 60;   // 0x03F3
uint16_t        g_prev_rpm_snap     = 0;    // 0x0430:0431
int8_t          g_pid_err           = 0;    // 0x0432
int8_t          g_pid_prev_err      = 0;    // 0x0433
int8_t          g_pid_err_min       = -15;  // 0x0434
int8_t          g_pid_err_max       = 15;   // 0x0435
uint8_t         g_tacho_alive_timer = 0;    // 0x0437
uint8_t         g_field_weaken_cnt  = 0;    // 0x0438
uint16_t        g_ramp_steps_rem    = 0;    // 0x0439:043A
uint16_t        g_ramp_ticks_reload = 0;    // 0x043B:043C
uint16_t        g_ramp_ticks_cnt    = 0;    // 0x043D:043E
uint16_t        g_relay_switch_dly  = 0;    // 0x043F:0440
uint8_t         g_stop_relay_timer  = 0;    // 0x030A
uint8_t         g_zero_rpm_debounce = 10;   // 0x030D

// Spin Virtual Machine (VM)
uint16_t        g_vm_script_start   = 0;    // 0x0459:045A
uint16_t        g_vm_pc_offset      = 0;    // 0x045D:045E (Program Counter)
uint16_t        g_vm_step_timer     = 0;    // 0x0461

// User Interface (UI), LED matrix and Service Mode
uint8_t         g_service_mode_active = 0;  // 0x03BA (1 = Service Mode / Autotest)
uint8_t         g_service_test_step   = 0;  // 0x02F8 (0..99)
uint8_t         g_ui_state_flags      = 1;  // 0x03C0 (1=Normal, 2=Service, 0x10=ExitService)
uint8_t         g_btn_event_flags     = 0;  // 0x03BD (Bits: 0=StartShort, 1=ResetLong, 2=ChildLock, 3..5=Options, 6=ShowErr)
uint8_t         g_detected_btn_id     = 3;  // 0x04A5 (3 = no button pressed)
uint8_t         g_btn_released_flag   = 1;  // 0x0332
uint16_t        g_btn_hold_timer      = 0;  // 0x04B5:04B6
uint8_t         g_led_solid_mask      = 0;  // 0x04B7 (Solid ON LEDs)
uint8_t         g_led_blink_mask      = 0;  // 0x0331 (Turned OFF / blinking LEDs)
uint8_t         g_last_logged_err     = 0;  // 0x03CA
uint8_t         g_err_display_timer   = 0;  // 0x0306
uint8_t         g_active_slot_idx     = 0;  // 0x0330

// Interrupt variables (Zero-Crossing, Tachogenerator, Scheduler)
volatile uint8_t  g_zc_period_ticks   = 0x3F; // 0x0307 (50Hz half-wave period ~ 63 ticks)
volatile uint8_t  g_zc_filter_cnt     = 0;    // 0x03F1
volatile uint8_t  g_mux_slot_counter  = 0;    // 0x03F2 (0..239)
volatile uint16_t g_tacho_captures[3] = {0, 0, 0}; // 0x02EA..0x02EF
volatile uint8_t  g_sched_cnt_100ms   = 50;   // 0x03BB
volatile uint8_t  g_sched_cnt_200ms   = 85;   // 0x03BC
volatile uint8_t  g_wdt_checkin_flags = 0;    // 0x0772

// ============================================================================
// 5. EEPROM MEMORY SUBSYSTEM (FUN_code_3696, 36f0, 3726, 3872, 38db, 3915, 3cd2)
// ============================================================================

/**
 * Write a single byte to EEPROM with verification (mapped from FUN_code_3c5c)
 */
void EEPROM_WriteVerifiedByte(uint16_t addr, uint8_t val)
{
    for (uint8_t retry = 0; retry < 3; retry++) {
        if (eeprom_read_byte((const uint8_t *)addr) == val) {
            return;
        }
        eeprom_write_byte((uint8_t *)addr, val);
    }
}

/**
 * Read data packet from EEPROM: [0xCA] + [Data: len] + [0x55] + [Checksum] (FUN_code_3696 / 38db)
 */
bool EEPROM_ReadPacket(uint16_t base_addr, uint8_t *dest, uint8_t len)
{
    if (eeprom_read_byte((const uint8_t *)base_addr) != 0xCA) {
        return false;
    }

    uint8_t checksum = 0xCA;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = eeprom_read_byte((const uint8_t *)(base_addr + 1 + i));
        dest[i] = b;
        checksum += b;
    }

    checksum += 0x55;
    uint8_t footer = eeprom_read_byte((const uint8_t *)(base_addr + 1 + len));
    uint8_t stored_crc = eeprom_read_byte((const uint8_t *)(base_addr + 2 + len));

    return (footer == 0x55) && (stored_crc == checksum);
}

/**
 * Write data packet to EEPROM with 0xCA header, 0x55 footer and checksum (FUN_code_36f0)
 */
void EEPROM_WritePacket(uint16_t base_addr, const uint8_t *src, uint8_t len)
{
    uint8_t checksum = 0xCA;
    EEPROM_WriteVerifiedByte(base_addr, 0xCA);

    for (uint8_t i = 0; i < len; i++) {
        EEPROM_WriteVerifiedByte(base_addr + 1 + i, src[i]);
        checksum += src[i];
        wdt_reset();
    }

    checksum += 0x55;
    EEPROM_WriteVerifiedByte(base_addr + 1 + len, 0x55);
    EEPROM_WriteVerifiedByte(base_addr + 2 + len, checksum);
}

/**
 * Push a new fault code to the front of the 10-element FIFO queue in EEPROM 0x004F..0x0058 (FUN_code_3cd2)
 */
void EEPROM_LogErrorToFIFO(uint8_t err_code)
{
    for (uint8_t i = 9; i > 0; i--) {
        uint8_t prev = eeprom_read_byte((const uint8_t *)(0x004F + i - 1));
        EEPROM_WriteVerifiedByte(0x004F + i, prev);
    }
    EEPROM_WriteVerifiedByte(0x004F, err_code);
}

/**
 * Read the most recent fault from the FIFO queue (FUN_code_3cc4)
 */
uint8_t EEPROM_GetLastError()
{
    return eeprom_read_byte((const uint8_t *)0x004F);
}

/**
 * Save wash cycle state in 5 redundant copies in EEPROM spaced by 31 bytes (FUN_code_3872)
 */
void EEPROM_SaveCycleState(bool skip_active_slot)
{
    g_cycle.saved_flags_0399 = g_status_flags_0399;
    g_cycle.saved_flags_039a = g_comm_flags_039a;

    for (uint8_t slot = 0; slot < 5; slot++) {
        wdt_reset();
        if (!skip_active_slot || (slot != g_active_slot_idx && slot != (uint8_t)(g_active_slot_idx - 1))) {
            uint16_t addr = (uint16_t)slot * 0x1F;
            EEPROM_WritePacket(addr, (const uint8_t *)&g_cycle, sizeof(CycleStateRecord));
        }
    }
    g_save_req_flags &= ~0x01;
}

/**
 * Restore wash cycle state at startup from 5 redundant EEPROM copies (FUN_code_3726)
 */
void EEPROM_RestoreCycleState()
{
    g_sys_flags_03be &= ~0x02;
    bool restored = false;

    for (uint8_t attempt = 0; attempt < 5 && !restored; attempt++) {
        CycleStateRecord temp_slots[2];
        bool valid[5] = {false};

        for (uint8_t s = 0; s < 5; s++) {
            uint16_t addr = (uint16_t)s * 0x1F;
            valid[s] = EEPROM_ReadPacket(addr, (uint8_t *)&g_cycle, sizeof(CycleStateRecord));
            if (s > 0 && valid[s] && valid[s - 1]) {
                // Two adjacent copies are valid -> restore state!
                g_sys_flags_03be |= 0x02;
                g_active_slot_idx = s;
                restored = true;
                break;
            }
        }
    }

    if (!restored) {
        // Initialize default idle state (FUN_code_3c6c)
        memset(&g_cycle, 0, sizeof(CycleStateRecord));
        g_cycle.system_mode = 2; // Stop / Ready
        g_cycle.cycle_step  = 0xFF;
        g_active_slot_idx   = 0xFE;
    }

    g_status_flags_0399 = g_cycle.saved_flags_0399 & 0xEF;
    g_comm_flags_039a   = g_cycle.saved_flags_039a;
    g_last_logged_err   = EEPROM_GetLastError();
    g_cycle.total_cycles_cnt++;
}

/**
 * Load 83-byte factory configuration or initialize default settings (FUN_code_3915)
 */
void EEPROM_LoadFactoryConfiguration()
{
    // Attempt to read 83 bytes (0x53) from EEPROM calibration area (address 0x0100 in EEPROM)
    if (EEPROM_ReadPacket(0x0100, (uint8_t *)&g_cfg, sizeof(FactoryConfigRecord))) {
        g_sys_flags_03be |= 0x01;
    } else {
        // Exact default values hardcoded in FUN_code_3915:
        g_sys_flags_03be &= ~0x01;
        memset(&g_cfg, 0, sizeof(FactoryConfigRecord));

        g_cfg.tacho_pulses       = 0xDA; // 218
        g_cfg.max_temp_limit     = 0x3C; // 60°C
        g_cfg.spin_table_2[1]    = 0x1E; // 300 RPM
        g_cfg.spin_table_2[2]    = 0x1E;
        g_cfg.spin_table_2[3]    = 0x28; // 400 RPM
        g_cfg.spin_table_2[4]    = 0x28;
        g_cfg.spin_table_2[5]    = 0x3C; // 600 RPM
        g_cfg.spin_table_2[6]    = 0x50; // 800 RPM
        g_cfg.spin_table_2[7]    = 0x64; // 1000 RPM
        g_cfg.spin_table_2[8]    = 0xFE;
        g_cfg.max_heater_cycles  = 8;

        g_cfg.opt_led_mask[0]    = 0x20;
        g_cfg.opt_led_mask[1]    = 0x02;
        g_cfg.opt_led_mask[2]    = 0x04;
        g_cfg.opt_led_mask[3]    = 0xFE;
        g_cfg.opt_led_mask[4]    = 0xFE;
        g_cfg.opt_led_mask[5]    = 0xFE;

        g_cfg.angle_min_power    = 0x99; // 153
        g_cfg.angle_max_wash     = 0x32; // 50
        g_cfg.angle_max_spin     = 0x19; // 25
        g_cfg.angle_initial      = 0x7F; // 127

        g_cfg.pid_wash_kp        = 100;  // 0x64
        g_cfg.pid_wash_ki        = 80;   // 0x50
        g_cfg.pid_wash_kd        = 128;  // 0x80
        g_cfg.accel_limit_wash   = 12;   // 0x0C

        g_cfg.pid_spin_kp        = 70;   // 0x46
        g_cfg.pid_spin_ki        = 53;   // 0x35
        g_cfg.pid_spin_kd        = 128;  // 0x80
        g_cfg.accel_limit_spin   = 12;   // 0x0C

        g_cfg.pid_dist_kp        = 110;  // 0x6E
        g_cfg.pid_dist_ki        = 70;   // 0x46
        g_cfg.pid_dist_kd        = 128;  // 0x80

        g_cfg.temp_eco_60        = 0x2F; // 47°C
        g_cfg.temp_eco_40        = 0x28; // 40°C
        g_cfg.temp_hysteresis    = 1;
        g_cfg.machine_model_type = 1;
        g_cfg.hw_board_variant   = 0;
        g_cfg.table_variant_flag = 1;
        g_cfg.max_agitate_reps   = 5;
        g_cfg.time_wash_60       = 0x41; // 65 min
        g_cfg.time_wash_60_short = 0x14; // 20 min
        g_cfg.time_wash_40       = 0x41;
        g_cfg.time_wash_40_short = 0x14;
    }
}

// ============================================================================
// 6. LOW-LEVEL I/O CONTROL (DECODED FROM 093d..09ea)
// ============================================================================

uint8_t IO_Read_PD4() { return (PORTD & (1 << Pins::PD_RELAY_MOTOR_B)) ? 1 : 0; } // FUN_code_093d

uint8_t IO_Read_AuxOut() // FUN_code_0944
{
    if (g_cfg.hw_board_variant == 1) return (PORTD & (1 << Pins::PD_RELAY_MOTOR_A)) ? 1 : 0;
    else                             return (PORTD & (1 << Pins::PD_RELAY_HEATER))  ? 1 : 0;
}

void IO_Set_Aux1_High() // code:095f
{
    if (g_cfg.hw_board_variant == 1) PORTD |= (1 << Pins::PD_RELAY_MOTOR_A);
    else                             PORTD |= (1 << Pins::PD_RELAY_HEATER);
}

void IO_Set_Aux1_Low() // code:096b
{
    if (g_cfg.hw_board_variant == 1) PORTD &= ~(1 << Pins::PD_RELAY_MOTOR_A);
    else                             PORTD &= ~(1 << Pins::PD_RELAY_HEATER);
}

void IO_Set_Aux2_High() // code:0977
{
    if (g_cfg.hw_board_variant == 1) PORTC |= (1 << Pins::PC_RELAY_AUX2);
    else                             PORTD |= (1 << Pins::PD_RELAY_MOTOR_A);
}

void IO_Set_Aux2_Low() // code:0983
{
    if (g_cfg.hw_board_variant == 1) PORTC &= ~(1 << Pins::PC_RELAY_AUX2);
    else                             PORTD &= ~(1 << Pins::PD_RELAY_MOTOR_A);
}

void IO_Set_FieldWeaken_High() { if (g_cfg.hw_board_variant == 1) PORTD |=  (1 << Pins::PD_RELAY_HEATER); } // FUN_code_098f
void IO_Set_FieldWeaken_Low()  { if (g_cfg.hw_board_variant == 1) PORTD &= ~(1 << Pins::PD_RELAY_HEATER); } // FUN_code_0995

/**
 * PWM modulation of valves, pump and heater outputs in a 10-step cycle (code:099b)
 */
void IO_UpdateOutputsPWM()
{
    uint8_t step = g_mux_slot_counter % 10;
    uint8_t duty_mask = pgm_read_byte(&OUTPUT_DUTY_MASK[step]);

    g_outputs_pwm_mask = g_outputs_req_mask & duty_mask;
    if (!(g_flags_038a & 0x04)) {
        g_outputs_pwm_mask = g_outputs_req_mask & 0x10;
    }

    if (g_outputs_pwm_mask & (1 << 0)) PORTB |= (1 << Pins::PB_VALVE_PREWASH);  else PORTB &= ~(1 << Pins::PB_VALVE_PREWASH);
    if (g_outputs_pwm_mask & (1 << 1)) PORTC |= (1 << Pins::PC_VALVE_MAIN);     else PORTC &= ~(1 << Pins::PC_VALVE_MAIN);
    if (g_outputs_pwm_mask & (1 << 2)) PORTB |= (1 << Pins::PB_VALVE_SOFTENER); else PORTB &= ~(1 << Pins::PB_VALVE_SOFTENER);
    if (g_outputs_pwm_mask & (1 << 3)) PORTC |= (1 << Pins::PC_DRAIN_PUMP);     else PORTC &= ~(1 << Pins::PC_DRAIN_PUMP);
    if (g_outputs_pwm_mask & (1 << 4)) PORTC |= (1 << Pins::PC_MOTOR_TRIAC);    else if (g_motor_state == 0) PORTC &= ~(1 << Pins::PC_MOTOR_TRIAC);
    if (g_outputs_pwm_mask & (1 << 5)) IO_Set_Aux1_High();                      else IO_Set_Aux1_Low();
}

// ============================================================================
// 7. ADC SUBSYSTEM, AC WAVEFORM CLASSIFICATION, NTC AND PRESSURE SWITCH (079b..0d3f)
// ============================================================================

uint16_t ADC_ReadAndStartNext(uint8_t next_slot_idx) // FUN_code_0867
{
    while (ADCSRA & (1 << ADSC)) {
        if (g_adc_timeout_cnt != 0) {
            g_adc_timeout_cnt--;
        } else {
            g_adc_timeout_cnt = 1000;
            break;
        }
    }

    uint16_t result = ADC;
    if (next_slot_idx >= 10) next_slot_idx = 0;

    uint8_t mux_ch = pgm_read_byte(&ADC_CHANNEL_MAP[next_slot_idx]);
    if (mux_ch == 0xFF) {
        mux_ch = pgm_read_byte(&ADC_CHANNEL_MAP[0]);
    }

    ADMUX  = (1 << REFS0) | (mux_ch & 0x07);
    ADCSRA |= (1 << ADSC);
    return result;
}

void ADC_ResetChannels() // FUN_code_20fe
{
    ADCSRA |= (1 << ADSC);
    ADC_ReadAndStartNext(0);
    g_adc_sample_cnt = 0;

    for (uint8_t i = 0; i < 8; i++) {
        g_adc_channels[i].avg_8bit  = 0;
        g_adc_channels[i].sum_10bit = 0;
        g_adc_channels[i].max_8bit  = 0;
        g_adc_channels[i].min_8bit  = 0x00FF;
    }
    g_adc_ready_flags &= ~0x08;
}

void ADC_ProcessSamplingCycle() // FUN_code_079b
{
    if (g_adc_sample_cnt >= 80) return;

    uint8_t slot = g_mux_slot_counter % 10;
    uint8_t skip = 0;

    if (g_adc_sample_cnt == 0 && slot != 0) {
        ADC_ResetChannels();
        skip = 0xFF;
    } else {
        g_adc_sample_cnt++;
    }

    g_current_adc_ch = pgm_read_byte(&ADC_CHANNEL_MAP[slot]);
    if (g_current_adc_ch != 0xFF && skip != 0xFF) {
        uint16_t raw = ADC_ReadAndStartNext(slot + 1);
        AdcChannelStats &ch = g_adc_channels[g_current_adc_ch];

        ch.sum_10bit += raw;
        uint16_t val8 = raw >> 2;
        if (ch.max_8bit < val8) ch.max_8bit = val8;
        if (val8 < ch.min_8bit) ch.min_8bit = val8;
    }

    if (g_adc_sample_cnt == 80) {
        g_adc_sample_cnt = 0;
        for (uint8_t i = 0; i < 8; i++) {
            g_adc_channels[i].avg_8bit  = g_adc_channels[i].sum_10bit >> 5;
            g_adc_avg_bytes[i]          = (uint8_t)g_adc_channels[i].avg_8bit;
            g_adc_channels[i].sum_10bit = 0;
        }
        g_adc_ready_flags |= 0x18;
    }
}

bool ADC_CheckChannelStability(uint8_t ch_idx, uint8_t max_ripple) // FUN_code_08ab
{
    AdcChannelStats &ch = g_adc_channels[ch_idx];
    bool stable = false;

    if ((ch.min_8bit + max_ripple >= ch.max_8bit) &&
        (ch.max_8bit >= ch.avg_8bit) &&
        (ch.avg_8bit >= ch.min_8bit) &&
        (g_adc_ready_flags & 0x08)) {
        stable = true;
    }

    ch.max_8bit = 0x0000;
    ch.min_8bit = 0x00FF;
    return stable;
}

/**
 * AC waveform shape classification on channels 1 and 2 (FUN_code_0b68):
 * 1 = full AC wave (0..255 amplitude around midpoint), 2 = half-wave (diode), 3 = ~2.5V DC level, 0 = none
 */
uint8_t Signal_ClassifyWindow(const SignalWindow *win) // FUN_code_0b68
{
    uint8_t avg = (uint8_t)win->sum_avg;
    if (avg > 0x62 && avg < 0x9E) {
        if (win->sum_min < 0x001E && win->sum_max >= 0x00E2) return 1; // Full AC waveform
        if (win->sum_min >= 0x0063 && win->sum_max < 0x009E) return 3; // Steady mid-scale DC level
        return 0;
    } else if ((avg > 0x22 && avg < 0x5E) || (avg > 0xA2 && avg < 0xDE)) {
        return 2; // Half-wave rectified signal (diode)
    }
    return 0;
}

void Signal_CheckOutputChangeAndResetWindows() // FUN_code_0b9a
{
    uint8_t bit3 = g_outputs_req_mask & 0x08;
    if (g_prev_out_bit3 != bit3 || (g_flags_038a & 0x04)) {
        g_prev_out_bit3            = bit3;
        g_win_samples_ch2          = 32;
        g_signal_state_ch2         = 0;
        g_win_ch2                  = {0, 0, 0};
        g_adc_channels[2].min_8bit = 0x00FF;
        g_adc_channels[2].max_8bit = 0x0000;
    }

    uint8_t bit4 = g_outputs_req_mask & 0x10;
    if (g_prev_out_bit4 != bit4) {
        g_prev_out_bit4            = bit4;
        g_win_samples_ch1          = 32;
        g_signal_state_ch1         = 0;
        g_win_ch1                  = {0, 0, 0};
        g_adc_channels[1].min_8bit = 0x00FF;
        g_adc_channels[1].max_8bit = 0x0000;
    }
}

void Signal_AccumulateWindows() // FUN_code_0c00
{
    if (!(g_adc_ready_flags & 0x08)) return;

    if (g_win_samples_ch2 != 0) {
        g_win_ch2.sum_avg += g_adc_avg_bytes[2];
        g_win_ch2.sum_min += g_adc_channels[2].min_8bit;
        g_win_ch2.sum_max += g_adc_channels[2].max_8bit;
        g_win_samples_ch2--;
    } else {
        g_win_ch2.sum_avg >>= 5;
        g_win_ch2.sum_min >>= 5;
        g_win_ch2.sum_max >>= 5;
        g_signal_state_ch2 = Signal_ClassifyWindow(&g_win_ch2);
        g_win_samples_ch2  = 32;
        g_win_ch2          = {0, 0, 0};
    }

    if (g_win_samples_ch1 != 0) {
        g_win_ch1.sum_avg += g_adc_avg_bytes[1];
        g_win_ch1.sum_min += g_adc_channels[1].min_8bit;
        g_win_ch1.sum_max += g_adc_channels[1].max_8bit;
        g_win_samples_ch1--;
    } else {
        g_win_ch1.sum_avg >>= 5;
        g_win_ch1.sum_min >>= 5;
        g_win_ch1.sum_max >>= 5;
        g_signal_state_ch1 = Signal_ClassifyWindow(&g_win_ch1);
        g_win_samples_ch1  = 32;
        g_win_ch1          = {0, 0, 0};
    }

    g_adc_channels[2].min_8bit = 0x00FF;
    g_adc_channels[2].max_8bit = 0x0000;
    g_adc_channels[1].min_8bit = 0x00FF;
    g_adc_channels[1].max_8bit = 0x0000;
}

void NTC_UpdateTemperature() // FUN_code_0cee
{
    uint8_t raw = g_adc_avg_bytes[5];
    if (!ADC_CheckChannelStability(5, 5)) return;

    if (raw < 0x05 || raw > 0xD2) {
        if (g_ntc_valid_cnt != 0) g_ntc_valid_cnt--;
        return;
    }

    g_ntc_valid_cnt = 5;
    uint8_t idx_start, idx_end;

    if (raw < pgm_read_byte(&NTC_LOOKUP_TABLE[25]))      { idx_start = 0;  idx_end = 25; }
    else if (raw < pgm_read_byte(&NTC_LOOKUP_TABLE[50])) { idx_start = 25; idx_end = 50; }
    else if (raw < pgm_read_byte(&NTC_LOOKUP_TABLE[75])) { idx_start = 50; idx_end = 75; }
    else                                                 { idx_start = 75; idx_end = 99; }

    uint8_t t = idx_start;
    while (t <= idx_end && raw >= pgm_read_byte(&NTC_LOOKUP_TABLE[t])) {
        t++;
    }
    g_temperature_c = t;
}

/**
 * Digital pressure switch (hydrostat) input handling on PINC0 (FUN_code_09eb + code:0a2c..0a66)
 */
void Hydrostat_UpdateState()
{
    // Fast pulse/state counting on PINC0 (code:0a30 sbis PINC, 0)
    if (PINC & (1 << Pins::PC_HYDROSTAT_IN)) {
        if (g_hydro_pulse_cnt < 240) g_hydro_pulse_cnt += 3;
    } else {
        if (g_hydro_pulse_cnt > 0) g_hydro_pulse_cnt--;
    }

    // Debouncing and water level state evaluation in g_water_level_state (FUN_code_09eb)
    if (!(g_motor_ctrl_flags & 0x20)) {
        g_hydro_debounce = 0;
    } else {
        if (g_hydro_debounce == 0) {
            g_hydro_debounce = 20;
        } else if (g_hydro_debounce > 1) {
            g_hydro_debounce--;
        } else {
            if (g_status_flags_0399 & 0x10) {
                if (g_hydro_pulse_cnt < 50) g_status_flags_0399 &= ~0x10;
                else                        g_water_level_state = 0;
            } else {
                g_water_level_state = (g_hydro_pulse_cnt >= 201) ? 2 : 0;
            }
        }
    }
}

// ============================================================================
// 8. HARDWARE INTERRUPTS AND PHASE-ANGLE MOTOR CONTROLLER WITH PID (24a2..2dda)
// ============================================================================

/**
 * Calculate triac firing delay from zero-crossing (FUN_code_2578)
 */
void Motor_CalcTriacDelay()
{
    if (!(g_motor_flags & 0x01)) {
        g_firing_angle = g_cfg.angle_initial;
    }
    // Convert phase angle to timer ticks (in original firmware: division by 18 in FUN_code_2578)
    g_triac_timer_ticks = (uint8_t)(((uint16_t)g_firing_angle * 7) / 18);
}

/**
 * INT0 Vector: Mains voltage zero-crossing detection (FUN_code_24a2)
 */
ISR(INT0_vect)
{
    uint8_t low_count = 0;
    for (uint8_t i = 0; i < 5; i++) {
        if ((PIND & (1 << Pins::PD_ZERO_CROSS_INT0)) == 0) {
            low_count++;
        }
    }

    if (low_count > 2 && g_zc_filter_cnt > 0x11 && (!(g_status_flags_0399 & 0x02) || g_service_mode_active != 0)) {
        g_flags_038a    &= ~0x04;
        g_zc_filter_cnt  = 0;

        // Mains frequency adaptation (50/60 Hz) within window 59..67 (0x3B..0x43)
        if (g_mux_slot_counter < 0x13)      g_zc_period_ticks--;
        else if (g_mux_slot_counter >= 0x14) g_zc_period_ticks++;

        if (g_zc_period_ticks < 0x3B) g_zc_period_ticks = 0x3B;
        if (g_zc_period_ticks > 0x43) g_zc_period_ticks = 0x43;

        g_outputs_pwm_mask = 0;
        Motor_CalcTriacDelay();

        // Field weakening control for high spin speeds in board variant 1
        if (g_cfg.hw_board_variant == 1) {
            if (g_field_weaken_cnt > 1) {
                g_field_weaken_cnt--;
                g_triac_timer_ticks = 0x14;
            }
            if (g_field_weaken_cnt == 3) {
                IO_Set_FieldWeaken_Low();
            }
        }

        // Arm Timer0 comparator to trigger the motor triac gate pulse (PC1)
        if (g_motor_state >= 4 && g_motor_target_rpm > 0) {
            OCR0A  = TCNT0 + g_triac_timer_ticks;
            TIFR0 |= (1 << OCF0A);
            TIMSK0 |= (1 << OCIE0A);
        }
    }
}

/**
 * TIMER0_COMPA Vector: Motor triac gate trigger pulse on pin PC1 (code:21a0)
 */
ISR(TIMER0_COMPA_vect)
{
    if (g_motor_state >= 4 && g_motor_target_rpm > 0) {
        PORTC |= (1 << Pins::PC_MOTOR_TRIAC);
        // Short gate pulse (~50 us)
        for (volatile uint8_t d = 0; d < 60; d++);
        PORTC &= ~(1 << Pins::PC_MOTOR_TRIAC);
    }
    TIMSK0 &= ~(1 << OCIE0A);
}

/**
 * TIMER1_CAPT Vector: Tachogenerator pulse period capture on pin PD6 (code:22fa)
 */
ISR(TIMER1_CAPT_vect)
{
    static uint16_t prev_icr1 = 0;
    static uint8_t  cap_idx   = 0;

    uint16_t current_icr1 = ICR1;
    g_tacho_captures[cap_idx] = current_icr1 - prev_icr1;
    prev_icr1 = current_icr1;

    if (++cap_idx >= 3) cap_idx = 0;
    g_tacho_alive_timer = 10;
}

/**
 * Motor direction relay switching CW / CCW / STOP (FUN_code_25b9, 25fe, 2643)
 */
void Motor_RelaysCW() // FUN_code_25b9
{
    PORTD &= ~(1 << Pins::PD_RELAY_MOTOR_B);
    IO_Set_Aux2_High();
    g_motor_ctrl_flags = (g_motor_ctrl_flags & 0x7F) | 0x40;
}

void Motor_RelaysCCW() // FUN_code_25fe
{
    IO_Set_Aux2_Low();
    PORTD |= (1 << Pins::PD_RELAY_MOTOR_B);
    g_motor_ctrl_flags = (g_motor_ctrl_flags & 0xBF) | 0x80;
}

void Motor_RelaysStop() // FUN_code_2643
{
    PORTC &= ~(1 << Pins::PC_MOTOR_TRIAC);
    PORTD &= ~(1 << Pins::PD_RELAY_MOTOR_B);
    IO_Set_Aux2_Low();
    g_motor_ctrl_flags &= 0x3F;
    if (g_cfg.hw_board_variant == 1) {
        IO_Set_FieldWeaken_High();
    }
}

/**
 * Median filter of 3 tachogenerator samples (FUN_code_43de) and RPM calculation (FUN_code_2b35)
 */
uint16_t Motor_Median3(uint16_t a, uint16_t b, uint16_t c)
{
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

void Motor_UpdateRPM() // FUN_code_2b35
{
    if (g_tacho_alive_timer == 0) {
        g_motor_curr_rpm = 0;
        return;
    }

    cli();
    uint16_t s0 = g_tacho_captures[0];
    uint16_t s1 = g_tacho_captures[1];
    uint16_t s2 = g_tacho_captures[2];
    sei();

    g_motor_tacho_med = Motor_Median3(s0, s1, s2);
    if (g_motor_tacho_med != 0 && g_cfg.tacho_pulses != 0) {
        // Mapped from 32-bit division FUN_code_56ad in FUN_code_2b35
        uint32_t num = 6000000UL / g_motor_tacho_med;
        g_motor_curr_rpm = (uint16_t)(num / g_cfg.tacho_pulses);
    } else {
        g_motor_curr_rpm = 0;
    }
}

/**
 * PID gain selection depending on operating mode: Wash, Spin Ramp, Spin Hold, Distribution (FUN_code_2ad9)
 */
void Motor_SelectPIDGains()
{
    if (!(g_motor_flags & 0x08)) {
        // Wash mode (low speed)
        g_active_kp      = g_cfg.pid_wash_kp;
        g_active_ki      = g_cfg.pid_wash_ki;
        g_active_kd      = g_cfg.pid_wash_kd;
        g_pid_sample_div = 5;
        g_pid_err_min    = -15;
        g_pid_err_max    = 15;
    } else {
        // Spin mode
        if (g_motor_target_rpm == g_motor_ramped_rpm) {
            g_active_kp = g_cfg.pid_spin_kp;
            g_active_ki = g_cfg.pid_spin_ki;
        } else {
            g_active_kp = 35; // 0x23
            g_active_ki = 27; // 0x1B
        }
        g_active_kd      = g_cfg.pid_spin_kd;
        g_pid_sample_div = 10;
        g_pid_err_min    = -100;
        g_pid_err_max    = 100;
    }

    if (g_motor_flags & 0x30) {
        // Load distribution mode
        g_active_kp      = g_cfg.pid_dist_kp;
        g_active_ki      = g_cfg.pid_dist_ki;
        g_active_kd      = g_cfg.pid_dist_kd;
        g_pid_sample_div = 5;
    }
}

/**
 * PID controller step calculation, slew rate limiting and acceleration compensation (FUN_code_2b5f..2dc1)
 */
void Motor_ExecutePIDTask() // FUN_code_2dc1
{
    if (++g_pid_sample_cnt < g_pid_sample_div) return;
    g_pid_sample_cnt = 0;

    Motor_SelectPIDGains();
    Motor_UpdateRPM();

    // 1. Control error clamped to [g_pid_err_min .. g_pid_err_max] (FUN_code_2b5f)
    int16_t diff = (int16_t)g_motor_ramped_rpm - (int16_t)g_motor_curr_rpm;
    if (diff > g_pid_err_max) diff = g_pid_err_max;
    if (diff < g_pid_err_min) diff = g_pid_err_min;
    g_pid_err = (int8_t)diff;

    // 2. Incremental PID equation (FUN_code_2ba9)
    int16_t p_term = (int16_t)g_pid_err * (int16_t)g_active_kp;
    int16_t i_term = (int16_t)g_pid_prev_err * (int16_t)g_active_ki;
    g_pid_prev_err = g_pid_err;

    int16_t delta = (p_term - i_term) / (int16_t)(g_active_kd ? g_active_kd : 1);
    if (delta >  125) delta =  125;
    if (delta < -125) delta = -125;

    if (g_motor_flags & 0x08) {
        if (g_motor_curr_rpm < 90 && delta < -3) delta = -3;
        if (g_motor_ramped_rpm == g_motor_target_rpm) {
            if (delta >  5) delta =  5;
            if (delta < -5) delta = -5;
        }
    }

    g_firing_angle_16 = (int16_t)g_firing_angle - delta;

    // Clamp triac firing angle between min power and max power angles
    if (g_firing_angle_16 > g_cfg.angle_min_power) {
        g_firing_angle_16 = g_cfg.angle_min_power;
    }
    uint8_t min_angle = (g_motor_flags & 0x08) ? g_cfg.angle_max_spin : g_cfg.angle_max_wash;
    if (g_firing_angle_16 < min_angle) {
        g_firing_angle_16 = min_angle;
    }

    // 3. Sudden RPM jump damping (anti-surge in FUN_code_2cc6)
    uint8_t check_period = (g_motor_flags & 0x08) ? 10 : 2;
    if (++g_unbal_sample_cnt > check_period) {
        g_unbal_sample_cnt = 0;
        if (g_motor_curr_rpm > g_prev_rpm_snap) {
            uint16_t jump = g_motor_curr_rpm - g_prev_rpm_snap;
            uint8_t  thr  = (g_motor_flags & 0x08) ? 20 : 0;
            if (jump > thr) {
                uint8_t gain = (g_motor_flags & 0x08) ? g_cfg.accel_limit_spin : g_cfg.accel_limit_wash;
                g_firing_angle_16 += ((uint16_t)jump * gain) >> 6;
                if (g_firing_angle_16 > g_cfg.angle_min_power) {
                    g_firing_angle_16 = g_cfg.angle_min_power;
                }
            }
        }
        g_prev_rpm_snap = g_motor_curr_rpm;
    }

    // 4. Soft-start over the first 5 steps after activation (FUN_code_2d99)
    if (g_soft_start_flag == 0) {
        g_firing_angle = (uint8_t)g_firing_angle_16;
    } else {
        if (++g_soft_start_step < 5) {
            g_firing_angle--;
        } else {
            g_soft_start_flag = 0;
        }
    }
}

/**
 * Stop motor and reset state variables (FUN_code_29ff and FUN_code_2a4d)
 */
void Motor_StopAndReset() // FUN_code_29ff
{
    g_motor_ramped_rpm = 0;
    g_tacho_alive_timer = 0;
    g_field_weaken_cnt = 0;
    g_motor_flags     &= ~0x01;
    g_stop_relay_timer = 1;
    g_motor_prev_state = g_motor_state;
    g_motor_state      = 0;
    g_motor_target_rpm = 0;
    g_motor_ctrl_flags |= 0x10;
}

void Motor_InitPIDState() // FUN_code_2a16
{
    g_firing_angle      = g_cfg.angle_initial;
    g_soft_start_flag   = 1;
    g_soft_start_step   = 0;
    g_pid_sample_cnt    = 0;
    g_unbal_sample_cnt  = 0;
    g_motor_ramped_rpm  = 0;
    g_tacho_alive_timer = 10;
    g_motor_flags       = (g_motor_flags & 0x8F) | 0x01;
}

/**
 * Command motor motion converting durations in seconds to 200ms ticks (FUN_code_268a)
 */
void Motor_CommandMove(uint16_t target_rpm, uint8_t hold_s, uint8_t ramp_s, uint8_t total_s)
{
    g_motor_total_time = (uint16_t)total_s * 10;
    g_motor_ramp_time  = (uint16_t)ramp_s  * 10;
    g_motor_hold_time  = (uint16_t)hold_s  * 10;
    if (g_motor_hold_time >= 14) {
        g_motor_hold_time -= 13;
    }

    if (g_cycle.target_spin_div10 != 0) {
        uint16_t max_allowed = (uint16_t)g_cycle.target_spin_div10 * 10;
        if (target_rpm > max_allowed) {
            target_rpm = max_allowed;
        }
    }

    g_motor_target_rpm = target_rpm;
    if (g_motor_target_rpm != 0) {
        g_motor_prev_state = g_motor_state;
        g_motor_state      = 1;
    }
}

/**
 * 7-state motor state machine called every 200 ms (FUN_code_2714)
 */
void Motor_StateMachine_200ms()
{
    switch (g_motor_state) {
        case 1: // Prepare direction relay switching
            if (g_motor_ctrl_flags & 0x04) {
                g_motor_hold_time  = 0;
                g_motor_ctrl_flags &= ~0x04;
            }
            g_relay_switch_dly = 0;
            // If changing direction CW <-> CCW, force stop and wait 2 cycles
            if (((g_motor_flags >> 1) & 1) != ((g_motor_flags >> 2) & 1)) {
                g_relay_switch_dly = 2;
                g_motor_ramped_rpm = 0;
                g_motor_flags     &= ~0x01;
            }
            g_motor_prev_state = g_motor_state;
            g_motor_state      = 2;
            break;

        case 2: // Wait for triac turn-off and release relay contacts
            if (g_relay_switch_dly == 0) {
                g_motor_prev_state = g_motor_state;
                g_motor_state      = 4;
                if (g_motor_flags & 0x02) Motor_RelaysCCW();
                else                      Motor_RelaysCW();
            } else {
                g_relay_switch_dly--;
                if (g_relay_switch_dly == 0) {
                    Motor_RelaysStop();
                    g_motor_flags = (g_motor_flags & 0xFD) | (((g_motor_flags >> 2) & 1) << 1);
                    g_relay_switch_dly = 2;
                    g_motor_prev_state = g_motor_state;
                    g_motor_state      = 3;
                }
            }
            break;

        case 3: // Relay contact switching dead-time (wait until RPM == 0)
            if (g_relay_switch_dly > 0) g_relay_switch_dly--;
            if (g_relay_switch_dly == 0 && (g_motor_ctrl_flags & 0x20)) {
                if (g_motor_flags & 0x02) Motor_RelaysCCW();
                else                      Motor_RelaysCW();
                g_motor_prev_state = g_motor_state;
                g_motor_state      = 4;
            }
            break;

        case 4: // Calculate number of speed ramp steps
            g_motor_prev_state = g_motor_state;
            g_motor_state      = (g_motor_hold_time == 0) ? 6 : 5;

            if (g_motor_target_rpm == 0) {
                Motor_StopAndReset();
            } else if (g_motor_target_rpm < g_motor_ramped_rpm) {
                g_ramp_steps_rem    = 0;
                g_ramp_ticks_reload = 0;
                g_ramp_ticks_cnt    = 0;
                g_motor_ramped_rpm  = g_motor_target_rpm;
            } else {
                uint16_t delta_rpm = g_motor_target_rpm - g_motor_ramped_rpm;
                uint16_t step_div  = (g_motor_flags & 0x08) ? 10 : 4;
                g_ramp_steps_rem   = (delta_rpm / step_div) + 1;
                if (g_ramp_steps_rem > 1) g_ramp_steps_rem--;

                g_ramp_ticks_reload = (g_ramp_steps_rem > 0) ? (g_motor_ramp_time / g_ramp_steps_rem) : 1;
                if (g_ramp_ticks_reload == 0) g_ramp_ticks_reload = 1;
                g_ramp_ticks_cnt    = g_ramp_ticks_reload;
            }
            break;

        case 5: // Initial speed hold phase (Hold)
            if (g_motor_hold_time == 0) g_motor_state = 6;
            else                        g_motor_hold_time--;
            break;

        case 6: // Linear speed ramp-up phase (Ramp)
            if (g_ramp_ticks_cnt > 0) {
                g_ramp_ticks_cnt--;
                if (g_ramp_ticks_cnt == 0) {
                    g_ramp_ticks_cnt = g_ramp_ticks_reload;
                    if (g_ramp_steps_rem == 0) {
                        g_motor_prev_state = g_motor_state;
                        g_motor_state      = 7;
                        g_motor_ramped_rpm = g_motor_target_rpm;
                    } else {
                        g_ramp_steps_rem--;
                        if (!(g_motor_flags & 0x01)) {
                            Motor_InitPIDState();
                        }
                        g_motor_ramped_rpm += (g_motor_flags & 0x08) ? 10 : 4;
                        if (g_motor_ramped_rpm > g_motor_target_rpm) {
                            g_motor_ramped_rpm = g_motor_target_rpm;
                        }
                    }
                }
            }
            break;

        case 7: // Target speed hold phase (Plateau)
            if (g_motor_total_time > 0) {
                g_motor_total_time--;
                if (g_motor_total_time == 0) {
                    Motor_StopAndReset();
                }
            }
            break;
    }
}

// ============================================================================
// 9. SPIN VIRTUAL MACHINE (BYTECODE INTERPRETER) (FUN_code_3093, 3534, 356a)
// ============================================================================

void SpinVM_Stop() // FUN_code_356a
{
    Motor_StopAndReset();
    g_vm_script_start  = 0;
    g_vm_pc_offset     = 0;
    g_motor_flags     &= ~0x08;
    g_motor_ctrl_flags &= ~0x08;
}

void SpinVM_StartScript(uint16_t script_offset) // FUN_code_3534
{
    g_vm_script_start   = script_offset;
    g_vm_pc_offset      = script_offset;
    g_vm_step_timer     = 0;
    g_motor_ctrl_flags |= 0x08;
}

/**
 * Interpreter for 7-byte spin instructions from PROGMEM (FUN_code_3093)
 */
void SpinVM_ExecuteStep()
{
    if (!(g_motor_ctrl_flags & 0x08)) return;

    // If motor is still executing previous command or VM timer pause is active, wait
    if (g_motor_state != 0) return;
    if (g_vm_step_timer > 0) {
        g_vm_step_timer--;
        return;
    }

    uint8_t ins[7];
    for (uint8_t i = 0; i < 7; i++) {
        ins[i] = pgm_read_byte(&SPIN_VM_SCRIPT_MAIN[g_vm_pc_offset + i]);
    }
    g_vm_pc_offset += 7;

    uint8_t opcode = ins[0];
    switch (opcode) {
        case 0x01: // PAUSE / COASTING: ins[1] = duration
            Motor_StopAndReset();
            g_vm_step_timer = ins[1];
            break;

        case 0x02: // MOTOR MOTION: ins[1]=Hold, ins[2:3]=RPM_BE, ins[4]=Ramp, ins[5]=Dir, ins[6]=Time
        {
            uint16_t rpm = ((uint16_t)ins[2] << 8) | ins[3];
            if (ins[5]) g_motor_flags |= 0x04;
            else        g_motor_flags &= ~0x04;
            if (rpm >= 300) g_motor_flags |= 0x08; // High-speed spin mode flag
            else            g_motor_flags &= ~0x08;
            Motor_CommandMove(rpm, ins[1], ins[4], ins[6]);
            break;
        }

        case 0x03: // DRUM UNBALANCE CHECK
        {
            uint16_t threshold = ((uint16_t)ins[2] << 8) | ins[3];
            // If speed fluctuations exceed threshold, restart load distribution script
            if ((uint16_t)abs(g_pid_err) * 16 > threshold) {
                g_unbalance_cnt++;
                g_vm_pc_offset = g_vm_script_start; // Jump back to distribution start
            }
            break;
        }

        case 0x06: // DRAIN PUMP CONTROL
            g_outputs_req_mask |= (1 << 3);
            break;

        case 0x05: // END OF SPIN SCRIPT
        case 0x07:
        default:
            SpinVM_Stop();
            g_sys_flags_03be |= 0x10; // Step completion flag
            break;
    }
}

// ============================================================================
// 10. FAULT SUPERVISOR (0..15) AND DOOR LOCK STATE MACHINE (FUN_code_0d40, 0d8b, 0e18, 13ed)
// ============================================================================

void Faults_LatchDelayed(uint8_t *slot_state, uint8_t bit_pos) // FUN_code_1565
{
    if (g_cycle.system_mode == 2 || g_cycle.system_mode == 3) return;

    if (*slot_state == 0) {
        if (!(g_cycle.config_flags_lo & 0x04)) {
            g_cycle.config_flags_lo |= 0x04; // Force emergency drain
            g_cycle.phase_substate   = 0;
        }
        *slot_state = 1;
    } else if (*slot_state == 1) {
        if (!(g_cycle.config_flags_lo & 0x04)) {
            g_latched_faults |= (1U << bit_pos);
            *slot_state = 4;
        }
    }
}

void Faults_ProcessActiveMask() // FUN_code_0d8b
{
    uint16_t mask = ((uint16_t)g_cycle.fault_mask_hi << 8) | g_cycle.fault_mask_lo;

    if (mask & (1U << 15)) { g_outputs_req_mask &= ~0x20; Faults_LatchDelayed(&g_fault_slot_03ab, 15); }
    if (mask & (1U << 14)) { Faults_LatchDelayed(&g_fault_slot_03ae, 14); }
    if (mask & (1U << 13)) { Faults_LatchDelayed(&g_fault_slot_03ad, 13); }
    if (mask & (1U << 12)) { Faults_LatchDelayed(&g_fault_slot_03af, 12); }
    if (mask & (1U << 10)) { g_outputs_req_mask &= 0xF8; g_latched_faults |= (1U << 10); }
    if (mask & (1U << 9))  { g_latched_faults |= (1U << 9); }
    if (mask & (1U << 8))  { Faults_LatchDelayed(&g_fault_slot_03b0, 8); }

    if (mask & (1U << 4)) {
        if (!(g_latched_faults & (1U << 4))) {
            g_latched_faults |= (1U << 4);
        } else {
            uint8_t mode_lim = pgm_read_byte(&FAULT_CONFIG_TABLE[4]) >> 6;
            if (g_cycle.system_mode != mode_lim) {
                g_cycle.sub_timer_sec    = 0;
                g_cycle.config_flags_lo &= ~0x08;
            }
        }
    }

    if (mask & (1U << 3)) {
        g_outputs_req_mask  &= ~0x20;
        g_computed_target_t  = 0;
        g_temperature_c      = 0;
        g_latched_faults    |= (1U << 3);
    }

    if (mask & (1U << 2)) {
        g_outputs_req_mask  &= ~0x20;
        g_computed_target_t  = 0;
        g_latched_faults    |= (1U << 2);
    }
}

void Faults_EvaluateAll() // FUN_code_0e18
{
    uint16_t new_faults  = 0;
    uint16_t prev_faults = ((uint16_t)g_cycle.fault_mask_hi << 8) | g_cycle.fault_mask_lo;

    // Fault 1: Signal on channel ADC1 while triac output is off
    if (!(g_outputs_req_mask & 0x10) && !(g_flags_038a & 0x04)) {
        if (g_signal_state_ch1 == 2 || g_signal_state_ch1 == 3) {
            new_faults |= (1U << 1);
        }
    }

    // Fault 2: Heater supervision (requires at least +2°C rise within 30 time units)
    if (prev_faults & (1U << 2)) {
        new_faults |= (1U << 2);
    } else if ((g_outputs_req_mask & 0x20) && !(prev_faults & (1U << 3)) && !(g_wash_ctrl_04bc & 0x40)) {
        if (g_temp_rise_timer == 0) {
            if (g_temperature_c < (uint8_t)(g_temp_snapshot + 2)) {
                if (g_status_flags_0399 & 0x40) {
                    new_faults          |= (1U << 2);
                    g_status_flags_0399 &= ~0x40;
                    g_temp_snapshot      = 0;
                } else if (g_adc_ready_flags & 0x10) {
                    g_status_flags_0399 |= 0x40;
                    g_temp_snapshot      = g_temperature_c;
                    g_temp_rise_timer    = 30;
                }
            }
        }
    } else {
        g_temp_snapshot      = 0;
        g_temp_rise_timer    = 10;
        g_status_flags_0399 &= ~0x40;
    }

    // Fault 3: NTC sensor failure
    if ((prev_faults & (1U << 3)) || g_ntc_valid_cnt == 0) {
        new_faults |= (1U << 3);
    }

    // Fault 4: Water fill timeout
    if (g_cycle.config_flags_lo & 0x08) {
        new_faults |= (1U << 4);
    }

    // Fault 5: Invalid program selector dial reading
    if ((g_status_flags_0399 & 0x08) && (g_cycle.system_mode == 2 || g_cycle.system_mode == 3)) {
        new_faults |= (1U << 5);
    }

    // Fault 6: EEPROM checksum error
    if (g_save_req_flags & 0x04) {
        new_faults |= (1U << 6);
    }

    // Fault 9: Exceeded 10 load distribution attempts before spin
    if ((prev_faults & (1U << 9)) || g_unbalance_cnt > 9) {
        new_faults     |= (1U << 9);
        g_unbalance_cnt = 0;
    }

    // Fault 12: Motor triac short-circuit (full AC wave present without gate drive)
    if (prev_faults & (1U << 12)) {
        new_faults |= (1U << 12);
    } else if ((g_outputs_req_mask & 0x10) && g_signal_state_ch1 == 1 && !(g_flags_038a & 0x04)) {
        if (g_triac_test_t1 == 0 && g_triac_test_t2 == 0) {
            g_triac_test_t1 = 7;
            g_triac_test_t2 = 1;
        } else if (g_triac_test_t1 == 0 && g_triac_test_t2 != 0) {
            g_triac_test_t1  = 0;
            g_triac_test_t2  = 0;
            g_fault_slot_03af = 0;
            new_faults      |= (1U << 12);
        }
    }

    // Fault 13: Door lock supervision (up to 8 attempts in FUN_code_0e18)
    if ((prev_faults & 0x6000) == 0 && g_signal_state_ch1 == 3) {
        if (g_motor_target_rpm >= 9 && (IO_Read_PD4() == 1 || IO_Read_AuxOut() == 1)) {
            if (!(g_motor_flags & 0x01)) {
                if (g_door_retry_timer > 0) g_door_retry_timer--;
                else {
                    if (g_door_retry_cnt < 8) {
                        g_door_retry_timer = 1200; // 0x04B0
                        g_door_retry_cnt++;
                        if (g_cycle.system_mode == 0) {
                            Motor_RelaysStop();
                            g_cycle.system_mode = 4;
                            g_sys_flags_03be   |= 0x04;
                        }
                    } else {
                        Motor_RelaysStop();
                        new_faults       |= (1U << 13);
                        g_fault_slot_03ad = 0;
                    }
                }
            }
        }
    } else if (prev_faults & (1U << 13)) {
        new_faults |= (1U << 13);
    }

    // Fault 14: Loss of tachogenerator signal during motor operation (up to 3 attempts)
    if ((prev_faults & 0x6000) == 0 && g_motor_target_rpm > 0 && (g_motor_flags & 0x01)) {
        if (g_motor_curr_rpm + 100 < g_motor_target_rpm && g_tacho_alive_timer == 0) {
            if (g_tacho_retry_timer > 0) g_tacho_retry_timer--;
            else {
                if (g_tacho_retry_cnt < 3) {
                    g_tacho_retry_timer = 300; // 0x012C
                    g_tacho_retry_cnt++;
                    Motor_RelaysStop();
                } else {
                    Motor_RelaysStop();
                    new_faults       |= (1U << 14);
                    g_fault_slot_03ae = 0;
                }
            }
        } else {
            g_tacho_retry_cnt = 0;
        }
    } else if (prev_faults & (1U << 14)) {
        new_faults |= (1U << 14);
    }

    // Fault 15: Overheating above 83°C (0x53)
    if (prev_faults & (1U << 15)) {
        new_faults |= (1U << 15);
    } else if (!(prev_faults & (1U << 3)) && g_temperature_c >= 0x53) {
        if (g_overheat_t1 == 0 && g_overheat_t2 == 0) {
            g_overheat_t1 = 1;
            g_overheat_t2 = 8;
        } else if (g_overheat_t1 == 1 && g_overheat_t2 == 0) {
            g_overheat_t1 = 0;
            new_faults   |= (1U << 15);
        }
    } else {
        g_overheat_t1 = 0;
        g_overheat_t2 = 0;
    }

    // Detect newly triggered fault and log to EEPROM FIFO (end of FUN_code_0e18)
    if (new_faults != prev_faults) {
        uint16_t diff = new_faults ^ prev_faults;
        uint8_t bit_idx = 0;
        while (diff != 0) {
            bit_idx++;
            diff >>= 1;
        }
        if (bit_idx > 0) bit_idx--;

        EEPROM_LogErrorToFIFO(bit_idx);
        EEPROM_WriteVerifiedByte(0x006E, bit_idx);
        g_last_logged_err  = bit_idx;
        g_cycle.error_index = bit_idx;
        g_save_req_flags  |= 0x01;
    }

    g_cycle.fault_mask_lo = (uint8_t)(new_faults & 0xFF);
    g_cycle.fault_mask_hi = (uint8_t)(new_faults >> 8);
}

/**
 * 230V AC mains voltage window check and global heater/triac cut-off (FUN_code_13ed, 0d40)
 */
void System_SafetySupervisor() // FUN_code_0d40 + FUN_code_13ed
{
    // 1. Mains voltage supervision on channel ADC7 (0x6B..0xE6 with hysteresis 0x70..0xE1)
    if (g_mains_voltage_raw < 0x6B || g_mains_voltage_raw > 0xE6) {
        if (g_cycle.system_mode == 0) {
            if (g_mains_fault_timer < 15) {
                g_mains_fault_timer++;
            } else {
                g_cycle.system_mode = 4;
                g_sys_flags_03be   |= 0x04;
                g_mains_fault_timer = 0;
                Motor_RelaysStop();
                g_comm_flags_039a  |= 0x80;
                g_status_flags_0399 |= 0x02;
            }
        }
    } else if (g_mains_voltage_raw > 0x6F && g_mains_voltage_raw < 0xE2) {
        g_mains_fault_timer  = 0;
        g_status_flags_0399 &= ~0x03;
        if ((g_comm_flags_039a & 0x11) == 0 && g_cycle.system_mode == 4 && (g_comm_flags_039a & 0x80)) {
            g_cycle.system_mode = 0;
            g_sys_flags_03be   |= 0x04;
        }
    }

    // 2. Enable power to door lock/triac circuit (bit 4 in g_outputs_req_mask)
    if (g_motor_curr_rpm < 50 && !(g_cycle.fault_mask_hi & 0x80)) {
        if (g_water_level_state != 0 || (g_cycle.fault_mask_hi & 0x01) || g_temperature_c >= 0x5A) {
            g_outputs_req_mask |= 0x10;
        } else {
            g_outputs_req_mask &= ~0x10;
        }
    } else {
        g_outputs_req_mask |= 0x10;
    }

    // 3. Enable heater relay activation (bit 5 in g_outputs_req_mask)
    if ((g_wash_ctrl_04bc & 0x04) &&
        g_water_level_state != 0 &&
        !(g_cycle.fault_mask_lo & 0x0C) &&
        g_cycle.cycle_step == 4) {
        g_outputs_req_mask |= 0x20;
    } else {
        g_outputs_req_mask &= ~0x20;
    }

    // 4. Evaluate faults and update system mode (FUN_code_0d40)
    Signal_CheckOutputChangeAndResetWindows();
    Faults_EvaluateAll();

    if (g_cycle.error_index == 0) {
        g_latched_faults = 0;
    } else {
        Faults_ProcessActiveMask();
        uint16_t err_bit = (1U << g_cycle.error_index);
        uint16_t active  = ((uint16_t)g_cycle.fault_mask_hi << 8) | g_cycle.fault_mask_lo;

        if ((active & err_bit) && (g_latched_faults & err_bit)) {
            uint8_t target_mode = pgm_read_byte(&FAULT_CONFIG_TABLE[g_cycle.error_index]) >> 6;
            if (g_cycle.system_mode < target_mode && g_cycle.system_mode != 2 && target_mode != 2) {
                g_cycle.system_mode = target_mode;
                g_sys_flags_03be   |= 0x04;
            }
        }
    }
}

// ============================================================================
// 11. WASH CYCLE STATE MACHINE, DRUM REVERSALS AND THERMOSTAT (440a..5222)
// ============================================================================

/**
 * Load parameters for the selected wash program from table (FUN_code_5222)
 */
void Wash_LoadSelectedProgram()
{
    if (g_cycle.selected_prog_id >= 1 && g_cycle.selected_prog_id <= 16) {
        g_cycle.program_offset = (g_cycle.selected_prog_id - 1) * 8;
    } else {
        g_cycle.program_offset = 0;
    }

    uint8_t p_idx = g_cycle.program_offset / 8;
    g_cycle.program_group     = pgm_read_byte(&WASH_PROGRAMS_FLASH[p_idx][0]);
    g_cycle.target_spin_div10 = pgm_read_byte(&WASH_PROGRAMS_FLASH[p_idx][5]);
    g_cycle.target_temp_c     = pgm_read_byte(&WASH_PROGRAMS_FLASH[p_idx][6]);
}

/**
 * Determine target heating temperature based on step and Eco options (FUN_code_518c)
 */
void Wash_ResolveTargetTemperature()
{
    g_computed_target_t = 0;
    if ((g_cycle.user_options & 0x04) || (g_cycle.fault_mask_lo & 0x0C)) {
        return; // Cold wash option or NTC/heater fault
    }

    if (g_cycle.cycle_step > 0 && g_cycle.cycle_step < 3) {
        g_computed_target_t = 25; // Prewash: 25°C (0x19)
    } else if (g_cycle.cycle_step > 2) {
        if (g_cycle.target_temp_c == 90) {
            g_computed_target_t = 80; // 0x50
        } else if (g_cycle.program_group == 0 && (g_wash_ctrl_04bc & 0x20) && g_cycle.target_temp_c == 60) {
            g_computed_target_t = g_cfg.temp_eco_60; // 47°C
        } else if (g_cycle.program_group == 0 && (g_wash_ctrl_04bc & 0x20) && g_cycle.target_temp_c == 40) {
            g_computed_target_t = g_cfg.temp_eco_40; // 40°C
        } else {
            g_computed_target_t = g_cycle.target_temp_c;
        }
    }
}

/**
 * Main wash thermostat control in step 4 (FUN_code_472a)
 */
void Wash_ThermostatControl()
{
    if (g_cycle.cycle_step != 4) {
        g_wash_ctrl_04bc &= ~0x04;
        return;
    }

    if (g_temperature_c < g_computed_target_t) {
        if (g_water_level_state != 0 && !(g_cycle.fault_mask_lo & 0x08)) {
            g_wash_ctrl_04bc |= 0x04; // Request heater activation
        }
    } else {
        g_wash_ctrl_04bc &= ~0x04; // Target temperature reached
        g_sys_flags_03be |= 0x10;  // Unlock transition to next step after heating
    }
}

/**
 * 5-phase water filling and reversing drum agitation state machine (FUN_code_494f)
 */
void Wash_AgitationAndFillMachine()
{
    switch (g_cycle.agitation_sub) {
        case 1: // Initialize water fill and select solenoid valve
            g_cycle.sub_timer_sec   = 0;
            g_cycle.agitation_sub   = 2;
            break;

        case 2: // Fill water via solenoid valve until pressure switch reports full (g_water_level_state == 2)
            g_outputs_req_mask |= 0x02; // Enable main valve
            if (g_cycle.sub_timer_sec < 240) {
                if (g_water_level_state >= 2) {
                    g_cycle.measured_fill_t = (uint8_t)g_cycle.sub_timer_sec;
                    g_cycle.sub_timer_sec   = 0;
                    g_outputs_req_mask     &= ~0x07; // Close valves
                    g_cycle.agitation_sub   = 3;
                }
            } else {
                // Water fill timeout exceeded -> Fault 4!
                g_cycle.config_flags_lo |= 0x08;
                g_outputs_req_mask      &= ~0x07;
            }
            break;

        case 3: // Short stabilization pause after closing valve (2 s)
            if (g_cycle.sub_timer_sec >= 2) {
                g_cycle.sub_timer_sec = 0;
                g_cycle.agitation_sub = 4;
            }
            break;

        case 4: // Active drum rotation phase (e.g., 52 RPM for calculated duration)
            if (g_motor_state == 0 && g_cycle.sub_timer_sec == 0) {
                g_motor_flags ^= 0x04; // Toggle direction CW <-> CCW
                Motor_CommandMove(52, 0, 2, 10);
            }
            if (g_cycle.sub_timer_sec >= 12) {
                g_cycle.sub_timer_sec = 0;
                g_cycle.agitation_sub = 5;
            }
            break;

        case 5: // Pause between reversals and water level check
            if (g_water_level_state < 2) {
                // Water level dropped (absorbed by laundry) -> top up!
                g_outputs_req_mask |= 0x02;
            } else {
                g_outputs_req_mask &= ~0x07;
                if (g_cycle.sub_timer_sec >= 6) {
                    g_cycle.sub_timer_sec = 0;
                    g_cycle.agitation_cycles++;
                    g_cycle.agitation_sub = 4; // Next reversal
                }
            }
            break;

        default:
            g_cycle.agitation_sub = 1;
            break;
    }
}

/**
 * 10-attempt water draining and load distribution procedure before spin (FUN_code_4879)
 */
void Wash_DrainAndDistributeBeforeSpin()
{
    if (g_cycle.unbalance_stage == 1) {
        g_outputs_req_mask |= (1 << 3); // Enable drain pump
        Motor_CommandMove(52, 0, 2, 15);
        g_cycle.unbalance_stage = 2;
        g_redist_timer          = 240;  // 0x00F0
        g_unbalance_cnt         = 0;
    } else if (g_cycle.unbalance_stage == 2) {
        if (g_redist_timer == 0) {
            if (g_unbalance_cnt < 10) {
                if (!(g_status_flags_0399 & 0x20)) {
                    g_unbalance_cnt++;
                    g_outputs_req_mask  &= ~(1 << 3);
                    g_redist_timer       = 120; // 0x78
                    g_status_flags_0399 |= 0x20;
                } else {
                    g_outputs_req_mask  |= (1 << 3);
                    g_redist_timer       = 240; // 0xF0
                    g_status_flags_0399 &= ~0x20;
                }
            } else {
                // 10 failed attempts -> Fault 9 and transition to failure mode 3
                g_cycle.system_mode      = 3;
                g_sys_flags_03be        |= 0x04;
                g_cycle.config_flags_lo &= ~0x04;
                g_cycle.cycle_step       = 0xFF;
                g_cycle.unbalance_stage  = 3;
                g_cycle.phase_substate   = 0;
            }
        } else if (g_water_level_state == 0) {
            // Water drained successfully
            g_redist_timer          = 30;
            g_unbalance_cnt         = 0;
            g_status_flags_0399    &= ~0x20;
            g_cycle.unbalance_stage = 3;
        }
    } else if (g_cycle.unbalance_stage == 3) {
        if (g_redist_timer == 0) {
            g_outputs_req_mask     &= ~(1 << 3);
            g_cycle.agitation_sub   = 1;
            g_cycle.unbalance_stage = 0;
        }
    }
}

/**
 * Emergency drain and program cancel after holding Start button (FUN_code_4598)
 */
void Wash_EmergencyDrainAndCancel()
{
    if (g_cycle.system_mode == 1) {
        g_cycle.system_mode = 0;
        g_sys_flags_03be   |= 0x04;
        return;
    }

    if (g_cycle.system_mode == 0) {
        if (g_cycle.phase_substate == 0) {
            SpinVM_Stop();
            g_cycle.agitation_sub  = 0;
            g_cycle.phase_substate = 2;
            g_cycle.unbalance_stage = 1;
            g_save_req_flags      |= 0x01;
        } else if (g_cycle.phase_substate == 2) {
            if (g_cycle.unbalance_stage != 0) {
                Wash_DrainAndDistributeBeforeSpin();
            } else {
                g_cycle.system_mode      = 3; // End / Unlock door
                g_sys_flags_03be        |= 0x04;
                g_cycle.config_flags_lo &= ~0x04;
                g_cycle.cycle_step       = 0xFF;
                g_cycle.phase_substate   = 0;
                g_save_req_flags        |= 0x01;
            }
        }
    }
}

/**
 * Advance to the next wash program step 0..28 -> 0xFF (FUN_code_4ccc)
 */
void Wash_AdvanceCycleStep(uint8_t reason)
{
    if (g_cycle.cycle_step == 0xFF) {
        uint8_t p_idx = g_cycle.program_offset / 8;
        g_cycle.cycle_step = pgm_read_byte(&WASH_PROGRAMS_FLASH[p_idx][1]);
        if (g_cycle.user_options & 0x21) {
            g_cycle.cycle_step = 1; // Prewash active
        }
    } else if (g_cycle.cycle_step < 0x1D && reason == 1) {
        g_cycle.cycle_step++;
        g_cycle.config_flags_lo  &= ~0x04;
        g_cycle.step_elapsed_time = 0;
        g_cycle.agitation_cycles  = 0;
        g_cycle.sub_timer_sec     = 0;
        g_cycle.phase_substate    = 0;

        if (g_cycle.cycle_step >= 0x1D) {
            // Entire wash program completed
            g_cycle.cycle_step  = 0xFF;
            g_cycle.system_mode = 3;
            g_sys_flags_03be   |= 0x04;
            SpinVM_Stop();
            g_outputs_req_mask  = 0;
        }
    }

    g_sys_flags_03be &= ~0x10;
    Wash_ResolveTargetTemperature();
    g_save_req_flags |= 0x01;
}

/**
 * Main active wash cycle dispatcher (FUN_code_440a)
 */
void Wash_ExecuteNormalCycle()
{
    if (g_sys_flags_03be & 0x10) {
        Wash_AdvanceCycleStep(1);
        return;
    }

    // Steps 1..18: Prewash, main wash and rinses (drum reversals + heating)
    if (g_cycle.cycle_step >= 1 && g_cycle.cycle_step <= 18) {
        Wash_ThermostatControl();
        Wash_AgitationAndFillMachine();
    }
    // Steps 19..25: Draining between rinses
    else if (g_cycle.cycle_step >= 19 && g_cycle.cycle_step <= 25) {
        if (g_cycle.unbalance_stage == 0) g_cycle.unbalance_stage = 1;
        Wash_DrainAndDistributeBeforeSpin();
        if (g_cycle.unbalance_stage == 0) {
            g_sys_flags_03be |= 0x10;
        }
    }
    // Steps 26..28: Final spin (launch Spin Virtual Machine FUN_code_4803)
    else if (g_cycle.cycle_step >= 26 && g_cycle.cycle_step <= 28) {
        if (g_cycle.phase_substate == 0) {
            g_cycle.unbalance_stage = 1;
            g_cycle.phase_substate  = 1;
        } else if (g_cycle.phase_substate == 1) {
            Wash_DrainAndDistributeBeforeSpin();
            if (g_cycle.unbalance_stage == 0) {
                g_outputs_req_mask |= (1 << 3); // Pump runs during spin
                SpinVM_StartScript(0);
                g_cycle.phase_substate = 3;
            }
        } else if (g_cycle.phase_substate == 3) {
            if (!(g_motor_ctrl_flags & 0x08)) {
                g_outputs_req_mask &= ~(1 << 3);
                g_sys_flags_03be   |= 0x10;
            }
        }
    }
}

// ============================================================================
// 12. USER INTERFACE: BUTTONS, DIAL, LED MATRIX AND SERVICE MODE
// ============================================================================

/**
 * Read program selector dial position from channel ADC4 (FUN_code_3d35 and FUN_code_54a5)
 */
uint8_t UI_DecodeProgramDial() // FUN_code_3d35
{
    if (!ADC_CheckChannelStability(4, 10)) return 0xFC;

    uint8_t raw = g_adc_avg_bytes[4];
    // 16 thresholds evenly spaced by 15 ADC units (as in FUN_code_3915: 0x0B..0xF8)
    for (uint8_t pos = 1; pos <= 16; pos++) {
        uint8_t thr = (uint8_t)(pos * 15);
        if (raw < thr) return pos;
    }
    return 16;
}

/**
 * Decode ADC0 button resistor ladder with short and long press detection (FUN_code_3d6c and 3dfc)
 */
void UI_ScanButtons()
{
    if (!ADC_CheckChannelStability(0, 5)) return;

    uint8_t raw = g_adc_avg_bytes[0];
    uint8_t window = 0;
    while (window < 7) {
        if (raw > pgm_read_byte(&BTN_ADC_LOW[window]) && raw < pgm_read_byte(&BTN_ADC_HIGH[window])) {
            break;
        }
        window++;
    }

    if (window == 7) return;

    if (window == 3) {
        // Idle state (no button pressed) -> if previously held briefly, trigger short press event
        if (g_btn_released_flag == 0 && g_btn_hold_timer > 0 && g_btn_hold_timer < 26) {
            if (g_detected_btn_id == 4)      g_btn_event_flags |= 0x01; // Start / Pause (short)
            else if (g_detected_btn_id == 6) g_btn_event_flags |= 0x08; // Option 1
            else if (g_detected_btn_id == 2) g_btn_event_flags |= 0x10; // Option 2
            else if (g_detected_btn_id == 0) g_btn_event_flags |= 0x20; // Option 3
        }
        g_btn_hold_timer    = 0;
        g_btn_released_flag = 1;
        g_detected_btn_id   = 3;
    } else {
        if (g_btn_released_flag == 1) {
            g_detected_btn_id   = window;
            g_btn_released_flag = 0;
        } else if (window == g_detected_btn_id) {
            if (g_btn_hold_timer < 0xFFFF) g_btn_hold_timer++;
            // Held for more than 26 cycles (~2.6 seconds):
            if (g_btn_hold_timer == 26) {
                if (g_detected_btn_id == 4)      g_btn_event_flags |= 0x02; // Long Start = Reset/Cancel
                else if (g_detected_btn_id == 5) g_btn_event_flags |= 0x04; // Combo = Child Lock
                else if (g_detected_btn_id == 2) g_btn_event_flags |= 0x40; // Show last error on LEDs
            }
        }
    }
}

/**
 * Process button events (FUN_code_1b74)
 */
void UI_ProcessButtonEvents()
{
    if (g_btn_event_flags & 0x02) {
        // Long press Start/Pause -> Cancel program and drain water
        if (!(g_cycle.config_flags_lo & 0x04)) {
            g_cycle.config_flags_lo |= 0x04;
            g_cycle.phase_substate   = 0;
            g_save_req_flags        |= 0x01;
        }
    } else if (g_btn_event_flags & 0x01) {
        // Short press Start/Pause
        if (g_cycle.system_mode == 1 || g_cycle.system_mode == 2) {
            if (g_cycle.cycle_step == 0xFF) {
                Wash_AdvanceCycleStep(0);
            }
            g_cycle.system_mode = 0; // Start operation
            g_sys_flags_03be   |= 0x04;
        } else if (g_cycle.system_mode == 0 || g_cycle.system_mode == 4) {
            g_cycle.system_mode = 1; // Pause
            g_sys_flags_03be   |= 0x04;
        }
    } else if (g_btn_event_flags & 0x04) {
        // Toggle Child Lock
        g_cycle.config_flags_lo ^= 0x01;
    } else if (g_btn_event_flags & 0x40) {
        // Display last error on LEDs for 10 cycles
        g_sys_flags_03be    |= 0x08;
        g_err_display_timer  = 10;
    } else if (g_cycle.system_mode == 1 || g_cycle.system_mode == 2) {
        // Toggle user options in pause/ready mode
        if (g_btn_event_flags & 0x08) g_cycle.user_options ^= 0x01; // Prewash
        if (g_btn_event_flags & 0x10) g_cycle.user_options ^= 0x02; // Extra rinse
        if (g_btn_event_flags & 0x20) g_cycle.user_options ^= 0x04; // Cold wash
    }

    g_btn_event_flags = 0;
}

/**
 * Encode state of the 6 front panel LEDs (FUN_code_3f20)
 */
void UI_UpdateLEDMatrix()
{
    g_led_solid_mask = 0;
    g_led_blink_mask = 0;

    // 1. Display error code (6 bits from FAULT_CONFIG_TABLE)
    if ((g_sys_flags_03be & 0x08) || g_cycle.error_index >= 4) {
        uint8_t err_id = (g_service_mode_active == 1) ? g_last_logged_err : g_cycle.error_index;
        uint8_t pattern = pgm_read_byte(&FAULT_CONFIG_TABLE[err_id & 0x0F]) & 0x3F;
        g_led_solid_mask = pattern;
        if (g_adc_ready_flags & 0x20) {
            g_led_blink_mask = pattern; // Blink at flag 0x20 frequency
        }
    } else {
        // 2. Normal indication of cycle progress and selected options
        if (g_cycle.system_mode == 0 || g_cycle.system_mode == 4) {
            g_led_solid_mask |= (1 << 0); // Run LED solid ON
        } else if (g_cycle.system_mode == 1) {
            if (g_adc_ready_flags & 0x20) g_led_solid_mask |= (1 << 0); // Run LED blinks in Pause
        }

        if (g_cycle.user_options & 0x01) g_led_solid_mask |= (1 << 1); // Option 1 LED
        if (g_cycle.user_options & 0x02) g_led_solid_mask |= (1 << 2); // Option 2 LED
        if (g_cycle.user_options & 0x04) g_led_solid_mask |= (1 << 3); // Option 3 LED
        if (g_cycle.cycle_step < 19)     g_led_solid_mask |= (1 << 4); // Wash Phase LED
        else if (g_cycle.cycle_step < 26) g_led_solid_mask |= (1 << 5); // Rinse Phase LED
    }

    // Drive lines PB2..PB7 according to calculated mask
    uint8_t portb_leds = (g_led_solid_mask & ~g_led_blink_mask) << 2;
    PORTB = (PORTB & 0x03) | (portb_leds & Pins::PB_LED_MASK);
}

/**
 * Detect Service Mode entry key combination during startup (FUN_code_187a)
 */
void ServiceMode_CheckStartupEntry()
{
    uint16_t loops = 200; // In original firmware: 0x07D0 (2000) startup loops
    bool     match = false;

    while (loops-- > 0) {
        ADC_ProcessSamplingCycle();
        if (++g_mux_slot_counter >= 240) g_mux_slot_counter = 0;

        if (g_adc_ready_flags & 0x08) {
            UI_ScanButtons();
            uint8_t dial = UI_DecodeProgramDial();
            g_adc_ready_flags &= ~0x08;

            // Condition from FUN_code_187a: Held Start button (window 4) + Program dial at pos. 8
            if (g_detected_btn_id == 4 && dial == 8 && (g_sys_flags_03be & 0x01)) {
                match = true;
            } else {
                match = false;
                break;
            }
        }
        wdt_reset();
    }

    g_mux_slot_counter = 0;
    if (match) {
        g_ui_state_flags      = 2; // Activate Service Mode
        g_service_mode_active = 1;
        g_service_test_step   = 0;
    } else {
        g_ui_state_flags      = 1; // Normal operating mode
        g_service_mode_active = 0;
    }
}

/**
 * Execute Service Autotest steps (FUN_code_1920)
 */
void ServiceMode_ExecuteTask()
{
    g_outputs_req_mask |= 0x10; // Lock door

    if (g_btn_event_flags & 0x01) {
        g_btn_event_flags = 0;
        g_service_test_step++;
        Motor_StopAndReset();
        g_outputs_req_mask = 0x10;
    }

    switch (g_service_test_step) {
        case 0: // Display last fault from EEPROM on LEDs
            g_sys_flags_03be |= 0x08;
            break;
        case 1: // Test prewash solenoid valve (PB0)
            g_sys_flags_03be   &= ~0x08;
            g_outputs_req_mask |= (1 << 0);
            break;
        case 2: // Test main wash solenoid valve (PC7)
            g_outputs_req_mask |= (1 << 1);
            break;
        case 3: // Test softener solenoid valve (PB1)
            g_outputs_req_mask |= (1 << 2);
            break;
        case 4: // Test heater (when drum is filled with water)
            if (g_water_level_state != 0) g_outputs_req_mask |= (1 << 5);
            break;
        case 5: // Test clockwise motor rotation (52 RPM CW)
            if (g_motor_state == 0) { g_motor_flags &= ~0x04; Motor_CommandMove(52, 0, 2, 60); }
            break;
        case 6: // Test counter-clockwise motor rotation (52 RPM CCW)
            if (g_motor_state == 0) { g_motor_flags |= 0x04; Motor_CommandMove(52, 0, 2, 60); }
            break;
        case 7: // Test drain pump (PC6)
            g_outputs_req_mask |= (1 << 3);
            break;
        case 8: // Test maximum spin speed (1200 RPM)
            g_outputs_req_mask |= (1 << 3);
            if (g_motor_state == 0) { g_motor_flags |= 0x08; Motor_CommandMove(1200, 0, 10, 60); }
            break;
        default: // Exit autotest (step 99 in FUN_code_1920)
            g_service_test_step   = 0;
            g_service_mode_active = 0;
            g_ui_state_flags      = 1;
            Motor_StopAndReset();
            g_outputs_req_mask    = 0;
            break;
    }
}

// ============================================================================
// 13. UART0 DIAGNOSTIC TELEMETRY (FROM FLASH POINTER TABLE code:007c)
// ============================================================================

void UART_SendTelemetryFrame()
{
    if (!Serial.available()) return;
    char cmd = Serial.read();

    if (cmd == 's' || cmd == 'S') {
        // Send a complete frame of key system variables from the 54-pointer table (code:007c)
        Serial.print(F("MODE:"));  Serial.print(g_cycle.system_mode);
        Serial.print(F(" STEP:")); Serial.print(g_cycle.cycle_step);
        Serial.print(F(" PROG:")); Serial.print(g_cycle.selected_prog_id);
        Serial.print(F(" TEMP:")); Serial.print(g_temperature_c);
        Serial.print(F("/"));      Serial.print(g_computed_target_t);
        Serial.print(F(" RPM:"));  Serial.print(g_motor_curr_rpm);
        Serial.print(F("/"));      Serial.print(g_motor_target_rpm);
        Serial.print(F(" ANG:"));  Serial.print(g_firing_angle);
        Serial.print(F(" H2O:"));  Serial.print(g_water_level_state);
        Serial.print(F(" OUT:"));  Serial.print(g_outputs_req_mask, HEX);
        Serial.print(F(" ERR:"));  Serial.print(g_cycle.error_index);
        Serial.print(F(" MASK:")); Serial.println(((uint16_t)g_cycle.fault_mask_hi << 8) | g_cycle.fault_mask_lo, HEX);
    }
}

// ============================================================================
// 14. THREE MAIN SCHEDULER TASKS (FUN_code_1641, 17b9, 17f2)
// ============================================================================

/**
 * 100 ms Task: Process timers, wash step supervision and fault counters (FUN_code_1641)
 */
void Task_100ms() // FUN_code_1641
{
    static uint8_t div_2s  = 20;
    static uint8_t div_60s = 30;

    if (--div_2s == 0) {
        div_2s = 20;
        PORTC ^= (1 << Pins::PC_DIAG_TICK_100MS); // ACSR ^= 4 in original firmware
        g_adc_ready_flags ^= 0x20;                // LED blink flag (~0.5 Hz)

        if (g_cycle.system_mode == 0) {
            if (g_cycle.sub_timer_sec < 0xFFFF) g_cycle.sub_timer_sec++;
            if (g_redist_timer > 0)             g_redist_timer--;
        }

        if (g_err_display_timer > 0) {
            if (--g_err_display_timer == 0) {
                g_sys_flags_03be &= ~0x08;
            }
        }

        if (g_triac_test_t1 > 0) g_triac_test_t1--;
        if (g_overheat_t2 > 0)   g_overheat_t2--;

        // Minute timer for wash steps
        if (--div_60s == 0) {
            div_60s = 30;
            if (g_cycle.system_mode == 0 && g_cycle.cycle_step < 0x1D) {
                uint8_t step_limit = pgm_read_byte(&STEP_DURATION_TABLE[g_cycle.cycle_step]);
                if (g_cycle.step_elapsed_time < step_limit) {
                    g_cycle.step_elapsed_time++;
                } else if (step_limit != 0) {
                    g_sys_flags_03be |= 0x10; // Step time elapsed -> advance to next step
                }
            }
            if (g_temp_rise_timer > 0) g_temp_rise_timer--;
        }
    }

    g_wdt_checkin_flags |= 0x02;
}

/**
 * 200 ms Task: Motor state machine, speed ramp, UI, LED matrix and pressure switch (FUN_code_17b9)
 */
void Task_200ms() // FUN_code_17b9
{
    PORTC ^= (1 << Pins::PC_DIAG_TICK_200MS); // ACSR ^= 8 in original firmware

    // 1. Delayed motor relay turn-off and 0 RPM detection (FUN_code_2a66)
    if ((g_motor_ctrl_flags & 0x10) && --g_stop_relay_timer == 0) {
        Motor_RelaysStop();
        g_motor_ctrl_flags &= ~0x10;
    }

    if (g_motor_curr_rpm < 6) {
        if (g_zero_rpm_debounce == 0) g_motor_ctrl_flags |= 0x20;
        else                          g_zero_rpm_debounce--;
    } else {
        g_zero_rpm_debounce = 10;
        g_motor_ctrl_flags &= ~0x20;
    }

    if (g_tacho_alive_timer > 0) g_tacho_alive_timer--;

    // 2. Main dispatcher for Normal / Service mode (FUN_code_1b13 + FUN_code_18f0)
    if (g_service_mode_active == 1) {
        ServiceMode_ExecuteTask();
    } else {
        System_SafetySupervisor();
        if (g_cycle.system_mode == 1 || g_cycle.system_mode == 2) {
            Wash_LoadSelectedProgram();
        }
        UI_ProcessButtonEvents();

        if (g_cycle.config_flags_lo & 0x04) {
            Wash_EmergencyDrainAndCancel();
        } else if (g_cycle.system_mode == 0) {
            Wash_ExecuteNormalCycle();
        }

        if (g_motor_ctrl_flags & 0x08) {
            SpinVM_ExecuteStep();
        }
    }

    // 3. Update LEDs, pressure switch and motor state machine
    UI_UpdateLEDMatrix();
    Hydrostat_UpdateState();

    if (g_motor_state != 0) {
        Motor_StateMachine_200ms();
        if (g_motor_prev_state != g_motor_state) {
            g_motor_prev_state = g_motor_state;
            g_save_req_flags  |= 0x01;
        }
    }

    g_wdt_checkin_flags |= 0x04;
}

/**
 * Fast Loop: ADC sampling, motor PID controller, AC window classification and EEPROM save (FUN_code_17f2)
 */
void Task_FastLoop() // FUN_code_17f2
{
    PORTC ^= (1 << Pins::PC_DIAG_TICK_FAST); // ACSR ^= 0x10 in original firmware

    ADC_ProcessSamplingCycle();
    IO_UpdateOutputsPWM();
    Motor_ExecutePIDTask();

    // When a new batch of 80 ADC samples is ready (bit 3 in g_adc_ready_flags):
    if (g_adc_ready_flags & 0x08) {
        UI_ScanButtons();
        uint8_t dial = UI_DecodeProgramDial();
        if (dial >= 1 && dial <= 16 && (g_cycle.system_mode == 1 || g_cycle.system_mode == 2)) {
            g_cycle.selected_prog_id = dial;
        }

        Signal_AccumulateWindows();
        NTC_UpdateTemperature();

        if (ADC_CheckChannelStability(7, 25)) {
            g_mains_voltage_raw = g_adc_avg_bytes[7];
        }
        g_adc_ready_flags &= ~0x08;
    }

    // Save state to EEPROM on request (when not in Service Mode)
    if ((g_save_req_flags & 0x01) && g_service_mode_active == 0) {
        EEPROM_SaveCycleState(false);
    }

    UART_SendTelemetryFrame();
    g_wdt_checkin_flags |= 0x08;
}

// ============================================================================
// 15. HARDWARE INITIALIZATION (setup) AND MAIN SYSTEM LOOP (loop)
// ============================================================================

void setup() // Mapped from FUN_code_15ef -> FUN_code_204f -> FUN_code_1e28
{
    // 1. Check reset cause (MCUSR) and initialize Watchdog
    uint8_t mcusr_val = MCUSR;
    MCUSR = 0;
    wdt_enable(WDTO_2S);

    // 2. Configure directions and initial states of all ports (FUN_code_1e28)
    DDRA  = 0x00; PORTA = 0x00; // PA0..PA7 as ADC inputs
    DDRB  = 0xFF; PORTB = 0xFF; // PB0..PB7 as valve and LED matrix outputs
    DDRC  = 0xFE; PORTC = 0x00; // PC0 pressure switch input, PC1..PC7 outputs
    DDRD  = 0xBA; PORTD = 0xB8; // PD0(RX), PD2(INT0), PD6(ICP1) inputs, rest outputs

    // 3. Initialize UART0 (UBRR0 = 25 -> 38400 bps @ 16MHz)
    Serial.begin(38400);

    // 4. Initialize ADC converter (ADCSRA = 0x85)
    ADMUX  = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS0);

    // 5. Initialize Timer1 in Input Capture mode for tachogenerator (TCCR1B = 0xC2)
    TCCR1A = 0x00;
    TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11);
    TIMSK1 = (1 << ICIE1);

    // 6. Initialize mains zero-crossing interrupt INT0 on falling edge (EICRA = 0x02)
    EICRA  = (1 << ISC01);
    EIMSK  = (1 << INT0);

    // 7. Initialize data structures, load calibration and restore state from EEPROM
    ADC_ResetChannels();
    EEPROM_LoadFactoryConfiguration();
    ServiceMode_CheckStartupEntry();
    EEPROM_RestoreCycleState();

    if (g_cycle.system_mode == 3) {
        g_cycle.system_mode = 2; // Transition from end mode to ready mode after reset
    }

    sei();
}

void loop() // Cooperative real-time scheduler from FUN_code_15ef
{
    static uint32_t last_2ms   = 0;
    static uint32_t last_100ms = 0;
    static uint32_t last_200ms = 0;

    uint32_t now = millis();

    // Fast loop (~2 ms): ADC sampling, output PWM, motor PID
    if (now - last_2ms >= 2) {
        last_2ms = now;
        if (++g_mux_slot_counter >= 240) g_mux_slot_counter = 0;
        if (g_zc_filter_cnt < 255)       g_zc_filter_cnt++;
        Task_FastLoop();
    }

    // 100 ms Task (50 x 2ms in original firmware: DAT_mem_03bb = 0x32)
    if (now - last_100ms >= 100) {
        last_100ms = now;
        Task_100ms();
    }

    // 200 ms Task (100 x 2ms in original firmware: DAT_mem_03bc = 0x64)
    if (now - last_200ms >= 200) {
        last_200ms = now;
        Task_200ms();
    }

    // Watchdog supervisor (FUN_code_5617): resets WDT only when all 3 tasks check in
    if ((g_wdt_checkin_flags & 0x0E) == 0x0E) {
        wdt_reset();
        g_wdt_checkin_flags &= ~0x0E;
    }
}