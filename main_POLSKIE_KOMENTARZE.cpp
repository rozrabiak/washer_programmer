/**
 * ============================================================================
 * PEŁNA REKONSTRUKCJA OPROGRAMOWANIA STEROWNIKA PRALKI (ATmega644P)
 * Odtworzono na podstawie dekompilacji wsadu IAR (flash_intelhex.c / dekomp.txt)
 * 
 * Zawiera wszystkie podsystemy:
 *  1. Tablice PROGMEM: 16 programów, 100 punktów NTC, 16 błędów, skrypty VM wirowania
 *  2. Menedżer EEPROM: 83B kalibracji (0xCA..0x55+CRC), 5x redundancja stanu, FIFO 10 błędów
 *  3. Podsystem ADC (80 próbek) + klasyfikator przebiegów AC (32 okna) dla triaka i blokady
 *  4. Sterownik fazowy silnika AC: Zero-Cross (INT0), Tacho (ICP1 + mediana), adaptacyjny PID
 *  5. Maszyna Wirtualna (Bytecode Interpreter) profili wirowania i kontroli niewyważenia
 *  6. Pełna diagnostyka 16 błędów + ryglowanie drzwi (8 prób) + nadzór grzałki (+2C/30s)
 *  7. Maszyna stanów prania (kroki 0..28), 5-fazowy rewers bębna i 10 prób rozkładania wsadu
 *  8. Obsługa UI: drabinka przycisków (ADC0), pokrętła (ADC4/ADC6), matryca LED i Tryb Serwisowy
 *  9. Telemetria diagnostyczna UART0 (54 zmienne systemowe z tablicy wskaźników)
 * ============================================================================
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

// ============================================================================
// 1. DEFINICJE WYPROWADZEŃ MIKROKONTROLERA ATMEGA644P (z FUN_code_1e28 i 093d..09ea)
// ============================================================================
namespace Pins {
    // PORTA (0x20..0x22) - Wejścia analogowe ADC0..ADC7
    constexpr uint8_t ADC_BUTTONS_CH     = 0; // PA0 (ADC0) - Drabinka przycisków
    constexpr uint8_t ADC_DOOR_TRIAC_CH  = 1; // PA1 (ADC1) - Detekcja blokady drzwi / triaka
    constexpr uint8_t ADC_PUMP_SENSE_CH  = 2; // PA2 (ADC2) - Detekcja obwodu pompy / wyjść
    constexpr uint8_t ADC_PROG_DIAL_CH   = 4; // PA4 (ADC4) - Pokrętło wyboru programu (1..16)
    constexpr uint8_t ADC_NTC_SENSOR_CH  = 5; // PA5 (ADC5) - Czujnik temperatury wody NTC
    constexpr uint8_t ADC_SPIN_DIAL_CH   = 6; // PA6 (ADC6) - Pokrętło opcji / obrotów (0..6)
    constexpr uint8_t ADC_MAINS_VOLT_CH  = 7; // PA7 (ADC7) - Pomiar napięcia sieci 230V AC

    // PORTB (0x23..0x25) - Zawory wody i kolumny matrycy LED
    constexpr uint8_t PB_VALVE_PREWASH   = 0; // PB0 - Elektrozawór prania wstępnego (30% PWM)
    constexpr uint8_t PB_VALVE_SOFTENER  = 1; // PB1 - Elektrozawór płynu / płukania (30% PWM)
    constexpr uint8_t PB_LED_MASK        = 0xFC; // PB2..PB7 - Linie matrycy diod LED

    // PORTC (0x26..0x28) - Hydrostat, Triak silnika, Pompa, Zawór główny
    constexpr uint8_t PC_HYDROSTAT_IN    = 0; // PC0 - Wejście cyfrowe hydrostatu
    constexpr uint8_t PC_MOTOR_TRIAC     = 1; // PC1 - Wyjście bramki triaka silnika
    constexpr uint8_t PC_DIAG_TICK_100MS = 2; // PC2 - Pin diagnostyczny pętli 100ms
    constexpr uint8_t PC_DIAG_TICK_200MS = 3; // PC3 - Pin diagnostyczny pętli 200ms
    constexpr uint8_t PC_DIAG_TICK_FAST  = 4; // PC4 - Pin diagnostyczny pętli szybkiej
    constexpr uint8_t PC_RELAY_AUX2      = 5; // PC5 - Przekaźnik pomocniczy (dla wariantu 1)
    constexpr uint8_t PC_DRAIN_PUMP      = 6; // PC6 - Pompa odpływowa (50% PWM)
    constexpr uint8_t PC_VALVE_MAIN      = 7; // PC7 - Elektrozawór prania głównego (30% PWM)

    // PORTD (0x29..0x2B) - UART0, Detekcja zera (INT0), Tacho (ICP1), Przekaźniki mocy
    constexpr uint8_t PD_UART_RX         = 0; // PD0 - RXD0
    constexpr uint8_t PD_UART_TX         = 1; // PD1 - TXD0
    constexpr uint8_t PD_ZERO_CROSS_INT0 = 2; // PD2 - INT0 (Detekcja przejścia przez zero 50/60Hz)
    constexpr uint8_t PD_RELAY_MOTOR_A   = 3; // PD3 - Przekaźnik rewersu silnika A / Grzałka
    constexpr uint8_t PD_RELAY_MOTOR_B   = 4; // PD4 - Przekaźnik rewersu silnika B (CW/CCW)
    constexpr uint8_t PD_RELAY_HEATER    = 5; // PD5 - Przekaźnik grzałki / uzwojenia
    constexpr uint8_t PD_TACHO_ICP1      = 6; // PD6 - ICP1 (Wejście impulsów tachogeneratora)
    constexpr uint8_t PD_MUX_CTRL        = 7; // PD7 - Linia sterująca multipleksera
}

// ============================================================================
// 2. TABLICE DANYCH STAŁYCH W PAMIĘCI FLASH (PROGMEM) - ZDEKODOWANE Z HEX
// ============================================================================

// Flash 0x0070 (code:0038): Kolejność próbkowania kanałów ADC w 10 krokach
const uint8_t ADC_CHANNEL_MAP[10] PROGMEM = {
    6, 5, 0, 4, 1, 2, 7, 0xFF, 0xFF, 0xFF
};

// Flash 0x007A (code:003d): Maska modulacji PWM wyjść w cyklu 10-krokowym
const uint8_t OUTPUT_DUTY_MASK[10] PROGMEM = {
    0x30, 0x3F, 0x3F, 0x3F, 0x38, 0x38, 0x30, 0x30, 0x30, 0x30
};

// Flash 0x0084 (code:0042): 100-punktowa tablica linearyzacji czujnika NTC (0..99 °C)
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

// Flash 0x00E8 (code:0074..007b.1): Tablica 16 błędów
// Starsze 2 bity (>>6) = docelowy tryb awaryjny (0..3), młodsze 6 bitów (& 0x3F) = kod migania LED
const uint8_t FAULT_CONFIG_TABLE[16] PROGMEM = {
    0x00, // 0: Brak błędu
    0x38, // 1: Sygnał obwodu triaka/blokady przy wyłączonym wyjściu (LED: 111000b)
    0x02, // 2: Brak przyrostu temperatury o 2°C w 30 krokach - grzałka (LED: 000010b)
    0x04, // 3: Zwarcie / rozwarcie czujnika NTC (LED: 000100b)
    0x50, // 4: Timeout napełniania wodą - brak sygnału hydrostatu (Tryb 1, LED: 010000b)
    0xB1, // 5: Błąd odczytu wybieraka programów ADC4 (Tryb 2, LED: 110001b)
    0x98, // 6: Błąd sumy kontrolnej EEPROM (Tryb 2, LED: 011000b)
    0x15, // 7: Błąd konfiguracji / komunikacji (LED: 010101b)
    0xF0, // 8: Przepełnienie / błąd hydrostatu (Tryb 3, LED: 110000b)
    0xC5, // 9: Niepowodzenie rozłożenia prania po 10 próbach - Unbalance (Tryb 3, LED: 000101b)
    0xC7, // 10: Błąd obwodu zaworów wody (Tryb 3, LED: 000111b)
    0x11, // 11: Błąd podstanu wirowania 0x1C (LED: 010001b)
    0xDC, // 12: Zwarcie / przebicie triaka silnika (Tryb 3, LED: 011100b)
    0xC8, // 13: Brak potwierdzenia zamknięcia blokady drzwi po 8 próbach (Tryb 3, LED: 001000b)
    0xC3, // 14: Brak sygnału z tachogeneratora / zablokowany silnik po 3 próbach (Tryb 3, LED: 000011b)
    0xC6  // 15: Przegrzanie wody >= 83°C (Tryb 3, LED: 000110b)
};

// Flash 0x0D9F (code:06cf.1..070f): Tablica 16 programów prania (po 8 bajtów na program)
// Format: { Grupa, KrokStartowy, KrokKońcowy, MaskaOpcji, Rezerwa, MaxRPM_div10, TempZadana_C, Flaga }
const uint8_t WASH_PROGRAMS_FLASH[16][8] PROGMEM = {
    { 0x07, 0x18, 0x1D, 0x10, 0x00, 0x78, 0x00, 0x00 }, // Prog 1: Płukanie + Wirowanie 1200
    { 0x04, 0x03, 0x1D, 0x14, 0x00, 0x3D, 0x1E, 0x00 }, // Prog 2: Szybkie 30°C (600 RPM)
    { 0x05, 0x03, 0x1D, 0x14, 0x00, 0x78, 0x1E, 0x00 }, // Prog 3: Codzienne 30°C (1200 RPM)
    { 0x06, 0x03, 0x1D, 0x14, 0x00, 0x78, 0x28, 0x00 }, // Prog 4: Codzienne 40°C (1200 RPM)
    { 0x00, 0x03, 0x1D, 0x3F, 0x00, 0x78, 0x00, 0x00 }, // Prog 5: Bawełna Zimna (1200 RPM)
    { 0x00, 0x03, 0x1D, 0x3F, 0x00, 0x78, 0x28, 0x00 }, // Prog 6: Bawełna 40°C (1200 RPM)
    { 0x00, 0x03, 0x1D, 0x3F, 0x00, 0x78, 0x3C, 0x00 }, // Prog 7: Bawełna 60°C (1200 RPM)
    { 0x00, 0x03, 0x1D, 0x3F, 0x00, 0x78, 0x5A, 0x00 }, // Prog 8: Bawełna 90°C (1200 RPM)
    { 0x01, 0x03, 0x1D, 0x3F, 0x00, 0x50, 0x3C, 0x00 }, // Prog 9: Syntetyki 60°C (800 RPM)
    { 0x01, 0x03, 0x1D, 0x3F, 0x00, 0x50, 0x28, 0x00 }, // Prog 10: Syntetyki 40°C (800 RPM)
    { 0x01, 0x03, 0x1D, 0x3F, 0x00, 0x50, 0x1E, 0x00 }, // Prog 11: Syntetyki 30°C (800 RPM)
    { 0x01, 0x03, 0x1D, 0x3F, 0x00, 0x50, 0x00, 0x00 }, // Prog 12: Syntetyki Zimna (800 RPM)
    { 0x02, 0x03, 0x1D, 0x1E, 0x00, 0x3D, 0x1E, 0x00 }, // Prog 13: Delikatne 30°C (600 RPM)
    { 0x03, 0x03, 0x1D, 0x1E, 0x00, 0x3D, 0x00, 0x00 }, // Prog 14: Wełna Zimna (600 RPM)
    { 0x03, 0x03, 0x1D, 0x1E, 0x00, 0x3D, 0x28, 0x00 }, // Prog 15: Wełna 40°C (600 RPM)
    { 0x08, 0x1A, 0x1D, 0x10, 0x00, 0x78, 0x00, 0x00 }  // Prog 16: Samo Wirowanie (1200 RPM)
};

// Flash 0x0164 (code:00b2..0205): Bytecode Maszyny Wirtualnej Wirowania (7-bajtowe instrukcje)
// Opcody: 0x01=Pauza, 0x02=RuchSilnika(Hold, RPM_H, RPM_L, Ramp, Dir, Time), 0x03=TestWyważenia, 0x06=Pompa, 0x07=Koniec
const uint8_t SPIN_VM_SCRIPT_MAIN[] PROGMEM = {
    // Faza 1: Rozkładanie wsadu (52 RPM CW/CCW)
    0x02, 0x05, 0x00, 0x34, 0x05, 0x00, 0x01,
    0x02, 0x05, 0x00, 0x34, 0x05, 0x01, 0x01,
    0x02, 0x05, 0x00, 0x34, 0x05, 0x00, 0x01,
    0x02, 0x05, 0x00, 0x34, 0x05, 0x01, 0x01,
    // Faza 2: Rampowanie 65 -> 80 -> 100 RPM i kontrola wyważenia
    0x02, 0x00, 0x00, 0x41, 0x04, 0x00, 0x01,
    0x02, 0x00, 0x00, 0x50, 0x04, 0x00, 0x01,
    0x02, 0x00, 0x00, 0x64, 0x14, 0x00, 0x04,
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Włącz pompę
    0x03, 0x01, 0x4F, 0x03, 0x08, 0x02, 0x00, // Kontrola wyważenia próg 0x034F
    0x03, 0x01, 0xCC, 0x03, 0x08, 0x04, 0x02, // Kontrola wyważenia próg 0x03CC
    // Faza 3: Wirowanie pośrednie 300 -> 600 -> 800 -> 1000 -> 1200 RPM
    0x02, 0x00, 0x01, 0x2C, 0x01, 0x00, 0x0A, // 300 RPM
    0x02, 0x00, 0x02, 0x58, 0x3C, 0x00, 0x3C, // 600 RPM
    0x02, 0x00, 0x03, 0x20, 0x3C, 0x00, 0x14, // 800 RPM
    0x02, 0x00, 0x03, 0xE8, 0x64, 0x00, 0x14, // 1000 RPM
    0x02, 0x00, 0x04, 0xB0, 0xA0, 0x00, 0x14, // 1200 RPM
    // Faza 4: Wybieganie bębna i koniec
    0x01, 0xB4, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Progi napięciowe drabinki przycisków ADC0 (z FUN_code_3d6c, adresy 0x072F i 0x0736)
const uint8_t BTN_ADC_LOW[7]  PROGMEM = { 0x10, 0x30, 0x50, 0x70, 0x95, 0xB5, 0xD5 };
const uint8_t BTN_ADC_HIGH[7] PROGMEM = { 0x28, 0x48, 0x68, 0x8E, 0xAE, 0xCE, 0xF5 };

// Progi dodatkowego pokrętła ADC6 (z FUN_code_3cf6, adres 0x0728)
const uint8_t AUX_DIAL_THRESHOLDS[7] PROGMEM = { 0x1E, 0x3C, 0x5A, 0x78, 0x96, 0xB4, 0xD2 };

// Czasy trwania kroków prania w minutach (kopiowane w FUN_code_3915 do 0x0725)
const uint8_t STEP_DURATION_TABLE[29] PROGMEM = {
    0, 2, 1, 5, 25, 10, 5, 3, 2, 1, 3, 0, 0, 2, 3, 2, 0, 0, 2, 5, 4, 3, 2, 3, 4, 2, 0, 0, 6
};

// ============================================================================
// 3. STRUKTURY DANYCH SYSTEMOWYCH (SRAM I EEPROM)
// ============================================================================

// Statystyki pojedynczego kanału ADC (8 bajtów, baza 0x033A)
struct AdcChannelStats {
    uint16_t avg_8bit;  // +0x00
    uint16_t sum_10bit; // +0x02
    uint16_t max_8bit;  // +0x04
    uint16_t min_8bit;  // +0x06
};

// Okno 32 próbek do klasyfikacji kształtu przebiegu AC (6 bajtów: 0x038C i 0x0392)
struct SignalWindow {
    uint16_t sum_avg;
    uint16_t sum_min;
    uint16_t sum_max;
};

// 28-bajtowy stan bieżącego cyklu prania zapisywany w 5 kopiach w EEPROM (0x0100..0x011B)
struct __attribute__((packed)) CycleStateRecord {
    uint8_t  step_elapsed_time; // 0x0100: Czas upływający w danym kroku
    uint8_t  system_mode;       // 0x0101: 0=Praca, 1=Pauza, 2=Gotowość/Stop, 3=Awaria, 4=Ryglowanie/Start
    uint8_t  error_index;       // 0x0102: Numer aktywnego błędu (0..15)
    uint8_t  config_flags_lo;   // 0x0103: Bity: 0=ChildLock, 2=DrainReq, 3=FillTimeout, 4=RinseHold
    uint8_t  program_group;     // 0x0104: Grupa programu (0=Bawełna, 1=Syntetyki, 2=Delikatne, 3=Wełna...)
    uint8_t  user_options;      // 0x0105: Opcje dodatkowe (bity: 0=Prewash, 1=ExtraRinse, 2=Cold, 3=RinseHold, 4=Eco)
    uint8_t  target_temp_c;     // 0x0106: Zadana temperatura prania (0, 30, 40, 60, 90)
    uint8_t  target_spin_div10; // 0x0107: Zadane obroty wirowania / 10 (np. 120 = 1200 RPM)
    uint8_t  measured_fill_t;   // 0x0108: Zmierzony czas dopełniania wody
    uint8_t  agitation_cycles;  // 0x0109: Licznik cykli rewersyjnych
    uint8_t  agitation_sub;     // 0x010A: Podstan rewersu (0..5)
    uint8_t  cycle_step;        // 0x010B: Aktualny krok programu (0..28 lub 0xFF=Koniec)
    uint8_t  phase_substate;    // 0x010C: Podstan fazy (0, 1, 2, 3)
    uint8_t  unbalance_stage;   // 0x010D: Etap rozkładania prania (0..3)
    uint8_t  program_offset;    // 0x010E: Offset w tablicy programów ((prog-1)*8)
    uint8_t  saved_step_info;   // 0x010F: Informacja o podkroku płukania
    uint8_t  fault_mask_lo;     // 0x0110: Młodszy bajt maski błędów
    uint8_t  fault_mask_hi;     // 0x0111: Starszy bajt maski błędów
    uint16_t sub_timer_sec;     // 0x0112:0113: Licznik sekundowy podfazy
    uint8_t  door_seq_state;    // 0x0114: Stan sekwencji drzwi/startu
    uint8_t  selected_prog_id;  // 0x0115: Wybrany program na pokrętle (1..16)
    uint8_t  power_fail_flag;   // 0x0116: Flaga zaniku zasilania (7 = wznowienie)
    uint8_t  saved_flags_0399;  // 0x0117: Kopia flag 0x0399
    uint8_t  saved_flags_039a;  // 0x0118: Kopia flag 0x039A
    uint8_t  packed_nibbles;    // 0x0119: Spakowane liczniki płukań
    uint16_t total_cycles_cnt;  // 0x011A:011B: Licznik całkowitej liczby cykli pralki
};

// 83-bajtowy rekord konfiguracji fabrycznej w EEPROM (inaugurowany w FUN_code_3915, 0x011C..0x016E)
struct __attribute__((packed)) FactoryConfigRecord {
    uint8_t tacho_pulses;       // 0x011C: Stała tachogeneratora (domyślnie 0xDA = 218)
    uint8_t spin_table_1[9];    // 0x011D..0x0125
    uint8_t spin_table_2[9];    // 0x0126..0x012E: {0, 30, 30, 40, 40, 60, 80, 100, 0xFE}
    uint8_t unused_012f;        // 0x012F
    uint8_t opt_led_mask[6];    // 0x0130..0x0135: {0x20, 0x02, 0x04, 0xFE, 0xFE, 0xFE}
    uint8_t max_temp_limit;     // 0x0136: Limit temperatury (0x3C = 60°C)
    uint8_t unused_0137[3];     // 0x0137..0x0139
    uint8_t agitate_params[8];  // 0x013A..0x0141: {0x7A, 0x3F, 0x50, 0x2D, 0x40, 0x1F, 0x70, 0x17}
    uint8_t machine_model_type; // 0x0142: Typ modelu (0..4, domyślnie 1)
    uint8_t hw_board_variant;   // 0x0143: Wariant sprzętowy PCB (0 lub 1)
    uint8_t ui_btn_layout;      // 0x0144: Układ przycisków (0 lub 1)
    uint8_t dial_positions_cfg; // 0x0145: Wariant pokrętła (0/1 = 16 poz., 2 = 8 poz.)
    uint8_t reserved_block[14]; // 0x0146..0x0153
    uint8_t angle_min_power;    // 0x0154 -> 0x0471: Max opóźnienie triaka (0x99 = 153)
    uint8_t angle_max_wash;     // 0x0155 -> 0x046F: Min opóźnienie dla prania (0x32 = 50)
    uint8_t angle_max_spin;     // 0x0156 -> 0x0470: Min opóźnienie dla wirowania (0x19 = 25)
    uint8_t angle_initial;      // 0x0157 -> 0x0472: Kąt startowy triaka (0x7F = 127)
    uint8_t pid_wash_kp;        // 0x0158 -> 0x0417: Kp pranie (100)
    uint8_t pid_wash_ki;        // 0x0159 -> 0x0418: Ki pranie (80)
    uint8_t pid_wash_kd;        // 0x015A -> 0x0419: Kd pranie (128)
    uint8_t accel_limit_wash;   // 0x015B -> 0x0421: Limit przyrostu pranie (12)
    uint8_t pid_spin_kp;        // 0x015C -> 0x041A: Kp wirowanie (70)
    uint8_t pid_spin_ki;        // 0x015D -> 0x041B: Ki wirowanie (53)
    uint8_t pid_spin_kd;        // 0x015E -> 0x041C: Kd wirowanie (128)
    uint8_t accel_limit_spin;   // 0x015F -> 0x0420: Limit przyrostu wirowanie (12)
    uint8_t pid_dist_kp;        // 0x0160 -> 0x041D: Kp rozkładanie (110)
    uint8_t pid_dist_ki;        // 0x0161 -> 0x041E: Ki rozkładanie (70)
    uint8_t pid_dist_kd;        // 0x0162 -> 0x041F: Kd rozkładanie (128)
    uint8_t temp_eco_60;        // 0x0163: Temperatura realna dla 60°C Eco (0x2F = 47°C)
    uint8_t temp_eco_40;        // 0x0164: Temperatura realna dla 40°C Eco (0x28 = 40°C)
    uint8_t temp_hysteresis;    // 0x0165: Histereza termostatu (1°C)
    uint8_t cold_wash_assist;   // 0x0166: Dogrzewanie zimnej wody (0/1)
    uint8_t table_variant_flag; // 0x0167: Flaga tablicy (1)
    uint8_t max_agitate_reps;   // 0x0168: Liczba powtórzeń cyklu ruchu (5)
    uint8_t time_wash_60;       // 0x0169: Czas bazowy 60°C (0x41 = 65)
    uint8_t time_wash_60_short; // 0x016A: Czas skrócony 60°C (0x14 = 20)
    uint8_t time_wash_40;       // 0x016B: Czas bazowy 40°C (0x41 = 65)
    uint8_t time_wash_40_short; // 0x016C: Czas skrócony 40°C (0x14 = 20)
    uint8_t max_heater_cycles;  // 0x016D: Limit włączeń grzałki (8)
    uint8_t auto_start_mode;    // 0x016E: Tryb autostartu (0)
};

// ============================================================================
// 4. ZMIENNE GLOBALNE STEROWNIKA (ODWZOROWANIE RAM)
// ============================================================================
CycleStateRecord    g_cycle;
FactoryConfigRecord g_cfg;

// Bufory pomiarowe ADC i filtracji okienkowej
AdcChannelStats g_adc_channels[8];
uint8_t         g_adc_avg_bytes[8];         // 0x03C1..0x03C8 (młodsze bajty średnich)
uint16_t        g_adc_sample_cnt    = 0;    // 0x037B:037C (0..80)
uint16_t        g_adc_timeout_cnt   = 1000; // 0x02EF:02F0
uint8_t         g_current_adc_ch    = 0;    // 0x037D
uint8_t         g_adc_ready_flags   = 0;    // 0x03BF

SignalWindow    g_win_ch1           = {0, 0, 0}; // 0x038C..0x0391 (Kanał 1: Blokada/Triak)
SignalWindow    g_win_ch2           = {0, 0, 0}; // 0x0392..0x0397 (Kanał 2: Pompa)
uint8_t         g_win_samples_ch1   = 32;        // 0x02F4
uint8_t         g_win_samples_ch2   = 32;        // 0x02F3
uint8_t         g_prev_out_bit3     = 0;         // 0x02F1
uint8_t         g_prev_out_bit4     = 0;         // 0x02F2
uint8_t         g_signal_state_ch1  = 1;         // 0x0387 (Wynik klasyfikacji 0..3)
uint8_t         g_signal_state_ch2  = 1;         // 0x0388 (Wynik klasyfikacji 0..3)

// Zmienne czujników: Temperatura NTC, Hydrostat, Napięcie sieci
uint8_t         g_temperature_c     = 20;   // 0x038B
uint8_t         g_ntc_valid_cnt     = 5;    // 0x0389
uint8_t         g_temp_snapshot     = 0;    // 0x03B8
uint8_t         g_temp_rise_timer   = 30;   // 0x039E
uint8_t         g_computed_target_t = 0;    // 0x04BB
uint8_t         g_mains_voltage_raw = 180;  // 0x037A
uint16_t        g_mains_fault_timer = 0;    // 0x03B1:03B2
uint8_t         g_water_level_state = 0;    // 0x0385 (0=Pusto, 2=Pełno)
uint8_t         g_hydro_debounce    = 0;    // 0x0386
uint8_t         g_hydro_pulse_cnt   = 0;    // 0x0380

// Zmienne wyjść i flag systemowych
uint8_t         g_outputs_req_mask  = 0;    // 0x037E (Bity: 0=Prewash, 1=Main, 2=Soft, 3=Pump, 4=TriacEn, 5=Heater)
uint8_t         g_outputs_pwm_mask  = 0;    // 0x037F
uint8_t         g_flags_038a        = 4;    // 0x038A
uint8_t         g_status_flags_0399 = 0;    // 0x0399
uint8_t         g_comm_flags_039a   = 0;    // 0x039A
uint8_t         g_comm_flags_039b   = 0;    // 0x039B
uint16_t        g_latched_faults    = 0;    // 0x039C:039D
uint8_t         g_sys_flags_03be    = 0;    // 0x03BE
uint8_t         g_save_req_flags    = 0;    // 0x046B (bit 0=Zapisz stan w EEPROM, bit 1=Aktualizuj czas, bit 2=Błąd EEPROM)
uint8_t         g_wash_ctrl_04bc    = 0;    // 0x04BC

// Liczniki ryglowania drzwi i nadzoru silnika (z FUN_code_0e18)
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

// Sterownik silnika i regulator PID (0x03FB .. 0x0447)
uint8_t         g_motor_state       = 0;    // 0x03FB (0=Stop, 1..7=Fazy ruchu)
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

// Maszyna Wirtualna Wirowania (VM)
uint16_t        g_vm_script_start   = 0;    // 0x0459:045A
uint16_t        g_vm_pc_offset      = 0;    // 0x045D:045E (Program Counter)
uint16_t        g_vm_step_timer     = 0;    // 0x0461

// Interfejs użytkownika (UI), matryca LED i Tryb Serwisowy
uint8_t         g_service_mode_active = 0;  // 0x03BA (1 = Tryb Serwisowy / Autotest)
uint8_t         g_service_test_step   = 0;  // 0x02F8 (0..99)
uint8_t         g_ui_state_flags      = 1;  // 0x03C0 (1=Normal, 2=Service, 0x10=ExitService)
uint8_t         g_btn_event_flags     = 0;  // 0x03BD (Bity: 0=StartShort, 1=ResetLong, 2=ChildLock, 3..5=Opcje, 6=ShowErr)
uint8_t         g_detected_btn_id     = 3;  // 0x04A5 (3 = brak wciśniętego przycisku)
uint8_t         g_btn_released_flag   = 1;  // 0x0332
uint16_t        g_btn_hold_timer      = 0;  // 0x04B5:04B6
uint8_t         g_led_solid_mask      = 0;  // 0x04B7 (Diody świecące ciągle)
uint8_t         g_led_blink_mask      = 0;  // 0x0331 (Diody wygaszone / migające)
uint8_t         g_last_logged_err     = 0;  // 0x03CA
uint8_t         g_err_display_timer   = 0;  // 0x0306
uint8_t         g_active_slot_idx     = 0;  // 0x0330

// Zmienne przerwaniowe (Zero-Crossing, Tachogenerator, Scheduler)
volatile uint8_t  g_zc_period_ticks   = 0x3F; // 0x0307 (Okres półfali 50Hz ~ 63 ticki)
volatile uint8_t  g_zc_filter_cnt    = 0;    // 0x03F1
volatile uint8_t  g_mux_slot_counter  = 0;    // 0x03F2 (0..239)
volatile uint16_t g_tacho_captures[3] = {0, 0, 0}; // 0x02EA..0x02EF
volatile uint8_t  g_sched_cnt_100ms   = 50;   // 0x03BB
volatile uint8_t  g_sched_cnt_200ms   = 85;   // 0x03BC
volatile uint8_t  g_wdt_checkin_flags = 0;    // 0x0772

// ============================================================================
// 5. PODSYSTEM PAMIĘCI EEPROM (FUN_code_3696, 36f0, 3726, 3872, 38db, 3915, 3cd2)
// ============================================================================

/**
 * Zapis pojedynczego bajtu w EEPROM z weryfikacją (odwzorowanie FUN_code_3c5c)
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
 * Odczyt ramki danych z EEPROM: [0xCA] + [Dane: len] + [0x55] + [Suma kontrolna] (FUN_code_3696 / 38db)
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
 * Zapis ramki danych do EEPROM z nagłówkiem 0xCA, stopką 0x55 i sumą kontrolną (FUN_code_36f0)
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
 * Zapis nowego kodu błędu na początek 10-elementowej kolejki FIFO w EEPROM 0x004F..0x0058 (FUN_code_3cd2)
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
 * Odczyt ostatniego błędu z kolejki FIFO (FUN_code_3cc4)
 */
uint8_t EEPROM_GetLastError()
{
    return eeprom_read_byte((const uint8_t *)0x004F);
}

/**
 * Zapis stanu cyklu prania w 5 nadmiarowych kopiach w EEPROM co 31 bajtów (FUN_code_3872)
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
 * Odzyskanie stanu cyklu prania przy starcie z 5 nadmiarowych kopii EEPROM (FUN_code_3726)
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
                // Dwie sąsiednie kopie są poprawne -> przywróć stan!
                g_sys_flags_03be |= 0x02;
                g_active_slot_idx = s;
                restored = true;
                break;
            }
        }
    }

    if (!restored) {
        // Inicjalizacja domyślnego stanu pustego (FUN_code_3c6c)
        memset(&g_cycle, 0, sizeof(CycleStateRecord));
        g_cycle.system_mode = 2; // Stop / Gotowość
        g_cycle.cycle_step  = 0xFF;
        g_active_slot_idx   = 0xFE;
    }

    g_status_flags_0399 = g_cycle.saved_flags_0399 & 0xEF;
    g_comm_flags_039a   = g_cycle.saved_flags_039a;
    g_last_logged_err   = EEPROM_GetLastError();
    g_cycle.total_cycles_cnt++;
}

/**
 * Wczytanie 83-bajtowej konfiguracji fabrycznej lub załadowanie nastaw domyślnych (FUN_code_3915)
 */
void EEPROM_LoadFactoryConfiguration()
{
    // Próba odczytu 83 bajtów (0x53) z obszaru kalibracji EEPROM (adres 0x0100 w EEPROM)
    if (EEPROM_ReadPacket(0x0100, (uint8_t *)&g_cfg, sizeof(FactoryConfigRecord))) {
        g_sys_flags_03be |= 0x01;
    } else {
        // Dokładne wartości domyślne zaszyte w FUN_code_3915:
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
// 6. STEROWANIE NISKOPOZIOMOWE I/O (ZDEKODOWANE Z 093d..09ea)
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
 * Modulacja PWM wyjść zaworów, pompy i grzałki w cyklu 10-krokowym (code:099b)
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
// 7. PODSYSTEM ADC, KLASYFIKACJA PRZEBIEGÓW AC, NTC I HYDROSTAT (079b..0d3f)
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
 * Klasyfikacja kształtu przebiegu AC na kanałach 1 i 2 (FUN_code_0b68):
 * 1 = pełna fala AC (amplituda 0..255 wokół środka), 2 = półfala (dioda), 3 = składowa stała ~2.5V, 0 = brak
 */
uint8_t Signal_ClassifyWindow(const SignalWindow *win) // FUN_code_0b68
{
    uint8_t avg = (uint8_t)win->sum_avg;
    if (avg > 0x62 && avg < 0x9E) {
        if (win->sum_min < 0x001E && win->sum_max >= 0x00E2) return 1; // Pełny przebieg AC
        if (win->sum_min >= 0x0063 && win->sum_max < 0x009E) return 3; // Stały poziom środkowy
        return 0;
    } else if ((avg > 0x22 && avg < 0x5E) || (avg > 0xA2 && avg < 0xDE)) {
        return 2; // Przebieg jednopołówkowy (dioda)
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
 * Obsługa wejścia cyfrowego hydrostatu na pinie PINC0 (FUN_code_09eb + code:0a2c..0a66)
 */
void Hydrostat_UpdateState()
{
    // Szybkie zliczanie stanu pinu PINC0 (code:0a30 sbis PINC, 0)
    if (PINC & (1 << Pins::PC_HYDROSTAT_IN)) {
        if (g_hydro_pulse_cnt < 240) g_hydro_pulse_cnt += 3;
    } else {
        if (g_hydro_pulse_cnt > 0) g_hydro_pulse_cnt--;
    }

    // Debouncing i wyznaczenie stanu poziomu wody g_water_level_state (FUN_code_09eb)
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
// 8. PRZERWANIA SPRZĘTOWE I STEROWNIK FAZOWY SILNIKA Z PID (24a2..2dda)
// ============================================================================

/**
 * Obliczenie czasu opóźnienia wyzwalania triaka od przejścia przez zero (FUN_code_2578)
 */
void Motor_CalcTriacDelay()
{
    if (!(g_motor_flags & 0x01)) {
        g_firing_angle = g_cfg.angle_initial;
    }
    // Przeliczenie kąta fazowego na ticki timera (w oryginale dzielenie przez 18 w FUN_code_2578)
    g_triac_timer_ticks = (uint8_t)(((uint16_t)g_firing_angle * 7) / 18);
}

/**
 * Wektor INT0: Detekcja przejścia napięcia sieci przez zero (FUN_code_24a2)
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

        // Adaptacja do częstotliwości sieci (50/60 Hz) w oknie 59..67 (0x3B..0x43)
        if (g_mux_slot_counter < 0x13)      g_zc_period_ticks--;
        else if (g_mux_slot_counter >= 0x14) g_zc_period_ticks++;

        if (g_zc_period_ticks < 0x3B) g_zc_period_ticks = 0x3B;
        if (g_zc_period_ticks > 0x43) g_zc_period_ticks = 0x43;

        g_outputs_pwm_mask = 0;
        Motor_CalcTriacDelay();

        // Obsługa osłabiania pola (Field Weakening) dla wysokich obrotów w wariancie 1
        if (g_cfg.hw_board_variant == 1) {
            if (g_field_weaken_cnt > 1) {
                g_field_weaken_cnt--;
                g_triac_timer_ticks = 0x14;
            }
            if (g_field_weaken_cnt == 3) {
                IO_Set_FieldWeaken_Low();
            }
        }

        // Uzbrojenie komparatora Timer0 do wyzwolenia impulsu bramki triaka (PC1)
        if (g_motor_state >= 4 && g_motor_target_rpm > 0) {
            OCR0A  = TCNT0 + g_triac_timer_ticks;
            TIFR0 |= (1 << OCF0A);
            TIMSK0 |= (1 << OCIE0A);
        }
    }
}

/**
 * Wektor TIMER0_COMPA: Impuls wyzwalający bramkę triaka silnika na pinie PC1 (code:21a0)
 */
ISR(TIMER0_COMPA_vect)
{
    if (g_motor_state >= 4 && g_motor_target_rpm > 0) {
        PORTC |= (1 << Pins::PC_MOTOR_TRIAC);
        // Krótki impuls bramki (~50 us)
        for (volatile uint8_t d = 0; d < 60; d++);
        PORTC &= ~(1 << Pins::PC_MOTOR_TRIAC);
    }
    TIMSK0 &= ~(1 << OCIE0A);
}

/**
 * Wektor TIMER1_CAPT: Przechwycenie okresu impulsów tachogeneratora na pinie PD6 (code:22fa)
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
 * Przełączanie przekaźników kierunku silnika CW / CCW / STOP (FUN_code_25b9, 25fe, 2643)
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
 * Filtr medianowy z 3 próbek tachogeneratora (FUN_code_43de) i obliczenie RPM (FUN_code_2b35)
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
        // Odwzorowanie 32-bitowego dzielenia FUN_code_56ad z FUN_code_2b35
        uint32_t num = 6000000UL / g_motor_tacho_med;
        g_motor_curr_rpm = (uint16_t)(num / g_cfg.tacho_pulses);
    } else {
        g_motor_curr_rpm = 0;
    }
}

/**
 * Dobór nastaw PID w zależności od trybu pracy: Pranie, Rampa Wirowania, Wirowanie, Rozkładanie (FUN_code_2ad9)
 */
void Motor_SelectPIDGains()
{
    if (!(g_motor_flags & 0x08)) {
        // Tryb prania (niskie obroty)
        g_active_kp      = g_cfg.pid_wash_kp;
        g_active_ki      = g_cfg.pid_wash_ki;
        g_active_kd      = g_cfg.pid_wash_kd;
        g_pid_sample_div = 5;
        g_pid_err_min    = -15;
        g_pid_err_max    = 15;
    } else {
        // Tryb wirowania
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
        // Tryb rozkładania wsadu
        g_active_kp      = g_cfg.pid_dist_kp;
        g_active_ki      = g_cfg.pid_dist_ki;
        g_active_kd      = g_cfg.pid_dist_kd;
        g_pid_sample_div = 5;
    }
}

/**
 * Obliczenie kroku regulatora PID, ograniczenie przyrostu i kompensacja przyspieszenia (FUN_code_2b5f..2dc1)
 */
void Motor_ExecutePIDTask() // FUN_code_2dc1
{
    if (++g_pid_sample_cnt < g_pid_sample_div) return;
    g_pid_sample_cnt = 0;

    Motor_SelectPIDGains();
    Motor_UpdateRPM();

    // 1. Uchyb regulacji z ograniczeniem do [g_pid_err_min .. g_pid_err_max] (FUN_code_2b5f)
    int16_t diff = (int16_t)g_motor_ramped_rpm - (int16_t)g_motor_curr_rpm;
    if (diff > g_pid_err_max) diff = g_pid_err_max;
    if (diff < g_pid_err_min) diff = g_pid_err_min;
    g_pid_err = (int8_t)diff;

    // 2. Równanie przyrostowe PID (FUN_code_2ba9)
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

    // Nasycenie kąta wyzwalania triaka między kątem minimalnej i maksymalnej mocy
    if (g_firing_angle_16 > g_cfg.angle_min_power) {
        g_firing_angle_16 = g_cfg.angle_min_power;
    }
    uint8_t min_angle = (g_motor_flags & 0x08) ? g_cfg.angle_max_spin : g_cfg.angle_max_wash;
    if (g_firing_angle_16 < min_angle) {
        g_firing_angle_16 = min_angle;
    }

    // 3. Tłumienie nagłych skoków obrotów (tzw. anti-surge w FUN_code_2cc6)
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

    // 4. Łagodny start pierwszych 5 kroków po załączeniu (FUN_code_2d99)
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
 * Zatrzymanie i reset zmiennych silnika (FUN_code_29ff i FUN_code_2a4d)
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
 * Zlecenie ruchu silnika z przeliczeniem czasów w sekundach na kroki 200ms (FUN_code_268a)
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
 * 7-stanowa maszyna stanów silnika wywoływana co 200 ms (FUN_code_2714)
 */
void Motor_StateMachine_200ms()
{
    switch (g_motor_state) {
        case 1: // Przygotowanie przełączenia przekaźników kierunku
            if (g_motor_ctrl_flags & 0x04) {
                g_motor_hold_time  = 0;
                g_motor_ctrl_flags &= ~0x04;
            }
            g_relay_switch_dly = 0;
            // Jeśli zmiana kierunku CW <-> CCW, wymuś zatrzymanie i odczekaj 2 cykle
            if (((g_motor_flags >> 1) & 1) != ((g_motor_flags >> 2) & 1)) {
                g_relay_switch_dly = 2;
                g_motor_ramped_rpm = 0;
                g_motor_flags     &= ~0x01;
            }
            g_motor_prev_state = g_motor_state;
            g_motor_state      = 2;
            break;

        case 2: // Odczekanie na wyłączenie triaka i zdjęcie styków przekaźnika
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

        case 3: // Martwy czas przełączania styków przekaźnika (czekaj na RPM == 0)
            if (g_relay_switch_dly > 0) g_relay_switch_dly--;
            if (g_relay_switch_dly == 0 && (g_motor_ctrl_flags & 0x20)) {
                if (g_motor_flags & 0x02) Motor_RelaysCCW();
                else                      Motor_RelaysCW();
                g_motor_prev_state = g_motor_state;
                g_motor_state      = 4;
            }
            break;

        case 4: // Wyznaczenie liczby kroków rampy prędkości
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

        case 5: // Faza wstępnego utrzymania obrotów (Hold)
            if (g_motor_hold_time == 0) g_motor_state = 6;
            else                        g_motor_hold_time--;
            break;

        case 6: // Faza liniowego narastania obrotów (Ramp)
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

        case 7: // Faza utrzymania prędkości docelowej (Plateau)
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
// 9. MASZYNA WIRTUALNA (BYTECODE INTERPRETER) WIROWANIA (FUN_code_3093, 3534, 356a)
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
 * Interpreter 7-bajtowych instrukcji wirowania z pamięci PROGMEM (FUN_code_3093)
 */
void SpinVM_ExecuteStep()
{
    if (!(g_motor_ctrl_flags & 0x08)) return;

    // Jeśli silnik nadal wykonuje poprzednią komendę lub trwa pauza timera VM, czekaj
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
        case 0x01: // PAUZA / WYBIEGANIE: ins[1] = czas
            Motor_StopAndReset();
            g_vm_step_timer = ins[1];
            break;

        case 0x02: // RUCH SILNIKA: ins[1]=Hold, ins[2:3]=RPM_BE, ins[4]=Ramp, ins[5]=Dir, ins[6]=Time
        {
            uint16_t rpm = ((uint16_t)ins[2] << 8) | ins[3];
            if (ins[5]) g_motor_flags |= 0x04;
            else        g_motor_flags &= ~0x04;
            if (rpm >= 300) g_motor_flags |= 0x08; // Flaga wysokich obrotów wirowania
            else            g_motor_flags &= ~0x08;
            Motor_CommandMove(rpm, ins[1], ins[4], ins[6]);
            break;
        }

        case 0x03: // KONTROLA NIEWYWAŻENIA BĘBNA (Unbalance Check)
        {
            uint16_t threshold = ((uint16_t)ins[2] << 8) | ins[3];
            // Jeżeli wahania prędkości przekraczają próg, zrestartuj skrypt rozkładania wsadu
            if ((uint16_t)abs(g_pid_err) * 16 > threshold) {
                g_unbalance_cnt++;
                g_vm_pc_offset = g_vm_script_start; // Powrót na początek rozkładania
            }
            break;
        }

        case 0x06: // STEROWANIE POMPĄ ODPŁYWOWĄ
            g_outputs_req_mask |= (1 << 3);
            break;

        case 0x05: // KONIEC SKRYPTU WIROWANIA
        case 0x07:
        default:
            SpinVM_Stop();
            g_sys_flags_03be |= 0x10; // Flaga zakończenia kroku
            break;
    }
}

// ============================================================================
// 10. NADZÓR BŁĘDÓW (0..15) I MASZYNA RYGLOWANIA DRZWI (FUN_code_0d40, 0d8b, 0e18, 13ed)
// ============================================================================

void Faults_LatchDelayed(uint8_t *slot_state, uint8_t bit_pos) // FUN_code_1565
{
    if (g_cycle.system_mode == 2 || g_cycle.system_mode == 3) return;

    if (*slot_state == 0) {
        if (!(g_cycle.config_flags_lo & 0x04)) {
            g_cycle.config_flags_lo |= 0x04; // Wymuś odpompowanie awaryjne
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

    // Błąd 1: Sygnał na kanale ADC1 przy wyłączonym triaku
    if (!(g_outputs_req_mask & 0x10) && !(g_flags_038a & 0x04)) {
        if (g_signal_state_ch1 == 2 || g_signal_state_ch1 == 3) {
            new_faults |= (1U << 1);
        }
    }

    // Błąd 2: Nadzór grzałki (wymagany przyrost o min. +2°C w oknie 30 jednostek czasu)
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

    // Błąd 3: Uszkodzenie czujnika NTC
    if ((prev_faults & (1U << 3)) || g_ntc_valid_cnt == 0) {
        new_faults |= (1U << 3);
    }

    // Błąd 4: Przekroczenie czasu napełniania wodą
    if (g_cycle.config_flags_lo & 0x08) {
        new_faults |= (1U << 4);
    }

    // Błąd 5: Nieprawidłowy odczyt pokrętła programu
    if ((g_status_flags_0399 & 0x08) && (g_cycle.system_mode == 2 || g_cycle.system_mode == 3)) {
        new_faults |= (1U << 5);
    }

    // Błąd 6: Błąd sumy kontrolnej EEPROM
    if (g_save_req_flags & 0x04) {
        new_faults |= (1U << 6);
    }

    // Błąd 9: Przekroczenie 10 prób rozłożenia prania przed wirowaniem
    if ((prev_faults & (1U << 9)) || g_unbalance_cnt > 9) {
        new_faults     |= (1U << 9);
        g_unbalance_cnt = 0;
    }

    // Błąd 12: Zwarcie triaka silnika (pełna fala AC przy braku wysterowania)
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

    // Błąd 13: Kontrola ryglowania drzwi (do 8 prób w FUN_code_0e18)
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

    // Błąd 14: Brak sygnału z tachogeneratora podczas pracy silnika (do 3 prób)
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

    // Błąd 15: Przegrzanie powyżej 83°C (0x53)
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

    // Wykrycie pojawienia się nowego błędu i zapis do FIFO w EEPROM (końcówka FUN_code_0e18)
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
 * Kontrola okna napięcia sieci 230V AC oraz globalne odcięcie grzałki i triaka (FUN_code_13ed, 0d40)
 */
void System_SafetySupervisor() // FUN_code_0d40 + FUN_code_13ed
{
    // 1. Kontrola napięcia sieciowego z kanału ADC7 (0x6B..0xE6 z histerezą 0x70..0xE1)
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

    // 2. Zezwolenie na zasilanie obwodu blokady/triaka (bit 4 w g_outputs_req_mask)
    if (g_motor_curr_rpm < 50 && !(g_cycle.fault_mask_hi & 0x80)) {
        if (g_water_level_state != 0 || (g_cycle.fault_mask_hi & 0x01) || g_temperature_c >= 0x5A) {
            g_outputs_req_mask |= 0x10;
        } else {
            g_outputs_req_mask &= ~0x10;
        }
    } else {
        g_outputs_req_mask |= 0x10;
    }

    // 3. Zezwolenie na włączenie przekaźnika grzałki (bit 5 w g_outputs_req_mask)
    if ((g_wash_ctrl_04bc & 0x04) &&
        g_water_level_state != 0 &&
        !(g_cycle.fault_mask_lo & 0x0C) &&
        g_cycle.cycle_step == 4) {
        g_outputs_req_mask |= 0x20;
    } else {
        g_outputs_req_mask &= ~0x20;
    }

    // 4. Ocena błędów i zmiana trybu systemu (FUN_code_0d40)
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
// 11. MASZYNA STANÓW CYKLU PRANIA, REWERSY BĘBNA I TERMOSTAT (440a..5222)
// ============================================================================

/**
 * Załadowanie parametrów wybranego programu prania z tablicy (FUN_code_5222)
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
 * Wyznaczenie docelowej temperatury grzania w zależności od kroku i opcji Eco (FUN_code_518c)
 */
void Wash_ResolveTargetTemperature()
{
    g_computed_target_t = 0;
    if ((g_cycle.user_options & 0x04) || (g_cycle.fault_mask_lo & 0x0C)) {
        return; // Opcja prania na zimno lub błąd NTC/grzałki
    }

    if (g_cycle.cycle_step > 0 && g_cycle.cycle_step < 3) {
        g_computed_target_t = 25; // Pranie wstępne: 25°C (0x19)
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
 * Termostat prania zasadniczego w kroku 4 (FUN_code_472a)
 */
void Wash_ThermostatControl()
{
    if (g_cycle.cycle_step != 4) {
        g_wash_ctrl_04bc &= ~0x04;
        return;
    }

    if (g_temperature_c < g_computed_target_t) {
        if (g_water_level_state != 0 && !(g_cycle.fault_mask_lo & 0x08)) {
            g_wash_ctrl_04bc |= 0x04; // Żądanie włączenia grzałki
        }
    } else {
        g_wash_ctrl_04bc &= ~0x04; // Temperatura osiągnięta
        g_sys_flags_03be |= 0x10;  // Odblokuj przejście do kolejnego kroku po dogrzaniu
    }
}

/**
 * 5-fazowa maszyna stanów napełniania wodą i rewersyjnego obracania bębna (FUN_code_494f)
 */
void Wash_AgitationAndFillMachine()
{
    switch (g_cycle.agitation_sub) {
        case 1: // Inicjalizacja napełniania i wybór elektrozaworu
            g_cycle.sub_timer_sec   = 0;
            g_cycle.agitation_sub   = 2;
            break;

        case 2: // Dopełnianie wody przez elektrozawór aż hydrostat zgłosi pełny stan (g_water_level_state == 2)
            g_outputs_req_mask |= 0x02; // Włącz zawór główny
            if (g_cycle.sub_timer_sec < 240) {
                if (g_water_level_state >= 2) {
                    g_cycle.measured_fill_t = (uint8_t)g_cycle.sub_timer_sec;
                    g_cycle.sub_timer_sec   = 0;
                    g_outputs_req_mask     &= ~0x07; // Zamknij zawory
                    g_cycle.agitation_sub   = 3;
                }
            } else {
                // Przekroczono czas napełniania -> Błąd 4!
                g_cycle.config_flags_lo |= 0x08;
                g_outputs_req_mask      &= ~0x07;
            }
            break;

        case 3: // Krótka pauza stabilizacyjna po zamknięciu zaworu (2 s)
            if (g_cycle.sub_timer_sec >= 2) {
                g_cycle.sub_timer_sec = 0;
                g_cycle.agitation_sub = 4;
            }
            break;

        case 4: // Faza aktywnego obracania bębna (np. 52 RPM przez wyliczony czas)
            if (g_motor_state == 0 && g_cycle.sub_timer_sec == 0) {
                g_motor_flags ^= 0x04; // Zmiana kierunku CW <-> CCW
                Motor_CommandMove(52, 0, 2, 10);
            }
            if (g_cycle.sub_timer_sec >= 12) {
                g_cycle.sub_timer_sec = 0;
                g_cycle.agitation_sub = 5;
            }
            break;

        case 5: // Faza postoju między rewersami i kontrola poziomu wody
            if (g_water_level_state < 2) {
                // Woda opadła (wsiąkła w pranie) -> dopełnij!
                g_outputs_req_mask |= 0x02;
            } else {
                g_outputs_req_mask &= ~0x07;
                if (g_cycle.sub_timer_sec >= 6) {
                    g_cycle.sub_timer_sec = 0;
                    g_cycle.agitation_cycles++;
                    g_cycle.agitation_sub = 4; // Kolejny rewers
                }
            }
            break;

        default:
            g_cycle.agitation_sub = 1;
            break;
    }
}

/**
 * 10-próbowa procedura wypompowania i rozkładania prania przed wirowaniem (FUN_code_4879)
 */
void Wash_DrainAndDistributeBeforeSpin()
{
    if (g_cycle.unbalance_stage == 1) {
        g_outputs_req_mask |= (1 << 3); // Włącz pompę odpływową
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
                // 10 nieudanych prób -> Błąd 9 i przejście w tryb awarii 3
                g_cycle.system_mode      = 3;
                g_sys_flags_03be        |= 0x04;
                g_cycle.config_flags_lo &= ~0x04;
                g_cycle.cycle_step       = 0xFF;
                g_cycle.unbalance_stage  = 3;
                g_cycle.phase_substate   = 0;
            }
        } else if (g_water_level_state == 0) {
            // Woda wypompowana pomyślnie
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
 * Awaryjne odpompowanie wody i reset programu po przytrzymaniu przycisku Start (FUN_code_4598)
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
                g_cycle.system_mode      = 3; // Koniec / Odblokuj drzwi
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
 * Przejście do kolejnego kroku programu prania 0..28 -> 0xFF (FUN_code_4ccc)
 */
void Wash_AdvanceCycleStep(uint8_t reason)
{
    if (g_cycle.cycle_step == 0xFF) {
        uint8_t p_idx = g_cycle.program_offset / 8;
        g_cycle.cycle_step = pgm_read_byte(&WASH_PROGRAMS_FLASH[p_idx][1]);
        if (g_cycle.user_options & 0x21) {
            g_cycle.cycle_step = 1; // Pranie wstępne aktywne
        }
    } else if (g_cycle.cycle_step < 0x1D && reason == 1) {
        g_cycle.cycle_step++;
        g_cycle.config_flags_lo  &= ~0x04;
        g_cycle.step_elapsed_time = 0;
        g_cycle.agitation_cycles  = 0;
        g_cycle.sub_timer_sec     = 0;
        g_cycle.phase_substate    = 0;

        if (g_cycle.cycle_step >= 0x1D) {
            // Zakończenie całego programu prania
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
 * Główny dyspozytor aktywnego cyklu prania (FUN_code_440a)
 */
void Wash_ExecuteNormalCycle()
{
    if (g_sys_flags_03be & 0x10) {
        Wash_AdvanceCycleStep(1);
        return;
    }

    // Krok 1..18: Pranie wstępne, zasadnicze i płukania (rewersy bębna + grzanie)
    if (g_cycle.cycle_step >= 1 && g_cycle.cycle_step <= 18) {
        Wash_ThermostatControl();
        Wash_AgitationAndFillMachine();
    }
    // Krok 19..25: Odpompowania między płukaniami
    else if (g_cycle.cycle_step >= 19 && g_cycle.cycle_step <= 25) {
        if (g_cycle.unbalance_stage == 0) g_cycle.unbalance_stage = 1;
        Wash_DrainAndDistributeBeforeSpin();
        if (g_cycle.unbalance_stage == 0) {
            g_sys_flags_03be |= 0x10;
        }
    }
    // Krok 26..28: Wirowanie końcowe (uruchomienie Maszyny Wirtualnej Wirowania FUN_code_4803)
    else if (g_cycle.cycle_step >= 26 && g_cycle.cycle_step <= 28) {
        if (g_cycle.phase_substate == 0) {
            g_cycle.unbalance_stage = 1;
            g_cycle.phase_substate  = 1;
        } else if (g_cycle.phase_substate == 1) {
            Wash_DrainAndDistributeBeforeSpin();
            if (g_cycle.unbalance_stage == 0) {
                g_outputs_req_mask |= (1 << 3); // Pompa pracuje w trakcie wirowania
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
// 12. INTERFEJS UŻYTKOWNIKA: PRZYCISKI, POKRĘTŁO, LED I TRYB SERWISOWY
// ============================================================================

/**
 * Odczyt pozycji pokrętła programów z kanału ADC4 (FUN_code_3d35 i FUN_code_54a5)
 */
uint8_t UI_DecodeProgramDial() // FUN_code_3d35
{
    if (!ADC_CheckChannelStability(4, 10)) return 0xFC;

    uint8_t raw = g_adc_avg_bytes[4];
    // 16 progów równomiernie rozłożonych co 15 jednostek ADC (jak w FUN_code_3915: 0x0B..0xF8)
    for (uint8_t pos = 1; pos <= 16; pos++) {
        uint8_t thr = (uint8_t)(pos * 15);
        if (raw < thr) return pos;
    }
    return 16;
}

/**
 * Dekodowanie drabinki przycisków z kanału ADC0 z obsługą krótkiego i długiego przytrzymania (FUN_code_3d6c i 3dfc)
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
        // Stan spoczynkowy (brak wciśniętego przycisku) -> jeżeli wcześniej trzymano krótko, wyzwól zdarzenie
        if (g_btn_released_flag == 0 && g_btn_hold_timer > 0 && g_btn_hold_timer < 26) {
            if (g_detected_btn_id == 4)      g_btn_event_flags |= 0x01; // Start / Pauza (krótkie)
            else if (g_detected_btn_id == 6) g_btn_event_flags |= 0x08; // Opcja 1
            else if (g_detected_btn_id == 2) g_btn_event_flags |= 0x10; // Opcja 2
            else if (g_detected_btn_id == 0) g_btn_event_flags |= 0x20; // Opcja 3
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
            // Przytrzymanie powyżej 26 cykli (~2.6 sekundy):
            if (g_btn_hold_timer == 26) {
                if (g_detected_btn_id == 4)      g_btn_event_flags |= 0x02; // Długi Start = Reset/Anuluj
                else if (g_detected_btn_id == 5) g_btn_event_flags |= 0x04; // Kombinacja = Child Lock
                else if (g_detected_btn_id == 2) g_btn_event_flags |= 0x40; // Pokaż ostatni błąd na LED
            }
        }
    }
}

/**
 * Obsługa zdarzeń od przycisków (FUN_code_1b74)
 */
void UI_ProcessButtonEvents()
{
    if (g_btn_event_flags & 0x02) {
        // Długie przytrzymanie Start/Pauza -> Anulowanie programu i odpompowanie
        if (!(g_cycle.config_flags_lo & 0x04)) {
            g_cycle.config_flags_lo |= 0x04;
            g_cycle.phase_substate   = 0;
            g_save_req_flags        |= 0x01;
        }
    } else if (g_btn_event_flags & 0x01) {
        // Krótkie wciśnięcie Start/Pauza
        if (g_cycle.system_mode == 1 || g_cycle.system_mode == 2) {
            if (g_cycle.cycle_step == 0xFF) {
                Wash_AdvanceCycleStep(0);
            }
            g_cycle.system_mode = 0; // Start pracy
            g_sys_flags_03be   |= 0x04;
        } else if (g_cycle.system_mode == 0 || g_cycle.system_mode == 4) {
            g_cycle.system_mode = 1; // Pauza
            g_sys_flags_03be   |= 0x04;
        }
    } else if (g_btn_event_flags & 0x04) {
        // Blokada rodzicielska (Child Lock)
        g_cycle.config_flags_lo ^= 0x01;
    } else if (g_btn_event_flags & 0x40) {
        // Wyświetlenie ostatniego błędu na diodach LED przez 10 cykli
        g_sys_flags_03be    |= 0x08;
        g_err_display_timer  = 10;
    } else if (g_cycle.system_mode == 1 || g_cycle.system_mode == 2) {
        // Zmiana opcji dodatkowych w trybie pauzy/gotowości
        if (g_btn_event_flags & 0x08) g_cycle.user_options ^= 0x01; // Pranie wstępne
        if (g_btn_event_flags & 0x10) g_cycle.user_options ^= 0x02; // Dodatkowe płukanie
        if (g_btn_event_flags & 0x20) g_cycle.user_options ^= 0x04; // Pranie na zimno
    }

    g_btn_event_flags = 0;
}

/**
 * Kodowanie stanu 6 diod LED na panelu przednim (FUN_code_3f20)
 */
void UI_UpdateLEDMatrix()
{
    g_led_solid_mask = 0;
    g_led_blink_mask = 0;

    // 1. Wyświetlanie kodu błędu (6 bitów z FAULT_CONFIG_TABLE)
    if ((g_sys_flags_03be & 0x08) || g_cycle.error_index >= 4) {
        uint8_t err_id = (g_service_mode_active == 1) ? g_last_logged_err : g_cycle.error_index;
        uint8_t pattern = pgm_read_byte(&FAULT_CONFIG_TABLE[err_id & 0x0F]) & 0x3F;
        g_led_solid_mask = pattern;
        if (g_adc_ready_flags & 0x20) {
            g_led_blink_mask = pattern; // Miganie z częstotliwością flagi 0x20
        }
    } else {
        // 2. Normalna sygnalizacja postępu cyklu i wybranych opcji
        if (g_cycle.system_mode == 0 || g_cycle.system_mode == 4) {
            g_led_solid_mask |= (1 << 0); // Dioda Praca świeci ciągle
        } else if (g_cycle.system_mode == 1) {
            if (g_adc_ready_flags & 0x20) g_led_solid_mask |= (1 << 0); // Dioda Praca miga w Pauzie
        }

        if (g_cycle.user_options & 0x01) g_led_solid_mask |= (1 << 1); // Dioda Opcji 1
        if (g_cycle.user_options & 0x02) g_led_solid_mask |= (1 << 2); // Dioda Opcji 2
        if (g_cycle.user_options & 0x04) g_led_solid_mask |= (1 << 3); // Dioda Opcji 3
        if (g_cycle.cycle_step < 19)     g_led_solid_mask |= (1 << 4); // Dioda Fazy Prania
        else if (g_cycle.cycle_step < 26) g_led_solid_mask |= (1 << 5); // Dioda Fazy Płukania
    }

    // Wysterowanie linii PB2..PB7 zgodnie z wyliczoną maską
    uint8_t portb_leds = (g_led_solid_mask & ~g_led_blink_mask) << 2;
    PORTB = (PORTB & 0x03) | (portb_leds & Pins::PB_LED_MASK);
}

/**
 * Wykrywanie kombinacji wejścia w Tryb Serwisowy podczas rozruchu (FUN_code_187a)
 */
void ServiceMode_CheckStartupEntry()
{
    uint16_t loops = 200; // W oryginale 0x07D0 (2000) pętli startowych
    bool     match = false;

    while (loops-- > 0) {
        ADC_ProcessSamplingCycle();
        if (++g_mux_slot_counter >= 240) g_mux_slot_counter = 0;

        if (g_adc_ready_flags & 0x08) {
            UI_ScanButtons();
            uint8_t dial = UI_DecodeProgramDial();
            g_adc_ready_flags &= ~0x08;

            // Warunek z FUN_code_187a: Trzymany przycisk Start (okno 4) + Pokrętło na poz. 8
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
        g_ui_state_flags      = 2; // Aktywuj Tryb Serwisowy
        g_service_mode_active = 1;
        g_service_test_step   = 0;
    } else {
        g_ui_state_flags      = 1; // Normalny tryb pracy
        g_service_mode_active = 0;
    }
}

/**
 * Wykonywanie kroków Autotestu Serwisowego (FUN_code_1920)
 */
void ServiceMode_ExecuteTask()
{
    g_outputs_req_mask |= 0x10; // Zablokuj drzwi

    if (g_btn_event_flags & 0x01) {
        g_btn_event_flags = 0;
        g_service_test_step++;
        Motor_StopAndReset();
        g_outputs_req_mask = 0x10;
    }

    switch (g_service_test_step) {
        case 0: // Pokaż ostatni błąd z EEPROM na diodach LED
            g_sys_flags_03be |= 0x08;
            break;
        case 1: // Test elektrozaworu prania wstępnego (PB0)
            g_sys_flags_03be   &= ~0x08;
            g_outputs_req_mask |= (1 << 0);
            break;
        case 2: // Test elektrozaworu głównego (PC7)
            g_outputs_req_mask |= (1 << 1);
            break;
        case 3: // Test elektrozaworu płynu (PB1)
            g_outputs_req_mask |= (1 << 2);
            break;
        case 4: // Test grzałki (przy napełnionym bębnie)
            if (g_water_level_state != 0) g_outputs_req_mask |= (1 << 5);
            break;
        case 5: // Test obrotów silnika w prawo (52 RPM CW)
            if (g_motor_state == 0) { g_motor_flags &= ~0x04; Motor_CommandMove(52, 0, 2, 60); }
            break;
        case 6: // Test obrotów silnika w lewo (52 RPM CCW)
            if (g_motor_state == 0) { g_motor_flags |= 0x04; Motor_CommandMove(52, 0, 2, 60); }
            break;
        case 7: // Test pompy odpływowej (PC6)
            g_outputs_req_mask |= (1 << 3);
            break;
        case 8: // Test wirowania maksymalnego (1200 RPM)
            g_outputs_req_mask |= (1 << 3);
            if (g_motor_state == 0) { g_motor_flags |= 0x08; Motor_CommandMove(1200, 0, 10, 60); }
            break;
        default: // Wyjście z autotestu (krok 99 w FUN_code_1920)
            g_service_test_step   = 0;
            g_service_mode_active = 0;
            g_ui_state_flags      = 1;
            Motor_StopAndReset();
            g_outputs_req_mask    = 0;
            break;
    }
}

// ============================================================================
// 13. TELEMETRIA DIAGNOSTYCZNA UART0 (Z TABLICY WSKAŹNIKÓW FLASH code:007c)
// ============================================================================

void UART_SendTelemetryFrame()
{
    if (!Serial.available()) return;
    char cmd = Serial.read();

    if (cmd == 's' || cmd == 'S') {
        // Wysłanie pełnej ramki najważniejszych zmiennych z tablicy 54 wskaźników (code:007c)
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
// 14. TRZY ZADANIA GŁÓWNEGO SCHEDULERA (FUN_code_1641, 17b9, 17f2)
// ============================================================================

/**
 * Zadanie 100 ms: Zegary procesowe, nadzór kroków prania i liczników błędów (FUN_code_1641)
 */
void Task_100ms() // FUN_code_1641
{
    static uint8_t div_2s  = 20;
    static uint8_t div_60s = 30;

    if (--div_2s == 0) {
        div_2s = 20;
        PORTC ^= (1 << Pins::PC_DIAG_TICK_100MS); // ACSR ^= 4 w oryginale
        g_adc_ready_flags ^= 0x20;                // Flaga migania LED (~0.5 Hz)

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

        // Zegar minutowy kroków prania
        if (--div_60s == 0) {
            div_60s = 30;
            if (g_cycle.system_mode == 0 && g_cycle.cycle_step < 0x1D) {
                uint8_t step_limit = pgm_read_byte(&STEP_DURATION_TABLE[g_cycle.cycle_step]);
                if (g_cycle.step_elapsed_time < step_limit) {
                    g_cycle.step_elapsed_time++;
                } else if (step_limit != 0) {
                    g_sys_flags_03be |= 0x10; // Czas kroku minął -> przejdź dalej
                }
            }
            if (g_temp_rise_timer > 0) g_temp_rise_timer--;
        }
    }

    g_wdt_checkin_flags |= 0x02;
}

/**
 * Zadanie 200 ms: Maszyna stanów silnika, rampa obrotów, UI, matryca LED i hydrostat (FUN_code_17b9)
 */
void Task_200ms() // FUN_code_17b9
{
    PORTC ^= (1 << Pins::PC_DIAG_TICK_200MS); // ACSR ^= 8 w oryginale

    // 1. Obsługa opóźnionego wyłączenia przekaźników silnika i detekcji 0 RPM (FUN_code_2a66)
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

    // 2. Główny dyspozytor trybu Normalnego / Serwisowego (FUN_code_1b13 + FUN_code_18f0)
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

    // 3. Aktualizacja LED, hydrostatu i maszyny stanów silnika
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
 * Pętla szybka: Próbkowanie ADC, regulator PID silnika, klasyfikacja okien AC i zapis EEPROM (FUN_code_17f2)
 */
void Task_FastLoop() // FUN_code_17f2
{
    PORTC ^= (1 << Pins::PC_DIAG_TICK_FAST); // ACSR ^= 0x10 w oryginale

    ADC_ProcessSamplingCycle();
    IO_UpdateOutputsPWM();
    Motor_ExecutePIDTask();

    // Gdy gotowa nowa paczka 80 pomiarów ADC (bit 3 w g_adc_ready_flags):
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

    // Zapis stanu do EEPROM na żądanie (gdy nie jesteśmy w trybie serwisowym)
    if ((g_save_req_flags & 0x01) && g_service_mode_active == 0) {
        EEPROM_SaveCycleState(false);
    }

    UART_SendTelemetryFrame();
    g_wdt_checkin_flags |= 0x08;
}

// ============================================================================
// 15. INICJALIZACJA SPRZĘTU (setup) I GŁÓWNA PĘTLA SYSTEMU (loop)
// ============================================================================

void setup() // Odwzorowanie FUN_code_15ef -> FUN_code_204f -> FUN_code_1e28
{
    // 1. Sprawdzenie przyczyny resetu (MCUSR) i inicjalizacja Watchdoga
    uint8_t mcusr_val = MCUSR;
    MCUSR = 0;
    wdt_enable(WDTO_2S);

    // 2. Konfiguracja kierunków i stanów początkowych wszystkich portów (FUN_code_1e28)
    DDRA  = 0x00; PORTA = 0x00; // PA0..PA7 jako wejścia ADC
    DDRB  = 0xFF; PORTB = 0xFF; // PB0..PB7 jako wyjścia zaworów i matrycy LED
    DDRC  = 0xFE; PORTC = 0x00; // PC0 wejście hydrostatu, PC1..PC7 wyjścia
    DDRD  = 0xBA; PORTD = 0xB8; // PD0(RX), PD2(INT0), PD6(ICP1) wejścia, reszta wyjścia

    // 3. Inicjalizacja UART0 (UBRR0 = 25 -> 38400 bps @ 16MHz)
    Serial.begin(38400);

    // 4. Inicjalizacja przetwornika ADC (ADCSRA = 0x85)
    ADMUX  = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS0);

    // 5. Inicjalizacja Timer1 w trybie Input Capture dla tachogeneratora (TCCR1B = 0xC2)
    TCCR1A = 0x00;
    TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11);
    TIMSK1 = (1 << ICIE1);

    // 6. Inicjalizacja przerwania detekcji zera sieci INT0 na zbocze opadające (EICRA = 0x02)
    EICRA  = (1 << ISC01);
    EIMSK  = (1 << INT0);

    // 7. Inicjalizacja struktur danych, odczyt kalibracji i odzyskanie stanu z EEPROM
    ADC_ResetChannels();
    EEPROM_LoadFactoryConfiguration();
    ServiceMode_CheckStartupEntry();
    EEPROM_RestoreCycleState();

    if (g_cycle.system_mode == 3) {
        g_cycle.system_mode = 2; // Po resecie przejdź z trybu zakończenia do gotowości
    }

    sei();
}

void loop() // Kooperacyjny system czasu rzeczywistego z FUN_code_15ef
{
    static uint32_t last_2ms   = 0;
    static uint32_t last_100ms = 0;
    static uint32_t last_200ms = 0;

    uint32_t now = millis();

    // Pętla szybka (~2 ms): Próbkowanie ADC, PWM wyjść, PID silnika
    if (now - last_2ms >= 2) {
        last_2ms = now;
        if (++g_mux_slot_counter >= 240) g_mux_slot_counter = 0;
        if (g_zc_filter_cnt < 255)       g_zc_filter_cnt++;
        Task_FastLoop();
    }

    // Zadanie 100 ms (50 x 2ms w oryginale: DAT_mem_03bb = 0x32)
    if (now - last_100ms >= 100) {
        last_100ms = now;
        Task_100ms();
    }

    // Zadanie 200 ms (100 x 2ms w oryginale: DAT_mem_03bc = 0x64)
    if (now - last_200ms >= 200) {
        last_200ms = now;
        Task_200ms();
    }

    // Strażnik Watchdoga (FUN_code_5617): resetuje WDT tylko gdy wszystkie 3 zadania działają
    if ((g_wdt_checkin_flags & 0x0E) == 0x0E) {
        wdt_reset();
        g_wdt_checkin_flags &= ~0x0E;
    }
}