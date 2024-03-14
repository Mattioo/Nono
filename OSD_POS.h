#pragma once

#include <stdint.h>

// OSD elements positions
// in betaflight configurator set OSD elements to your desired positions and in CLI type "set osd" to retrieve the numbers.
// 234 -> not visible. Horizontally 2048-2074(spacing 1), vertically 2048-2528(spacing 32). 26 characters X 15 lines

const uint16_t osd_altitude_pos = 234;                     // Wysokość
const uint16_t osd_numerical_vario_pos = 234;              // Wario numeryczne
const uint16_t osd_avg_cell_voltage_pos = 234;             // Średnie napięcie ogniwa
const uint16_t osd_main_batt_voltage_pos = 234;            // Główne napięcie baterii
const uint16_t osd_crosshairs_pos = 234;                   // Celownik
const uint16_t osd_craft_name_pos = 2048;                  // Nazwa maszyny

const uint16_t osd_gps_sats_pos = 234;                     // Satelity GPS
const uint16_t osd_home_dir_pos = 234;                     // Kierunek domu
const uint16_t osd_home_dist_pos = 234;                    // Odległość do domu
const uint16_t osd_gps_speed_pos = 234;                    // Prędkość GPS
const uint16_t osd_gps_lat_pos = 234;                      // Szerokość geograficzna GPS
const uint16_t osd_gps_lon_pos = 234;                      // Długość geograficzna GPS

const uint16_t osd_pitch_angle_pos = 234;                  // Kąt pitchu
const uint16_t osd_roll_angle_pos = 234;                   // Kąt rollu
const uint16_t osd_rssi_value_pos = 234;                   // Wartość sygnału RSSI
const uint16_t osd_display_name_pos = 234;                 // 
const uint16_t osd_flymode_pos = 234;                      // Tryb lotu
const uint16_t osd_current_draw_pos = 234;                 // Bieżące zużycie prądu
const uint16_t osd_mah_drawn_pos = 234;                    // Wykorzystane miliamperogodziny

const uint16_t osd_throttle_pos_pos = 234;                 // Pozycja przepustnicy
const uint16_t osd_vtx_channel_pos = 234;                  // Kanał VTX
const uint16_t osd_roll_pids_pos = 234;                    // PID-y rollu
const uint16_t osd_pitch_pids_pos = 234;                   // PID-y pitchu
const uint16_t osd_yaw_pids_pos = 234;                     // PID-y yawu
const uint16_t osd_power_pos = 234;                        // Moc
const uint16_t osd_pidrate_profile_pos = 234;              // Profil PIDRATE
const uint16_t osd_warnings_pos = 234;                     // Ostrzeżenia
const uint16_t osd_debug_pos = 234;                        // Tryb debugowania
const uint16_t osd_artificial_horizon_pos = 234;           // Sztuczny horyzont
const uint16_t osd_horizon_sidebars_pos = 234;             // Boczne paski horyzontu
const uint16_t osd_item_timer_1_pos = 234;                 // Licznik czasu 1
const uint16_t osd_item_timer_2_pos = 234;                 // Licznik czasu 2
const uint16_t osd_main_batt_usage_pos = 234;              // Zużycie głównej baterii
const uint16_t osd_disarmed_pos = 2284;                    // Rozbrojenie
const uint16_t osd_numerical_heading_pos = 234;            // Kierunek numeryczny
const uint16_t osd_compass_bar_pos = 234;                  // Pasek kompasu
const uint16_t osd_esc_tmp_pos = 234;                      // Temperatura ESC
const uint16_t osd_esc_rpm_pos = 234;                      // RPM ESC
const uint16_t osd_remaining_time_estimate_pos = 2550;     // Szacowany pozostały czas
const uint16_t osd_rtc_datetime_pos = 234;                 // Data/godzina RTC
const uint16_t osd_adjustment_range_pos = 234;             // Zakres regulacji
const uint16_t osd_core_temperature_pos = 234;             // Temperatura rdzenia
const uint16_t osd_anti_gravity_pos = 234;                 // Antygrawitacja
const uint16_t osd_g_force_pos = 234;                      // Siła G
const uint16_t osd_motor_diag_pos = 234;                   // Diagnostyka silnika
const uint16_t osd_log_status_pos = 234;                   // Status logowania
const uint16_t osd_flip_arrow_pos = 234;                   // Strzałka obracania
const uint16_t osd_link_quality_pos = 234;                 // Jakość połączenia
const uint16_t osd_flight_dist_pos = 234;                  // Dystans lotu
const uint16_t osd_stick_overlay_left_pos = 234;           // Nakładka lewej gałki
const uint16_t osd_stick_overlay_right_pos = 234;          // Nakładka prawej gałki
const uint16_t osd_esc_rpm_freq_pos = 234;                 // Częstotliwość RPM ESC
const uint16_t osd_rate_profile_name_pos = 234;            // Nazwa profilu rate
const uint16_t osd_pid_profile_name_pos = 234;             // Nazwa profilu PID
const uint16_t osd_profile_name_pos = 234;                 // Nazwa profilu
const uint16_t osd_rssi_dbm_value_pos = 234;               // Wartość RSSI w dBm
const uint16_t osd_rc_channels_pos = 234;                  // Kanały RC