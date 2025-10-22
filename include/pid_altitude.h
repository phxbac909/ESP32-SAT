#ifndef PID_ALTITUDE_H
#define PID_ALTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

// PID tuning parameters - Có thể update từ bên ngoài
extern double PID_ALTITUDE_KP;
extern double PID_ALTITUDE_KI;
extern double PID_ALTITUDE_KD;

void pid_altitude_init(void);
void pid_altitude_compute(void);
void pid_altitude_receive_command(int command);
int pid_altitude_get_current_throttle(void);
int pid_altitude_get_final_throttle(void);
void pid_altitude_set_base_throttle(int throttle);
int pid_altitude_get_base_throttle(void);
bool pid_altitude_is_hold_enabled(void);
void pid_altitude_update_tunings(void);
void pid_altitude_stop_task(void);


#ifdef __cplusplus
}
#endif

#endif