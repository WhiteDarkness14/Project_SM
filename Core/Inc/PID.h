#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
	/* Parametry regulatora */
	float Kp;
	float Ki;
	float Kd;
	
	/* stala czasowa dla rzeczywistego czlonu rozniczkujacego */
	float tau;

	/* Saturacja */
	float limMin;
	float limMax;

	/* Czas probkowania */
	float Tp;

	/* Pamiec regulatora */
	float integrator;
	float prevE;
	float differentiator;
	float prevMeasurement;

	/* Sygnal sterujacy */
	float u; //szerokosc impulsu PWM

} PIDController;

void PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif //PID_CONTROLLER_H
