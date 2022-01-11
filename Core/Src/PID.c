#include "PID.h"

void PIDController_Init(PIDController *pid){
	/* Ustawienie wszystkich wartosci na 0 */
	pid->integrator = 0.0f;
	pid->prevE = 0.0f;
	
	pid->differentiator = 0.0f;
	pid->prevMeasurement = 0.0f;

	/* Ustawienie wartosci regulatora */
	pid->Kp = 1.0f;
	pid->Ki = 0.1f;
	pid->Kd = 0.07f;
	pid->tau = 0.1f;
	pid->limMin = 0.0f;
	pid->limMax = 1000.0f;
	pid->Tp = 0.250f;

}

float PIDController_Update(PIDController *pid, float setpoint, float measurement){
	/* Uchyb */
	float error = setpoint - measurement;
	
	/* Czlon proporcjonalny */
	float proportional = pid->Kp*error;
	
	/* Czlon calkujacy */
	pid->integrator = pid->Ki*pid->Tp*0.5f*(error + pid->prevE) + pid->integrator;


	/* Obliczanie ograniczen czlonu calkujacego */
	float limMinInt, limMaxInt;

	if (pid->limMax > proportional) {
		limMaxInt = pid->limMax - proportional;
	} else {
		limMaxInt = 0.0f;
	}

	if (pid->limMin < proportional) {
		limMinInt = pid->limMin - proportional;
	} else {
		limMinInt = 0.0f;
	}
	
	if(pid->integrator > limMaxInt) {
		pid->integrator = limMaxInt;
	} else if (pid->integrator <limMinInt){
		pid->integrator = limMinInt;
	}

	/* Czlon rozniczkujacy */
	pid->differentiator = (2.0f*pid->Kd*(measurement - pid->prevMeasurement)
			    + (2.0f*pid->tau - pid->Tp) * pid->differentiator)
			    / (2.0f*pid->tau + pid->Tp);

	/* sygnal sterujacy */
	pid->u = proportional + pid->integrator + pid->differentiator;

	/* Ograniczenie syngalu */

	if(pid->u > pid->limMax) {
		pid->u = pid->limMax;
	} else if(pid->u < pid->limMin){
		pid->u = pid->limMin;
	}

	/* Zapisanie nowego 'poprzedniego' bledu oraz pomiaru */
	pid->prevE = error;
	pid->prevMeasurement = measurement;

	/* Przekazanie sygnalu sterujacego */
	return pid->u;
}
