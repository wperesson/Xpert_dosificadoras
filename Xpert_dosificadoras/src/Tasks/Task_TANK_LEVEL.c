/*
 * TaskTANK_LEVEL.c
 *
 *  Created on: 22/07/2019
 *      Author: Walter Peresson
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Definiciones.h"

#define FILTER_ARR_SZ	5

void RecalcMB(volatile Var_Type *Var) {

	Var->M = ((float) sp_pump.SpanTankCap - (float) sp_pump.ZeroTankCap)
			/ ((float) sp_pump.SpanTankEng - (float) sp_pump.ZeroTankEng);
	Var->B = sp_pump.SpanTankCap - Var->M * sp_pump.SpanTankEng;
}

float calcula_capacidad(SP_Pump_Type *sp, bool max_cap, float level) {
	float tempfloat;
	float _level, rad;

	switch (sp->tnk_shp) {
	case Vert_Cyl:
		if (max_cap)
			_level = sp->tnk_d2_VT_height_HT_lenght_RT_height;
		else
			_level = level;

		if (_level <= sp->tnk_d2_VT_height_HT_lenght_RT_height) {
			tempfloat = M_PI * (powf(sp->tnk_d1_VT_HT_diam_RT_width, 2) / 4)
					* _level;
		} else {
			tempfloat = sp_pump.TankCap / 1000; //Para mantener el escalado
		}
		break;
	case Horiz_Cyl:
		if (max_cap)
			_level = sp->tnk_d2_VT_height_HT_lenght_RT_height;
		else
			_level = level;

		if (_level <= sp->tnk_d1_VT_HT_diam_RT_width) {
			rad = sp_pump.tnk_d1_VT_HT_diam_RT_width / 2;
			tempfloat = sp_pump.tnk_d2_VT_height_HT_lenght_RT_height
					* (powf(rad, 2.0) * acos((rad - _level) / rad)
							- (rad - _level)
									* sqrt(
											(2 * rad * _level)
													- powf(_level, 2.0)));
		} else {
			tempfloat = sp_pump.TankCap / 1000; //Para mantener el escalado
		}
		break;
	case Rect:
		if (max_cap)
			_level = sp->tnk_d2_VT_height_HT_lenght_RT_height;
		else
			_level = level;
		tempfloat = sp->tnk_d1_VT_HT_diam_RT_width * sp->tnk_d3_RT_length
				* _level;
		break;
	default:
		break;
	}
	if ((sp->flow_unit == Galons_hour) || (sp->flow_unit == Galons_day))
		return tempfloat * 264.172; //Conversion de m3 a Gal
	else
		return tempfloat * 1000; //Conversion de m3 a Litros
}

static portTASK_FUNCTION(vTANK_LEVEL_Task,pvParameters) {
	static uint8_t canal;
	static uint8_t cont_process_tl = 0;
	float tempfloat;
	static float arr_Eng[FILTER_ARR_SZ];
	double result;
	uint8_t i, j = 0;
	Parameter_upd_Type newParameter;
	mqtt_event_type mqtt_st;
	Bool guardar_en_eeprom;

//	portTickType xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();

//Busca si hubo cambio de canal para el tank level==========
	canal = 0;
	while (canal < 6) {
		if (SP.An[canal].Src == Tank_Level) {
			break;
		} else {
			canal++;
		}
	}
	//----------------------------------------------------------

	/*Calcula la capacidad máxima del tanque*/
	if (sp_pump.tnk_shp != ref_points)
		sp_pump.TankCap = calcula_capacidad(&sp_pump, true, 0);

	while (1) {
		guardar_en_eeprom = false;
		while (xQueueReceive(queTANK_SETT_upd, &newParameter, 1000) != pdFAIL) {
			guardar_en_eeprom = true;
			switch (newParameter.parameter) {
			case SP_AL_TankLevel:
				if (newParameter.value >= 0
						|| newParameter.value < sp_pump.TankCap) {
					sp_pump.level_alarm_limit_T1 = newParameter.value;
				}
				break;
			case SP_War_TankLevel:
				if (newParameter.value >= 0
						|| newParameter.value < sp_pump.TankCap) {
					sp_pump.level_warning_limit_T1 = newParameter.value;
				}
				break;
			case SP_tnk_sensor_hgt:
				sp_pump.tnk_sensor_hgt = newParameter.value;
				break;
			case SP_tnk_dens:
				sp_pump.tnk_dens = newParameter.value;
				break;
			case SP_d1_tnk_VT_HT_diam_RT_width:
				sp_pump.tnk_d1_VT_HT_diam_RT_width = newParameter.value;
				break;
			case SP_d2_tnk_VT_height_HT_lenght_RT_height:
				sp_pump.tnk_d2_VT_height_HT_lenght_RT_height = newParameter.value;
				break;
			case SP_d3_tnk_RT_length:
				sp_pump.tnk_d3_RT_length = newParameter.value;
				break;
			case SP_tnk_Shp:
				sp_pump.tnk_shp = newParameter.value;
				break;
			default:
				guardar_en_eeprom = false;
				break;
			}
			/*Por las dudas vuelvo a calcular la maxima capacidad del tanque*/
			if (sp_pump.tnk_shp != ref_points)
				sp_pump.TankCap = calcula_capacidad(&sp_pump, true, 0);
		}
		/*Guardar en eeprom los cambios*/
		if (guardar_en_eeprom) {
			guardar_eeprom((char*) &sp_pump, OFF_sp_pump, sizeof(sp_pump));

			/*Refrescar los valores de los controles via web*/
			mqtt_st = mqtt_ev_publish_sett;
			xQueueSend(queMQTT_ev, &mqtt_st, 0);
		}

		//Almacena la señal para filtrar
		arr_Eng[j++] = Var.an[canal].Eng_Un;
		if (j == FILTER_ARR_SZ)
			j = 0;

		cont_process_tl++;
		if (cont_process_tl == 5) {
			cont_process_tl = 0;

			result = 0;
			for (i = 0; i < FILTER_ARR_SZ; i++) {
				result += arr_Eng[i];
			}
			result /= FILTER_ARR_SZ;

			if (canal < 6) { //Tank Level asignado a algún canal

				/*Calculo la altura de la columna de líquido teniendo en cuenta la altura del sensor */
				if (sp_pump.tnk_shp != ref_points) {
					Var.tank_Lev = (result * 6.894e3 / (sp_pump.tnk_dens * 9.78));
					if (Var.tank_Lev >= sp_pump.tnk_sensor_hgt) {
						Var.tank_Lev -= sp_pump.tnk_sensor_hgt;
					} else {
						Var.tank_Lev = 0;
					}
				}

				Var.tank_vol = calcula_capacidad(&sp_pump, false, Var.tank_Lev);

				/*Calculo el volumen en porcentaje*/
				tempfloat = 100 * Var.tank_vol / sp_pump.TankCap;
				if (tempfloat > 100)
					/*El tanque esta saturado*/
					Var.tank_vol_perc = 100;
				else
					Var.tank_vol_perc = tempfloat;

			} else { //Tank Level no fue asignado a ningún canal
				Var.tank_vol = 0;
			}

			//Antes de salir busca si hubo cambio de canal para el tank level==========
			canal = 0;
			while (canal < 6) {
				if (SP.An[canal].Src == Tank_Level) {
					break;
				} else {
					canal++;
				}
			}
		}
	}
}

void Create_TANK_LEVEL_task(void) {

	xTaskCreate(vTANK_LEVEL_Task, (char*) "TANK LEVEL TSK", 150, NULL,
	Prio_TANK_LEVEL_Task, &tsk_tank_level_handler);
}

