/*
 * Codigo.h
 *
 *  Created on: 23 ago. 2017
 *      Author: Walter
 */

#ifndef SRC_CODIGO_H_
#define SRC_CODIGO_H_

void Mediana(uint8_t, uint32_t*, uint32_t*);
Status Check_CRC(char*, uint16_t);
void RefreshData(void);
Bool guardar_eeprom(char *buf, uint16_t offset, uint16_t len);
Bool leer_eeprom(char *buf, uint16_t offset, uint16_t len);
void snd_tank_sett(Parameter_upd_Type*, float, Modb_RegAdd,
		cmd_SP_Param_Src_Type);
void snd_head_flow_sett(Parameter_upd_Type*, float, Modb_RegAdd,
		cmd_SP_Param_Src_Type);
void read_flow_Vol_unit(SP_Type *sp, Var_Type *var);

bool processSmsCommand(MOBILE_T*, char*);
bool verifyPumpOk(void);

#endif /* SRC_CODIGO_H_ */
