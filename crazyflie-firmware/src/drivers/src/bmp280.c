/*
 ****************************************************************************
 * Copyright (C) 2012 - 2015 Bosch Sensortec GmbH
 *
 * File : bmp280.h
 *
 * Date : 2015/03/27
 *
 * Revision : 2.0.4(Pressure and Temperature compensation code revision is 1.1)
 *
 * Usage: Sensor Driver for BMP280 sensor
 *
 ****************************************************************************
 *
 * \section License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holder nor the names of the
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 **************************************************************************/
#include "bmp280.h"
static bmp280_t *p_bmp280; /**< pointer to BMP280 */

/*!
 *	@brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the BMP280 sensor
 *	chip id is read in the register 0xD0 bit from 0 to 7
 *
 *	@param *bmp280 structure pointer.
 *
 *	@note While changing the parameter of the p_bmp280
 *	@note consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bmp280_init( bmp280_t *bmp280){
	/* variable used to return communication result*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8;

	p_bmp280 = bmp280;/* assign BMP280 ptr */
	if(p_bmp280!=0){
		/* read chip id */
		p_bmp280->bus_read(p_bmp280->dev_addr, BMP280_CHIP_ID_REG, &v_data_u8,1);

		/* read Chip Id */
		if(v_data_u8==0x58){
			p_bmp280->chip_id = v_data_u8;

			/* readout bmp280 calibparam structure */
			bmp280_get_calib_param();
			com_rslt = BSTDR_OK;
		}
	}else com_rslt = BSTDR_E_NULL_PTR;

	return com_rslt;
}

bstdr_ret_t bmp280_test_connection(){
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8;

	if(p_bmp280!=0){
		p_bmp280->bus_read(p_bmp280->dev_addr, BMP280_CHIP_ID_REG, &v_data_u8,1);
		if(v_data_u8==0x58){
			com_rslt = BSTDR_OK;
		}
	}else com_rslt = BSTDR_E_NULL_PTR;
	return com_rslt;
}
/*!
 *	@brief This API is used to read uncompensated temperature
 *	in the registers 0xFA, 0xFB and 0xFC
 *	@note 0xFA -> MSB -> bit from 0 to 7
 *	@note 0xFB -> LSB -> bit from 0 to 7
 *	@note 0xFC -> LSB -> bit from 4 to 7
 *
 *	@param v_uncomp_temperature_s32 : The uncompensated temperature.
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bmp280_read_uncomp_temperature(int32_t *v_uncomp_temperature_s32)
{
	/* variable used to return communication result*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t a_data_u8r[3] = {0};
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 != 0) {
		/* read temperature data */
		p_bmp280->bus_read(p_bmp280->dev_addr,BMP280_TEMPERATURE_MSB_REG,a_data_u8r, 3);
		*v_uncomp_temperature_s32 = (int32_t)((((uint32_t) (a_data_u8r[0]))<< 12) |	(((uint32_t)(a_data_u8r[1]))<< 4)| ((uint32_t)a_data_u8r[2]>> 4));
	}
	return com_rslt;
}
/*!
 *	@brief Reads actual temperature
 *	from uncompensated temperature
 *	@note Returns the value in 0.01 degree Centigrade
 *	@note Output value of "5123" equals 51.23 DegC.
 *
 *
 *
 *  @param v_uncomp_temperature_s32 : value of uncompensated temperature
 *
 *
 *
 *  @return Actual temperature output as int32_t
 *
 */
int32_t bmp280_compensate_temperature_int32(int32_t v_uncomp_temperature_s32)
{
	int32_t v_x1_u32r;
	int32_t v_x2_u32r;
	int32_t temperature;
	/* calculate true temperature*/
	/*calculate x1*/
	v_x1_u32r  = ((((v_uncomp_temperature_s32 >> 3) - ((int32_t) p_bmp280->calib_param.dig_T1 << 1))) * \
			((int32_t)p_bmp280->calib_param.dig_T2)) >> 11;
	/*calculate x2*/
	v_x2_u32r  = (((((v_uncomp_temperature_s32 >> 4) -	((int32_t)p_bmp280->calib_param.dig_T1)) * \
			((v_uncomp_temperature_s32 >> 4) - ((int32_t)p_bmp280->calib_param.dig_T1))) >> 12) * \
			((int32_t)p_bmp280->calib_param.dig_T3)) >> 14;
	/*calculate t_fine*/
	p_bmp280->calib_param.t_fine = v_x1_u32r + v_x2_u32r;
	/*calculate temperature*/
	temperature  = (p_bmp280->calib_param.t_fine * 5 + 128)	>> 8;

	return temperature;
}
/*!
 *	@brief This API is used to read uncompensated pressure.
 *	in the registers 0xF7, 0xF8 and 0xF9
 *	@note 0xF7 -> MSB -> bit from 0 to 7
 *	@note 0xF8 -> LSB -> bit from 0 to 7
 *	@note 0xF9 -> LSB -> bit from 4 to 7
 *
 *
 *
 *	@param v_uncomp_pressure_s32 : The value of uncompensated pressure
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bmp280_read_uncomp_pressure(
		int32_t *v_uncomp_pressure_s32)
{
	/* variable used to return communication result*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;

	uint8_t a_data_u8[3] = {0};
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == 0) {
		return  BSTDR_E_NULL_PTR;
	} else {
		com_rslt = p_bmp280->bus_read(p_bmp280->dev_addr,BMP280_PRESSURE_MSB_REG,a_data_u8, 3);
		*v_uncomp_pressure_s32 = (int32_t)((((uint32_t)(a_data_u8[0]))<< 12) |(((uint32_t)(a_data_u8[1]))<< 4) |
								((uint32_t)a_data_u8[2] >>4));
	}
	return com_rslt;
}
/*!
 *	@brief Reads actual pressure from uncompensated pressure
 *	and returns the value in Pascal(Pa)
 *	@note Output value of "96386" equals 96386 Pa =
 *	963.86 hPa = 963.86 millibar
 *
 *
 *
 *
 *  @param  v_uncomp_pressure_s32: value of uncompensated pressure
 *
 *
 *
 *  @return Returns the Actual pressure out put as int32_t
 *
 */
uint32_t bmp280_compensate_pressure_int32(int32_t v_uncomp_pressure_s32)
{
	int32_t v_x1_u32r;
	int32_t v_x2_u32r;
	uint32_t v_pressure_u32;
	/* calculate x1*/
	v_x1_u32r = (((int32_t)p_bmp280->calib_param.t_fine)>> 1) -	(int32_t)64000;
	/* calculate x2*/
	v_x2_u32r = (((v_x1_u32r >> 2) * (v_x1_u32r >> 2)) >> 11) *	((int32_t)p_bmp280->calib_param.dig_P6);
	v_x2_u32r = v_x2_u32r + ((v_x1_u32r * ((int32_t)p_bmp280->calib_param.dig_P5)) << 1);
	v_x2_u32r = (v_x2_u32r >> 2) + (((int32_t)p_bmp280->calib_param.dig_P4) << 16);
	/* calculate x1*/
	v_x1_u32r = (((p_bmp280->calib_param.dig_P3 *(((v_x1_u32r >> 2) * (v_x1_u32r >> 2)) >> 13)) >> 3) +	((((int32_t)p_bmp280->calib_param.dig_P2) *	v_x1_u32r) >> 1)) >> 18;
	v_x1_u32r = ((((32768 + v_x1_u32r)) * ((int32_t)p_bmp280->calib_param.dig_P1))>> 15);
	/* calculate pressure*/
	v_pressure_u32 = (((uint32_t)(((int32_t)1048576) - v_uncomp_pressure_s32) -	(v_x2_u32r >> 12))) * 3125;
	/* check overflow*/
	if (v_pressure_u32 < 0x80000000)
		/* Avoid exception caused by division by zero */
		if (v_x1_u32r != 0)
			v_pressure_u32 = (v_pressure_u32 << 1) / ((uint32_t)v_x1_u32r);
		else
			return 0;
	else
		/* Avoid exception caused by division by zero */
		if (v_x1_u32r != 0)
			v_pressure_u32 = (v_pressure_u32 / (uint32_t)v_x1_u32r) * 2;
		else
			return 0;
	/* calculate x1*/
	v_x1_u32r = (((int32_t)
			p_bmp280->calib_param.dig_P9) *
			((int32_t)(((v_pressure_u32
					>> 3)
					* (v_pressure_u32 >> 3))
					>> 13)))
				>> 12;
	/* calculate x2*/
	v_x2_u32r = (((int32_t)(v_pressure_u32
			>> 2)) *
			((int32_t)p_bmp280->calib_param.dig_P8))
				>> 13;
	/* calculate true pressure*/
	v_pressure_u32 = (uint32_t)
				((int32_t)v_pressure_u32 +
						((v_x1_u32r + v_x2_u32r +
								p_bmp280->calib_param.dig_P7)
								>> 4));

	return v_pressure_u32;
}
/*!
 * @brief reads uncompensated pressure and temperature
 *
 *
 * @param  v_uncomp_pressure_s32: The value of uncompensated pressure.
 * @param  v_uncomp_temperature_s32: The value of uncompensated temperature.
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
bstdr_ret_t bmp280_read_uncomp_data( int32_t *v_uncomp_pressure_s32, int32_t *v_uncomp_temperature_s32)
{
	/* variable used to return communication result*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t a_data_u8[6] = {0};
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 != 0){
		com_rslt = p_bmp280->bus_read(p_bmp280->dev_addr,BMP280_PRESSURE_MSB_REG,a_data_u8, 6);
		/*Pressure*/
		*v_uncomp_pressure_s32 = (int32_t)((((uint32_t)(a_data_u8[0]))<< 12) | (((uint32_t)(a_data_u8[1]))<< 4) |((uint32_t)a_data_u8[2] >> 4));
		/* Temperature */
		*v_uncomp_temperature_s32 = (int32_t)((( (uint32_t) (a_data_u8[3])) << 12) |(((uint32_t)(a_data_u8[4]))<< 4) | ((uint32_t)a_data_u8[5] >> 4));
	} else
		com_rslt = BSTDR_E_NULL_PTR;
	return com_rslt;
}
/*!
 * @brief This API reads the true pressure and temperature
 *
 *
 *  @param  v_pressure_u32 : The value of compensated pressure.
 *  @param  v_temperature_s32 : The value of compensated temperature.
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bmp280_read_data(uint32_t *v_pressure_u32,int32_t *v_temperature_s32)
{
	/* variable used to return communication result*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	int32_t v_uncomp_pressure_s32;
	int32_t v_uncomp_temperature_s32;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == 0) {
		return  BSTDR_E_NULL_PTR;
	} else {
		com_rslt = bmp280_read_uncomp_data(&v_uncomp_pressure_s32, &v_uncomp_temperature_s32);
		/* read trure pressure and temperature*/
		*v_temperature_s32 = bmp280_compensate_temperature_int32(v_uncomp_temperature_s32);
		*v_pressure_u32 = bmp280_compensate_pressure_int32(v_uncomp_pressure_s32);
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to
 *	calibration parameters used for calculation in the registers
 *
 *  parameter | Register address |   bit
 *------------|------------------|----------------
 *	dig_T1    |  0x88 and 0x89   | from 0 : 7 to 8: 15
 *	dig_T2    |  0x8A and 0x8B   | from 0 : 7 to 8: 15
 *	dig_T3    |  0x8C and 0x8D   | from 0 : 7 to 8: 15
 *	dig_P1    |  0x8E and 0x8F   | from 0 : 7 to 8: 15
 *	dig_P2    |  0x90 and 0x91   | from 0 : 7 to 8: 15
 *	dig_P3    |  0x92 and 0x93   | from 0 : 7 to 8: 15
 *	dig_P4    |  0x94 and 0x95   | from 0 : 7 to 8: 15
 *	dig_P5    |  0x96 and 0x97   | from 0 : 7 to 8: 15
 *	dig_P6    |  0x98 and 0x99   | from 0 : 7 to 8: 15
 *	dig_P7    |  0x9A and 0x9B   | from 0 : 7 to 8: 15
 *	dig_P8    |  0x9C and 0x9D   | from 0 : 7 to 8: 15
 *	dig_P9    |  0x9E and 0x9F   | from 0 : 7 to 8: 15
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bmp280_get_calib_param(void)
{
	/* variable used to return communication result*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t a_data_u8[24] = {0};
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == 0) {
		com_rslt = BSTDR_E_NULL_PTR;
	}
	else {

		com_rslt = p_bmp280->bus_read(p_bmp280->dev_addr,BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG,a_data_u8,24);

		/* read calibration values*/
		p_bmp280->calib_param.dig_T1 = (uint16_t)((((uint16_t)((uint8_t)a_data_u8[1]))<< 8) | a_data_u8[0]);
		p_bmp280->calib_param.dig_T2 = (int16_t)((((int16_t)((int8_t)a_data_u8[3]))<< 8) | a_data_u8[2]);
		p_bmp280->calib_param.dig_T3 = (int16_t)((((int16_t)((int8_t)a_data_u8[5]))<< 8) | a_data_u8[4]);
		p_bmp280->calib_param.dig_P1 = (uint16_t)((((uint16_t)((uint8_t)a_data_u8[7]))<< 8)	| a_data_u8[6]);
		p_bmp280->calib_param.dig_P2 = (int16_t)((((int16_t)((int8_t)a_data_u8[9]))<< 8) | a_data_u8[8]);
		p_bmp280->calib_param.dig_P3 = (int16_t)((((int16_t)((int8_t)a_data_u8[11]))<< 8) | a_data_u8[10]);
		p_bmp280->calib_param.dig_P4 = (int16_t)((((int16_t)((int8_t)a_data_u8[13]))<< 8) | a_data_u8[12]);
		p_bmp280->calib_param.dig_P5 = (int16_t)((((int16_t)((int8_t)a_data_u8[15]))<< 8) | a_data_u8[14]);
		p_bmp280->calib_param.dig_P6 = (int16_t)((((int16_t)((int8_t)a_data_u8[17]))<< 8) | a_data_u8[16]);
		p_bmp280->calib_param.dig_P7 = (int16_t)((((int16_t)((int8_t)a_data_u8[19]))<< 8) | a_data_u8[18]);
		p_bmp280->calib_param.dig_P8 = (int16_t)((((int16_t)((int8_t)a_data_u8[21]))<< 8) | a_data_u8[20]);
		p_bmp280->calib_param.dig_P9 = (int16_t)((((int16_t)((int8_t)a_data_u8[23]))<< 8) | a_data_u8[22]);
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to set
 *	the temperature oversampling setting in the register 0xF4
 *	bits from 5 to 7
 *
 *        value             | Temperature oversampling
 *  ------------------------|------------------------------
 *       0x00               |  BMP280_OVERSAMP_SKIPPED
 *       0x01               |  BMP280_OVERSAMP_1X
 *       0x02               |  BMP280_OVERSAMP_2X
 *       0x03               |  BMP280_OVERSAMP_4X
 *       0x04               |  BMP280_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP280_OVERSAMP_16X
 *
 *
 *  @param v_value_u8 :The value of temperature over sampling
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bmp280_set_osr_temperature(uint8_t v_value_u8)
{
	/* variable used to return communication result*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == 0) {
		return  BSTDR_E_NULL_PTR;
	} else {
		p_bmp280->bus_read(p_bmp280->dev_addr, BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,&v_data_u8, 1);

		/* write over sampling*/
		v_data_u8 = BSTDR_SET_BITSLICE(v_data_u8,BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS,\
				BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__LEN,	 v_value_u8);
		com_rslt +=	p_bmp280->bus_write(p_bmp280->dev_addr,	BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,	&v_data_u8, 1);
		p_bmp280->oversamp_temperature = v_value_u8;
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to set
 *	the pressure oversampling setting in the register 0xF4
 *	bits from 2 to 4
 *
 *        value             | Pressure oversampling
 *  ------------------------|------------------------------
 *       0x00               |  BMP280_OVERSAMP_SKIPPED
 *       0x01               |  BMP280_OVERSAMP_1X
 *       0x02               |  BMP280_OVERSAMP_2X
 *       0x03               |  BMP280_OVERSAMP_4X
 *       0x04               |  BMP280_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP280_OVERSAMP_16X
 *
 *
 *  @param  v_value_u8 : The value of pressure over sampling
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bmp280_set_osr_pressure(uint8_t v_value_u8)
{
	/* variable used to return communication result*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == 0) {
		return  BSTDR_E_NULL_PTR;
	} else {
		com_rslt = p_bmp280->bus_read( p_bmp280->dev_addr, BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,	&v_data_u8, 1);
			/* write pressure over sampling */
			v_data_u8 = BSTDR_SET_BITSLICE(v_data_u8,BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS,\
					BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__LEN,v_value_u8);
			p_bmp280->bus_write( p_bmp280->dev_addr,BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,&v_data_u8, 1);
			p_bmp280->oversamp_pressure = v_value_u8;
	}
	return com_rslt;
}


/*!
 *	@brief This API used to set the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	@param v_power_mode_u8 : The value of power mode value
 *  value            |   Power mode
 * ------------------|------------------
 *	0x00             | BMP280_SLEEP_MODE
 *	0x01 and 0x02    | BMP280_FORCED_MODE
 *	0x03             | BMP280_NORMAL_MODE
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bmp280_set_power_mode(uint8_t v_power_mode_u8)
{
	/* variable used to return communication result*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t v_mode_u8;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == 0) {
		return  BSTDR_E_NULL_PTR;
	} else {
		if (v_power_mode_u8 <= BMP280_NORMAL_MODE) {
			/* write the power mode*/
			v_mode_u8 = (p_bmp280->oversamp_temperature << 5) + (p_bmp280->oversamp_pressure <<	2) + v_power_mode_u8;
			com_rslt = p_bmp280->bus_write(p_bmp280->dev_addr, BMP280_CTRL_MEAS_REG_POWER_MODE__REG, &v_mode_u8, 1);
		} else {
			com_rslt = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 * @brief Used to reset the sensor
 * The value 0xB6 is written to the 0xE0 register
 * the device is reset using the
 * complete power-on-reset procedure.
 * Softreset can be easily set using bmp280_set_softreset().
 *
 * @note Usage Hint : bmp280_set_softreset()
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bmp280_set_soft_rst(void)
{
	/* variable used to return communication result*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = BMP280_SOFT_RESET_CODE;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == 0) {
		return  BSTDR_E_NULL_PTR;
	} else {
		/* write soft reset */
		com_rslt = p_bmp280->bus_write(
				p_bmp280->dev_addr,
				BMP280_RST_REG, &v_data_u8,
				1);
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to write filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	@param v_value_u8 : The value of filter coefficient
 *	value	    |	Filter coefficient
 * -------------|-------------------------
 *	0x00        | BMP280_FILTER_COEFF_OFF
 *	0x01        | BMP280_FILTER_COEFF_2
 *	0x02        | BMP280_FILTER_COEFF_4
 *	0x03        | BMP280_FILTER_COEFF_8
 *	0x04        | BMP280_FILTER_COEFF_16
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bmp280_set_filter(uint8_t v_value_u8)
{
	bstdr_ret_t com_rslt = BSTDR_OK;
	uint8_t v_data_u8;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 != 0) {
		/* write filter*/
		p_bmp280->bus_read(p_bmp280->dev_addr,BMP280_CONFIG_REG_FILTER__REG,	&v_data_u8, 1);
		v_data_u8 = BSTDR_SET_BITSLICE(	v_data_u8,BMP280_CONFIG_REG_FILTER__POS,BMP280_CONFIG_REG_FILTER__LEN, v_value_u8);
		p_bmp280->bus_write(p_bmp280->dev_addr,BMP280_CONFIG_REG_FILTER__REG,&v_data_u8, 1);
	}
	return com_rslt;
}

/*!
 *	@brief This API used to Read the
 *	standby duration time from the sensor
 *	in the register 0xF5 bit 5 to 7
 *	@note Normal mode comprises an
 *	automated perpetual cycling between an (active)
 *	Measurement period and an (inactive) standby period.
 *	@note The standby time is determined
 *	by the contents of the register t_sb.
 *	Standby time can be set using BMP280_STANDBYTIME_125_MS.
 *
 *	@note bme280_set_standby_durN(BMP280_STANDBYTIME_125_MS)
 *
 *
 *
 *	@param v_standby_durn_u8 : The standby duration time value.
 *  value     |  standby duration
 * -----------|--------------------
 *    0x00    | BMP280_STANDBYTIME_1_MS
 *    0x01    | BMP280_STANDBYTIME_63_MS
 *    0x02    | BMP280_STANDBYTIME_125_MS
 *    0x03    | BMP280_STANDBYTIME_250_MS
 *    0x04    | BMP280_STANDBYTIME_500_MS
 *    0x05    | BMP280_STANDBYTIME_1000_MS
 *    0x06    | BMP280_STANDBYTIME_2000_MS
 *    0x07    | BMP280_STANDBYTIME_4000_MS
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bmp280_set_standby_durn(uint8_t v_standby_durn_u8)
{
	/* variable used to return communication result*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == 0) {
		return  BSTDR_E_NULL_PTR;
	} else {
		/* write the standby duration*/
		com_rslt = p_bmp280->bus_read(
				p_bmp280->dev_addr,
				BMP280_CONFIG_REG_STANDBY_DURN__REG,
				&v_data_u8, 1);
		if (com_rslt == BSTDR_OK) {
			v_data_u8 =	BSTDR_SET_BITSLICE(v_data_u8,BMP280_CONFIG_REG_STANDBY_DURN__POS,\
							BMP280_CONFIG_REG_STANDBY_DURN__LEN,v_standby_durn_u8);
			p_bmp280->bus_write(p_bmp280->dev_addr,	BMP280_CONFIG_REG_STANDBY_DURN__REG,&v_data_u8, 1);
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to write
 *	 the working mode of the sensor
 *
 *
 *  @param v_work_mode_u8 : The value of work mode
 *   value      |  mode
 * -------------|-------------
 *    0         | BMP280_ULTRA_LOW_POWER_MODE
 *    1         | BMP280_LOW_POWER_MODE
 *    2         | BMP280_STANDARD_RESOLUTION_MODE
 *    3         | BMP280_HIGH_RESOLUTION_MODE
 *    4         | BMP280_ULTRA_HIGH_RESOLUTION_MODE
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bmp280_set_work_mode(uint8_t v_work_mode_u8)
{
	/* variable used to return communication result*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8;

	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == 0) {
		return  BSTDR_E_NULL_PTR;
	} else {
		if (v_work_mode_u8 <= BMP280_ULTRA_HIGH_RESOLUTION_MODE) {

			com_rslt = p_bmp280->bus_read(p_bmp280->dev_addr,	BMP280_CTRL_MEAS_REG,&v_data_u8, 1);

			if (com_rslt == BSTDR_OK) {
				switch (v_work_mode_u8) {
				/* write work mode*/
				case BMP280_ULTRA_LOW_POWER_MODE:
					p_bmp280->oversamp_temperature = BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE;
					p_bmp280->oversamp_pressure = BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE;
					break;
				case BMP280_LOW_POWER_MODE:
					p_bmp280->oversamp_temperature = BMP280_LOWPOWER_OVERSAMP_TEMPERATURE;
					p_bmp280->oversamp_pressure = BMP280_LOWPOWER_OVERSAMP_PRESSURE;
					break;
				case BMP280_STANDARD_RESOLUTION_MODE:
					p_bmp280->oversamp_temperature = BMP280_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE;
					p_bmp280->oversamp_pressure = BMP280_STANDARDRESOLUTION_OVERSAMP_PRESSURE;
					break;
				case BMP280_HIGH_RESOLUTION_MODE:
					p_bmp280->oversamp_temperature = BMP280_HIGHRESOLUTION_OVERSAMP_TEMPERATURE;
					p_bmp280->oversamp_pressure = BMP280_HIGHRESOLUTION_OVERSAMP_PRESSURE;
					break;
				case BMP280_ULTRA_HIGH_RESOLUTION_MODE:
					p_bmp280->oversamp_temperature = BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE;
					p_bmp280->oversamp_pressure = BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE;
					break;
				}

				v_data_u8 = BSTDR_SET_BITSLICE(v_data_u8,BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS,\
						BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__LEN,	p_bmp280->oversamp_temperature);

				v_data_u8 = BSTDR_SET_BITSLICE(v_data_u8,BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS,\
						BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__LEN,	p_bmp280->oversamp_pressure);

				com_rslt += p_bmp280->bus_write(p_bmp280->dev_addr,	BMP280_CTRL_MEAS_REG,&v_data_u8, 1);
			}
		} else {
			com_rslt = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

/*!
 * @brief This API used to read
 * actual temperature from uncompensated temperature
 * @note Returns the value in Degree centigrade
 * @note Output value of "51.23" equals 51.23 DegC.
 *
 *
 *
 *  @param v_uncomp_temperature_s32 : value of uncompensated temperature
 *
 *
 *
 *  @return
 *	Actual temperature in floating point
 *
 */
float bmp280_compensate_temperature_float(int32_t v_uncomp_temperature_s32)
{
	float v_x1_u32r;
	float v_x2_u32r;
	float temperature;

	/* calculate x1*/
	v_x1_u32r  = (((float)v_uncomp_temperature_s32) / 16384.0f -((float)p_bmp280->calib_param.dig_T1) / 1024.0f) * ((float)p_bmp280->calib_param.dig_T2);
	/* calculate x2*/
	v_x2_u32r  = ((((float)v_uncomp_temperature_s32) / 131072.0f - ((float)p_bmp280->calib_param.dig_T1) / 8192.0f) *
			(((float)v_uncomp_temperature_s32) / 131072.0f - ((float)p_bmp280->calib_param.dig_T1) / 8192.0f)) * ((float)p_bmp280->calib_param.dig_T3);

	/* calculate t_fine*/
	p_bmp280->calib_param.t_fine = (int32_t)(v_x1_u32r + v_x2_u32r);

	/* calculate true pressure*/
	temperature  = (v_x1_u32r + v_x2_u32r) / 5120.0f;

	return temperature;
}

/*!
 *	@brief Reads actual pressure from uncompensated pressure
 *	and returns pressure in Pa as float.
 *	@note Output value of "96386.2"
 *	equals 96386.2 Pa = 963.862 hPa.
 *
 *
 *
 *  @param v_uncomp_pressure_s32 : value of uncompensated pressure
 *
 *
 *
 *  @return
 *	Actual pressure in floating point
 *
 */
float bmp280_compensate_pressure_float(int32_t v_uncomp_pressure_s32)
{
	float v_x1_u32r;
	float v_x2_u32r;
	float pressure;

	v_x1_u32r = ((float)p_bmp280->calib_param.t_fine/2.0f) - 64000.0f;
	v_x2_u32r = v_x1_u32r * v_x1_u32r *	((float)p_bmp280->calib_param.dig_P6) / 32768.0f;
	v_x2_u32r = v_x2_u32r + v_x1_u32r *	((float)p_bmp280->calib_param.dig_P5) * 2.0f;
	v_x2_u32r = (v_x2_u32r / 4.0f) +	(((float)p_bmp280->calib_param.dig_P4) * 65536.0f);
	v_x1_u32r = (((float)p_bmp280->calib_param.dig_P3) * v_x1_u32r * v_x1_u32r / 524288.0f + ((float)p_bmp280->calib_param.dig_P2) * v_x1_u32r) / 524288.0f;
	v_x1_u32r = (1.0f + v_x1_u32r / 32768.0f) * ((float)p_bmp280->calib_param.dig_P1);
	pressure = 1048576.0f - (float)v_uncomp_pressure_s32;
	/* Avoid exception caused by division by zero */
	if (v_x1_u32r != 0.0f)
		pressure = (pressure - (v_x2_u32r / 4096.0f)) * 6250.0f / v_x1_u32r;
	else
		return 0;
	v_x1_u32r = ((float)p_bmp280->calib_param.dig_P9) *	pressure * pressure / 2147483648.0f;
	v_x2_u32r = pressure * ((float)p_bmp280->calib_param.dig_P8) / 32768.0f;
	pressure = pressure + (v_x1_u32r + v_x2_u32r + ((float)p_bmp280->calib_param.dig_P7)) / 16.0f;

	return pressure;
}

