#include "htu21d.h"
#include "freertos/task.h"


#ifdef __cplusplus
extern "C" {
#endif

// Sensor I2C address
#define HTU21D_ADDR		0x40

// HTU21D commands
#define HTU21D_CMD_TRIGGER_TEMP_MEASURE_HOLD  		((uint8_t) 0xE3)
#define HTU21D_CMD_TRIGGER_HUMID_MEASURE_HOLD  		((uint8_t) 0xE5)
//#define HTU21D_CMD_TRIGGER_TEMP_MEASURE_NOHOLD  	((uint8_t) 0xF3)
//#define HTU21D_CMD_TRIGGER_HUMD_MEASURE_NOHOLD  	((uint8_t) 0xF5)
#define HTU21D_CMD_WRITE_USER_REG  				    ((uint8_t) 0xE6)
#define HTU21D_CMD_READ_USER_REG  					((uint8_t) 0xE7)
#define HTU21D_CMD_SOFT_RESET  					    ((uint8_t) 0xFE)



#define RESET_TIME                                            15            // ms value

// Processing constants
#define HTU21_TEMPERATURE_COEFFICIENT                        (float)(-0.15)
#define HTU21_CONSTANT_A                                    (float)(8.1332)
#define HTU21_CONSTANT_B                                    (float)(1762.39)
#define HTU21_CONSTANT_C                                    (float)(235.66)

// Coefficients for temperature computation
#define TEMPERATURE_COEFF_MUL                                (175.72)
#define TEMPERATURE_COEFF_ADD                                (-46.85)

// Coefficients for relative humidity computation
#define HUMIDITY_COEFF_MUL                                    (125)
#define HUMIDITY_COEFF_ADD                                    (-6)

// Conversion timings
// (These values were ajusted based on real devices)
#define HTU21_TEMPERATURE_CONVERSION_TIME_14b		52
#define HTU21_TEMPERATURE_CONVERSION_TIME_13b		30
#define HTU21_TEMPERATURE_CONVERSION_TIME_12b		21
#define HTU21_TEMPERATURE_CONVERSION_TIME_11b		11
#define HTU21_HUMIDITY_CONVERSION_TIME_12b			21
#define HTU21_HUMIDITY_CONVERSION_TIME_11b			12
#define HTU21_HUMIDITY_CONVERSION_TIME_10b			12
#define HTU21_HUMIDITY_CONVERSION_TIME_8b			11


// HTU21D User Register masks and bit position
#define HTU21D_USER_REG_RESOLUTION_MASK				0x81
#define HTU21D_USER_REG_ENABLE_HEATER_MASK			0x04


// HTU User Register values
// Resolution
#define HTU21_USER_REG_RESOLUTION_T_14b_RH_12b		0x00
#define HTU21_USER_REG_RESOLUTION_T_13b_RH_10b		0x80
#define HTU21_USER_REG_RESOLUTION_T_12b_RH_8b		0x01
#define HTU21_USER_REG_RESOLUTION_T_11b_RH_11b		0x81

// End of battery status
#define HTU21_USER_REG_END_OF_BATTERY_VDD_ABOVE_2_25V	0x00
#define HTU21_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V	0x40
// Enable on chip heater
#define HTU21_USER_REG_HEATER_ENABLE					0x04

#define HTU21_USER_REG_OTP_RELOAD_DISABLE				0x02


i2c_port_t m_i2c_port = -1;
TickType_t m_temperature_conv_time;
TickType_t m_humidity_conv_time;

static esp_err_t htu21dReadUserReguster(uint8_t *user_reg);
static esp_err_t htu21dWriteUserReguster(uint8_t user_reg);
static esp_err_t htu21dReadValue(uint16_t *dst, uint8_t command, TickType_t conv_time_millis);
static uint8_t calculateValueCRC(uint16_t value);
static htu21d_result_t mapEspResultToHtuResult(esp_err_t esp_result);
static float htu21dComputeCompensatedHumidity(float rh, float temperature);


htu21d_result_t htu21dInit(i2c_port_t port, int sda_pin, int scl_pin,  gpio_pullup_t sda_internal_pullup,  gpio_pullup_t scl_internal_pullup) {
	
	htu21d_result_t ret;
	m_i2c_port = port;

	m_temperature_conv_time = HTU21_TEMPERATURE_CONVERSION_TIME_14b;
	m_humidity_conv_time = HTU21_HUMIDITY_CONVERSION_TIME_12b;
	
	// setup i2c controller
	i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
	    .sda_io_num = sda_pin,
	    .scl_io_num = scl_pin,
	    .sda_pullup_en = sda_internal_pullup,
	    .scl_pullup_en = scl_internal_pullup,
	    .master.clk_speed = 100000,
    };
	
	ret = i2c_param_config(port, &conf);
	if(ret != ESP_OK) {
        return HTU21D_ERR_CONFIG;
    }
	
	// install the driver
	ret = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
	if(ret != ESP_OK) {
        return HTU21D_ERR_INSTALL;
    }
	
	// verify if a sensor is present	
	//ret = htu21dCheckPresent();

    return ret;
}


htu21d_result_t htu21dCheckPresent(void) {
    esp_err_t result;
    i2c_cmd_handle_t cmd;

    if (m_i2c_port < 0) {
        return HTU21D_ERR_NOINIT;
    }

    cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (HTU21D_ADDR << 1) | I2C_MASTER_WRITE, true);
	i2c_master_stop(cmd);
    result = i2c_master_cmd_begin(m_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return (result == ESP_OK) ? ESP_OK : HTU21D_ERR_NOTFOUND;
}

htu21d_result_t htu21dSoftReset(void) {
    esp_err_t result;
    i2c_cmd_handle_t cmd;

    if (m_i2c_port < 0) {
        return HTU21D_ERR_NOINIT;
    }


    // send the command
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (HTU21D_ADDR << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, HTU21D_CMD_SOFT_RESET, true);
	i2c_master_stop(cmd);
	result = i2c_master_cmd_begin(m_i2c_port, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	m_temperature_conv_time = HTU21_TEMPERATURE_CONVERSION_TIME_14b;
	m_humidity_conv_time = HTU21_HUMIDITY_CONVERSION_TIME_12b;
	
	return mapEspResultToHtuResult(result);
}

htu21d_result_t htu21dSetResolution(htu21d_resolution_t resolution) {
	uint8_t resol_reg_val;
	uint8_t user_reg_value;
	TickType_t t_conv_time, rh_conv_time;	// Use intermediate values because we can return with error
	esp_err_t result;

	if (m_i2c_port < 0) {
        return HTU21D_ERR_NOINIT;
    }

	switch(resolution) {
		case htu21d_resolution_t14b_rh12b:
			resol_reg_val = HTU21_USER_REG_RESOLUTION_T_14b_RH_12b;
			t_conv_time = HTU21_TEMPERATURE_CONVERSION_TIME_14b;
			rh_conv_time = HTU21_HUMIDITY_CONVERSION_TIME_12b;
			break;
		case htu21d_resolution_t13b_rh10b:
			resol_reg_val = HTU21_USER_REG_RESOLUTION_T_13b_RH_10b;
			t_conv_time = HTU21_TEMPERATURE_CONVERSION_TIME_13b;
			rh_conv_time = HTU21_HUMIDITY_CONVERSION_TIME_10b;
			break;
		case htu21d_resolution_t12b_rh8b:
			resol_reg_val = HTU21_USER_REG_RESOLUTION_T_12b_RH_8b;
			t_conv_time = HTU21_TEMPERATURE_CONVERSION_TIME_12b;
			rh_conv_time = HTU21_HUMIDITY_CONVERSION_TIME_8b;
			break;
		case htu21d_resolution_t11b_rh11b:
			resol_reg_val = HTU21_USER_REG_RESOLUTION_T_11b_RH_11b;
			t_conv_time = HTU21_TEMPERATURE_CONVERSION_TIME_11b;
			rh_conv_time = HTU21_HUMIDITY_CONVERSION_TIME_11b;
			break;
		default:
			return HTU21D_ERR_INVALID_ARG;
	}

	// Read User register value
	result = htu21dReadUserReguster(&user_reg_value);
	if (result != ESP_OK) {
		return mapEspResultToHtuResult(result);
	}

	// Modify User register value
	user_reg_value &= ~HTU21D_USER_REG_RESOLUTION_MASK;					// Clear resolution bits
	user_reg_value |= resol_reg_val & HTU21D_USER_REG_RESOLUTION_MASK;	// Set necessary resolution bits from new value

	// Write User register value
	result = htu21dWriteUserReguster(user_reg_value);
	if (result != ESP_OK) {
		return mapEspResultToHtuResult(result);
	}

	m_temperature_conv_time = t_conv_time;
	m_humidity_conv_time = rh_conv_time;

    return HTU21D_OK;
}



htu21d_result_t htu21dGetResolution(htu21d_resolution_t *p_resolution) {
	esp_err_t result;
	uint8_t user_reg_value;

	// Read User register value
	result = htu21dReadUserReguster(&user_reg_value);
	if (result != ESP_OK) {
		return mapEspResultToHtuResult(result);
	} else {
		user_reg_value &= HTU21D_USER_REG_RESOLUTION_MASK;	// Clear all bits except resolution-related
		switch (user_reg_value)
		{
			case HTU21_USER_REG_RESOLUTION_T_14b_RH_12b:
				*p_resolution = htu21d_resolution_t14b_rh12b;
				break;
			case HTU21_USER_REG_RESOLUTION_T_13b_RH_10b:
				*p_resolution = htu21d_resolution_t13b_rh10b;
				break;
			case HTU21_USER_REG_RESOLUTION_T_12b_RH_8b:
				*p_resolution = htu21d_resolution_t12b_rh8b;
				break;
			case HTU21_USER_REG_RESOLUTION_T_11b_RH_11b:
				*p_resolution = htu21d_resolution_t11b_rh11b;
				break;
			default:
				return HTU21D_ERR_INVALID_VALUE;
		}		
		return HTU21D_OK;
	}
}


htu21d_result_t htu21dGetHeaterStatus(htu21d_heater_status_t *p_heater) {
	uint8_t user_reg_value;
	esp_err_t result;

	// Read User register value
	result = htu21dReadUserReguster(&user_reg_value);
	if (result == ESP_OK) {
		*p_heater = (user_reg_value & HTU21D_USER_REG_ENABLE_HEATER_MASK) ? htu21d_heater_on : htu21d_heater_off;	
		return HTU21D_OK;
	} else {
		return mapEspResultToHtuResult(result);
	}
}

htu21d_result_t htu21dTurnHeaterOn(void) {
	uint8_t user_reg_value;
	esp_err_t result;

	// Read User register value
	result = htu21dReadUserReguster(&user_reg_value);
	if (result != ESP_OK) {
		return mapEspResultToHtuResult(result);
	}

	// Modify User register value
	user_reg_value &= ~HTU21D_USER_REG_ENABLE_HEATER_MASK;
	user_reg_value |= HTU21_USER_REG_HEATER_ENABLE;

	// Write User register value
	result = htu21dWriteUserReguster(user_reg_value);
	return mapEspResultToHtuResult(result);
}


htu21d_result_t htu21dTurnHeaterOff(void) {
	uint8_t user_reg_value;
	esp_err_t result;

	// Read User register value
	result = htu21dReadUserReguster(&user_reg_value);
	if (result != ESP_OK) {
		return mapEspResultToHtuResult(result);
	}

	// Modify User register value
	user_reg_value &= ~HTU21D_USER_REG_ENABLE_HEATER_MASK;

	// Write User register value
	result = htu21dWriteUserReguster(user_reg_value);
	return mapEspResultToHtuResult(result);
}



htu21d_result_t htu21dReadTemperature(float *temperature) {
	esp_err_t result;
	uint16_t adc_value;

	result = htu21dReadValue(&adc_value, HTU21D_CMD_TRIGGER_TEMP_MEASURE_HOLD, m_temperature_conv_time);
	if (result == ESP_OK) {
		*temperature = (float) ((double) adc_value * TEMPERATURE_COEFF_MUL / ((uint32_t) 0x10000) + TEMPERATURE_COEFF_ADD);
		return HTU21D_OK;
	} else {
		return mapEspResultToHtuResult(result);
	}
}

htu21d_result_t htu21dReadHumidity(float *rh, htu21d_humidity_type_t compensation_mode, float temperature) {
	esp_err_t result;
	uint16_t adc_value;
	float humid;

	result = htu21dReadValue(&adc_value, HTU21D_CMD_TRIGGER_HUMID_MEASURE_HOLD, m_humidity_conv_time);
	if (result == ESP_OK) {
		humid = (float) ((double) adc_value * HUMIDITY_COEFF_MUL / ((uint32_t) 0x10000) + HUMIDITY_COEFF_ADD);
		switch (compensation_mode) {
			case htu21d_humidity_compensated:
				*rh = htu21dComputeCompensatedHumidity(humid, temperature);
				return HTU21D_OK;
			case htu21d_humidity_non_compensated:
				*rh = humid;
				return HTU21D_OK;
			default:
				return HTU21D_ERR_INVALID_ARG;
		}
		
	} else {
		return mapEspResultToHtuResult(result);
	}
}

htu21d_result_t htu21dGetResolutionValuesFromType(htu21d_resolution_t sensor_resolution, uint8_t *t_res, uint8_t *rh_res) {
    switch (sensor_resolution)
    {
        case htu21d_resolution_t14b_rh12b:
            *t_res = 14;
            *rh_res = 12;
            break;
        case htu21d_resolution_t13b_rh10b:
            *t_res = 13;
            *rh_res = 10;
            break;
        case htu21d_resolution_t12b_rh8b:
            *t_res = 12;
            *rh_res = 8;
            break;
        case htu21d_resolution_t11b_rh11b:
            *t_res = 11;
            *rh_res = 11;
            break;
        default:
            return HTU21D_ERR_INVALID_ARG;
    }
    return HTU21D_OK;
}








































static esp_err_t htu21dReadUserReguster(uint8_t *user_reg) {
	esp_err_t ret;
	uint8_t reg_value;
	
	// send the command
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (HTU21D_ADDR << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, HTU21D_CMD_READ_USER_REG, true);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(m_i2c_port, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret != ESP_OK) {
		return ret;
	}
	
	// receive the answer
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (HTU21D_ADDR << 1) | I2C_MASTER_READ, true);
	i2c_master_read_byte(cmd, &reg_value, I2C_MASTER_NACK);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(m_i2c_port, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret == ESP_OK) {
		*user_reg = reg_value;
	}
	
	return ret;
}

static esp_err_t htu21dWriteUserReguster(uint8_t user_reg) {
	esp_err_t result;
	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (HTU21D_ADDR << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, HTU21D_CMD_WRITE_USER_REG, true);
	i2c_master_write_byte(cmd, user_reg, true);
	i2c_master_stop(cmd);
	result = i2c_master_cmd_begin(m_i2c_port, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	
	return result;
}

static esp_err_t htu21dReadValue(uint16_t *dst, uint8_t command, TickType_t conv_time_millis) {
	esp_err_t result;
	uint8_t buffer[3] = {0};
	uint16_t adc_value;
	uint8_t crc_must;
	uint8_t crc_actual;

	// send the command
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (HTU21D_ADDR << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, command, true);
	i2c_master_stop(cmd);
	result = i2c_master_cmd_begin(m_i2c_port, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(result != ESP_OK) {
		return result;
	}

	// Wait for conversion
	vTaskDelay(conv_time_millis / portTICK_RATE_MS);

	// Read bytes of value and CRC
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (HTU21D_ADDR << 1) | I2C_MASTER_READ, true);
	i2c_master_read(cmd, buffer, sizeof(buffer), I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);
	result = i2c_master_cmd_begin(m_i2c_port, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(result != ESP_OK) {
		return result;
	}

	// Compose value
	adc_value = (((uint16_t)buffer[0]) << 8) | ((uint16_t) buffer[1]);
	crc_must = buffer[2];

	// Calculate and check CRC
	crc_actual = calculateValueCRC(adc_value);
	if (crc_actual != crc_must) {
		return ESP_ERR_INVALID_CRC;
	}

	*dst = adc_value;
	return ESP_OK;
}

static uint8_t calculateValueCRC(uint16_t value) {
	uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
    uint32_t msb = 0x800000;
    uint32_t mask = 0xFF8000;
    uint32_t result = (uint32_t) value << 8; // Pad with zeros as specified in spec
	uint8_t crc;

    while (msb != 0x80) {

        // Check if msb of current value is 1 and apply XOR mask
        if (result & msb) {
			result = ((result ^ polynom) & mask) | (result & ~mask);
		}

        // Shift by one
        msb >>= 1;
        mask >>= 1;
        polynom >>= 1;
    }
	crc = (uint8_t)result;
	return crc;
}

static htu21d_result_t mapEspResultToHtuResult(esp_err_t esp_result) {
	switch(esp_result) {
		case ESP_OK:
			return HTU21D_OK;

		case ESP_ERR_INVALID_ARG:
			return HTU21D_ERR_INVALID_ARG;
			
		case ESP_FAIL:
			return HTU21D_ERR_FAIL;
		
		case ESP_ERR_INVALID_STATE:
			return HTU21D_ERR_INVALID_STATE;
		
		case ESP_ERR_TIMEOUT:
			return HTU21D_ERR_TIMEOUT;

		default:
			return (htu21d_result_t) esp_result;
	}
}

static float htu21dComputeCompensatedHumidity(float rh, float temperature) {
	float result = (rh + (25 - temperature) * HTU21_TEMPERATURE_COEFFICIENT);
	return result;
}

#ifdef __cplusplus
}
#endif