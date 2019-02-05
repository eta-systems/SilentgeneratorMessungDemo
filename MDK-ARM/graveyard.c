/**
 * GRAVEYARD
 * TEMP
 * for later use
 */

/** @todo work on the LTC3886 library */
		/*
		asdf[0] = 65; asdf[1] = 65; asdf[2] = 65; asdf[3] = 65; asdf[4] = 65;
		//LTC3886_Read8(&dcdc12VDriver, LTC3886_MFR_ID, &asdf);
		pwmspeed1 = LTC3886_MFR_ID;
		//HAL_I2C_Master_Transmit(&hi2c1, SG_ADDRESS_LTC3886, &pwmspeed1, 1, 10);
		//HAL_I2C_Master_Receive(&hi2c1, SG_ADDRESS_LTC3886, &asdf[0], 5, 10);
		HAL_I2C_Mem_Read(&hi2c1, SG_ADDRESS_LTC3886, LTC3886_MFR_ID, 1, asdf, 5, 10);
		printf("MFR ID: %X %X %X %X %X \n", asdf[0], asdf[1], asdf[2], asdf[3], asdf[4]);
		printf("string: %s %s %s\n", &asdf[1], &asdf[2], &asdf[3]);
		*/
		
		// Fan Controller Chip
		float temp1 = 0.0f, temp2 = 0.0;
		//MAX6615_ReadTemperature(&fanDriver, 1, &temp1);
		//MAX6615_ReadTemperature(&fanDriver, 2, &temp2);
		//MAX6615_PWM_SetPWM(&fanDriver, 1, pwmspeed);
		//MAX6615_PWM_SetPWM(&fanDriver, 2, pwmspeed);
		// MAX6615_Read8(&fanDriver, MAX6615_PWM_1_INSTA_DC, &pwmspeed1);
		// MAX6615_Read8(&fanDriver, MAX6615_PWM_2_INSTA_DC, &pwmspeed2);
		// printf("Temp 1: %.3f°C Temp 2: %.3f°C Fan 1: %d%% Fan 2: %d%%\n", temp1, temp2, (int)(pwmspeed1/2.4f), (int)(pwmspeed2/2.4));
		//(pwmspeed >= 99) ? pwmspeed = 0 : (pwmspeed++);
		
		/*
		uint8_t bytes[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_RESET);  // slave select LOW
		uint8_t command = 0x9E;
		HAL_SPI_Transmit(&hspi2, &command, 1, 10);
		HAL_SPI_Receive(&hspi2, bytes, 3, 10);
		HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_SET);  // slave select HIGH
		printf("status: 0x%02X 0x%02X 0x%02X\n", bytes[0], bytes[1], bytes[2]);
		*/
		
    //printf("Solar: %0.4f \tI_Out: %0.4f \tVint: %0.4f \tVUSB: %0.4f \tTemp: %0.4f \tVref: %0.4f \t(VBat: %0.4f)\n", \
    SG_ADC2Volt(ADC_Buf[0]), (SG_ADC2Volt(ADC_Buf[1])/0.01f)/50.0f, SG_ADC2Volt(ADC_Buf[2]), SG_ADC2Volt(ADC_Buf[3]), \
    SG_ADC2Volt(ADC_Buf[4]), SG_ADC2Volt(ADC_Buf[5]), SG_ADC2Volt(ADC_Buf[6]));
		
		

/*
static int op_sector_erase(struct ringfs_flash_partition *flash, int address)
{
	(void) flash;
	//flashsim_sector_erase(sim, address);
	return 0;
}

static size_t op_program(struct ringfs_flash_partition *flash, int address, const void *data, size_t size)
{
	(void) flash;
	//flashsim_program(sim, address, data, size);
	// (poll until write ok?)
	// WRITE_ENABLE (1 byte)
	uint8_t write_enable = 0x06;
	HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_RESET);  // slave select LOW
	HAL_SPI_Transmit(&hspi2, &write_enable, 1, 10);
	HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_SET);  // slave select HIGH
	// PAGE_PROGRAM (4 bytes + data)
	
	return size;
}

static size_t op_read(struct ringfs_flash_partition *flash, int address, void *data, size_t size)
{
	(void) flash;
	//flashsim_read(sim, address, data, size);
	return size;
}

static struct ringfs_flash_partition flash = {
    .sector_size = FLASH_SECTOR_SIZE,
    .sector_offset = FLASH_PARTITION_OFFSET,
    .sector_count = FLASH_PARTITION_SIZE,

    .sector_erase = op_sector_erase,
    .program = op_program,
    .read = op_read,
};
*/

/*
		uint16_t adc_vals[3] = {0,0,0};
		MAX11615_ADC_Read(&adcDriver_1, 5, &adc_vals[0]);
		MAX11615_ADC_Read(&adcDriver_1, 6, &adc_vals[1]);
		MAX11615_ADC_Read(&adcDriver_1, 7, &adc_vals[2]);
		printf("ADC: %.02fV %.02fV %.02fV\n", SG_MAX_ADC2Volt(adc_vals[0]), SG_MAX_ADC2Volt(adc_vals[1]), SG_MAX_ADC2Volt(adc_vals[2]));
		
		if(SG_MAX_ADC2Volt(adc_vals[2]) >= 1.0f){
			printf("PSU is on\n");
			HAL_GPIO_WritePin(SG_ACDC_On_GPIO_Port, SG_ACDC_On_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		} else{
			HAL_GPIO_WritePin(SG_ACDC_On_GPIO_Port, SG_ACDC_On_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		}
		
		if(SG_MAX_ADC2Volt(adc_vals[1]) >= 1.0f){
			printf("Solar is on\n");
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		} else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		}
		*/
		
		
