		

		// ALL LED TEST
        MAX7313_Pin_Write(ioDrivers[ CHIP_LED_ERROR ], PORT_LED_ERROR, 14);
		HAL_Delay(500);
		
		for(uint8_t i = 0; i<4; i++){
			MAX7313_Pin_Write(ioDrivers[ MAX7313_LED_GRN_Chips[i] ], MAX7313_LED_GRN_Ports[i], 14);
			HAL_Delay(500);
			MAX7313_Pin_Write(ioDrivers[ MAX7313_LED_RED_Chips[i] ], MAX7313_LED_RED_Ports[i], 14);
			HAL_Delay(500);
		}
		MAX7313_Pin_Write(ioDrivers[ MAX7313_LED_GRN_Chips[4] ], MAX7313_LED_GRN_Ports[4], 14);
		HAL_Delay(500);
		
		for(uint8_t i = 1; i<120; i += 10){
			Silentgenerator_battery_bar(i, 14, 15);
			HAL_Delay(100);
			printf("%d\n", i);
		}
		
		for(uint8_t i = 0; i<4; i++){
			MAX7313_Pin_Write(ioDrivers[ MAX7313_LED_GRN_Chips[i] ], MAX7313_LED_GRN_Ports[i], 15);
			HAL_Delay(500);
			MAX7313_Pin_Write(ioDrivers[ MAX7313_LED_RED_Chips[i] ], MAX7313_LED_RED_Ports[i], 15);
			HAL_Delay(500);
		}
		MAX7313_Pin_Write(ioDrivers[ MAX7313_LED_GRN_Chips[4] ], MAX7313_LED_GRN_Ports[4], 15);
		HAL_Delay(500);
		MAX7313_Pin_Write(ioDrivers[ CHIP_LED_ERROR ], PORT_LED_ERROR, 15);
		HAL_Delay(500);




















		switch(led_mode){
			case 0:							// 1 LED blinkt
				set_all_leds(15);
				if(laufvariable > 10){
					// MAX7313_Pin_Write(&ioDriver_1, 14, 0);
					MAX7313_Pin_Write(&ioDriver_1, 14, 0);
					HAL_GPIO_WritePin(USBPowerOn_GPIO_Port, USBPowerOn_Pin, GPIO_PIN_SET);
				} else {
					HAL_GPIO_WritePin(USBPowerOn_GPIO_Port, USBPowerOn_Pin, GPIO_PIN_RESET);
				}
				break;
			case 1:							// alle LEDs blinken
				if(laufvariable > 10)
					set_all_leds(0);
				else
					set_all_leds(15);
				break;
			case 2: 						// alle LEDs ein
				set_all_leds(0);
				break;
			case 3:							// alle LEDs aus
				set_all_leds(15);
				break;
			default:
				led_mode = 0;
		}
		HAL_Delay(100);
		
		uint8_t phase00, phase01, phase10, phase11;
		MAX7313_Read8(&ioDriver_2, MAX7313_BLINK_PHASE_0_00_07, &phase00);
		MAX7313_Read8(&ioDriver_2, MAX7313_BLINK_PHASE_0_08_15, &phase01);
		//printf("P0: "BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(phase01), BYTE_TO_BINARY(phase00));
		printf("P0 %#04x%02x\n", phase01, phase00);
		*/
		
		// Analog Werte an USART3 printen
		// PA0 - PA1 - PA4 - PA5 - TempInt - VrefInt - VBatInt
		
		//printf("Solar: %04i \tI_Out: %04i \tVint: %04i \tVUSB: %04i \tTemp: %04i \tVref: %04i \t(VBat: %04i)\n", \
		ADC_Buf[0], ADC_Buf[1], ADC_Buf[2], ADC_Buf[3], ADC_Buf[4], ADC_Buf[5], ADC_Buf[6]);
		//printf("Solar: %.03fV \tI_Out: %.03fV \tVint: %.03fV \tVUSB: %.03fV \tTemp: %2.03f°C \tVref: %.03fV \t(VBat: %.03fV)\n", \
		ADC2Volt(ADC_Buf[0]), ADC2Volt(ADC_Buf[1]), ADC2Volt(ADC_Buf[2]), ADC2Volt(ADC_Buf[3]), \
		TEMPIntCelsius(ADC2Volt(ADC_Buf[4])), ADC2Volt(ADC_Buf[5]), ADC2Volt(ADC_Buf[6]));
		
		// printf("V_int: %.03fV\n", ADC2Volt(ADC_Buf[2]));
		uint16_t adc_bits = 0;
		float adc_volt = 0.0;
		
		/*
		for(uint8_t i=0; i<8; i++){
			MAX11615_ADC_Read(&adcDriver_1, i, &adc_bits);
			// printf("%d:\t%05i\n", i, adc_bits);
			adc_volt = MAX_ADC2Volt(adc_bits);
			printf("%d:\t%04i   %.04fV\n", i, adc_bits, adc_volt);
		}*/
		printf("\n");

		laufvariable ++;
		if(laufvariable > 20)
			laufvariable = 0;