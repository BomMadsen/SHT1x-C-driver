#include "sht15.h"

#include <math.h>

#define cmd_meas_Temp       0b00000011
#define cmd_meas_RH         0b00000101
#define cmd_read_stat       0b00000111
#define cmd_write_stat      0b00000110
#define cmd_soft_reset      0b00011110

#define statusMask_Heater	2
#define statusMask_OTP      1
#define statusMask_Res		0

#define clock_freq          100000              // Hz
#define clock_Hperiod       5                   // us, Half period (time high or low for clock bus)
                                                // (1/(clock_freq*2))*1000000 = 5 us
#define start_delay         20      			//us

#define use_checksum        false
#define RH_temp_comp        true

    // Internal functions
void start_transmission(void);
void send_command(uint8_t cmd);
uint16_t read_command(uint8_t bytes_to_read);
uint8_t read_byte(uint8_t sendAck);
void reset_serial(void);
uint8_t read_statusReg(void);
void write_statusReg(uint8_t bitMask, uint8_t value);
float calc_Temp(uint16_t measurement);
float calc_RH(uint16_t measurement);
float calc_DewPoint(float RH, float Temp);
float logN(float value);

float lastTemp;
float thisTemp;
float thisRH;

void SHT_init(void){
	// Setup outputs to sensor
	SHT15_CLK = LOW;
	SHT15_DAT = HIGH;
	SHT15_CLK_dir = DIR_OUTPUT;
	SHT15_DAT_dir = DIR_OUTPUT;

	// Wait for SHT to wake up
	delay_ms(15);

  // Set to high resolution
  write_statusReg(statusMask_Heater, 0);
  write_statusReg(statusMask_OTP, 0);
  write_statusReg(statusMask_Res, 0);
}

float SHT_getTemp(void){
	SHT_init();

	start_transmission();
	send_command(cmd_meas_Temp);
	while(!SHT15_DAT){;}
	delay_ms(10);
	SHT15_DAT_dir = DIR_INPUT;          // Set data line to input
	while(SHT15_DAT){};                 // Wait for SHT to pull DATA low after measurement
	uint16_t result = read_command(2);
	thisTemp = calc_Temp(result);
	lastTemp = thisTemp;

	return thisTemp;
}

float SHT_getHum(void){
	SHT_init();

	start_transmission();
	send_command(cmd_meas_RH);
	while(!SHT15_DAT){;}
	delay_ms(10);
	SHT15_DAT_dir = DIR_INPUT;          // Set data line to input
	while(SHT15_DAT){;}                 // Wait for SHT to pull DATA low after measurement
	uint16_t result = read_command(2);
	thisRH = calc_RH(result);

	return thisRH;
}

#define CHK_TempChange			0.5				// degC - increase needed after heater applied
#define CHK_RHchange			  0.1				// %RH  - decrease needed after heater applied
#define CHK_DewThreshold		1.5				// degC - threshold for equal dew points
#define CHK_HeaterTime			1000			// ms - time of which heater should be applied

bool SHT_checkForError(void){
	float Temp1 = SHT_getTemp();
	float RH1 = SHT_getHum();
	
	// Apply heater for some time
	write_statusReg(statusMask_Heater, true);
	delay_ms(CHK_HeaterTime);
	
	float Temp2 = SHT_getTemp();
	float RH2 = SHT_getHum();
	
	// Turn off heater
	write_statusReg(statusMask_Heater, false);
	
	// Calculate dew points
	float dew1 = calc_DewPoint(RH1, Temp1);
	float dew2 = calc_DewPoint(RH2, Temp2);
	
	bool errorState = false;
	
	// Ensure Temp increases and Hum decreases
	if(Temp1 + CHK_TempChange > Temp2){
		// Temperature did not increase sufficiently
		errorState = true;
	}
	
	if(RH1 - CHK_RHchange < RH2){
		// Humidity did not decrease sufficiently
		errorState = true;
	}
	
	float dewDifference = dew1 - dew2;
    if(fabs(dewDifference)>CHK_DewThreshold){
        // Dew points changed too much
		errorState = true;
    }
	
	
	
	return errorState;
}

uint8_t read_statusReg(void){
    start_transmission();
    send_command(cmd_read_stat);
    uint8_t result = (uint8_t) read_command(1);

    return result;
}

void write_statusReg(uint8_t bitMask, uint8_t value){
    uint8_t statusRegister;

    statusRegister = read_statusReg();

    // Set the bitMask bit to value - keep other values
    statusRegister ^= (-value^statusRegister) & (0b1<<bitMask);

    start_transmission();
    send_command(cmd_write_stat);
    send_command(statusRegister);

    delay_ms(10);

    uint8_t newStatusRegister = read_statusReg();

    if(statusRegister == newStatusRegister){
        // Success
        NOP();
    }
}


void start_transmission(void){
    reset_serial();

    SHT15_DAT = HIGH;
    delay_us(start_delay);
    SHT15_CLK = HIGH;
    delay_us(start_delay);
    SHT15_DAT = LOW;
    delay_us(start_delay);
    SHT15_CLK = LOW;
    delay_us(start_delay);
    SHT15_CLK = HIGH;
    delay_us(start_delay);
    SHT15_DAT = HIGH;
    delay_us(start_delay);
    SHT15_CLK = LOW;
    delay_us(start_delay);
}

void reset_serial(void){
    SHT15_DAT_dir = DIR_OUTPUT;
    SHT15_CLK_dir = DIR_OUTPUT;
    SHT15_DAT = HIGH;
    uint8_t i = 0;
    while(i++<10){
        SHT15_CLK = !SHT15_CLK;
        delay_us(clock_Hperiod);
    }
    SHT15_CLK = LOW;
}

void send_command(uint8_t cmd){
    SHT15_DAT_dir = DIR_OUTPUT;

    uint8_t bits;
    uint8_t ptr = 7;
    uint8_t i = 0;
    while(i++<8){
        bits = (cmd>>ptr--)&0b1;
        SHT15_DAT = bits;
        delay_us(clock_Hperiod);
        SHT15_CLK = HIGH;
        delay_us(clock_Hperiod);
        SHT15_CLK = LOW;
    }

    // ACK
    SHT15_DAT_dir = DIR_INPUT;
    delay_us(clock_Hperiod);
    while(SHT15_DAT){;}
    SHT15_CLK = HIGH;
    delay_us(clock_Hperiod);
    SHT15_CLK = LOW;
    
}

uint16_t read_command(uint8_t bytes_to_read){
    uint16_t result = 0;

    uint8_t i = bytes_to_read;
    uint8_t result8 = 0;
    while(i--){
        if(i){
            result8 = read_byte(1);
        }else{
            result8 = read_byte(0);
        }
        result |= (((uint16_t)result8) & 0b11111111)<<(8*i);
    }

    return result;
}

uint8_t read_byte(uint8_t sendAck){
    SHT15_DAT_dir = DIR_INPUT;
    SHT15_CLK_dir = DIR_OUTPUT;

    uint8_t result = 0;
    uint8_t this_bit;
    uint8_t i = 8;
    while(i){
        SHT15_CLK = HIGH;
        delay_us(clock_Hperiod);
        this_bit = SHT15_DAT;
        SHT15_CLK = LOW;
        delay_us(clock_Hperiod);
        result = result | (this_bit<<(i-1));
        i--;
    }
    SHT15_DAT_dir = DIR_OUTPUT;
    SHT15_DAT = !(sendAck);
    SHT15_CLK = HIGH;
    delay_us(clock_Hperiod);
    SHT15_CLK = LOW;
    delay_us(clock_Hperiod);
    SHT15_DAT = HIGH;

    return result;
}

float calc_Temp(uint16_t measurement){
    float measFloat = (float) measurement;
    float d1, d2;
    if(SHT_VDDVOLTAGE == 3.3){
        d1 = -39.700000;
    }
    else if(SHT_VDDVOLTAGE == 5.0){
        d1 = -40.100000;
    }
    d2 = 0.010000;

    float TempC = d1+(d2*measFloat);
    
    return TempC;
}

float calc_RH(uint16_t measurement){
    // V4 optimized values
    //const float c1 = -2.0468;
    //const float c2 = 0.0367;
    //const float c3 = -0.0000015955;
    // V3 and V4 values
    const float c1 = -4.0;
    const float c2 = 0.0405;
    const float c3 = -0.0000028;
    
    float RH_lin = c1 + (c2*measurement) + (c3*measurement*measurement);
    if(RH_temp_comp){
        const float t1 = 0.01;
        const float t2 = 0.00008;
        float RH_true = (lastTemp - 25)*(t1+(t2*measurement))+RH_lin;
        return RH_true;
    }
    else{
        return RH_lin;
    }
}


float calc_DewPoint(float RH, float Temp){
	float dewPoint;
	float Tn, m;
	if(Temp >= 0){
		Tn = 243.120000;
		m = 17.620000;
	}else{
		Tn = 272.620000;
		m = 22.460000;
	}
	
	float logToRH 	= 	logN(RH/100.0);
	float Tfactor 	= 	(m*Temp)/(Tn+Temp);
	
	float dividend 	= 	logToRH + Tfactor;
	float divisor 	= 	m - logToRH - Tfactor;
	
	if(divisor!=0.0){
		dewPoint = Tn*(dividend/divisor);
	} else{
		dewPoint = Tn*(dividend/0.000001);
	}
	
	return dewPoint;
}

float logN(float value){
	// May be compiler dependent, ensure that the "log()" function is the natural log
	return log(value);
}
