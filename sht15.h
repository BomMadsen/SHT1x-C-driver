/* 
 * File:   sht15.h
 * Author: Jacob Bom Madsen
 *
 */

#ifndef SHT15_H
#define	SHT15_H

// Requirements
// The sht15.c file must have access to a blocking delay function "delay_us(t)" and "delay_ms(t)"

// System setup
#define SHT_VDDVOLTAGE		3.3				// Volts, supply voltage for SHT15, can be 3.3 or 5.0

// Necessary pin assignments
#define DIR_OUTPUT          0					// TRIS value for output
#define DIR_INPUT           1					// TRIS value for input
#define HIGH                1					// PORT value for high (source, VDD)
#define LOW                 0					// PORT value for low (sink, VSS)

#define SHT15_CLK_dir       TRISB3			// Tri-state control register for CLK pin
#define SHT15_CLK           LATB3				// Output driver register for CLK pin

#define SHT15_DAT_dir       TRISB4			      // Tri-state control register for DAT pin
#define SHT15_DAT           PORTBbits.RB4     // In-out register for DAT pin

/* Initializes sensor
 * Should be called at start-up
 * If either CLK or DAT pins has other usages, should also be called before a measurement
 */
void SHT_init(void);

/* Measures the temperature
 * Returns temperature in degC
 */
float SHT_getTemp(void);

/* Measures the relative humidity
 * Returns relative humidity as float in %RH
 * Should be called AFTER getTemp, as it uses last temperature to perform temperature compensation
 */
float SHT_getHum(void);

/* Performs a diagnostics check of the SHT15
 * Should not be called frequently, as it uses the heater element. 
 * Measures T and RH, activates heater, repeats measurement
 * After heater operation, T should increase, RH should decrease and the dew point should remain the same.
 * Adjust your thresholds in system settings above
 * Returns true if beyond thresholds, false otherwise
 */
bool SHT_checkForError(void);


#endif	/* SHT15_H */
