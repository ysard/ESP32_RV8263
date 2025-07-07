/*
 * RTCDriver.h
 *
 *  Created on: 2022. jún. 26.
 *      Author: zilizii
 */
using namespace std;

#ifndef MAIN_RTCDRIVER_H_
#define MAIN_RTCDRIVER_H_
#include <cstring>
#include <iostream>
#include <map>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "i2c_cmd.h"
#include "SavingInterfaceClass.h"

// i2c Address
#define ADDRESS_RTC   			0x51
#define RTC_BIAS_YEAR			2000

// Register Addresses
#define REG_ADDR_ALL			0x00
#define REG_ADDR_CONTROL1		0x00
#define REG_ADDR_CONTROL2		0x01
#define REG_ADDR_OFFSET			0x02
#define REG_ADDR_RAM			0x03
#define REG_ADDR_TIME			0x04
#define REG_ADDR_SECS			0x04
#define REG_ADDR_MINS			0x05
#define REG_ADDR_HOURS			0x06
#define REG_ADDR_DATE			0x07
#define REG_ADDR_WEEKDAY		0x08
#define REG_ADDR_MONTH			0x09
#define REG_ADDR_YEAR			0x0A

#define REG_ADDR_TIMER_VALUE	0x10
#define REG_ADDR_TIMER_MODE		0x11

#define RESET_RTC				0x58

//Filter default values
#define FILTER_SECS				0x7F
#define FILTER_MINS				0x7F
#define FILTER_HOURS			0x7F
#define FILTER_DATE				0x3F
#define FILTER_WEEKDAY			0x03
#define FILTER_MONTH			0x1F
#define FILTER_YEAR				0xFF

#define FILTER_TIMER_VALUE		0xFF

#define EPOCH_YEAR				CONFIG_EPOCH_YEAR
#define EPOCH_BIAS_MONTH		1
#define TIME_ZONE				CONFIG_TIME_ZONE

#define AIE						0x80 /*Alarm Interrupt Enable*/
#define AF						0x40 /*Alarm Flag*/
#define MI						0x20 /*Minute Interrupt Enable*/
#define HMI						0x10 /*Half Minute Interrupt Enable*/
#define TF						0x08 /*Timer Flag*/
#define TF_CHECK(x) 			((x & TF) >> 3)
#define FD 						0x07 /*CLKOUT Frequency*/

#define FD_32kHz				0x00 /*32.768 kHz – Default value*/
#define FD_16kHz				0x01 /*16.384 kHz*/
#define FD_8kHz					0x02 /* 8.192 kHz*/
#define FD_4kHz					0x03 /* 4.096 kHz*/
#define FD_2kHz					0x04 /* 2.048 kHz*/
#define FD_1kHz					0x05 /* 1.024 kHz*/
#define FD_1Hz					0x06 /* 1      Hz*/
#define FD_CLKOUT_LOW			0x07 /*CLKOUT = LOW*/

#define TD						0x18 /*Timer Clock Frequency */
#define TD_CHECK(x)				((x & TD) >> 3)
#define TE						0x04 /*Timer Enable*/
#define TE_CHECK(x)				((x & TE) >> 2)
#define TIE						0x02 /*Timer Interrupt Enable*/
#define TIE_CHECK(x)			((x & TIE) >> 1)
#define TI_TP					0x01 /*Timer Interrupt Mode*/
#define TI_TP_CHECK(x)			(x & TI_TP)

#define TD_4kHz					0x00 /*4.096 kHz*/
#define TD_64Hz					0x01 /*64     Hz*/
#define TD_1Hz					0x02 /* 1     Hz*/
#define TD_1_60Hz				0x03 /* 1/60  Hz - Default value*/

#define HOURS_SECS				3600

enum eTimeClockFreq { _4kHz = TD_4kHz, _64Hz = TD_64Hz,  sec = TD_1Hz,  Min =  TD_1_60Hz};

std::ostream& operator<<(std::ostream& os, eTimeClockFreq e);



/*
 * Stucture to store the RTC values in memory, Coded with BCD
 * */

// DONE operator [] overload for struct
typedef struct  __attribute__ ((packed)) {
	uint8_t	Control1;		// 0x00
	uint8_t	Control2;		// 0x01
	uint8_t	Offset;			// 0x02
	uint8_t RAM;			// 0x03
	uint8_t Seconds;    	// 0x04
	uint8_t Minutes;		// 0x05
	uint8_t Hours;			// 0x06
	uint8_t Date;			// 0x07
	uint8_t Weekday;		// 0x08
	uint8_t Month;			// 0x09
	uint8_t Year;			// 0x0A
	uint8_t Seconds_Alarm;	// 0x0B
	uint8_t Minutes_Alarm;	// 0x0C
	uint8_t Hours_Alarm;	// 0x0D
	uint8_t Date_Alarm;		// 0x0E
	uint8_t Weekday_Alarm;	// 0x0F
	uint8_t Timer_Value;	// 0x10
	uint8_t Timer_Mode;		// 0x11
	uint8_t& operator[](std::size_t idx) {
		return *(uint8_t*)((size_t)this + idx * sizeof(uint8_t));
	}

}_ttime;

typedef struct {
	char command[255];
	uint8_t ID;
}_scommand;


//esp_err_t i2c_master_read_slave (i2c_port_t i2c_num, uint8_t Address, uint8_t Offset, uint8_t *data_rd, size_t size)
//esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t Address, uint8_t Offset ,uint8_t *data_wr, size_t size)

// Typedef for the function pointers --> seems both the read and write use the similar definition...

typedef esp_err_t (*fncPntr)(i2c_port_t, uint8_t, uint8_t, uint8_t *, size_t );



class RTCDriver : public SavingInterfaceClass {
private:
	SemaphoreHandle_t  * smph;
	uint8_t intToBCD(uint8_t num);
	uint8_t bcdToInt(uint8_t bcd);
	fncPntr _fp_writei2c = nullptr;
	fncPntr _fp_readi2c = nullptr;
    int8_t _timeZone = TIME_ZONE;
    int8_t _dls = 0;
	QueueHandle_t queueCommand;
	unsigned int topicSize = CONFIG_TOPIC_SIZE;
	_ttime sttime = {};
public:
	//SavingInterfaceClass interface functions
	cJSON* Save();
	void Load(cJSON*);

	RTCDriver(std::string name, SemaphoreHandle_t *, fncPntr readI2CFnc, fncPntr writeI2CFnc);
	QueueHandle_t getCommandQueue(void);

	esp_err_t readAllRegsFromRTC(void);
	esp_err_t writeAllRegsToRTC(void);
	esp_err_t readControl1Reg(uint8_t *);
	esp_err_t writeControl1Reg(uint8_t);
	esp_err_t resetRTC(void);
	esp_err_t readControl2Reg(uint8_t *);
	esp_err_t writeControl2Reg(uint8_t);
	esp_err_t writeTimeToRTC(void);
	esp_err_t readTimeFromRTC(void);

	long getEpoch(void);
	long getEpochUTC(void);
	void updateTimeFromEpoch(long, bool = true);
	esp_err_t writeTimeFromEpochToRTC(long, bool = true);
	void setTimeZone(int8_t, bool = false);
	int8_t getTimeZone(void);
	
	void ResetDLS(void);
	esp_err_t CheckDLS();
	bool IsDst(int day, int month, int dow);
	esp_err_t ForcedDLSUpdate(void);
	 

	esp_err_t writeYearToRTC(uint16_t);
	esp_err_t readYearFromRTC(uint16_t *);
	esp_err_t writeMonthToRTC(uint8_t);
	esp_err_t readMonthFromRTC(uint8_t *);
	esp_err_t writeDateToRTC(uint8_t);
	esp_err_t readDateFromRTC(uint8_t *);
	esp_err_t writeSecondsToRTC(uint8_t);
	esp_err_t readSecondsFromRTC(uint8_t *);
	esp_err_t writeMinutesToRTC(uint8_t);
	esp_err_t readMinutesFromRTC(uint8_t *);
	esp_err_t writeHoursToRTC(uint8_t);
	esp_err_t readHoursFromRTC(uint8_t *);

	// Timer Functionality supporting implementation
	esp_err_t isTimerWakeUp(bool, bool *);
	esp_err_t readTimerValueFromRTC(uint8_t *);
	esp_err_t writeTimerValueToRTC(uint8_t);
	esp_err_t readTimerModeFromRTC(uint8_t *);
	esp_err_t writeTimerModeToRTC(uint8_t);

	// RAM byte implementation
	esp_err_t readRAMFromRTC(uint8_t *);
	esp_err_t writeRAMToRTC(uint8_t);


	//testing purpose functions
	esp_err_t printAllRegs(bool);
	virtual ~RTCDriver();
};

#endif /* MAIN_RTCDRIVER_H_ */
