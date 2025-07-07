/*
 * RTCDriver.cpp
 *
 *  Created on: 2022. jún. 26.
 *      Author: Dell
 */

#include "RTCDriver.h"


std::ostream& operator<<(std::ostream& os, eTimeClockFreq e)
{
	switch(e)
	{
		case _4kHz : os << "4kHz";    	break;
		case _64Hz : os << "64Hz";    	break;
		case sec   : os << "Second ";   break;
		case Min   : os << "Minute";    break;
		default	   : throw std::out_of_range("The eTimeClockFreq enumerator used with unknown value");
	}
	return os;
}

RTCDriver::RTCDriver(std::string name,SemaphoreHandle_t *Smpf, fncPntr preadI2CFnc, fncPntr pwriteI2CFnc) : SavingInterfaceClass(name) {
	this->smph = Smpf;
	this->_fp_readi2c = preadI2CFnc;
	this->_fp_writei2c = pwriteI2CFnc;
	queueCommand =  xQueueCreate(topicSize, sizeof(_scommand) );
}

QueueHandle_t RTCDriver::getCommandQueue(void) {
	return this->queueCommand;
}



esp_err_t RTCDriver::isTimerWakeUp(bool updateRequired, bool * bReturn) {
	esp_err_t ret;
	uint8_t reg =0;
	if(updateRequired == true) {
		xSemaphoreTake(*(this->smph), portMAX_DELAY);
		ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_CONTROL2 , &reg , 1);
		xSemaphoreGive(*(this->smph));
		if(ret != ESP_OK)
			return ret;
	} else {
		//register filled from internal structure variable
		reg = this->sttime.Control2;
		ret = ESP_OK;
	}
	// Checking the bit
	if ((reg >> 3 & 0x1) == 1) {
		// Timer wake up!
		*bReturn = true;
	}else{
		*bReturn = false;
	}
	return ret;
}

esp_err_t RTCDriver::readControl1Reg(uint8_t * reg){
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_CONTROL1 , (uint8_t *)(&(this->sttime.Control1)), 1);
	xSemaphoreGive(*(this->smph));
	//if(ret == ESP_OK) {
		*reg = (this->sttime.Control1);
	//}
	return ret;
}

esp_err_t RTCDriver::writeControl1Reg(uint8_t reg){
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_CONTROL1 , &reg , 1);
	xSemaphoreGive(*(this->smph));
	return ret;
}
esp_err_t RTCDriver::resetRTC(void) {
	return writeControl1Reg(RESET_RTC);
}

esp_err_t RTCDriver::readControl2Reg(uint8_t * reg) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_CONTROL2 , (uint8_t *)(&(this->sttime.Control2)), 1);
	xSemaphoreGive(*(this->smph));
	//if(ret == ESP_OK) {
		*reg = (this->sttime.Control2);
	//}
	return ret;
}

esp_err_t RTCDriver::writeControl2Reg(uint8_t reg) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_CONTROL2 , &reg , 1);
	xSemaphoreGive(*(this->smph));
	return ret;
}

esp_err_t RTCDriver::readAllRegsFromRTC(void) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_ALL , (uint8_t *)(&(this->sttime)), sizeof(_ttime)/sizeof(uint8_t));
	xSemaphoreGive(*(this->smph));
	return ret;
}

esp_err_t RTCDriver::writeAllRegsToRTC(void) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_ALL , (uint8_t *)(&(this->sttime)), sizeof(_ttime)/sizeof(uint8_t));
	xSemaphoreGive(*(this->smph));
	return ret;
}

esp_err_t RTCDriver::readTimeFromRTC(void) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_TIME , (uint8_t *)(&(this->sttime.Seconds)), 7);
	xSemaphoreGive(*(this->smph));
	return ret;
}


esp_err_t RTCDriver::writeTimeToRTC(void) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_TIME , (uint8_t *)(&(this->sttime.Seconds)), 7 );
	xSemaphoreGive(*(this->smph));
	return ret;
}

esp_err_t RTCDriver::writeYearToRTC(uint16_t year) {
	uint16_t ny = year - RTC_BIAS_YEAR;
	uint8_t _year;
	_year = RTCDriver::intToBCD( (uint8_t) ny);
	this->sttime.Year = _year;
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_YEAR , &_year , 1);
	xSemaphoreGive(*(this->smph));
	return ret;
}

esp_err_t RTCDriver::readYearFromRTC(uint16_t * year) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_YEAR , (uint8_t *)(&(this->sttime.Year)), 1);
	xSemaphoreGive(*(this->smph));
	*year = RTCDriver::bcdToInt(this->sttime.Year & FILTER_YEAR);
	*year += RTC_BIAS_YEAR;
	return ret;
}

esp_err_t RTCDriver::writeMonthToRTC(uint8_t month) {
	esp_err_t ret;
	uint8_t _month;
	_month = RTCDriver::intToBCD( month);
	this->sttime.Month = _month;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_MONTH , &_month , 1);
	xSemaphoreGive(*(this->smph));
	return ret;
}

esp_err_t RTCDriver::readMonthFromRTC(uint8_t *pmonth) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_MONTH , (uint8_t *)(&(this->sttime.Month)), 1);
	xSemaphoreGive(*(this->smph));
	*pmonth = RTCDriver::bcdToInt(this->sttime.Month & FILTER_MONTH);
	return ret;
}


esp_err_t RTCDriver::writeDateToRTC(uint8_t date) {
	esp_err_t ret;
	uint8_t _date;
	_date = RTCDriver::intToBCD( date);
	this->sttime.Month = _date;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret =  this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_DATE , &_date , 1);
	xSemaphoreGive(*(this->smph));
	return ret;
}

esp_err_t RTCDriver::readDateFromRTC(uint8_t *pdate) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_DATE , (uint8_t *)(&(this->sttime.Date)), 1);
	xSemaphoreGive(*(this->smph));
	*pdate = RTCDriver::bcdToInt(this->sttime.Date & FILTER_DATE);
	return ret;
}

esp_err_t RTCDriver::writeSecondsToRTC(uint8_t secs) {
	esp_err_t ret;
	uint8_t _secs;
	_secs = RTCDriver::intToBCD(secs);
	this->sttime.Seconds = _secs;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret =  this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_SECS , &_secs , 1);
	xSemaphoreGive(*(this->smph));
	return ret;
}

esp_err_t RTCDriver::readSecondsFromRTC(uint8_t *psecs) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_SECS , (uint8_t *)(&(this->sttime.Seconds)), 1);
	xSemaphoreGive(*(this->smph));
	*psecs = RTCDriver::bcdToInt(this->sttime.Seconds & FILTER_SECS);
	return ret;
}

esp_err_t RTCDriver::writeMinutesToRTC(uint8_t mins) {
	esp_err_t ret;
	uint8_t _mins;
	_mins = RTCDriver::intToBCD(mins);
	this->sttime.Minutes = _mins;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret =  this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_MINS , &_mins , 1);
	xSemaphoreGive(*(this->smph));
	return ret;
}

esp_err_t RTCDriver::readMinutesFromRTC(uint8_t *pmins) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_MINS, (uint8_t *)(&(this->sttime.Minutes)), 1);
	xSemaphoreGive(*(this->smph));
	*pmins = RTCDriver::bcdToInt(this->sttime.Minutes & FILTER_MINS);
	return ret;
}

esp_err_t RTCDriver::writeHoursToRTC(uint8_t hours) {
	esp_err_t ret;
	uint8_t _hours;
	_hours = RTCDriver::intToBCD(hours);
	this->sttime.Hours = _hours;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret =  this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_HOURS, &_hours, 1);
	xSemaphoreGive(*(this->smph));
	return ret;
}

esp_err_t RTCDriver::readHoursFromRTC(uint8_t *phours) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_HOURS , (uint8_t *)(&(this->sttime.Hours)), 1);
	xSemaphoreGive(*(this->smph));
	if(ESP_OK == ret)
		*phours = RTCDriver::bcdToInt(this->sttime.Hours & FILTER_HOURS);
	return ret;
}


long RTCDriver::getEpoch(void) {
	struct tm t;
	memset(&t, 0, sizeof(tm)); // Initalize to all 0's
	t.tm_year = RTCDriver::bcdToInt(this->sttime.Year & FILTER_YEAR) + RTC_BIAS_YEAR - EPOCH_YEAR; 	// This is year-1900, so RTC store from 2000, not sure which epoch has to be used
	t.tm_mon  = RTCDriver::bcdToInt(this->sttime.Month & FILTER_MONTH) - EPOCH_BIAS_MONTH;       			// According to https://en.cppreference.com/w/cpp/chrono/c/mktime subtract is required
	t.tm_mday = RTCDriver::bcdToInt(this->sttime.Date & FILTER_DATE);
	t.tm_hour = RTCDriver::bcdToInt(this->sttime.Hours & FILTER_HOURS);
	t.tm_min  = RTCDriver::bcdToInt(this->sttime.Minutes & FILTER_MINS);
	t.tm_sec  = RTCDriver::bcdToInt(this->sttime.Seconds & FILTER_SECS);

	return mktime(&t) ;
}


long RTCDriver::getEpochUTC(void) {
	return (RTCDriver::getEpoch() - ((long)(this->_timeZone) * HOURS_SECS));
}

void RTCDriver::updateTimeFromEpoch(long epoch, bool timeZoneUpdateReq) {
	struct tm *tm;
	long long e = epoch;
	if(timeZoneUpdateReq == true) {
		e +=  ((long)this->_timeZone * HOURS_SECS);
	}
	tm = gmtime( &e);
	//cout <<" This is: Y:"<< tm->tm_year <<"- M:"<< tm->tm_mon + EPOCH_BIAS_MONTH<<"- D: "<<tm->tm_mday<<" T "<<tm->tm_hour + TIME_ZONE <<"-"<<tm->tm_min<<"-"<<tm->tm_sec<<endl;
	this->sttime.Year		= RTCDriver::intToBCD( tm->tm_year + EPOCH_YEAR - RTC_BIAS_YEAR);
	this->sttime.Month		= RTCDriver::intToBCD( tm->tm_mon  + EPOCH_BIAS_MONTH);
	this->sttime.Date  		= RTCDriver::intToBCD( tm->tm_mday);
	this->sttime.Hours		= RTCDriver::intToBCD( tm->tm_hour);
	this->sttime.Minutes	= RTCDriver::intToBCD( tm->tm_min);
	this->sttime.Seconds	= RTCDriver::intToBCD( tm->tm_sec);
}

esp_err_t RTCDriver::writeTimeFromEpochToRTC(long epoch, bool timeZoneUpdateReq) {
	RTCDriver::updateTimeFromEpoch(epoch, timeZoneUpdateReq);
	return RTCDriver::writeTimeToRTC();
}

esp_err_t RTCDriver::readTimerValueFromRTC(uint8_t *pTimerValue) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_TIMER_VALUE , pTimerValue, 1);
	xSemaphoreGive(*(this->smph));
	return ret;
}
esp_err_t RTCDriver::writeTimerValueToRTC(uint8_t timerValue) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret =  this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_TIMER_VALUE, &timerValue, 1);
	xSemaphoreGive(*(this->smph));
	this->sttime.Timer_Value = timerValue;
	return ret;
}

esp_err_t RTCDriver::readTimerModeFromRTC(uint8_t *pTimerMode) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_TIMER_MODE , pTimerMode, 1);
	xSemaphoreGive(*(this->smph));
	return ret;
}

esp_err_t RTCDriver::writeTimerModeToRTC(uint8_t timerMode) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret =  this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_TIMER_MODE, &timerMode, 1);
	xSemaphoreGive(*(this->smph));
	this->sttime.Timer_Mode = timerMode;
	return ret;
}

esp_err_t RTCDriver::readRAMFromRTC(uint8_t *pData) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_RAM , pData, 1);
	xSemaphoreGive(*(this->smph));
	return ret;
}

esp_err_t RTCDriver::writeRAMToRTC(uint8_t data) {
	esp_err_t ret;
	xSemaphoreTake(*(this->smph), portMAX_DELAY);
	ret =  this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_RAM, &data, 1);
	xSemaphoreGive(*(this->smph));
	return ret;
}

esp_err_t RTCDriver::printAllRegs(bool updateRequired) {
	esp_err_t ret;

	if(updateRequired == true) {
		ret = this->RTCDriver::readAllRegsFromRTC();
	}
	else {
		ret = ESP_OK;
	}
	printf("Data Content : \n");
	uint8_t *p = (uint8_t *)(&(this->sttime));
	for (uint8_t i= 0; i<(sizeof(_ttime)/sizeof(uint8_t)); i++) {
		printf("0x%02X : %02X\n", i, *(p+i) );
	}
	return ret;

}

RTCDriver::~RTCDriver() {
	vQueueDelete(queueCommand);
}

uint8_t RTCDriver::intToBCD(uint8_t num) {
	return ((num / 10) << 4) | (num%10);
}

uint8_t RTCDriver::bcdToInt(uint8_t bcd) {
	// 0x10
	return ((bcd >> 4) * 10) + (bcd & 0x0f);;
}

void RTCDriver::setTimeZone(int8_t timeZone, bool timeupdate) {

	if(this->_timeZone != timeZone){
		this->setToChanged();
	}

	// in case of update requires the EPOCH shall be read out with the old TimeZone settings
	// before re-calculated with the new TimeZone value.
	if(timeupdate == true)
	{
		long epoch;
		epoch = this->getEpochUTC();
		this->_timeZone = timeZone;
		this->writeTimeFromEpochToRTC(epoch);
	}
	else
	{
		this->_timeZone = timeZone;
	}
}

int8_t RTCDriver::getTimeZone(void) {
	return this->_timeZone;
}

void RTCDriver::Load(cJSON * p_json) {
	_timeZone = cJSON_GetObjectItem(p_json,"TimeZone")->valueint;
	_dls = cJSON_GetObjectItem(p_json,"DayLightSave")->valueint;
}

cJSON* RTCDriver::Save() {
	cJSON * RTCObject;
	RTCObject = cJSON_CreateObject();
	cJSON_AddNumberToObject(RTCObject, "TimeZone", _timeZone);
	cJSON_AddNumberToObject(RTCObject, "DayLightSave", _dls);

	return RTCObject;
}

void RTCDriver::ResetDLS(void) {
	_dls = 0;
}

esp_err_t RTCDriver::CheckDLS() {
	esp_err_t ret;
	// get up to date time data from RTC
	ret = this->readAllRegsFromRTC();
	if(ret != ESP_OK)
	{
		return ret;
	}

	struct tm *l;
	long long epoch;
	int8_t l_dsl;
	epoch = getEpoch();
	l = gmtime(&epoch);
	cout<< "l->tm_mday, l->tm_mon, l->tm_wday" << l->tm_mday<< " " << l->tm_mon + EPOCH_BIAS_MONTH<< " " <<l->tm_wday<<endl;
	l_dsl =  IsDst(l->tm_mday, l->tm_mon + EPOCH_BIAS_MONTH, l->tm_wday) ? 1 :0;
	cout << "DLS : " << unsigned(l_dsl) << " stored DLS : " << unsigned(_dls) << endl;
	if(l_dsl != _dls )
	{
		//If the DLS changed in a Sunday --> hopefully the day of the DSL changes, check the time is over,
		// other wise change can happen immediately
		//if(      ( (l->tm_wday == 0) && ((l->tm_hour >= 3) && (l->tm_mon + EPOCH_BIAS_MONTH)==10 ) || (l->tm_hour >= 2 && (l->tm_mon + EPOCH_BIAS_MONTH)==3 )) || (l->tm_wday > 0))
		if ((l->tm_wday > 0) || ( (l->tm_wday == 0) &&  ( (l->tm_hour >= 3) && (((l->tm_mon + EPOCH_BIAS_MONTH)==10 ) || (((l->tm_hour >= 2) && (l->tm_mon + EPOCH_BIAS_MONTH)==3 )))) ) )
		{
			this->setToChanged();
			_dls = l_dsl;
			if(l_dsl == 0)
				l_dsl = -1;
			epoch = epoch + l_dsl * HOURS_SECS;
			writeTimeFromEpochToRTC(epoch,false);
		}else{
			cout << "No action until time ..." << endl;

		}
	}
	return ret;
}

esp_err_t RTCDriver::ForcedDLSUpdate(void) {
	esp_err_t ret;
	// get up to date time data from RTC
	ret = this->readAllRegsFromRTC();
	if(ret != ESP_OK)
	{
		return ret;
	}

	struct tm *l;
	long long epoch;
	int8_t l_dsl;
	epoch = getEpoch();
	l = gmtime(&epoch);
	
	ResetDLS();
	
	cout<< "l->tm_mday, l->tm_mon, l->tm_wday" << l->tm_mday<< " " << l->tm_mon + EPOCH_BIAS_MONTH<< " " <<l->tm_wday<<endl;
	l_dsl =  IsDst(l->tm_mday, l->tm_mon + EPOCH_BIAS_MONTH, l->tm_wday) ? 1 :0;
	cout << "DLS : " << unsigned(l_dsl) << " stored DLS : " << unsigned(_dls) << endl;
	
	
	this->setToChanged();
	_dls = l_dsl;
	if(l_dsl == 0)
		l_dsl = -1;
	epoch = epoch + l_dsl * HOURS_SECS;
	writeTimeFromEpochToRTC(epoch,false);
	return ret;
	
}


bool RTCDriver::IsDst(int day, int month, int dow) {
	if (month < 3 || month > 10)  return false;
	if (month > 3 && month < 10)  return true;

	int previousSunday = day - dow;

	if (month == 3) return previousSunday >= 25;
	if (month == 10) return previousSunday < 25;

	return false; // this line never gonna happend
}
