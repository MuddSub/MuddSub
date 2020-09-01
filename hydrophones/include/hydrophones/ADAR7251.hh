#pragma once

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <vector>
#include "ros/ros.h"
#include "ros/package.h"
#include "hydrophones/regmap.hh"
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>


namespace MuddSub::Acoustics
{

class Hydrophones{

typedef uint16_t word_t;
typedef uint32_t raspiReg_t;

static const int channel = 0;
static const char addr15 = 0;

//max words we can write at once
static const int maxWrite = 4;

public:
	Hydrophones();
	~Hydrophones();

	// ------- SPI Interface: -------
	//read/write to register map
	void spiWrite(word_t addr, word_t data);
	void spiWrite(word_t addr, std::vector<word_t> data);
	word_t spiRead(word_t addr);

	//read the given address, changes specified bits, then re-writes
	word_t regSetBit(word_t addr, int pos, bool value);
	word_t regSetBits(word_t addr, word_t mask, bool value);
	word_t regSetRange(word_t addr, int startPos, int endPos, word_t value);


	// ------- Sampling -------
	void sample(float secs);
	void processSamples();

	raspiReg_t readGPIOBank();

private:
	bool spiErr;

	//given a word, set the given bit to the given value
	void setBit(word_t& data, int pos, bool value);
	//given a word and a mask, set the bits corresponding to 1s in the mask to value
	void setBits(word_t& data, word_t mask, bool value);

	bool aquiring = false;
	inline void startAquisition(){
		digitalWrite(RPI_N_CONV_START_PIN, LOW);
		aquiring = true;
	}
	inline void stopAquisition(){
		digitalWrite(RPI_N_CONV_START_PIN, HIGH);
		aquiring = false;
	}

	inline void powerOn(){
		digitalWrite(RPI_N_RESET_PWDN_PIN, HIGH);
	}
	inline void powerOff(){
		digitalWrite(RPI_N_RESET_PWDN_PIN, LOW);
	}

	inline bool dataReady(){return digitalRead(RPI_DATA_READY_PIN);}
	inline bool clockState(){return digitalRead(RPI_SCLK_ADC_PIN);}

	std::vector<raspiReg_t> rawSamples;
	std::vector<word_t> samples;
	bool samplesProcessed{false};
	std::vector<std::string> rawSampleFiles;
	std::vector<std::string> processedSampleFiles;

	std::string rawFilePath;
	std::string processedFilePath;

	unsigned int sampleCounter{0};

	uint8_t getByteFromReg(raspiReg_t reg);

	ros::NodeHandle nh_;

	unsigned int dataPins[8] = {DIN_PIN_0, DIN_PIN_1, DIN_PIN_2, DIN_PIN_3,
	                            DIN_PIN_4, DIN_PIN_5, DIN_PIN_6, DIN_PIN_7};

  volatile raspiReg_t* gpioReg_;
	bool gpioRegOpen_{false};

};

} // namespace MuddSub::Acoustics
