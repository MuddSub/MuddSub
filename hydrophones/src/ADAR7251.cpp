#include "hydrophones/ADAR7251.hh"
#include <bitset>
#include <time.h>
#include <chrono>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace MuddSub::Acoustics
{

Hydrophones::Hydrophones()
{

	wiringPiSetupGpio();
	wiringPiSPISetup(0, 500000);

	for(auto i : raspiInputPins)
		pinMode(i, INPUT);
	for(auto i : raspiOutputPins)
		pinMode(i, OUTPUT);

	powerOff();
	stopAquisition();
	digitalWrite(RPI_N_STATUS_R_PIN, HIGH);
	digitalWrite(RPI_N_STATUS_G_PIN, HIGH);
	digitalWrite(RPI_N_STATUS_B_PIN, HIGH);

	powerOn();

	//Create directories for storing stuff
	std::string path = ros::package::getPath("hydrophones");
	int t = (int)ros::Time::now().toSec();
	std::string filePathBase = path + "/data/" + std::to_string(t);
	if (mkdir(filePathBase.c_str(), 0777) == -1)
		std::cerr << "Error :  " << strerror(errno) << std::endl;
	else
		std::cout << "Directory created";

	rawFilePath = filePathBase+"/raw";
	processedFilePath = filePathBase+"/processed";

	if (mkdir(processedFilePath.c_str(), 0777) == -1)
        std::cerr << "Error :  " << strerror(errno) << std::endl;
  else
    std::cout << "Directory created";

	if (mkdir(rawFilePath.c_str(), 0777) == -1)
    std::cerr << "Error :  " << strerror(errno) << std::endl;
  else
  	std::cout << "Directory created";
	word_t test;
	bool skipOne = true;
	while(ros::ok())
	{
		if(skipOne)
		{
			skipOne = false;
			continue;
		}
		test = spiRead(ADC_SETTING1);
		// Needed a register which defaults to non-zero, but we also change most of those later...
		if(test == 0x304) break;
		ROS_ERROR("SPI Read Test on ADC_SETTING1 failed with value %d. Trying again",test);
		sleep(1);
	}
	std::cout << "Test passed" << std::endl;


	//Disable CRC
	while(ros::ok())
	{
		std::vector<word_t> data = {0x0001, 0x3307};
		spiWrite(CRC_EN, data);
		if(spiRead(CRC_EN)) break;
		ROS_ERROR("Failed to disable CRC. Trying again");

		sleep(1);
	}

	//PLL CONFIG (set frequency of PLL)
	regSetRange(PLL_CTRL, 11, 15, 6); //pll_integer_div=6
	regSetRange(PLL_CTRL, 4, 7, 1);   //pll_input_prescale=1
	regSetBit(PLL_CTRL, 0, 1);      //enable pll


	regSetBit(CLK_CTRL, 0, 0);       //use pll clock

	//TODO: timeout
	while(!spiRead(PLL_LOCK) && ros::ok()){
		ROS_WARN("Waiting for PLL Lock");
		continue;
	}
	ROS_INFO("PLL Lock Aquired");


	//Power Enable
	//regSetBit(POWER_ENABLE, 0, 0);   //Disable clock generator
	//regSetBit(POWER_ENABLE, 1, 0);   //Disable Serial Output

	//Decimator set to 300kHz
	regSetRange(DECIM_RATE, 0, 2, 7);

	//High Pass Filter
	regSetRange(HIGH_PASS, 1, 5, 0b10010);  //HP shift value 18
	regSetBit(HIGH_PASS, 0, 1);

	//Configure output
	regSetBit(OUTPUT_MODE, 0, 1);   //parallel mode
	regSetBit(PARALLEL_MODE, 2, 1); //byte mode

	//Pullups/downs enabled
	regSetBit(SPI_CLK_PIN, 2, 1);
	regSetBit(MISO_PIN, 2, 1);
	regSetBit(MOSI_PIN, 2, 1);
	regSetBit(MISO_PIN, 2, 1);
	regSetBit(SCLK_ADC_PIN, 2, 1);
	regSetBit(CS_PIN, 2, 1);
	regSetBit(SS_PIN, 2, 1);
	regSetBit(FAULT_PIN, 2, 1);

	regSetBit(ADC_DOUT0_PIN, 2, 1);
	regSetBit(ADC_DOUT1_PIN, 2, 1);
	regSetBit(ADC_DOUT2_PIN, 2, 1);
	regSetBit(ADC_DOUT3_PIN, 2, 1);
	regSetBit(ADC_DOUT4_PIN, 2, 1);
	regSetBit(ADC_DOUT5_PIN, 2, 1);
	regSetBit(ADDR15_PIN, 2, 1);
	regSetBit(FS_ADC_PIN, 2, 1);


	//TODO: how does peak detect work?

	//ADC Settings
	regSetBit(ADC_SETTING1, 1, 1);
	regSetRange(ADC_SETTING2, 0, 4, 0b10011);
	regSetRange(ADC_SETTING3, 2, 3, 0);


	//Enable it
	regSetBit(MASTER_ENABLE, 0, 1);

	//check for error
	skipOne = 1;
	while(spiRead(ASIL_FLAG))
	{
		if(skipOne)
		{
			skipOne = 0;
			continue;
		}
		ROS_ERROR("ADAR7251 Error Code %d", spiRead(ASIL_ERROR));
		regSetBit(ASIL_CLEAR, 0, 1);
		sleep(2);
	}
	ROS_INFO("ADAR7251 Initialization Complete; no errors reported.");


  // Open gpio register

  int fd = open("/dev/gpiomem", O_RDWR | O_SYNC);

  if (fd < 0)
    ROS_ERROR("failed to open /dev/gpiomem");
	else
	{
		gpioRegOpen_ = true;
		ROS_INFO("Opened GPIO memory");
	}

  gpioReg_ = (volatile uint32_t *)mmap(NULL, 0xB4, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);

  close(fd);
  return;
}


Hydrophones::~Hydrophones()
{
	powerOff();
	stopAquisition();
}

Hydrophones::word_t Hydrophones::spiRead(word_t addr)
{
	unsigned char buffer[5];
	buffer[0] = 0x1 | (addr15<<1); //set bit 1 to 1 or 0 accordingly, bit 0 is always 1 for read mode
	buffer[1] = addr >> 8;     //shift to get only MSB
	buffer[2] = (addr & 0xFF); //mask MSB with zeros
	buffer[3] = 0;
	buffer[4] = 0;
	int err = wiringPiSPIDataRW(channel, buffer, 5);
	word_t result = buffer[4];
	result |= (buffer[3] << 8);
	return result;
}

void Hydrophones::spiWrite(word_t addr, std::vector<word_t> data)
{
	unsigned char buffer[maxWrite*2 + 1];
	int len = 2*data.size() + 3;
	buffer[0] = addr15 << 1; //Set address, and bit 0 is always 0 for write mode
	buffer[1] = addr >> 8;     //shift to get only MSB
	buffer[2] = (addr & 0x00FF); //mask MSB with zeros
	for(int i = 0; i < data.size(); ++i)
	{
		buffer[2*i+3] = data[i] >> 8;     //shift to get only MSB
		buffer[2*i+4] = (data[i] & 0xFF); //mask MSB with zeros
	}
	for(int i = data.size(); i < maxWrite; ++i)
	{
		buffer[2*i+3] = 0;
		buffer[2*i+4] = 0;
	}
	int err = wiringPiSPIDataRW(channel, buffer, len);
}

void Hydrophones::spiWrite(word_t addr, word_t data)
{
	std::vector<word_t> v;
	v.push_back(data);
	spiWrite(addr, v);
}

void Hydrophones::setBit(word_t& data, int pos, bool value)
{
	word_t mask = (1 << pos);
	if(value == 1)
	{
		data |= mask;
		return;
	}
	//invert the mask
	mask = ~mask;
	data &= mask;
}

void Hydrophones::setBits(word_t& data, word_t mask, bool value)
{
	if(value == 1)
	{
		data |= mask;
		return;
	}
	mask = ~mask;
	data &= mask;
}

Hydrophones::word_t Hydrophones::regSetBit(word_t addr, int pos, bool value)
{
	word_t data = spiRead(addr);
	setBit(data, pos, value);
	spiWrite(addr,data);
	return data;
}

Hydrophones::word_t Hydrophones::regSetBits(word_t addr, word_t mask, bool value)
{
	word_t data = spiRead(addr);
	setBits(data, mask, value);
	spiWrite(addr,data);
	return data;
}

Hydrophones::word_t Hydrophones::regSetRange(word_t addr, int startPos, int endPos, word_t value)
{
	word_t data = spiRead(addr);

	//mask for setting zeros to ones
	word_t mask = value << startPos;

	data |= mask;

	//mask for setting ones to zeros needs padding of 1s on both sides of the value mask
	mask |= 0xFFFF << endPos | 0xFFFF >> (16-startPos);
	data &= mask;

	spiWrite(addr, data);

	return data;
}

Hydrophones::raspiReg_t Hydrophones::readGPIOBank()
{
	if(!gpioRegOpen_)
	{
		ROS_ERROR("Can't read GPIO register; failed to open.");
		return 0;
	}
	unsigned int GPIO_LEV_0 = 13;
	return *(gpioReg_ + GPIO_LEV_0);
}

void Hydrophones::sample(float secs)
{

	rawSamples = std::vector<raspiReg_t>();
	samplesProcessed = false;

	clock_t numTicks = secs * CLOCKS_PER_SEC;

	bool prevSclk = true;
	clock_t t = clock();
	startAquisition();
	bool skipOne = true;
	while(!dataReady() && ros::ok())
	{
		if(skipOne)
		{
			skipOne = false;
			continue;
		}
		std::string isError = spiRead(ASIL_FLAG)? "due to an ASIL error." : "not due to an ASIL error.";
		ROS_WARN("Data not ready. This was %s", isError.c_str());
	}
	//If needed: approximate clock cycles per loop, and just loop that many times
	bool sclk;
	do{
		sclk = clockState();
		if(!sclk && prevSclk){
			raspiReg_t val = readGPIOBank();
			rawSamples.push_back(val);
		}
		prevSclk = sclk;
	}while(clock() - t < numTicks);

	stopAquisition();

	//write to txt
	std::string path = rawFilePath + "/" + std::to_string(sampleCounter);
	std::ofstream file;
	file.open(path);
	rawSampleFiles.push_back(path);
	for(auto i : rawSamples)
		file << i << std::endl;

	++sampleCounter;
}

void Hydrophones::processSamples(){
	//We need an even number of readings, since each measurement is two readings

	int len = rawSamples.size();

	if(rawSamples.size() % 2 == 1){
		--len;
		rawSamples.pop_back();
	}

	for(int i = 0; i < len/2; ++i){
		raspiReg_t highReg = rawSamples.at(2*i);
		raspiReg_t lowReg = rawSamples.at(2*i+1);

		uint8_t msb = getByteFromReg(highReg);
		uint8_t lsb = getByteFromReg(lowReg);

		//assemble the bits
		word_t sample = (msb << 8) | lsb;
		samples.push_back(sample);
	}

	//write to txt
	std::string path = processedFilePath + "/" + std::to_string(sampleCounter);
	std::ofstream file;
	file.open(path);
	processedSampleFiles.push_back(path);
	for(auto i : samples)
		file << i << std::endl;
}

uint8_t Hydrophones::getByteFromReg(raspiReg_t reg){
	uint8_t result = 0;
	for(int i = 0; i < 8; ++i){
		int pos = dataPins[i];

		//wipe out all the bits but the one we care about
		raspiReg_t mask = (1 << pos);
		uint8_t bit= (mask & pos) >> pos;
		result |= (bit << i);
	}
	return result;
}

} // namespace MuddSub::Acoustics
