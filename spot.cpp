/*
 * spot.c
 *
 * Edited on: Dec 22, 2025
 * Author: Wojciech Kaczmarski, SP5WWP
 *         M17 Foundation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cerrno>
#include <atomic>
#include <math.h>
#include <stdarg.h>

#include <netinet/ip_icmp.h>
#include <netinet/udp.h>
#include <netinet/tcp.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>

#include <fcntl.h> 
#include <sys/ioctl.h>
#include <time.h>
#include <signal.h>

#include "CC1200.h"
#include "RingBuffer.h"

//spot commands
#include "interface_cmds.h"

//libm17
#include <m17.h>

#include "term.h" //colored terminal font

#define MAX_UDP_LEN				65535

#define RX_SYMBOL_SCALING_COEFF	(1.0f/(0.8f/(40.0e3f/2097152*0xAD)*130.0f))
// CC1200 User's Guide, p. 24
// 0xAD is `DEVIATION_M`, 2097152=2^21
// +1.0 is the symbol for +0.8kHz
// 40.0e3 is F_TCXO in kHz
// 129 is `CFM_RX_DATA_OUT` register value at max. F_DEV (130 is 1 off but offers a better symbol map)
// datasheet might have this wrong (it says 64)

#define TX_SYMBOL_SCALING_COEFF	(0.8f/((40.0e3f/2097152)*0xAD)*64.0f)
// 0xAD is `DEVIATION_M`, 2097152=2^21
// +0.8kHz is the deviation for symbol +1
// 40.0e3 is F_TCXO in kHz
// 64 is `CFM_TX_DATA_IN` register value for max. F_DEV

static std::atomic<bool> keep_running = true;

//internet
struct sockaddr_in source, dest; 
int sockt;
struct iphdr *iph;
struct sockaddr_in saddr;
struct sockaddr_in daddr;
struct sockaddr_in serv_addr;
uint32_t saddr_size=sizeof(saddr);

uint8_t tx_buff[512]={0};
uint8_t rx_buff[1000]={0};
int tx_len=0, rx_len=0;

//config stuff
struct config_t
{
	char log_path[128];
	char uart[64];
	uint32_t uart_rate;
	char node[10];
	char refl_addr[20];
	uint16_t refl_port;
	char reflector[8];
	char module;
	uint8_t enc_node[6];
	int16_t freq_corr;
	float tx_pwr;
	uint32_t rx_freq;
	uint32_t tx_freq;
	bool afc;

	//GPIO Pins
	uint16_t boot0;
	uint16_t nrst;
} config;

//M17
struct m17stream_t
{
	uint16_t sid;
	lsf_t lsf;
	uint16_t fn;
	uint8_t pld[16];
} m17stream;

enum rx_state_t
{
	RX_IDLE,
	RX_SYNCD
};

enum tx_state_t
{
	TX_IDLE,
	TX_ACTIVE
};

enum err_t
{
	ERR_OK,					//all good
	ERR_TRX_PLL,			//TRX PLL lock error
	ERR_TRX_SPI,			//TRX SPI comms error
	ERR_RANGE,				//value out of range
	ERR_CMD_MALFORM,		//malformed command
	ERR_BUSY,				//busy!
	ERR_BUFF_FULL,			//buffer full
	ERR_NOP,				//nothing to do
	ERR_OTHER
};

//log traffic to file
FILE* logfile = nullptr;

time_t last_refl_ping;

//debug printf
void CCC1200::printMsg(const char* color_code, const char* fmt, ...)
{
	char str[1000];	// plenty of room
	va_list ap;

	va_start(ap, fmt);
	vsprintf(str, fmt, ap);
	va_end(ap);

	if(color_code!=nullptr)
	{
		fputs(color_code, stdout);
		fputs(str, stdout);
		fputs(TERM_DEFAULT, stdout);
	}
	else
	{
		fputs(str, stdout);
	}
	fflush(stdout);
}

void CCC1200::timeStamp()
{
	struct timeval now;
	gettimeofday(&now, nullptr);
	struct tm* tm = ::localtime(&now.tv_sec);
	printMsg(TERM_SKYBLUE, "[%02d/%02d %02d:%02d:%02d.%03lld] ", tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, now.tv_usec / 1000LL);
}

uint32_t CCC1200::getMS(void)
{
	struct timespec spec;

	clock_gettime(CLOCK_REALTIME, &spec);

	time_t s = spec.tv_sec;
	uint32_t ms = roundf(spec.tv_nsec/1.0e6); //convert nanoseconds to milliseconds
	if(ms>999)
	{
		s++;
		ms=0;
	}

	return s*1000 + ms;
}

speed_t CCC1200::getBaud(unsigned baud)
{
	switch(baud)
	{
		default:
			return B0;
		case 9600:
			return B9600;
		case 19200:
			return B19200;
		case 38400:
			return B38400;
		case 57600:
			return B57600;
		case 115200:
			return B115200;
		case 230400:
			return B230400;
		case 460800:
			return B460800;
		case 500000:
			return B500000;
		case 576000:
			return B576000;
		case 921600:
			return B921600;
		case 1000000:
			return B1000000;
		case 1152000:
			return B1152000;
		case 1500000:
			return B1500000;
		case 2000000:
			return B2000000;
		case 2500000:
			return B2500000;
		case 3000000:
			return B3000000;
		case 3500000:
			return B3500000;
		case 4000000:
			return B4000000;
	}
}

bool CCC1200::setAttributes(unsigned speed, int parity)
{
	struct termios tty;
	if(tcgetattr(fd, &tty))
	{
		printMsg(TERM_RED, "tcgetattr() error: %s\n", strerror(errno));
		return true;
 	}

	auto baud = getBaud(speed);
	if (B0 == baud)
	{
		printMsg(TERM_YELLOW, "%u is not a valid baud rate, trying 460800 ", speed);
		baud = B460800;
	}
	cfsetospeed(&tty, baud);
	cfsetispeed(&tty, baud);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;	//8-bit chars
	//disable IGNBRK for mismatched speed tests; otherwise receive break
	//as \000 chars
	tty.c_iflag &= ~IGNBRK;			//disable break processing
	tty.c_lflag = 0;				//no signaling chars, no echo,
									//no canonical processing
	tty.c_oflag = 0;				//no remapping, no delays
	tty.c_cc[VMIN]  = 1;			//read returns when 1 byte available
	tty.c_cc[VTIME] = 5;			//5*0.5=0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);	//ignore modem controls,
										//enable reading
	tty.c_cflag &= ~(PARENB | PARODD);	//shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if(tcsetattr(fd, TCSANOW, &tty))
	{		
		printMsg(TERM_RED, " tcsetattr() error: %s\n", strerror(errno));
		return true; 
	}
	
	return false;
}

static bool load_config(struct config_t *cfg)
{
	//load defaults
	sprintf(cfg->log_path, "/home/tom/rpi-interface/dash.log"); //empty string - disabled
	sprintf(cfg->uart, "/dev/ttyAMA0");
	cfg->uart_rate = 460800;
	sprintf(cfg->node, "KK7SUV  H");
	sprintf(cfg->refl_addr, "10.0.0.181");
	cfg->refl_port=17749;
	sprintf(cfg->reflector, "M17-TAE");
	cfg->module = 'A';
	cfg->rx_freq = 446500000U;
	cfg->tx_freq = 446500000U;
	cfg->freq_corr = 0;
	cfg->tx_pwr = 10.0f;
	cfg->afc = true;
	cfg->nrst = 21;
	cfg->boot0 = 20;
	return false;
}

struct gpiod_line_request *CCC1200::gpioLineRequest(unsigned offset, int value, const char *consumer)
{
	struct gpiod_request_config *req_cfg = nullptr;
	struct gpiod_line_request *request = nullptr;
	struct gpiod_line_settings *settings;
	struct gpiod_line_config *line_cfg = nullptr;

	if (nullptr == gpio_chip) 
		return nullptr;

	settings = gpiod_line_settings_new();
	if (nullptr == settings)
	{
		printMsg(TERM_RED, "Could not create settings for gpio line #%u\n", offset);
	} else {
		if (gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT) or gpiod_line_settings_set_output_value(settings, value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE))
		{
			printMsg(TERM_RED, "Could not adjust settings for gpio line #%u\n", offset);
		} else {
			line_cfg = gpiod_line_config_new();
			if (nullptr == line_cfg)
			{
				printMsg(TERM_RED, "Could not create new config for gpio line #%u\n", offset);
			} else {
				if (gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings))
				{
					printMsg(TERM_RED, "could not add settings to config of gpio line #%u\n", offset);
				} else {
					req_cfg = gpiod_request_config_new();
					if (req_cfg)
					{
						gpiod_request_config_set_consumer(req_cfg, (not consumer) ? "SPOT" : consumer);
						request = gpiod_chip_request_lines(gpio_chip, req_cfg, line_cfg);
						if (nullptr == request)
							printMsg(TERM_RED, "Could not open offset %u on configured gpio device\n", offset);
					}
				}
			}
		}
	}

	if (req_cfg)
		gpiod_request_config_free(req_cfg);
	if (line_cfg)
		gpiod_line_config_free(line_cfg);
	if (settings)
		gpiod_line_settings_free(settings);

	return request;
}

// returns true on error
bool CCC1200::gpioSetValue(unsigned offset, int value)
{
	gpiod_line_request *lr = nullptr;
	if (config.boot0 == offset)
		lr = boot0_line;
	else if (config.nrst == offset)
		lr = nrst_line;
	else {
		printMsg(TERM_RED, "gpioSetValue error: offset %u not confiugred\n", offset);
		return true;
	}

	if (gpiod_line_request_set_value(lr, offset, value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE))
	{
		printMsg(TERM_RED, "Could not set gpio line #%u to %d\n", offset, value);
		return true;
	}
	return false;
}

bool CCC1200::gpioInit(const std::string &consumer)
{
	const char *chip = "/dev/gpiochip0";
	gpio_chip = gpiod_chip_open(chip);
	if (gpio_chip) {
		printMsg(TERM_GREEN, "%s opened\n", chip);
	} else {
		printMsg(TERM_RED, "Could not open %s\n", chip);
		return true;
	}
	boot0_line = gpioLineRequest(config.boot0, 0, consumer.c_str());
	if (nullptr == boot0_line)
		return true;
	nrst_line = gpioLineRequest(config.nrst, 0, consumer.c_str());
	if (nullptr == nrst_line)
		return true;
	return false;
}

// Release GPIO resources
void CCC1200::gpioCleanup()
{
	if (boot0_line)
	{
		gpioSetValue(config.boot0, 0);
		gpiod_line_request_release(boot0_line);
	}
	if (nrst_line)
	{
		gpioSetValue(config.nrst, 0);
		gpiod_line_request_release(nrst_line);
	}
	printMsg(TERM_GREEN, "GPIO lines set to low\n");
	if (gpio_chip)
		gpiod_chip_close(gpio_chip);
	printMsg(TERM_GREEN, "GPIO resources released\n");
}

//M17 stuff
static void refl_send(const uint8_t* msg, uint16_t len)
{
	if(sendto(sockt, msg, len, 0, (const struct sockaddr*)&serv_addr, sizeof(serv_addr))<0)
	{
		printf("Error while sending data to reflector.\nExiting.\n");
		exit(EXIT_FAILURE);
	}
}


bool CCC1200::readDev(void *vbuf, int size)
{
	uint8_t *buf = static_cast<uint8_t *>(vbuf);
	int rd = 0;
	while (rd < size)
	{
		int r = read(fd, buf + rd, size - rd);
		if (r < 0) {
			printMsg(TERM_RED, "read() %s returned error: %s", config.uart, strerror(errno));
			return true;
		} else if (r == 0) {
			printMsg(TERM_RED, "read() %s returned zero bytes\n", config.uart);
			return true;
		}
		rd += r;
	}
	return false;
}

void CCC1200::writeDev(void *buf, int size, const char *where)
{
	ssize_t n = write(fd, buf, size);
	if (n < 0) {
		printMsg(TERM_YELLOW, "In %s, write() error: %s\n", where, strerror(errno));
	} else if (n != size) {
		printMsg(TERM_YELLOW, "write() only wrote %d of %d in %s\n", n, size, where);
	}
	return;
}

//device config funcs
bool CCC1200::pingDev()
{
	uint8_t cid = CMD_PING;
	uint8_t cmd[3] = { cid, 3, 0 };
	uint8_t resp[7] = { 0 };

	uart_lock = true;      // prevent main loop from reading
    tcflush(fd, TCIFLUSH); // clear leftover bytes

    writeDev(cmd, 3, "pingDev");

    if (readDev(resp, sizeof(resp)))
	{
		uart_lock = false;
		return true;
	}

    uart_lock = false;

	const uint8_t good[7] { cid, 7, 0, 0, 0, 0, 0 };
    if (0 == memcmp(resp, good, 7))
	{
		printMsg(TERM_GREEN, "PONG OK\n"); //OK
        return false;
    }

	uint32_t dev_err;
	memcpy((uint8_t*)&dev_err, &resp[3], sizeof(uint32_t));
    printMsg(TERM_RED, "%02x %02x %02x PONG error code: 0x%04X\n", resp[0], resp[1], resp[2], dev_err);
    return true;
}

bool CCC1200::setRxFreq(uint32_t freq)
{
	uint8_t cid = CMD_SET_RX_FREQ;
	uint8_t cmd[3+4] = {cid, 7, 0};
	memcpy(&cmd[3], (uint8_t*)&freq, sizeof(freq));
	uint8_t resp[4] = {0};

	uart_lock = true;            //prevent main loop from reading
    tcflush(fd, TCIFLUSH);    //clear leftover bytes

    writeDev(cmd, 7, "setRxFreq");

    if (readDev(resp, sizeof(resp)))
	{
		uart_lock = false;
		return true;
	}

    uart_lock = false;

	const uint8_t good[4] = { cid, 4, 0, ERR_OK };
    if (0 == memcmp(resp, good, 4))
	{
		printMsg(0, "RX frequency: ");
		printMsg(TERM_GREEN, "%lu Hz\n", freq); //OK
        return false;
    }

    printMsg(TERM_RED, "Error %d setting RX frequency: %u Hz\n", resp[3], freq); //error
    return true;
}

bool CCC1200::setTxFreq(uint32_t freq)
{
	uint8_t cid = CMD_SET_TX_FREQ;
	uint8_t cmd[3+4] = {cid, 7, 0};
	memcpy(&cmd[3], (uint8_t*)&freq, sizeof(freq));
	uint8_t resp[4] = { 0 };

	uart_lock = true;      // prevent main loop from reading
    tcflush(fd, TCIFLUSH);    //clear leftover bytes

    writeDev(cmd, 7, "setTxFreq");

    if (readDev(resp, sizeof(resp)))
	{
		uart_lock = false;
		return true;
	}

    uart_lock = false;

	const uint8_t good[4] { cid, 4, 0, ERR_OK };
    if (0 == memcmp(resp, good, 4))
	{
		printMsg(0, "TX frequency: ");
		printMsg(TERM_GREEN, "%lu Hz\n", freq); //OK
        return false;
    }

    printMsg(TERM_RED, "Error %d setting TX frequency: %u Hz\n", resp[3], freq); //error
    return true;
}

bool CCC1200::setFreqCorr(int16_t corr)
{
	uint8_t cid = CMD_SET_FREQ_CORR;
	uint8_t cmd[5] = {cid, 5, 0, uint8_t(corr&0xffu), uint8_t((corr>>8)&0xffu)};
	uint8_t resp[4] = { 0 };

	uart_lock = true;            //prevent main loop from reading
    tcflush(fd, TCIFLUSH);    //clear leftover bytes

    writeDev(cmd, 5, "setFreqCorr");

	if (readDev(resp, sizeof(resp)))
	{
		uart_lock = false;
		return true;
	}

	uart_lock = false;

	const uint8_t good[4] { cid, 4, 0, ERR_OK };
    if (0 == memcmp(resp, good, 4))
	{
		printMsg(0, "Frequency correction: ");
		printMsg(TERM_GREEN, "%d\n", corr); //OK
        return false;
    }

    printMsg(TERM_RED, "Error %d setting frequency correction: %d\n", resp[3], corr); //error
    return true;
}

bool CCC1200::setAfc(bool en)
{
	uint8_t cid = CMD_SET_AFC;
	uint8_t cmd[3+1] = { cid, 4, 0, uint8_t(en ? 0 : 1) };
	uint8_t resp[4] = { 0 };

	uart_lock = true;            //prevent main loop from reading
    tcflush(fd, TCIFLUSH);    //clear leftover bytes

    writeDev(cmd, 4, "setAfc");

    if (readDev(resp, sizeof(resp)))
	{
		uart_lock = false;
		return true;
	}

    uart_lock = false;

	const uint8_t good[4] { cid, 4, 0, ERR_OK };
    if (0 == memcmp(resp, good, 4))
	{
		printMsg(0, "AFC: ");
		printMsg(TERM_GREEN, "%s\n", en==0?"disabled":"enabled"); //OK
        return false;
    }

    printMsg(TERM_RED, "Error setting AFC\n"); //error
    return true;
}

bool CCC1200::setTxPower(float power) //powr in dBm
{
	uint8_t cid = CMD_SET_TX_POWER;
	uint8_t cmd[4] = { cid, 4, 0, uint8_t(roundf(power*4.0f)) };
	uint8_t resp[4] = { 0 };

	uart_lock = true;            //prevent main loop from reading
    tcflush(fd, TCIFLUSH);    //clear leftover bytes

    writeDev(cmd, 4, "setTxPower");

    if (readDev(resp, sizeof(resp)))
	{
		uart_lock = false;
		return true;
	}

    uart_lock = false;

	uint8_t good[4] { cid, 4, 0, ERR_OK };
    if (0 == memcmp(resp, good, 4))
	{
		printMsg(0, "TX power: ");
		printMsg(TERM_GREEN, "%2.2f dBm\n", power); //OK
        return false;
    }

    printMsg(TERM_RED, "Error %d setting TX power: %2.2f dBm\n", resp[3], power); //error
    return true;
}

bool CCC1200::txrxControl(uint8_t cid, uint8_t onoff, const char *what)
{
	uint8_t cmd[4] { cid, 4, 0, onoff };
	uint8_t resp[4] = { 0 };

	uart_lock = true;          //prevent main loop from reading
	tcflush(fd, TCIFLUSH);    //clear leftover bytes

	writeDev(cmd, 4, what);

	if (readDev(resp, sizeof(resp)))
	{
		uart_lock = false;
		return true;
	}

	uart_lock = false;

	const uint8_t good[3] { cid, 4, 0 };
	if (memcmp(resp, good, 3) or (ERR_OK != resp[3] and ERR_NOP != resp[3]))
	{
		printMsg(TERM_RED, "Doing %s, cmd returned %02x %02x %02x %02x\n", what, resp[0], resp[1], resp[2], resp[3]);
		return true;
	}

	return false;
}

bool CCC1200::startRx(void)
{
	timeStamp();
	printMsg(TERM_YELLOW, "Starting Rx\n");
	return txrxControl(CMD_RX_START, 1, "startRx");
}

bool CCC1200::stopRx(void)
{
	timeStamp();
	printMsg(TERM_YELLOW, "Stopping Rx\n");
	return txrxControl(CMD_RX_START, 0, "stopRx");
}

bool CCC1200::startTx(void)
{
	timeStamp();
	printMsg(TERM_YELLOW, "Starting Tx\n");
	return txrxControl(CMD_TX_START, 1, "startTx");
}

bool CCC1200::stopTx(void)
{
	timeStamp();
	printMsg(TERM_YELLOW, "Stopping Tx\n");
	return txrxControl(CMD_TX_START, 0, "stopTx");
}

void sigint_handler(int)
{
	sprintf((char*)tx_buff, "DISCxxxxxx"); //that "xxxxxx" is just a placeholder
	memcpy(&tx_buff[4], config.enc_node, sizeof(config.enc_node));
	refl_send(tx_buff, 4+6); //DISC

	keep_running = false;
}

//new, polyphase filter implementation
void CCC1200::filterSymbols(int8_t* __restrict out, const int8_t* __restrict in, const float* __restrict flt, uint8_t phase_inv)
{
	#define FLT_LEN 41
    #define TAPS_PER_PHASE 9

	//history
	static float sr[TAPS_PER_PHASE * 2] = {0};
	static uint8_t w = 0;

	//flush filter state
	if (in == nullptr)
	{
		memset(sr, 0, sizeof(sr));
		w = 0;
		return;
	}

	//precompute gain and sign once
    static const float gain = TX_SYMBOL_SCALING_COEFF*sqrtf(5.0f);
	const float sign = phase_inv ? -1.0f : 1.0f;

	for (uint16_t i = 0; i < SYM_PER_FRA; i++)
	{
		//insert new sample per symbol
		const float x = (float)in[i] * sign;

		//store once, duplicated for linear access
		float * __restrict hp = &sr[w];
		hp[0]			   = x;
		hp[TAPS_PER_PHASE] = x;

		//phase pointer
		const float * __restrict tp = flt;

		//generate sps (5) output samples
		for (uint8_t ph = 0; ph < 5; ph++)
		{
			float acc;

			//fully unrolled 9-tap dot product
			acc  = hp[0] * tp[0];
			acc += hp[1] * tp[1];
			acc += hp[2] * tp[2];
			acc += hp[3] * tp[3];
			acc += hp[4] * tp[4];
			acc += hp[5] * tp[5];
			acc += hp[6] * tp[6];
			acc += hp[7] * tp[7];
			acc += hp[8] * tp[8];

			out[i*5 + ph] = (int8_t)(acc * gain);

			//advance to next phase coefficients
			tp += TAPS_PER_PHASE;
		}

		//circular index update without modulo
		if (w == 0)
			w = TAPS_PER_PHASE-1;
		else
			w--;
	}
}

bool CCC1200::Start()
{
	srand(time(nullptr));
	printMsg(TERM_GREEN, "Starting up spot\n");

	//check write access to the log file
	if(strlen(config.log_path) > 0)
	{
		logfile=fopen(config.log_path, "awb");
		if(logfile)
		{
			printMsg(0, "Dashboard log in %s\n", config.log_path);
		}
		else
		{
			printMsg(TERM_RED, "Cannot access %s\nExiting\n", config.log_path);
			return true;
		}
	}
	else
	{
		printMsg(0, "Traffic logging ");
		printMsg(TERM_GREEN, "disabled\n");
	}

	//------------------------------------gpio init------------------------------------
	printMsg(0, "GPIO init: ");
	if (gpioInit("Spot"))
		return true;
	if (gpioSetValue(config.nrst, 0)) //both pins should be at logic low already, but better be safe than sorry
		return true;
	usleep(250000U); //250ms
	if (gpioSetValue(config.nrst, 1))
		return true;
	usleep(1000000U); //1s for device boot-up
	printMsg(TERM_GREEN, " OK\n");

	//-----------------------------------device part-----------------------------------
	printMsg(0, "UART init: %s at %d baud: ", (char*)config.uart, config.uart_rate);
	fd = open((char*)config.uart, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0)
	{
		printMsg(TERM_RED, "open(%s) error: %s\n", config.uart, strerror(errno));
		return true;
	}
	
	if (setAttributes(config.uart_rate, 0))
		return true;
	printMsg(TERM_GREEN, " OK\n");

	//PING-PONG test
	printMsg(0, "Radio board's reply to PING... ");
	if (pingDev())
		return true;

	//config the device
	if (setRxFreq(config.rx_freq) or
		setTxFreq(config.tx_freq) or
		setFreqCorr(config.freq_corr) or
		setTxPower(config.tx_pwr) or
		setAfc(config.afc))
	{
		return true;
	}

	//-----------------------------------internet part-----------------------------------
	printMsg(0, "Connecting to %s:%d (%s) module %c as \"%s\": ", config.refl_addr, config.refl_port, config.reflector, config.module, config.node);

	//server
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(config.refl_addr);
	serv_addr.sin_port = htons(config.refl_port);

	//Create a socket
	sockt = socket(AF_INET, SOCK_DGRAM, 0);
	if(sockt < 0)
	{
		printMsg(TERM_RED, "socket() error: %s\n", strerror(errno));
		return true;
	}
	memset(&daddr, 0, sizeof(daddr));

	//encode M17 callsign from argv
	encode_callsign_bytes(config.enc_node, (uint8_t*)config.node);

	//send "CONN"
	sprintf((char*)tx_buff, "CONNxxxxxx%c", config.module);
	memcpy(&tx_buff[4], config.enc_node, sizeof(config.enc_node));
	refl_send(tx_buff, 4+6+1);
	printMsg(TERM_GREEN, "OK\n");

	//start RX
	while (stopTx())
		usleep(40e3);
	while (startRx())
		usleep(40e3);
	timeStamp();
	printMsg(TERM_GREEN, "Device start - RX\n");
	return false;
}

void CCC1200::Stop()
{
	keep_running = false;
	gpioCleanup();
	if (logfile)
		fclose(logfile);
	if (sockt)
		close(sockt);
	printMsg(TERM_GREEN, "All resources closed\n");
}

#define FLOATBUFSIZE 2042
void CCC1200::Run()
{
	//UART comms
	bool uart_rx_sync = false;
	bool uart_rx_data_valid = false;
	bool got_lsf = false;
	bool first_frame = true;
	int8_t rx_bsb_sample = 0;
	int8_t raw_bsb_rx[960];
	uint8_t rx_samp_buff[1024] { 0 };
	uint8_t lsf_b[30];
	uint8_t lich_parts = 0;
	uint16_t fn;
	uint16_t sample_cnt = 0;
	uint16_t rx_buff_cnt = 0;
	uint16_t last_fn = 0xffffu;
	RingBuffer<int8_t, 41> flt_buff;
	RingBuffer<float, 2000> f_flt_buff;
	const int8_t lsf_sync_ext[16] { +3, -3, +3, -3, +3, -3, +3, -3, +3, +3, +3, +3, -3, -3, +3, -3 };
	const int8_t eot_symbols[8]   { +3, +3, +3, +3, +3, +3, -3, +3 };


	lsf_t lsf;

	enum rx_state_t rx_state = RX_IDLE;
	float f_sample;

	// for packets
	uint8_t last_pkt_fn = 0xffu;
	uint8_t pkt_fn;
	uint32_t tx_timer = 0;
	enum tx_state_t tx_state = TX_IDLE;

	//file for debug data dumping
	//FILE *fp=fopen("test_dump.bin", "wb");

	last_refl_ping = time(nullptr);

	fd_set rfds;
	int maxfd = (fd > sockt) ? fd : sockt;

	float lmin = 144, pmin = 72, smin = 72;

	while(keep_running)
	{
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		FD_SET(sockt, &rfds);

		auto sval = select(maxfd+1, &rfds, nullptr, nullptr, nullptr);
		if (sval < 0)
		{
			if (EINTR != errno)
				printMsg(TERM_RED, "select() error: %s\n", strerror(errno));
			keep_running = false;
			break;
		}

		//are there any new baseband samples to process?
		if (!uart_lock && FD_ISSET(fd, &rfds))
		{
			if(readDev(&rx_bsb_sample, 1))
			{
				keep_running = false;
				break;
			}

			//wait for rx baseband data header
			if (!uart_rx_sync)
			{
				rx_samp_buff[0] = rx_samp_buff[1];
				rx_samp_buff[1] = rx_samp_buff[2];
				rx_samp_buff[2] = rx_bsb_sample;

				if (rx_samp_buff[0]==CMD_RX_DATA && rx_samp_buff[1]==0xC3 && rx_samp_buff[2]==0x03)
				{
					uart_rx_sync = true;
					rx_buff_cnt = 3;
				}
			}
			else
			{
				rx_samp_buff[rx_buff_cnt++] = rx_bsb_sample;
			}

			if (uart_rx_sync && rx_buff_cnt>=963)
			{
				//printMsg(TERM_YELLOW, "Baseband packet received\n");
				memcpy(raw_bsb_rx, &rx_samp_buff[3], sizeof(raw_bsb_rx));
				memset(rx_samp_buff, 0, sizeof(rx_samp_buff));
				uart_rx_data_valid = true;
				uart_rx_sync = false;
				rx_buff_cnt = 0;
			}
		}

		if (uart_rx_data_valid)
		{
			for (uint16_t ii=0; ii<960; ii++)
			{
				//push the next sample into the buffer
				flt_buff.Push(raw_bsb_rx[ii]);

				// filter the buffer to get the new sample
				f_sample = 0.0f;
				for(uint8_t i=0; i<flt_buff.Size(); i++)
					f_sample += rrc_taps_5[i] * float(flt_buff[i]);

				// push the sample on into the float buffer
				f_flt_buff.Push(f_sample*RX_SYMBOL_SCALING_COEFF);

				//L2 norm check against syncword
				float symbols[16];
				for(uint8_t i=0; i<16; i++)
					symbols[i]=f_flt_buff[i*5];

				float sed_lsf = sed(symbols, lsf_sync_ext, 16);
				float sed_pma = sed(symbols, pkt_sync_symbols, 8);
				float sed_sma = sed(symbols, str_sync_symbols, 8);
				for(uint8_t i=0; i<16; i++)
					symbols[i]=f_flt_buff[960+i*5];
				float sed_eot = sed(symbols, eot_symbols,      8);
				float sed_pmb = sed(symbols, pkt_sync_symbols, 8);
				float sed_smb = sed(symbols, str_sync_symbols, 8);
				float sed_pkt = sed_pma + (sed_pmb < sed_eot) ? sed_pmb : sed_eot;
				float sed_str = sed_sma + (sed_smb < sed_eot) ? sed_smb : sed_eot;
				if (sed_lsf < lmin) {
					lmin = sed_lsf;
					timeStamp();
					printMsg(TERM_GREEN, "lmin=%6.2f smin=%6.2f pmin=%6.2f\n", lmin, smin, pmin);
				}
				if (sed_str < smin) {
					smin = sed_str;
					timeStamp();
					printMsg(TERM_GREEN, "lmin=%6.2f smin=%6.2f pmin=%6.2f\n", lmin, smin, pmin);
				}
				if (sed_pkt < pmin) {
					pmin = sed_pkt;
					timeStamp();
					printMsg(TERM_GREEN, "lmin=%6.2f smin=%6.2f pmin=%6.2f\n", lmin, smin, pmin);
				}

				//printMsg(TERM_YELLOW, "%.3u %6.2f %6.2f %6.2f\n", ii, sed_lsf, sed_pkt, sed_str);

				//LSF received at idle state
				if(sed_lsf<=20.25f && rx_state==RX_IDLE)
				{
					//find minimum
					uint8_t sample_offset = 0;
					for(uint8_t i=1; i<=2; i++)
					{
						for(uint8_t j=0; j<16; j++)
							symbols[j] = f_flt_buff[j*5+i];

						float d = sed(symbols, lsf_sync_ext, 16);

						if(d < sed_lsf)
						{
							sed_lsf = d;
							sample_offset = i;
						}
					}

					float pld[SYM_PER_PLD];

					for(uint16_t i=0; i<SYM_PER_PLD; i++)
					{
						pld[i]=f_flt_buff[16*5+i*5+sample_offset]; //add symbol timing correction
					}

					uint32_t e = decode_LSF(&lsf, pld);

					uint8_t call_dst[10], call_src[10], can;
					uint16_t type, crc;
					decode_callsign_bytes(call_dst, lsf.dst);
					decode_callsign_bytes(call_src, lsf.src);
					type=((uint16_t)lsf.type[0]<<8|lsf.type[1]);
					can=(type>>7)&0xFU;
					crc=(((uint16_t)lsf.crc[0]<<8)|lsf.crc[1]);

					timeStamp();
					printMsg(TERM_YELLOW, " RF LSF:");

					if(LSF_CRC(&lsf)==crc) //if CRC valid
					{
						got_lsf = true;
						rx_state=RX_SYNCD;	//change RX state
						sample_cnt=0;		//reset rx timeout timer

						last_fn=0xFFFFU;

						printMsg(TERM_GREEN, " CRC OK ");
						printMsg(TERM_YELLOW, "| DST: %s | SRC: %-9s | TYPE: %04X (CAN=%d) | MER: %-3.1f%%\n",
							call_dst, call_src, type, can, (float)e/0xFFFFU/SYM_PER_PLD/2.0f*100.0f);

						if(type&1) //if stream
						{
							m17stream.fn=0;
							m17stream.sid=rand()%0x10000U;

							uint8_t refl_pld[(32+16+224+16+128+16)/8];					//single frame
							sprintf((char*)&refl_pld[0], "M17 ");						//MAGIC
							*((uint16_t*)&refl_pld[4])=m17stream.sid;					//SID
							memcpy(&refl_pld[6], &lsf, 224/8);							//LSD
							*((uint16_t*)&refl_pld[34])=m17stream.fn;					//FN
							memset(&refl_pld[36], 0, 128/8);							//payload (zeros, because this is LSF)
							uint16_t crc_val=CRC_M17(refl_pld, 52);						//CRC
							*((uint16_t*)&refl_pld[52])=(crc_val>>8)|(crc_val<<8);		//endianness swap
							refl_send(refl_pld, sizeof(refl_pld));						//send a single frame to the reflector

							if (logfile)
							{
								const auto rawtime = time(nullptr);
								const auto timeinfo=localtime(&rawtime);
								fprintf(logfile, "\"%02d:%02d:%02d\" \"%s\" \"%s\" \"RF\" \"%d\" \"%3.1f%%\"\n",
									timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
									call_src, call_dst, can, (float)e/0xFFFFU/SYM_PER_PLD/2.0f*100.0f);
							}
						}
					}
					else
					{
						printMsg(TERM_RED, " CRC ERR\n");
					}
				}

				//stream frame received
				else if(sed_str <= 25.0f)
				{
					rx_state = RX_SYNCD;
					sample_cnt = 0;		//reset rx timeout timer

					//find L2's minimum
					uint8_t sample_offset=0;
					for(uint8_t i=1; i<=2; i++)
					{
						for(uint8_t j=0; j<16; j++)
							symbols[j]=f_flt_buff[j*5+i];
						
						float tmp_a = sed(symbols, str_sync_symbols, 8);
						// check the next frame, look for another data frame or EOT frame
						for(uint8_t j=0; j<16; j++)
							symbols[j] = f_flt_buff[960+j*5+i];
						float tmp_b = sed(symbols, str_sync_symbols, 8);
						float tmp_c = sed(symbols, eot_symbols, 8);
						float d = tmp_a + ((tmp_b < tmp_c) ? tmp_b : tmp_c);

						if(d < sed_str)
						{
							sed_str = d;
							sample_offset = i;
						}
					}

					float pld[SYM_PER_PLD];
					
					for(uint16_t i=0; i<SYM_PER_PLD; i++)
					{
						pld[i]=f_flt_buff[16*5+i*5+sample_offset];
					}

					uint8_t lich[6];
					uint8_t lich_cnt;
					uint8_t frame_data[128/8];
					uint32_t e = decode_str_frame(frame_data, lich, &fn, &lich_cnt, pld);
					
					//set the last FN number to FN-1 if this is a late-join and the frame data is valid
					if(first_frame and (fn%6)==lich_cnt)
					{
						last_fn=fn-1;
					}
					
					if(((last_fn+1)&0xFFFFU)==fn) //new frame. TODO: maybe a timeout would be better
					{
						if(lich_parts!=0x3FU) //6 chunks = 0b111111
						{
							//reconstruct LSF chunk by chunk
							memcpy(&lsf_b[lich_cnt*5], lich, 40/8); //40 bits
							lich_parts|=(1<<lich_cnt);
							if(lich_parts==0x3FU && got_lsf==false) //collected all of them?
							{
								if(!CRC_M17(lsf_b, 30)) //CRC check
								{
									got_lsf = true;
									m17stream.sid=rand()%0x10000U;

									uint8_t call_dst[12]={0}, call_src[12]={0};
									uint16_t type=((uint16_t)lsf_b[12]<<8)|lsf_b[13];
									uint8_t can=(type>>7)&0xF;

									decode_callsign_bytes(call_dst, &lsf_b[0]);
									decode_callsign_bytes(call_src, &lsf_b[6]);

									timeStamp();
									printMsg(TERM_YELLOW, "LSF REC: DST: %-9s | SRC: %-9s | TYPE: %04X (CAN=%d)\n",
										call_dst, call_src, type, can);

									if(logfile)
									{
										const auto rawtime = time(nullptr);
										const auto timeinfo=localtime(&rawtime);
										fprintf(logfile, "\"%02d:%02d:%02d\" \"%s\" \"%s\" \"RF\" \"%d\" \"--\"\n",
											timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
											call_src, call_dst, can);
									}
								}
								else
								{
									printMsg(TERM_YELLOW, "LSF CRC ERR\n");
									lich_parts=0; //reset flags
								}
							}
						}

						timeStamp();
						printMsg(TERM_YELLOW, " RF FRM: ");
						printMsg(TERM_YELLOW, " FN:%04X | LICH_CNT:%d", fn, lich_cnt);
						/*printMsg(TERM_YELLOW, " | PLD: ");
						for(uint8_t i=0; i<128/8; i++)
							printMsg(TERM_YELLOW, "%02X", frame_data[2+i]);*/
						printMsg(TERM_YELLOW, " | MER: %-3.1f%%\n",
							(float)e/0xFFFFU/SYM_PER_PLD/2.0f*100.0f);

						if(got_lsf)
						{
							m17stream.fn=(fn>>8)|((fn&0xFF)<<8);
							uint8_t refl_pld[(32+16+224+16+128+16)/8];					//single frame
							sprintf((char*)&refl_pld[0], "M17 ");						//MAGIC
							*((uint16_t*)&refl_pld[4])=m17stream.sid;					//SID
							memcpy(&refl_pld[6], &lsf_b[0], 224/8);						//LSD
							*((uint16_t*)&refl_pld[34])=m17stream.fn;					//FN
							memcpy(&refl_pld[36], frame_data, 128/8);					//payload
							uint16_t crc_val=CRC_M17(refl_pld, 52);						//CRC
							*((uint16_t*)&refl_pld[52])=(crc_val>>8)|(crc_val<<8);		//endianness swap
							refl_send(refl_pld, sizeof(refl_pld));						//send a single frame to the reflector
						}

						last_fn=fn;
					}

					first_frame = false;
				}

				//TODO: handle packet mode reception over RF
				else if(sed_pkt <= 25.0f && rx_state == RX_SYNCD)
				{
					//find L2's minimum
					uint8_t sample_offset = 0;
					for(uint8_t i=1; i<=2; i++)
					{
						for(uint8_t j=0; j<8; j++)
							symbols[j]=f_flt_buff[j*5+i];
							
						float tmp_a = sed(symbols, pkt_sync_symbols, 8);
						for(uint8_t j=0; j<16; j++)
							symbols[j] = f_flt_buff[960+j*5+i];
						float tmp_b = sed(symbols, pkt_sync_symbols, 8);
						float tmp_c = sed(symbols, eot_symbols, 8);
						float d = tmp_a + ((tmp_b < tmp_c) ? tmp_b : tmp_c);

						if(d < sed_pkt)
						{
							sed_pkt = d;
							sample_offset = i;
						}
					}

					float pld[SYM_PER_PLD];
					uint8_t pkt_frame_data[25] = {0};
					uint8_t eof = 0;
					
					for(uint16_t i=0; i<SYM_PER_PLD; i++)
					{
						pld[i]=f_flt_buff[8*5+i*5+sample_offset];
					}

					//debug data dump
					//fwrite((uint8_t*)&f_flt_buff[sample_offset], SYM_PER_FRA*5*sizeof(float), 1, fp);

					/*uint32_t e = */decode_pkt_frame(pkt_frame_data, &eof, &pkt_fn, pld);

					//TODO: this will only properly decode single-framed packets
					if(last_pkt_fn==0xffu && eof==1 && CRC_M17(pkt_frame_data, strlen((char*)pkt_frame_data)+3)==0)
					{
						sample_cnt=0;		//reset rx timeout timer
						last_pkt_fn = pkt_fn;

						timeStamp();
						printMsg(TERM_YELLOW, " RF PKT: ");
						/*for(uint8_t i=0; i<25; i++)
							printMsg(0, "%02X ", pkt_frame_data[i]);
						printMsg(0, "\n");*/
						printMsg(0, "%s\n", (char*)&pkt_frame_data[1]);
						uint8_t refl_pld[4+sizeof(lsf)+strlen((char*)pkt_frame_data)+3];					//single frame
						sprintf((char*)&refl_pld[0], "M17P");						//MAGIC
						memcpy(&refl_pld[4], &lsf, sizeof(lsf));					//LSF
						memcpy(&refl_pld[34], &pkt_frame_data, strlen((char*)pkt_frame_data)+3); //PKT data + CRC
						/*debug logging
						time(&rawtime);
						timeinfo=localtime(&rawtime);

						printMsg(TERM_SKYBLUE, "[%02d:%02d:%02d]",
							timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
						printMsg(TERM_YELLOW, " refl_pld: ");
						for(uint8_t i=0; i<sizeof(refl_pld); i++)
							printMsg(0, "%02X ", refl_pld[i]);
						printMsg(0, "\n");
						*/
						refl_send(refl_pld, 4+sizeof(lsf)+strlen((char*)pkt_frame_data)+3);						//send to the reflector
					}
				}
				
				//RX sync timeout
				if(rx_state==RX_SYNCD)
				{
					sample_cnt++;
					if(sample_cnt==960*2)
					{
						rx_state=RX_IDLE;
						sample_cnt=0;
						first_frame = true;
						last_fn=0xFFFFU; //TODO: there's a small chance that this will cause problems (it's a valid frame number)
						last_pkt_fn = 0xffu;
						lich_parts=0;
						got_lsf = false;
					}
				}
			}
			//all data has been used
			uart_rx_data_valid = false;
		}

		//receive a packet - blocking
		if (FD_ISSET(sockt, &rfds))
		{
			auto bsize = sizeof(rx_buff);
			rx_len = recvfrom(sockt, rx_buff, bsize, 0, (struct sockaddr*)&saddr, (socklen_t*)&saddr_size);

			//debug
			//printMsg(0, "Size:%d\nPayload:%s\n", rx_len, rx_buff);

			//PING-PONG
			if(strstr((char*)rx_buff, "PING")==(char*)rx_buff)
			{
				last_refl_ping = time(nullptr);
				sprintf((char*)tx_buff, "PONGxxxxxx"); //that "xxxxxx" is just a placeholder
				memcpy(&tx_buff[4], config.enc_node, sizeof(config.enc_node));
				refl_send(tx_buff, 4+6); //PONG
				//printMsg(TERM_YELLOW, "PING\n");
			}

			//M17 stream frame data - "Steaming Mode IP Packet, Single Packet Method"
			else if(strstr((char*)rx_buff, "M17 ")==(char*)rx_buff)
			{
				tx_timer = getMS();

				m17stream.sid=((uint16_t)rx_buff[4]<<8)|rx_buff[5];
				m17stream.fn=((uint16_t)rx_buff[34]<<8)|rx_buff[35];
				static uint8_t dst_call[10]={0};
				static uint8_t src_call[10]={0};
				memcpy(m17stream.pld, &rx_buff[(32+16+224+16)/8U], 16);

				int8_t frame_symbols[SYM_PER_FRA];						//raw frame symbols
				int8_t bsb_samples[963] = {CMD_TX_DATA, -61, 3};	//baseband samples wrapped in a frame

				if(tx_state==TX_IDLE) //first received frame
				{
					tx_state=TX_ACTIVE;

					//TODO: this needs to happen every time a new transmission appears
					//stopRx();
					//printMsg(0, "RX stop\n");
					usleep(10e3);

					//extract data
					memcpy(m17stream.lsf.dst, "\xFF\xFF\xFF\xFF\xFF\xFF", 6);
					memcpy(m17stream.lsf.src, &rx_buff[6+6], 6);
					decode_callsign_bytes(dst_call, m17stream.lsf.dst);
					decode_callsign_bytes(src_call, m17stream.lsf.src);

					//set TYPE field
					memcpy(m17stream.lsf.type, &rx_buff[18], 2);
					m17stream.lsf.type[1]|=0x2U<<5; //no encryption, so the subtype field defines the META field contents: extended callsign data

					//generate META field
					//remove trailing spaces and suffixes
					uint8_t trimmed_src[10], enc_trimmed_src[6];
					for(uint8_t i=0; i<10; i++)
					{
						if(src_call[i]!=' ')
							trimmed_src[i]=src_call[i];
						else
						{
							trimmed_src[i]=0;
							break;
						}
					}
					encode_callsign_bytes(enc_trimmed_src, trimmed_src);

					uint8_t ext_ref[12], enc_ext_ref[6];
					sprintf((char*)ext_ref, "%s %c", config.reflector, config.module);
					encode_callsign_bytes(enc_ext_ref, ext_ref);

					memcpy(&m17stream.lsf.meta[0], m17stream.lsf.src, 6); //originator
					memcpy(&m17stream.lsf.meta[6], enc_ext_ref, 6); //reflector
					memset(&m17stream.lsf.meta[12], 0, 2);
					memcpy(m17stream.lsf.src, enc_trimmed_src, 6);

					//append CRC
					uint16_t ccrc=LSF_CRC(&m17stream.lsf);
					m17stream.lsf.crc[0]=ccrc>>8;
					m17stream.lsf.crc[1]=ccrc&0xFF;

					//log to file
					if(logfile)
					{
						const auto rawtime = time(nullptr);
						const auto timeinfo=localtime(&rawtime);
						fprintf(logfile, "\"%02d:%02d:%02d\" \"%s\" \"%s\" \"Internet\" \"--\" \"--\"\n",
							timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
							src_call, dst_call);
					}

					timeStamp();
					printMsg(TERM_GREEN, " Stream TX start\n");

					//stop RX, set PA_EN=1 and initialize TX
					while (stopRx())
						usleep(40e3);
					usleep(2e3);
					while (startTx())
						usleep(40e3);
					usleep(10e3);

					//flush the RRC baseband filter
					filterSymbols(nullptr, nullptr, nullptr, 0);
				
					//generate frame symbols, filter them and send out to the device
					//we need to prepare 3 frames to begin the transmission - preamble, LSF and stream frame 0
					//let's start with the preamble
					uint32_t frame_buff_cnt=0;
					gen_preamble_i8(frame_symbols, &frame_buff_cnt, PREAM_LSF);

					//filter and send out to the device
					filterSymbols(bsb_samples+3, frame_symbols, rrc_taps_5_poly, 0);
					writeDev(bsb_samples, sizeof(bsb_samples), "SM Pream");

					//now the LSF
					gen_frame_i8(frame_symbols, nullptr, FRAME_LSF, &(m17stream.lsf), 0, 0);

					//filter and send out to the device
					filterSymbols(bsb_samples+3, frame_symbols, rrc_taps_5_poly, 0);
					writeDev(bsb_samples, sizeof(bsb_samples), "SM LSF");

					//finally, the first frame
					gen_frame_i8(frame_symbols, m17stream.pld, FRAME_STR, &(m17stream.lsf), (m17stream.fn&0x7FFFU)%6, m17stream.fn);

					//filter and send out to the device
					filterSymbols(bsb_samples+3, frame_symbols, rrc_taps_5_poly, 0);
					writeDev(bsb_samples, sizeof(bsb_samples), "SM first Frame");
				}
				else
				{
					//only one frame is needed
					gen_frame_i8(frame_symbols, m17stream.pld, FRAME_STR, &(m17stream.lsf), (m17stream.fn&0x7FFFU)%6, m17stream.fn);

					//filter and send out to the device
					filterSymbols(bsb_samples+3, frame_symbols, rrc_taps_5_poly, 0);
					writeDev(bsb_samples, sizeof(bsb_samples), "SM Frame");
				}

				if(m17stream.fn&0x8000U) //last stream frame
				{
					//send the final EOT marker
					uint32_t frame_buff_cnt=0;
					gen_eot_i8(frame_symbols, &frame_buff_cnt);

					//filter and send out to the device
					filterSymbols(bsb_samples+3, frame_symbols, rrc_taps_5_poly, 0);
					writeDev(bsb_samples, sizeof(bsb_samples), "SM EOT");

					timeStamp();
					printMsg(TERM_GREEN, " Stream TX end\n");
					usleep(8*40e3); //wait 320ms (8 M17 frames) - let the transmitter consume all the buffered samples

					//restart RX
					while (stopTx())
						usleep(40e3);
					while (startRx())
						usleep(40e3);
					timeStamp();
					printMsg(TERM_GREEN, " RX start\n");

					tx_state=TX_IDLE;
				}
			}

			//M17 packet data - "Packet Mode IP Packet"
			else if(strstr((char*)rx_buff, "M17P")==(char*)rx_buff)
			{
				timeStamp();
				printMsg(TERM_GREEN, " M17 Inet packet received\n");

				uint8_t call_dst[10], call_src[10], can, type;
				decode_callsign_bytes(call_dst, &rx_buff[4+0]);
				decode_callsign_bytes(call_src, &rx_buff[4+6]);
				can=(*((uint16_t*)&rx_buff[4+6+6])>>7)&0xF;
				type=rx_buff[4+240/8];
				
				printMsg(TERM_DEFAULT, " ├ "); printMsg(TERM_YELLOW, "DST: "); printMsg(TERM_DEFAULT, "%s\n", call_dst);
				printMsg(TERM_DEFAULT, " ├ "); printMsg(TERM_YELLOW, "SRC: "); printMsg(TERM_DEFAULT, "%s\n", call_src);
				printMsg(TERM_DEFAULT, " ├ "); printMsg(TERM_YELLOW, "CAN: "); printMsg(TERM_DEFAULT, "%d\n", can);
				if(type!=5) //assuming 1-byte type specifier
				{
					printMsg(TERM_DEFAULT, " └ "); printMsg(TERM_YELLOW, "TYPE: "); printMsg(TERM_DEFAULT, "%d\n", type);
				}
				else
				{
					printMsg(TERM_DEFAULT, " ├ "); printMsg(TERM_YELLOW, "TYPE: "); printMsg(TERM_DEFAULT, "SMS\n");
					printMsg(TERM_DEFAULT, " └ "); printMsg(TERM_YELLOW, "MSG: ");  printMsg(TERM_DEFAULT, "%s\n", &rx_buff[4+240/8+1]);
				}

				//TODO: handle TX here
				int8_t frame_symbols[SYM_PER_FRA];						//raw frame symbols
				int8_t bsb_samples[SYM_PER_FRA*5];						//filtered baseband samples = symbols*sps
				uint8_t bsb_chunk[963] = {CMD_TX_DATA, 0xC3, 0x03};		//baseband samples wrapped in a frame

				if(logfile)
				{
					const auto rawtime = time(nullptr);
					const auto timeinfo=localtime(&rawtime);
					fprintf(logfile, "\"%02d:%02d:%02d\" \"%s\" \"%s\" \"Internet\" \"--\" \"--\"\n", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, call_src, call_dst);
				}
				
				timeStamp();
				printMsg(TERM_GREEN, " Packet TX start\n");

				//stop RX, set PA_EN=1 and initialize TX
				while (stopRx())
					usleep(40e3);
				usleep(2e3);

				while (startTx())
					usleep(40e3);
				usleep(10e3);
				
				//flush the RRC baseband filter
				filterSymbols(nullptr, nullptr, nullptr, 0);
				
				//generate frame symbols, filter them and send out to the device
				//we need to prepare 3 frames to begin the transmission - preamble, LSF and stream frame 0
				//let's start with the preamble
				uint32_t frame_buff_cnt=0;
				gen_preamble_i8(frame_symbols, &frame_buff_cnt, PREAM_LSF);
				
				//filter and send out to the device
				filterSymbols(bsb_samples, frame_symbols, rrc_taps_5_poly, 0);
				memcpy(&bsb_chunk[3], bsb_samples, sizeof(bsb_samples));
				writeDev(bsb_samples, sizeof(bsb_samples), "PM LSF Pream");
				
				//now the LSF
				gen_frame_i8(frame_symbols, nullptr, FRAME_LSF, (lsf_t*)&rx_buff[4], 0, 0);
				
				//filter and send out to the device
				filterSymbols(bsb_samples, frame_symbols, rrc_taps_5_poly, 0);
				memcpy(&bsb_chunk[3], bsb_samples, sizeof(bsb_samples));
				writeDev(bsb_samples, sizeof(bsb_samples), "PM LSF");
				
				//packet frames
				uint16_t pld_len=rx_len-(4+240/8); //"M17P" plus 240-bit LSD
				uint8_t frame=0;
				uint8_t pld[26];
				
				while(pld_len>25)
				{
					memcpy(pld, &rx_buff[4+240/8+frame*25], 25);
					pld[25]=frame<<2;
					gen_frame_i8(frame_symbols, pld, FRAME_PKT, nullptr, 0, 0);
					filterSymbols(bsb_samples, frame_symbols, rrc_taps_5_poly, 0);
					memcpy(&bsb_chunk[3], bsb_samples, sizeof(bsb_samples));
					writeDev(bsb_samples, sizeof(bsb_samples), "PM Frame");
					pld_len-=25;
					frame++;
					usleep(40*1000U);
				}
				memset(pld, 0, 26);
				memcpy(pld, &rx_buff[4+240/8+frame*25], pld_len);
				pld[25]=(1<<7)|(pld_len<<2); //EoT flag set, amount of remaining data in the 'frame number' field
				gen_frame_i8(frame_symbols, pld, FRAME_PKT, nullptr, 0, 0);
				filterSymbols(bsb_samples, frame_symbols, rrc_taps_5_poly, 0);
				memcpy(&bsb_chunk[3], bsb_samples, sizeof(bsb_samples));
				writeDev(bsb_samples, sizeof(bsb_samples), "PM Frame");
				usleep(40*1000U);

				//now the final EOT marker
				frame_buff_cnt=0;
				gen_eot_i8(frame_symbols, &frame_buff_cnt);

				//filter and send out to the device
				filterSymbols(bsb_samples, frame_symbols, rrc_taps_5_poly, 0);
				memcpy(&bsb_chunk[3], bsb_samples, sizeof(bsb_samples));
				writeDev(bsb_samples, sizeof(bsb_samples), "PM EOT");

				timeStamp();
				printMsg(TERM_GREEN, " PKT TX end\n");
				usleep(3*40e3); //wait 120ms (3 M17 frames)

				//restart RX
				while (stopTx())
					usleep(40e3);
				while (startRx())
					usleep(40e3);
				timeStamp();
				printMsg(TERM_GREEN, " RX start\n");

				tx_state = TX_IDLE;
			}

			//clear the rx_buff
			memset((uint8_t*)rx_buff, 0, rx_len);
		}

		//tx timeout
		if(tx_state==TX_ACTIVE && (getMS()-tx_timer)>240) //240ms timeout
		{
			timeStamp();
			printMsg(TERM_GREEN, " TX timeout\n");
			//usleep(10*40e3); //wait 400ms (10 M17 frames)

			//restart RX
			while (stopTx())
				usleep(40e3);
			while (startRx())
				usleep(40e3);
			timeStamp();
			printMsg(TERM_GREEN, " RX start\n");

			tx_state=TX_IDLE;
		}

		//connection with the reflector borken
		if(time(nullptr)-last_refl_ping>30)
		{
			keep_running = false;
			//for now, just cry about it and quit
			printMsg(TERM_RED, "Lost connection with the reflector\nExiting");
		}
	}
	printMsg(TERM_GREEN, "run loop terminated\n");
}

/**
 * @brief Calculate squared Euclidean distance between two n-dimensional vectors.
 * It is the sum of squared differences.
 *
 * @param v1 Vector 1 - floats.
 * @param v2 Vector 2 - signed ints.
 * @param n Vectors' size.
 * @return float Squared distance between two points.
 */
float CCC1200::sed(const float *v1, const int8_t *v2, const unsigned n) const
{
	float r = 0.0f;
	for (unsigned i=0; i<n; i++)
	{
		auto x = v1[1] - float(v2[i]);
		r += x * x;
	}
	return r;
}


int main()
{
	signal(SIGINT, sigint_handler);

	if (load_config(&config))
		return EXIT_FAILURE;

	CCC1200 modem;
	if (modem.Start())
	{
		modem.Stop();
		return EXIT_FAILURE;
	}
	modem.Run();

	modem.Stop();

	return EXIT_SUCCESS;
}
