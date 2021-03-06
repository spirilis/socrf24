/* Simple utility to dump nRF24L01+ packets received by the daemon */

#include <socrf24.h>
#include <getopt.h>
#include <libsoc_debug.h>
#include <fcntl.h>
#include <errno.h>

// SPI#1, P9_24 aka GPIO0_15, P9_23 aka GPIO1_17, P8_26 aka GPIO1_29
SoCrf24<1, 15, 49, 61> radio;
uint8_t basestation_addr[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01};
unsigned int channel = 0, addrlen = 5;
enum rf24_speed rfspeed = SPEED_1MBPS;

const char * nrf_state(enum rf24_state s)
{
	switch (s) {
		case NOT_PRESENT:
			return "TRANSCEIVER NOT PRESENT";
			break;
		case POWERDOWN:
			return "LOW-POWER SLEEP";
			break;
		case STANDBY:
			return "STANDBY";
			break;
		case PTX:
			return "PTX";
			break;
		case PRX:
			return "PRX";
			break;
	}
}

const char * nrf_speed(enum rf24_speed s)
{
	switch (s) {
		case SPEED_250KBPS:
			return "250Kbps";
			break;
		case SPEED_1MBPS:
			return "1Mbps";
			break;
		case SPEED_2MBPS:
			return "2Mbps";
			break;
	}
}

char * nrfaddr_explode(const uint8_t *addr, size_t len = 5)
{
	static char buf[128];
	if (len < 3 || len > 5)
		return NULL;
	switch (len) {
		case 3:
			sprintf(buf, "%02X %02X %02X", addr[0], addr[1], addr[2]);
			break;
		case 4:
			sprintf(buf, "%02X %02X %02X %02X", addr[0], addr[1], addr[2], addr[3]);
			break;
		case 5:
			sprintf(buf, "%02X %02X %02X %02X %02X", addr[0], addr[1], addr[2], addr[3], addr[4]);
			break;
	}
	return buf;
}

// This function returns a shared buffer containing the address in question, or NULL if there is a problem
// with the input (length must == addrlen*2 and all characters must include 0-9, A-F or a-f.
const uint8_t * nrfaddr_interpret(const char *a, size_t alen = 5)
{
	size_t i, l = strlen(a);
	char c, d;
	uint8_t v;
	static uint8_t addr[5];

	// Scrub input
	if (l != alen*2)
		return NULL;
	for (i=0; i < alen*2; i++) {
		c = a[i];
		if (c >= '0' && c <= '9')
			continue;
		if (c >= 'A' && c <= 'F')
			continue;
		if (c >= 'a' && c <= 'f')
			continue;
		return NULL;
	}

	for (i=0; i < alen; i++) {
		c = a[i*2];
		d = a[i*2+1];
		if (c >= '0' && c <= '9')
			v = (c - '0') * 0x10;
		if (c >= 'A' && c <= 'F')
			v = (c - 'A' + 10) * 0x10;
		if (c >= 'a' && c <= 'f')
			v = (c - 'a' + 10) * 0x10;
		if (d >= '0' && d <= '9')
			v += (d - '0');
		if (d >= 'A' && d <= 'F')
			v += (d - 'A' + 10);
		if (d >= 'a' && d <= 'f')
			v += (d - 'a' + 10);
		addr[i] = v;
	}
	return addr;
}

void dumpregs()
{
	uint8_t addrs[19], buf[19];
	int i;

	radio.dumpregs(addrs, buf);
	printf("nRF24L01+ Register Dump:\n");
	for (i = 0; i < 19; i++) {
		printf("  R#%02x = %02X\n", addrs[i], buf[i]);
	}
}

int main(int argc, char *argv[])
{
	//libsoc_set_debug(1);

	static struct option long_opts[] = {
		{"addr", required_argument, NULL, 'a'},
		{"addrlen", required_argument, NULL, 'l'},
		{"channel", required_argument, NULL, 'c'},
		{"speed", required_argument, NULL, 's'},
		{0, 0, 0, 0}
	};
	int optidx = 0, c = 0;
	const uint8_t *a;
	while ( (c = getopt_long(argc, argv, "l:a:c:s:", long_opts, &optidx)) != -1 ) {
		switch (c) {
			case 'l':
				c = atoi(optarg);
				if (c >= 3 && c <= 5) {
					addrlen = c;
				} else {
					printf("Invalid address length \"%s\" specified (valid address lengths include 3, 4 and 5)\n", optarg);
				}
				break;
			case 'a':
				a = nrfaddr_interpret(optarg, addrlen);
				if (a != NULL) {
					memcpy(basestation_addr, a, addrlen);
				} else {
					printf("Invalid %d-byte nRF24 address specified: %s\n", optarg);
				}
				break;
			case 'c':
				c = atoi(optarg);
				if (c >= 0 && c <= 125) {
					channel = c;
				} else {
					printf("Invalid channel specified: %s\n", optarg);
				}
				break;
			case 's':
				c = atoi(optarg);
				switch (c) {
					case 250:
						rfspeed = SPEED_250KBPS;
						break;
					case 1000:
						rfspeed = SPEED_1MBPS;
						break;
					case 2000:
						rfspeed = SPEED_2MBPS;
						break;
					default:
						printf("Invalid speed specified: %s\n", optarg);
				}
		}
	}

	printf("nrfdump: listening on %s at channel %u, speed = %s\n", nrfaddr_explode(basestation_addr, addrlen), channel, nrf_speed(rfspeed));

	if (!radio.begin(rfspeed, channel)) {
		printf("radio.begin() failed; exiting\n");
		exit(1);
	}

	radio.setAddressLength(addrlen);
	radio.setCRC(CRC16);
	radio.setRXaddress(0, basestation_addr);
	radio.open(0, 0, true);
	radio.enableRX();

	printf("Transceiver state: %s\n", nrf_state(radio.radioState()));

	uint8_t buf[32], pipeid, len;
	int i;

	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);  // Make read() on STDIN non-blocking so our loop doesn't hang
	dumpregs();
	while (1) {
		if (radio.available(false)) {
			printf("RX event-\n");
			len = radio.recv(buf, 32, &pipeid);
			if (len) {
				printf("Pipe = %d, length = %d\n", pipeid, len);
				for (i=0; i < len; i++) {
					printf("%02X ", buf[i]);
					if (i % 8 == 7)
						printf("\n");
				}
				if (i % 8 != 0)
					printf("\n");
				printf("\n");
			}
		}
		i = read(STDIN_FILENO, buf, 32);
		if (i != -1) {
			if (buf[0] == '\n' || (buf[0] == '\r' && buf[1] == '\n')) {
				dumpregs();
			}
		}
		//printf("IRQ = %d\n", radio.getIRQState());
	}

}

