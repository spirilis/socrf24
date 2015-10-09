/* Simple utility to shut down nRF24L01+ transceivers into DEEPSLEEP mode */

#include <socrf24.h>
#include <getopt.h>
#include <libsoc_debug.h>

// SPI#1, P9_24 aka GPIO0_15, P9_23 aka GPIO1_17, P8_26 aka GPIO1_29
SoCrf24<1, 15, 49, 61> radio;

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

	printf("nrfoff: Attempting to contact transceiver and shut it down.\n");

	if (!radio.begin()) {
		printf("radio.begin() failed; exiting\n");
		exit(1);
	}

	printf("Transceiver state: %s\n", nrf_state(radio.radioState()));

	radio.end();
}

