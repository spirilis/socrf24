/* socrf24 - C++ nRF24L01+ driver using libsoc for Linux SoC boards */

#ifndef SOCRF24_H
#define SOCRF24_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <libsoc_gpio.h>
#include <libsoc_spi.h>
#include <libsoc_debug.h>

#include "nRF24L01.h"


/* Enums for data types and potential values */

enum rf24_crc {
	CRC_NONE = 0,
	CRC8,
	CRC16
};

enum rf24_speed {
	250KBPS = 0,
	1MBPS = 1,
	2MBPS = 2
};


/* Main class */

template <uint8_t spi_inst, unsigned int gpio_spics, unsigned int gpio_ce, unsigned int gpio_irq>
class Socrf24 {
	private:
		uint8_t rf_status, rf_address_width;

		/* SPI and GPIO I/O and initialization */
		gpio *spics, *ce, *irq;
		spi *SPI;
		volatile bool irq_is_flagged;

		__noinline
		void _irq_callback(void *arg) {
			irq_is_flagged = true;
		};

		__noinline
		bool _init_gpio() {
			spics = libsoc_gpio_request(gpio_spics, LS_GREEDY);
			ce = libsoc_gpio_request(gpio_ce, LS_GREEDY);
			irq = libsoc_gpio_request(gpio_irq, LS_GREEDY);
			if (!spi_cs) {
				libsoc_debug("_init_gpio", "Error requesting gpio for SPI CS (%d)", gpio_spics);
				return false;
			}
			if (!ce) {
				libsoc_debug("_init_gpio", "Error requesting gpio for CE (%d)", gpio_ce);
				return false;
			}
			if (!irq) {
				libsoc_debug("_init_gpio", "Error requesting gpio for IRQ (%d)", gpio_irq);
				return false;
			}

			libsoc_gpio_set_direction(spics, OUTPUT);
			libsoc_gpio_set_level(spics, HIGH);  // CS = HIGH at rest

			libsoc_gpio_set_direction(ce, OUTPUT);
			libsoc_gpio_set_level(ce, LOW);

			libsoc_gpio_set_direction(irq, INPUT);
			libsoc_gpio_set_level(irq, HIGH);  // Not sure if this actually "does" anything
			libsoc_gpio_set_edge(irq, FALLING);
			libsoc_gpio_callback_interrupt(irq, _irq_callback, NULL);

			return true;
		};

		__noinline
		bool _init_spi() {
			SPI = libsoc_spi_init(spi_inst, 0);
			if (!SPI) {
				libsoc_debug("_init_spi", "Error requesting SPI handle for SPI bus#%d", spi_inst);
				return false;
			}

			libsoc_spi_set_mode(SPI, MODE_0);
			libsoc_spi_set_bits_per_word(SPI, BITS_8);
			libsoc_spi_set_speed(SPI, 6000000UL);
			return true;
		};

		__inline
		void ce_en(void) {
			libsoc_gpio_set_level(ce, HIGH);
		};

		__inline
		void ce_dis(void) {
			libsoc_gpio_set_level(ce, LOW);
		};

		__inline
		void csn_en(void) {
			libsoc_gpio_set_level(spics, LOW);
		};

		__inline
		void csn_dis(void) {
			libsoc_gpio_set_level(spics, HIGH);
		};

		__noinline
		void w_reg(uint8_t reg, uint8_t val) {
			csn_en();
			reg |= RF24_W_REGISTER;
			libsoc_spi_rw(SPI, &reg, &rf_status, 1);
			libsoc_spi_write(SPI, &val, 1);
			csn_dis();
		};

		__noinline
		uint8_t r_reg(uint8_t reg) {
			uint8_t val;
			csn_en();
			reg |= RF24_R_REGISTER;
			libsoc_spi_rw(SPI, &reg, &rf_status, 1);
			libsoc_spi_read(SPI, &val, 1);
			csn_dis();
			return val;
		};

		__noinline
		void w_reg_lsbstring(uint8_t reg, const void *data, size_t len) {
			csn_en();
			reg |= RF24_W_REGISTER;
			libsoc_spi_rw(SPI, &reg, &rf_status, 1);
			for (int i = len; i >= 0; i--) {
				libsoc_spi_write(SPI, (const uint8_t *)data[i], 1);
			}
			csn_dis();
		};

		__noinline
		size_t r_reg_lsbstring(uint8_t reg, void *data, size_t len) {
			csn_en();
			reg |= RF24_R_REGISTER;
			libsoc_spi_rw(SPI, &reg, &rf_status, 1);
			for (int i = len; i >= 0; i--) {
				libsoc_spi_read(SPI, (const uint8_t *)data+i, 1);
			}
			csn_dis();
			return len;
		};

		__noinline
		void w_msbstring(uint8_t cmd, const void *data, size_t len) {
			csn_en();
			libsoc_spi_rw(SPI, &cmd, &rf_status, 1);
			libsoc_spi_write(SPI, (const uint8_t *)data, len);
			csn_dis();
		};

		__noinline
		size_t r_msbstring(uint8_t cmd, void *data, size_t len) {
			csn_en();
			libsoc_spi_rw(SPI, &cmd, &rf_status, 1);
			libsoc_spi_read(SPI, (uint8_t *)data, len);
			csn_dis();
			return len;
		};


	public:
		__noinline
		Socrf24() {
			spics = NULL;
			ce = NULL;
			irq = NULL;
			irq_is_flagged = false;
			rf_status = 0;
			rf_address_width = 5;
		};

		/* Address management */
		__inline
		void setTXaddress(const void *addr) {
			return w_reg_lsbstring(RF24_TX_ADDR, addr, rf_address_width);
		};

		__inline
		void setRXaddress(uint8_t pipe, const void *addr) {
			if (pipe < 6)
				return w_reg_lsbstring(RF24_RX_ADDR_P0 + pipe, addr, rf_address_width);
		};

		__noinline
		void setAddressLength(uint8_t width) {
			if (width >= 3 && width <= 5) {
				rf_address_width = width;
				w_reg(RF24_SETUP_AW, width);
			};
		};

		/* Pipe management */
		__noinline
		void open(uint8_t pipeid, uint8_t pktsize, bool autoack) {
			if (pipeid < 6 && pktsize < 33) {
				uint8_t rxen, enaa;

				rxen = r_reg(RF24_EN_RXADDR);
				enaa = r_reg(RF24_EN_AA);

				if (autoack)
					enaa |= (1 << pipeid);
				else
					enaa &= ~(1 << pipeid);
				rxen |= (1 << pipeid);
				w_reg(RF24_EN_RXADDR, rxen);
				w_reg(RF24_EN_AA, enaa);
				if (pktsize > 0) {  // pktsize==0 means Dynamic Payload
					w_reg(RF24_RX_PW_P0 + pipeid, pktsize);
				} else {
					w_reg(RF24_RX_PW_P0 + pipeid, 32);
					uint8_t dynpd = r_reg(RF24_DYNPD);
					dynpd |= (1 << pipeid);
					w_reg(RF24_DYNPD, dynpd);
				}
			}
		};

		__noinline
		void close(uint8_t pipeid) {
			if (pipeid < 6) {
				uint8_t rxen, enaa;

				rxen = r_reg(RF24_EN_RXADDR);
				enaa = r_reg(RF24_EN_AA);

				rxen &= ~(1 << pipeid);
				enaa &= ~(1 << pipeid);
				w_reg(RF24_EN_RXADDR, rxen);
				w_reg(RF24_EN_AA, enaa);
			}
		};

		__noinline
		void setAutoAckParams(uint8_t autoretry_count, uint16_t autoretry_timeout) {
			// TODO: autoretry_timeout is in microseconds, base this on RF_SPEED somehow.
		};

		/* CRC */
		__noinline
		enum rf24_crc getCRC() {
			uint8_t cfg = r_reg(RF24_CONFIG);
			if (cfg & 0x08) {
				if (cfg & 0x04)
					return CRC16;
				return CRC8;
			}
			return CRC_NONE;
		};

		__noinline
		void setCRC(enum rf24_crc crc) {
			uint8_t cfg = r_reg(RF24_CONFIG) & ~0x0C;

			switch (crc) {
				case CRC8:
					cfg |= 0x08;
					break;
				case CRC16:
					cfg |= 0x0C;
					break;
			}
			w_reg(RF24_CONFIG, cfg);
		};

		/* Speed */
		__noinline
		enum rf24_speed getSpeed(void) {
			uint8_t rfsp = r_reg(RF24_RF_SETUP) & 0x28;
			if (rfsp == 0x00)
				return 1MBPS;
			if (rfsp == 0x20)
				return 250KBPS;
			if (rfsp == 0x08)
				return 2MBPS;
			return 250KBPS;  // Reserved ... not sure what else to do here
		};

		__noinline
		void setSpeed(enum rf24_speed s) {
			uint8_t rfsp = r_reg(RF24_RF_SETUP) & ~0x28;  // Clear speed bits by default (this is also 1Mbps by default)
			switch (s) {
				case 250KBPS:
					rfsp |= 0x20;
					break;
				case 2MBPS:
					rfsp |= 0x08;
					break;
			}
			w_reg(RF24_RF_SETUP, rfsp);
		};

		/* TX power */
		__noinline
		int getTXpower(void) {
			uint8_t rfsp = r_reg(RF24_RF_SETUP) & 0x07;
			if (rfsp == 0x07)
				return 7;
			if (rfsp == 0x06)
				return 0;
			if (rfsp == 0x04)
				return -6;
			if (rfsp == 0x02)
				return -12;
			return -18;
		};

		__noinline
		void setTXpower(int t) {
			uint8_t rfsp = r_reg(RF24_RF_SETUP) & ~0x07;  // Clear TXpower bits by default
			uint8_t pwr = 0x07;  // 7dBm
			if (t < 7)
				pwr = 0x06;  // 0dBm
			if (t < 0)
				pwr = 0x04;  // -6dBm
			if (t < -6)
				pwr = 0x02;  // -12dBm
			if (t < -12)
				pwr = 0x00;  // -18dBm
			w_reg(RF24_RF_SETUP, rfsp | pwr);
		};
};


#endif /* SOCRF24_H */
