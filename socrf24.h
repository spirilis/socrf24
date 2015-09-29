/* socrf24 - C++ nRF24L01+ driver using libsoc for Linux SoC boards
 * Tested on BeagleBone Black.
 */

#ifndef SOCRF24_H
#define SOCRF24_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <sched.h>
#include <stdint.h>
#include <stdbool.h>

#include <libsoc_gpio.h>
#include <libsoc_spi.h>
#include <libsoc_debug.h>

#ifndef BITF
  #define BIT0 0x0001
  #define BIT1 0x0002
  #define BIT2 0x0004
  #define BIT3 0x0008
  #define BIT4 0x0010
  #define BIT5 0x0020
  #define BIT6 0x0040
  #define BIT7 0x0080
  #define BIT8 0x0100
  #define BIT9 0x0200
  #define BITA 0x0400
  #define BITB 0x0800
  #define BITC 0x1000
  #define BITD 0x2000
  #define BITE 0x4000
  #define BITF 0x8000
#endif

#include "nRF24L01.h"

#ifndef __noinline
#define __noinline __attribute__((noinline))
#endif
#ifndef __inline
#define __inline inline
#endif

/* Enums for data types and potential values */

enum rf24_crc {
	CRC_NONE = 0,
	CRC8,
	CRC16
};

enum rf24_speed {
	SPEED_250KBPS = 0,
	SPEED_1MBPS = 1,
	SPEED_2MBPS = 2
};

enum rf24_state {
	NOT_PRESENT = 0,
	POWERDOWN,
	STANDBY,
	PTX,
	PRX
};

extern "C" {
int socrf24_irq_handler(void *);
};

/* Main class */

template <uint8_t spi_inst, unsigned int gpio_spics, unsigned int gpio_ce, unsigned int gpio_irq>
class SoCrf24 {
	private:
		uint8_t rf_status, rf_address_width;
		uint8_t rx_pipe_open;

		uint8_t _writebuf[32], _txaddr[5];
		size_t _writelen;

		/* SPI and GPIO I/O and initialization */
		gpio *spics, *ce, *irq;
		spi *SPI;
		volatile bool irq_is_flagged;

		__noinline
		int _irq_callback(void *arg) {
			irq_is_flagged = true;
			return 0;
		};

		__noinline
		bool _init_gpio() {
			spics = libsoc_gpio_request(gpio_spics, LS_GREEDY);
			ce = libsoc_gpio_request(gpio_ce, LS_GREEDY);
			irq = libsoc_gpio_request(gpio_irq, LS_GREEDY);
			if (!spics) {
				fprintf(stderr, "_init_gpio: Error requesting gpio for SPI CS (%d)\n", gpio_spics);
				return false;
			}
			if (!ce) {
				fprintf(stderr, "_init_gpio: Error requesting gpio for CE (%d)\n", gpio_ce);
				return false;
			}
			if (!irq) {
				fprintf(stderr, "_init_gpio: Error requesting gpio for IRQ (%d)\n", gpio_irq);
				return false;
			}

			libsoc_gpio_set_direction(spics, OUTPUT);
			libsoc_gpio_set_level(spics, HIGH);  // CS = HIGH at rest

			libsoc_gpio_set_direction(ce, OUTPUT);
			libsoc_gpio_set_level(ce, LOW);

			libsoc_gpio_set_direction(irq, INPUT);
			libsoc_gpio_set_edge(irq, FALLING);
			libsoc_gpio_callback_interrupt(irq, socrf24_irq_handler, (void *)&irq_is_flagged);

			return true;
		};

		__noinline
		bool _init_spi() {
			SPI = libsoc_spi_init(spi_inst, 0);
			if (!SPI) {
				fprintf(stderr, "_init_spi: Error requesting SPI handle for SPI bus#%d\n", spi_inst);
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

		__inline
		void get_rf_status(void) {
			csn_en();
			libsoc_spi_read(SPI, &rf_status, 1);
			csn_dis();
		};

		__noinline
		void w_reg_lsbstring(uint8_t reg, const void *data, size_t len) {
			csn_en();
			reg |= RF24_W_REGISTER;
			libsoc_spi_rw(SPI, &reg, &rf_status, 1);
			for (int i = len; i >= 0; i--) {
				libsoc_spi_write(SPI, (uint8_t *)data+i, 1);
			}
			csn_dis();
		};

		__noinline
		size_t r_reg_lsbstring(uint8_t reg, void *data, size_t len) {
			csn_en();
			reg |= RF24_R_REGISTER;
			libsoc_spi_rw(SPI, &reg, &rf_status, 1);
			for (int i = len; i >= 0; i--) {
				libsoc_spi_read(SPI, (uint8_t *)data+i, 1);
			}
			csn_dis();
			return len;
		};

		__noinline
		void w_msbstring(uint8_t cmd, const void *data, size_t len) {
			csn_en();
			libsoc_spi_rw(SPI, &cmd, &rf_status, 1);
			libsoc_spi_write(SPI, (uint8_t *)data, len);
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
		SoCrf24() {
			spics = NULL;
			ce = NULL;
			irq = NULL;
			irq_is_flagged = false;
			rf_status = 0;
			rf_address_width = 5;
			_writelen = 0;
			rx_pipe_open = 0;
		};

		/* User-called chip initialization */
		__noinline
		bool begin(enum rf24_speed datarate = SPEED_1MBPS, uint8_t chan = 0) {
			int i;

			if (!_init_gpio()) {
				fprintf(stderr, "begin: Fault; chip is in an unknown or uninitialized state.\n");
				return false;
			}
			if (!_init_spi()) {
				fprintf(stderr, "begin: Fault; chip is in an unknown or uninitialized state.\n");
				return false;
			}
			if (!isPresent()) {
				fprintf(stderr, "begin: GPIO+SPI init complete but no transceiver found; aborting begin()\n");
				return false;
			}

			setAddressLength(rf_address_width);
			irq_is_flagged = false;
			for (i=0; i < 6; i++) {
				close(i);
			}
			w_reg(RF24_CONFIG, 0x00);
			setChannel(chan);
			setSpeed(datarate);
			setTXpower(-18);  // Defaults at minimum
			w_reg(RF24_FEATURE, 0x04);  // EN_DPL=1, EN_ACK_PAY=0, EN_DYN_ACK=0

			return true;
		};

		__noinline
		void end(void) {
			int i;

			w_reg(RF24_CONFIG, 0x00);
			for (i=0; i < 6; i++) {
				close(i);
			}
			libsoc_spi_free(SPI);
			libsoc_gpio_free(spics);
			libsoc_gpio_free(ce);
			libsoc_gpio_callback_interrupt_cancel(irq);
			libsoc_gpio_free(irq);
		};

		/* Address management */
		__noinline
		void setTXaddress(const void *addr) {
			if (addr != NULL) {
				memcpy(_txaddr, (const uint8_t *)addr, rf_address_width);
				w_reg_lsbstring(RF24_TX_ADDR, (const uint8_t *)addr, rf_address_width);
			}
		};

		__inline
		void setRXaddress(uint8_t pipe, const void *addr) {
			if (pipe < 6 && addr != NULL)
				return w_reg_lsbstring(RF24_RX_ADDR_P0 + pipe, (const uint8_t *)addr, rf_address_width);
		};

		__noinline
		void setAddressLength(uint8_t width) {
			if (width >= 3 && width <= 5) {
				rf_address_width = width;
				w_reg(RF24_SETUP_AW, width-2);
			};
		};

		__inline
		void getTXaddress(void *buf, size_t *getsize) {
			if (buf != NULL) {
				if (getsize != NULL)
					*getsize = rf_address_width;
				memcpy((uint8_t *)buf, _txaddr, rf_address_width);
			}
		};

		__noinline
		void getRXaddress(uint8_t pipeid, void *buf, size_t *getsize) {
			if (pipeid < 6 && buf != NULL) {
				if (getsize != NULL)
					*getsize = rf_address_width;
				r_reg_lsbstring(RF24_RX_ADDR_P0 + pipe, (uint8_t *)buf, rf_address_width);
			}
		};

		__noinline
		bool isPresent(void) {  // Use SETUP_AW to determine if SPI, GPIO's are working right
			uint8_t saw = r_reg(RF24_SETUP_AW);
			if ( (saw & 0xFC) == 0x00 && (saw & 0x03) != 0x00 )
				return true;
			return false;
		};

		/* Pipe management */
		__noinline
		void open(uint8_t pipeid, uint8_t pktsize = 0, bool autoack = false) {
			if (pipeid < 6 && pktsize < 33) {
				uint8_t rxen, enaa, dynpd;

				rxen = r_reg(RF24_EN_RXADDR);
				enaa = r_reg(RF24_EN_AA);
				dynpd = r_reg(RF24_DYNPD);

				if (autoack)
					enaa |= (1 << pipeid);
				else
					enaa &= ~(1 << pipeid);

				rxen |= (1 << pipeid);
				w_reg(RF24_EN_RXADDR, rxen);
				w_reg(RF24_EN_AA, enaa);
				if (pktsize > 0) {  // pktsize==0 means Dynamic Payload
					w_reg(RF24_RX_PW_P0 + pipeid, pktsize);
					dynpd &= ~(1 << pipeid);
				} else {
					w_reg(RF24_RX_PW_P0 + pipeid, 32);
					dynpd |= (1 << pipeid);
				}
				w_reg(RF24_DYNPD, dynpd);
				rx_pipe_open |= (1 << pipeid);
			}
		};

		__noinline
		void close(uint8_t pipeid) {
			if (pipeid < 6) {
				uint8_t rxen, enaa, dynpd;

				rxen = r_reg(RF24_EN_RXADDR);
				enaa = r_reg(RF24_EN_AA);
				dynpd = r_reg(RF24_DYNPD);

				rxen &= ~(1 << pipeid);
				enaa &= ~(1 << pipeid);
				dynpd &= ~(1 << pipeid);
				w_reg(RF24_EN_RXADDR, rxen);
				w_reg(RF24_EN_AA, enaa);
				w_reg(RF24_DYNPD, dynpd);

				rx_pipe_open &= ~(1 << pipeid);
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
				return SPEED_1MBPS;
			if (rfsp == 0x20)
				return SPEED_250KBPS;
			if (rfsp == 0x08)
				return SPEED_2MBPS;
			return SPEED_250KBPS;  // Reserved ... not sure what else to do here
		};

		__noinline
		void setSpeed(enum rf24_speed s) {
			uint8_t rfsp = r_reg(RF24_RF_SETUP) & ~0x28;  // Clear speed bits by default (this is also 1Mbps by default)
			switch (s) {
				case SPEED_250KBPS:
					rfsp |= 0x20;
					break;
				case SPEED_2MBPS:
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

		/* Channel */
		__inline
		uint8_t getChannel(void) {
			return r_reg(RF24_RF_CH);
		};

		__inline
		void setChannel(uint8_t ch) {
			if (ch <= 125)
				w_reg(RF24_RF_CH, ch);
		};

		__inline
		bool rfSignalDetected(void) {  // Is there RF activity on the current channel
			return (bool) r_reg(RF24_RPD);
		};

		/* Transceiver state administration */
		__noinline
		void powerdown(void) {
			uint8_t cfg = r_reg(RF24_CONFIG);
			cfg &= ~(RF24_PWR_UP | RF24_PRIM_RX);
			ce_dis();
			w_reg(RF24_CONFIG, cfg);
		};

		__noinline
		void enableRX(void) {
			uint8_t cfg = r_reg(RF24_CONFIG);
			cfg |= RF24_PWR_UP | RF24_PRIM_RX;
			w_reg(RF24_CONFIG, cfg);
			ce_en();
		};

		__noinline
		void disableRX(void) {
			ce_dis();
			uint8_t cfg = r_reg(RF24_CONFIG);
			cfg &= ~RF24_PRIM_RX;
			w_reg(RF24_CONFIG, cfg);
		};

		/* Administrative queries */
		__noinline
		enum rf24_state radioState(void) {
			if (!isPresent())
				return NOT_PRESENT;
			uint8_t cfg = r_reg(RF24_CONFIG);
			if (!(cfg & 0x02))
				return POWERDOWN;
			if (libsoc_gpio_get_level(ce) == LOW)
				return STANDBY;
			if (!(cfg & 0x01))
				return PTX;
			return PRX;
		};

		/* User I/O */
		__noinline
		bool available(bool short_method=false) {
			// Short method quickly returns false if the IRQ line is not triggered.
			// Only use this if you're absolutely certain you've drained the RX FIFOs on the transceiver
			// and are currently waiting for fresh packets.
			if (short_method) {
				if (!irq_is_flagged && libsoc_gpio_get_level(irq) == HIGH)
					return false;
			}
			uint8_t fifo = r_reg(RF24_FIFO_STATUS);
			if (fifo & RF24_RX_EMPTY)
				return false;
			return true;
		};

		__noinline
		size_t recv(void *inbuf, size_t maxlen = 32, uint8_t *pipenum = NULL) {
			uint8_t plwidth, pn;

			// Input checking
			if (inbuf == NULL)
				return 0;

			r_msbstring(RF24_R_RX_PL_WID, &plwidth, 1);
			if (plwidth == 0 || plwidth > 32) {
				w_msbstring(RF24_FLUSH_RX, NULL, 0);  // Issue FLUSH_RX
				w_reg(RF24_STATUS, RF24_RX_DR);  // and clear any pending RX IRQs since we just purged all RX FIFOs
				return 0;
			}

			// rf_status now contains the RX pipe#
			pn = (rf_status & 0x0E) >> 1;
			if (pipenum != NULL)
				*pipenum = pn;

			if (maxlen < plwidth)
				plwidth = maxlen;
			r_msbstring(RF24_R_RX_PAYLOAD, (uint8_t *)inbuf, plwidth);

			/* Optimization - check FIFO_STATUS, if RX buffer is empty clear RX_DR.
			 * By *not* clearing RX_DR after reading with a non-empty FIFO, the IRQ line will remain triggered so
			 * running available(true) will return true, thus ensuring poorly written user firmware will know there
			 * is still data waiting to be read.
			 */
			uint8_t fifo = r_reg(RF24_FIFO_STATUS);
			if (fifo & RF24_RX_EMPTY) {
				w_reg(RF24_STATUS, RF24_RX_DR);
			}
			return plwidth;
		};

		__noinline
		bool sendto(const void *txaddr, const void *outbuf, size_t len, bool autoack = false) {
			uint8_t old_p0addr[5];
			bool had_rx_fifo = false;
			bool rx0_was_open = false;

			// Input checking
			if (txaddr == NULL || outbuf == NULL)
				return false;
			if (len < 1 || len > 32)
				return false;

			w_reg_lsbstring(RF24_TX_ADDR, (const uint8_t *)txaddr, rf_address_width);
			w_msbstring(RF24_W_TX_PAYLOAD, (const uint8_t *)outbuf, len);
			if (autoack) {  // Prepare pipe#0 for receiving the auto ACKs
				if (rx_pipe_open & 0x01) {
					rx0_was_open = true;
					getRXaddress(0, old_p0addr);
				}
				open(0, 0, true);
				w_reg_lsbstring(RF24_RX_ADDR_P0, (const uint8_t *)txaddr, rf_address_width);
			}

			// Validate the IRQ line is un-triggered
			if (libsoc_gpio_get_level(irq) == LOW) {
				// What are we waiting for?
				get_rf_status();
				if (rf_status & (RF24_TX_DS | RF24_MAX_RT))
					w_reg(RF24_STATUS, RF24_TX_DS | RF24_MAX_RT);
				if (rf_status & (RF24_RX_DR))  // Clear RX IRQ so we can sense a TX IRQ
					w_reg(RF24_STATUS, RF24_RX_DR);
				if ( (rf_status & 0x0E) != 0x0E )  // But if RX FIFO not empty, remember this so later we can manually trigger irq_is_flagged
					had_rx_fifo = true;
				get_rf_status();
				if (rf_status & (RF24_TX_DS | RF24_MAX_RT | RF24_RX_DR))
					return false;  // Give up
			}

			// Fire up the transceiver
			uint8_t cfg = r_reg(RF24_CONFIG);
			uint8_t cfg_orig = cfg;
			if ( (cfg & (RF24_PWR_UP | RF24_PRIM_RX)) == (RF24_PWR_UP | RF24_PRIM_RX) ) {
				// Presently in PRX mode; disable that
				cfg &= ~RF24_PRIM_RX;
				ce_dis();
			}
			if (!(cfg & RF24_PWR_UP)) {
				cfg |= RF24_PWR_UP;
			}
			w_reg(RF24_CONFIG, cfg);
			if (!(cfg_orig & RF24_PWR_UP)) {
				usleep(5000);  // 5ms to wake up
			}
			ce_en();
			usleep(150);  // at least 130us required from CE=1 until TX is active
			while (!irq_is_flagged)
				sched_yield();
			// Read IRQ
			get_rf_status();
			if (!(rf_status & (RF24_TX_DS | RF24_MAX_RT))) {
				ce_dis();
				return false;  // No idea what went wrong?
			}
			bool retval;
			if (rf_status & RF24_TX_DS)
				retval = true;
			else
				retval = false;
			ce_dis();

			// Restore initial config (i.e. switch back to PRX mode if that was on before sendto() was called)
			if (cfg_orig & RF24_PRIM_RX) {
				w_reg(RF24_CONFIG, cfg_orig);
				ce_en();
				usleep(10);
			}
			if (autoack) {
				if (rx0_was_open) {
					setRXaddress(0, old_p0addr);
				} else {
					close(0);
				}
			}
			if (had_rx_fifo)
				irq_is_flagged = true;  // Manually trigger irq_is_flagged so available() returns true later
			// Transceiver is left in Standby mode if it was previously in powerdown mode.
			return retval;
		};

		/* Managed user I/O - single or arbitrary-length write's that queue into a 32-byte buffer
		 * It is mandatory that the receiver side of this transaction have Dynamic Payload enabled for their RX pipe.
		 */
		__noinline
		bool flush(void) {
			bool retval = sendto(_txaddr, _writebuf, _writelen, false);
			_writelen = 0;
			return retval;
		};

		__noinline
		void purge(void) {
			_writelen = 0;
		};

		__noinline
		size_t write(const void *inbuf, size_t len) {
			if ( (_writelen + len) < 33 ) {  // Will this be a simple call?
				memcpy(_writebuf+_writelen, (const uint8_t *)inbuf, len);
			} else {
				// Nope; this is where it gets fun...
				size_t i = 0, idx = 0;  // idx == position within user's inbuf[]
				size_t bytes_left = len;
				do {
					i = 32 - _writelen;  // Amount to write this time through the loop
					memcpy(_writebuf+_writelen, inbuf+idx, i);
					_writelen += i;
					flush();  // TODO: Find a way to enable auto-ack here

					idx += i;
					bytes_left -= i;
				} while (bytes_left > 32);

				if (bytes_left) {
					memcpy(_writebuf+_writelen, inbuf+idx, bytes_left);
					_writelen += i;
				}
			}
		};

		__noinline
		size_t write(uint8_t c) {
			if (_writelen >= 32)
				flush();
			_writebuf[_writelen++] = c;
		};
};


#endif /* SOCRF24_H */
