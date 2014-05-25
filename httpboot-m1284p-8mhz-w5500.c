// Internet-Bootloader for Atmega1284p and WizNet W5500, developped using Atmel Studio.
// See http://s.wangnick.de/doku.php?id=iot-basisstation for target hardware.
// (C) Copyright 2014 Sebastian Wangnick.

#define F_CPU 8000000
#define BAUD 57600
#include <util/setbaud.h>

// Fuses: Low:0xE2 High:0x90 Extended:0xFD
//  Internal oscillator, 6CK+65ms
//  Boot reset vector enabled, boot section size 4096 words start address $F000 (0x1E000)
//  Preserve EEPROM through chip erase cycle
//  SPI enabled
//  JTAG enabled
//  Brown-out detection at 2.7V

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdlib.h>

// Pin connections
#define SS (1<<PINB4)
#define MOSI (1<<PINB5)
#define MISO (1<<PINB6)
#define SCK (1<<PINB7)
#define W5500_POWER (1<<PINB2)
#define W5500_INT (1<<PINB3) // Unused
#define W5500_SS SS
#define SD_SS (1<<PINB1)

#define EEPROM_BASE	        ((uint8_t*) 0x0000)
#define EEPROM_GWIP         EEPROM_BASE+0x10 // Not required for DHCP
#define EEPROM_MYMASK       EEPROM_BASE+0x14 // Not required for DHCP
#define EEPROM_MAC	        EEPROM_BASE+0x18
#define EEPROM_MYIP	        EEPROM_BASE+0x1E // First byte 0xFF requests use of DHCP
#define EEPROM_DNSIP        EEPROM_BASE+0x22 // Not required for DHCP
#define EEPROM_URL          EEPROM_BASE+0x26 // Syntax: "host/pathqueryfragment\0" "&loaderid=<swid>" will be appended.

#define DEBUG_PHASES
//#define DEBUG_DHCP
//#define DEBUG_DHCP_VERBOSE
//#define DEBUG_DNS
#define DEBUG_PROGRESS
//#define DEBUG_PARSER
//#define DEBUG_SPI_WRITE
//#define DEBUG_SPI
//#define DEBUG_W5500
//#define DEBUG_SEND
//#define DEBUG_RECEIVE
//#define DEBUG_FLASH
//#define DEBUG_MEM

#define DO_VERIFY_FLASH
#define DO_SD_SPI

#define TIMEOUT F_CPU/1024*5 // 5 seconds timeout for serial communication start

// All our PROGMEM data is in the same extended range as the FLASHEND
#define pgm_read_byte_progmem(ptr) pgm_read_byte_far((FLASHEND&0xFFFF0000)|(uint16_t)(ptr))

void uart_putc (char c);
void uart_puts (char* s);
void uart_putp (PGM_P s);
void uart_puthex2 (uint8_t val);
#define uart_puthex4(val) do{uart_puthex2(((uint16_t)(val))>>8);uart_puthex2(((uint16_t)(val))&0xFF);}while(0)
#define uart_puthex8(val) do{uart_puthex4(((uint32_t)(val))>>16);uart_puthex4(((uint32_t)(val))&0xFFFF);}while(0)
char uart_getc ();

uint8_t attempt_bootload (void);
void write_mem (void);
void read_mem (void);

void init (void) __attribute__ ((naked)) __attribute__ ((section (".vectors")));
void init (void) {asm volatile ( "rjmp start" );}

void __do_copy_data (void) __attribute__ ((naked)) __attribute__ ((section (".text9")));
void __do_copy_data (void) {}
void __do_clear_bss (void) __attribute__ ((naked)) __attribute__ ((section (".text9")));
void __do_clear_bss (void) {}

const char SWID[] PROGMEM = __FILE__ " " __DATE__ " " __TIME__;
const char* swid;

typedef enum {MEM_EEPROM='E', MEM_FLASH='F', MEM_SRAM='S', MEM_REG='R'} Mem; Mem mem;

#define SPI(data) do {SPDR = data; while (!(SPSR & 1<<SPIF)) continue;} while (0)
#define SPIBEGIN(pin) PORTB &= ~(pin)
#define SPIEND(pin) PORTB |= (pin)

#ifdef DO_SD_SPI
void sd_spi(void) {
    uint8_t i, result;
#ifdef DEBUG_PHASES
    uart_putc('s');
#endif
    PORTB |= SD_SS;     // Make sure SS is HIGH when activating OUTPUT
    DDRB |= SD_SS;
    SPCR |= 1<<SPR1;    // SD standard requires 100-400kHz during init. Slow down SPI clock to F_CPU/32=250kHz
    // SD standard allows card to require max of 1ms and 74 clock cycles with CS high as from VCC ramp-up
    // We send 256 clock cycles, this lasts >1ms at 250kHz
    for (i=0; i<32; i++) { //
        SPI(0xFF);
    }
    // Command to go idle in SPI mode
    SPIBEGIN(SD_SS);
    SPI(0x40);
    SPI(0x00);
    SPI(0x00);
    SPI(0x00);
    SPI(0x00);
    SPI(0x95);
    result = 0xFF;
    for (i=0; result&0x80 && i<255; i++) {
        SPI(0xFF);
        result = SPDR;
    }
    SPIEND(SD_SS);
#ifdef DEBUG_PHASES
    uart_puthex2(result);
#endif
    SPCR &= ~(1<<SPR1); // Speed up SPI clock back to F_CPU/2
    DDRB &= ~SD_SS;      // Set SD_SS port back to default
    PORTB &= ~SD_SS;     // Set SD_SS port back to default
}
#endif

uint8_t mcusr;

void start (void) __attribute__ ((naked)) __attribute__ ((section (".init0")));
void start (void) {
	asm volatile ( "ldi	16, %0" :: "i" (RAMEND >> 8) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_HI_ADDR) );
	asm volatile ( "ldi	16, %0" :: "i" (RAMEND & 0x0ff) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_LO_ADDR) );
	asm volatile ( "clr __zero_reg__" );									// GCC depends on register r1 set to 0
	asm volatile ( "out %0, __zero_reg__" :: "I" (_SFR_IO_ADDR(SREG)) );	// set SREG to 0
	
	mcusr = MCUSR;
	MCUSR =	0;
    wdt_reset();
    wdt_disable();

    // Set up timer for misc timing purposes. 
	TCCR1B = 1<<CS02 | 1<<CS00;                  // Prescaler 1024 -> 1/8ms per tick at 8MHz
    
    // Activate UART
	UBRR0 = UBRR_VALUE;
	#if USE_2X
	UCSR0A = 1<<U2X0;
	#endif
	UCSR0C = 1<<UCSZ01 | 1<<UCSZ00;				 // Asynchronous 8N1 (this is the default)
	UCSR0B = 1<<RXEN0 | 1<<TXEN0;                // Switch on UART RX und TX

    // Activate SPI
    PORTB = SS; // Make sure SS is HIGH when activating OUTPUT
    DDRB = SS | MOSI | SCK; // SS MUST be output to enable SPI
    SPCR = 1<<SPE | 1<<MSTR; // Enable SPI, master mode 0, maximum speed
    SPSR = 1<<SPI2X;		  // Enable SPI double speed clock, F_CPU/2 = 4MHz

#ifdef DO_SD_SPI
    sd_spi();
#endif    
	
    swid = SWID+sizeof(SWID)-1;                  // AVR Studio compiles using ./../ in front of filename, have to strip
    while (swid!=SWID && pgm_read_byte_progmem(swid-1)!='/') swid--;
    
    uart_putc('\n');
    uart_putp(swid);
    uart_putp(PSTR(", MCUSR="));
    uart_puthex2(mcusr);
    uart_putc('\n');
    
    uint8_t exec = 0;
	while (!exec) {
		uart_putp(PSTR("> "));
        TCNT1 = 0;
		char c = uart_getc();
		TCCR1B = 0;
		TCNT1 = 0;
		uart_putc(c);
		switch (c) {
		case 'R':
			// Fall through
		case 'W':
            mem = uart_getc();
            if (mem!=MEM_EEPROM && mem!=MEM_FLASH && mem!=MEM_SRAM && mem!=MEM_REG) mem = '?';
            uart_putc(mem);
            uart_putc('\n');
            if (mem!='?') {
                if (c=='R') read_mem(); else write_mem();
            }
            break;
        case 'M':
            uart_putp(PSTR("\nMCUSR: "));
            uart_puthex2(mcusr);
            break;
		case 0:
			c = mcusr&(1<<PORF|1<<EXTRF|1<<BORF)? 'G': 'B';
			// Fall through
		case 'B':
    		// Fall through
		case 'T':
			// Fall through
		case 'G':
			exec = c;
			break;
		default:
			uart_putp(PSTR("R[EFSR], W[EFSR], M, B, T, G"));
		}
	    uart_putc('\n');
	}
    
    wdt_enable(WDTO_8S);
	
	uart_putc(exec);
    if (exec=='B') {
		exec = attempt_bootload()? 'G': 'T';
	}
	uart_putc(exec);
	_delay_ms(1);

	// Return registers to default values after reset
  	DDRB = 0;
   	PORTB = 0;
   	SPCR = 0;
   	SPSR = 0;
	UCSR0B = 0;
#if USE_2X
	UCSR0A = 0;
#endif	
	UBRR0 = 0;

	if (exec=='G') { // Start application by jumping to address 0
        wdt_reset();
		asm volatile(					
			"jmp    0"
			);
	} else { // Perform a processor reset via watchdog.
		wdt_enable(WDTO_250MS);
		for (;;) continue;
	}
}

#ifdef __BYTE_ORDER__
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define HTONS(x) ((uint16_t) (((uint16_t) (x) << 8) | ((uint16_t) (x) >> 8)))
#define HTONL(x) ((uint32_t) (((uint32_t) HTONS(x) << 16) | HTONS((uint32_t) (x) >> 16)))
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define HTONS(x) ((uint16_t) (x))
#define HTONL(x) ((uint32_t) (x))
#else
#error Byte order not supported!
#endif
#else
#error Byte order not defined!
#endif

#define NTOHS(x) HTONS(x)
#define NTOHL(x) HTONL(x)


uint8_t read_eeprom (uint32_t addr) {
    return eeprom_read_byte((uint8_t*)(uint16_t)addr);
}
uint8_t read_flash (uint32_t addr) {
    return pgm_read_byte_far(addr);
}
uint8_t read_sram (uint32_t addr) {
    return *(uint8_t*)(uint16_t)addr;
}

#define HEXBLOCK 16

void read_mem (void) {
	uint32_t addr = mem==MEM_SRAM? RAMSTART: 0;
    uint32_t size = (mem==MEM_EEPROM? E2END: mem==MEM_FLASH? FLASHEND: mem==MEM_SRAM? RAMEND: RAMSTART-1)+1;
    uint8_t (*reader) (uint32_t addr) = mem==MEM_EEPROM? read_eeprom: mem==MEM_FLASH? read_flash: read_sram;
	while (size) {
		size--;
		if ((*reader)(size)!=0xFF) break;
	}
	uint8_t cksum, len, val, i;
	for (addr=0; addr<=size; addr+=HEXBLOCK) {
        if (addr && !(addr&0xFFFF)) {
 		    uart_putp(PSTR(":02000002"));
            uart_puthex2(val = addr>>12); cksum = 4+val;
            uart_puthex2(val = addr>>4);  cksum += val;
    		uart_puthex2(0x100-cksum);
    		uart_putc('\n');
        }        
		uart_putc(':');
		len = size-addr+1>HEXBLOCK? HEXBLOCK: (uint8_t)(size-addr+1);
		uart_puthex2(len); cksum = len;
		uart_puthex2(val = addr>>8); cksum += val;
		uart_puthex2(val = addr); cksum += val;
		uart_puthex2(0x00); // cksum += 0x00;
		for (i=0; i<len; i++) {
			val = (*reader)(addr+i);
			uart_puthex2(val); cksum += val;
		}
		uart_puthex2(0x100-cksum);
		for (;i<HEXBLOCK;i++) {
			uart_putp(PSTR("  "));
		}
		uart_putp(PSTR(" ;"));
		for (i=0; i<len; i++) {
			val = (*reader)(addr+i);
			uart_putc(val>0x20&&val<128?val:'.');
		}
		uart_putc('\n');
	}
	uart_putp(PSTR(":00000001FF\n"));
}

#define SPM_PAGEMASK ~(SPM_PAGESIZE-1)
#define SPM_NOPAGE 0xFFFFFFFF
uint16_t eepromwait, pa;
uint32_t spmaddr, spmpage;
uint8_t spmdata[SPM_PAGESIZE]; // Note that this memory is also used below for the DNS answer UDP frame
uint8_t spmdirty;

void flashinit (void) {
	eepromwait = 0;
	spmpage = SPM_NOPAGE;
    spmaddr = 0;
    spmdirty = 0;
}

void flashaddr (uint32_t addr) {
	spmaddr = addr;
}

void flashflush (void) {
    uint16_t pa;
	if (spmdirty) {
#ifdef DEBUG_PROGRESS
        uart_putc('!');
#endif
#ifdef DEBUG_FLASH
        uart_putc('F');
        uart_puthex8(spmpage);
        uart_putc('\n');
#endif
		if (!eepromwait) {
			eeprom_busy_wait();
			eepromwait = 1;
		}
      	for (pa=0; pa<SPM_PAGESIZE; pa+=2) {
            boot_page_fill(pa,spmdata[pa+1]<<8|spmdata[pa]); // low byte is written first, then high byte as second            
        }
		boot_page_erase(spmpage);
		boot_spm_busy_wait();		// Wait until the memory is erased.
		boot_page_write(spmpage);   // Store buffer in flash page. This also erases the temporary buffer again.
		boot_spm_busy_wait();       // Wait until the memory is written.
        boot_rww_enable();          // Allow read access again such that a) verification can be done and b) the next page can be read
#ifdef DO_VERIFY_FLASH
        uint8_t val;
        for (pa=0; pa<SPM_PAGESIZE; pa++) {
            val = pgm_read_byte_far(spmpage+pa);
            if (val!=spmdata[pa]) {
                // TODO: Report this back to the internet server somehow
                uart_putp(PSTR("Flash programming failed at "));
                uart_puthex8(spmpage+pa);
                uart_putp(PSTR(", wrote "));
                uart_puthex2(spmdata[pa]);
                uart_putp(PSTR(", got "));
                uart_puthex2(val);
                uart_putc('\n');
                break;
            }
        }
#endif
        spmdirty = 0;
	} else {
#ifdef DEBUG_PROGRESS
        uart_putc(':');
#endif       
    }
}

void flashwrite (uint8_t data) {
    uint16_t pa;
	if ((spmaddr&SPM_PAGEMASK)!=spmpage) {
		flashflush();
		spmpage = spmaddr&SPM_PAGEMASK;
        for (pa=0; pa<SPM_PAGESIZE; pa++) {
            spmdata[pa] = pgm_read_byte_far(spmpage+pa);
        }
	}
    pa = spmaddr&~SPM_PAGEMASK;
    if (spmdata[pa]!=data) {
#ifdef DEBUG_FLASH
        uart_putc('F');
        uart_puthex8(spmaddr);
        uart_putc(':');
        uart_puthex2(data);
        uart_putc('\n');
#endif
        spmdata[pa] = data;
        spmdirty = 1;
    }
	spmaddr++;
}

void flashfinish (void) {
	flashflush();
}

void uart_putc (char c) {
    while (!(UCSR0A&1<<UDRE0)) continue;
    UDR0 = c;                      /* sende Zeichen */
}
void uart_puts (char* s) {
    while (*s) {
        uart_putc(*s++);
    }
}
void uart_putp (PGM_P s) {
    char c;
    while ((c = pgm_read_byte_progmem(s++))) {
        uart_putc(c);
    }
}
#define hex(digit) ((digit)+((digit)>9?'A'-10:'0'))
void uart_puthex2 (uint8_t val) {
    uart_putc(hex(val>>4)); 
    uart_putc(hex(val&0xF));
}

char uart_getc () {
    while (!(UCSR0A & 1<<RXC0)) {
        if (TCNT1>TIMEOUT) return 0; // Bail out after timeout if timer 1 is running
    }
    return UDR0;
}

uint16_t hex2num (const uint8_t *ascii, uint8_t num) {
	uint16_t val = 0;
    uint8_t c;
	while (num--) {
		c = *ascii++;
		c = (c>='A'? c-'A'+10: c)&0x0F;
		val = val<<4 | c;
	}
	return val;
}

enum HEX_STATE {
	STATE_HEAD,
	STATE_CR,
	STATE_CRLF,
	STATE_CRLFCR,
	STATE_START,
	STATE_SIZE,
	STATE_ADDRESS,
	STATE_TYPE,
	STATE_DATA,
	STATE_ENDLINE,
	STATE_ENDLINECR,
	STATE_FINALLINE,
	STATE_FINALLINECR,
	STATE_END,
	STATE_ERROR
};

const char HEX_START = ':';
uint16_t addr, esar, hex;
uint8_t state, bytes, cnt, cksum, size, type;
Mem mem;

void process_start () {
#ifdef DEBUG_PARSER
	uart_putc('S');
#endif
	esar = 0;
	state = STATE_HEAD;
	if (mem==MEM_FLASH) {
		flashinit();
	}
}

void process_hex (char c) {
    uint16_t ptr;
#ifdef DEBUG_PARSER
	uart_putc('X');
	uart_putul(state);
#endif
	switch (state) {
	case STATE_HEAD:
	case STATE_CRLF:
		if (c=='\r') state++;
		else if (c=='\n') state += 2;
		else state = STATE_HEAD;
		break;
	case STATE_CR:
	case STATE_CRLFCR:
		if (c=='\n') state++;
		else state = STATE_HEAD;
		break;
	case STATE_END:
		// state = STATE_ERROR; // This is too rigid.
	case STATE_ERROR:
		break;
	case STATE_START:
		if (c==HEX_START) {
			state++;
			cnt = 0;
			hex = 0;
			cksum = 0;
			bytes = 0;
		}
		break;
	case STATE_SIZE:
	case STATE_ADDRESS:
	case STATE_TYPE:
	case STATE_DATA:
	    if ((c>='0'&&c<='9') || (c>='A'&&c<='F') || (c>='a'&&c<='f')) {
			hex = hex<<4 | (c>='A'? c-'A'+10: c)&0x0F;
			cnt++;
			if (state==STATE_ADDRESS) {
				if (cnt==4) {
					addr = hex;
					cksum += (addr>>8) + (addr&0xFF);
					cnt = 0;
					if (mem==MEM_FLASH) {
						flashaddr((uint32_t)esar<<4|addr);
					}
					state++;
				}
			} else {
				if (cnt==2) {
					cksum += hex;
					cnt = 0;
					if (state==STATE_SIZE) {
						size = hex;
						state++;
					} else if (state==STATE_TYPE) {
						type = hex;
						if (type>=3 || type>0 && (addr || size!=(type==1?0:2))) {
							state = STATE_ERROR;
						} else {
							state++;
						}
					} else {
						if (bytes++==size) {
							state = cksum?STATE_ERROR:type==1?STATE_END:STATE_START;
						} else if (type==0) {
							if (mem==MEM_FLASH) {
								flashwrite(hex);
							} else {
								ptr = (uint16_t)esar<<4|addr++;
#ifdef DEBUG_MEM								
								uart_putc('M');
								uart_putul(ptr);
								uart_putc(':');
								uart_putul(hex);
								uart_putc(',');
#endif
                                if (mem==MEM_EEPROM) {
                                    if (eeprom_read_byte((uint8_t*)ptr)!=hex) {
								        eeprom_write_byte((uint8_t*)ptr,hex);
                                    }                                    
                                } else {
                                    *(uint8_t*)ptr = hex;
                                }                                                                
							}
						} else if (type==2) {
							esar = esar<<8 | hex;
						} else {
							state = STATE_ERROR;
						}
					}
                    hex = 0;
				}
			}
		} else {
			state = STATE_ERROR;
		}
		break;
	}
#ifdef DEBUG_PARSER
	uart_putc('>');
	uart_puthex2(state);
	uart_putc('\n');
#endif
}

void process_end (void) {
#ifdef DEBUG_PARSER
	uart_putc('N');
#endif
	if (mem==MEM_FLASH) {
		flashfinish();
	}
}

void write_mem (void) {
    char c;
	process_start();
	state = STATE_START;
	while (state<STATE_END) {
		c = uart_getc();
		uart_putc(c);
		process_hex(c);
	}
	uart_putp(state==STATE_END?PSTR("OK"):PSTR("ERROR"));
}

#include "w5500.h"

uint16_t W51_xfer (uint16_t addr, uint8_t cb, uint8_t* ptr, uint16_t len, Mem mem) {
    uint8_t val;
#ifdef DEBUG_SPI
	uart_putc('x');
#endif
	SPIBEGIN(W5500_SS);
	SPI(addr>>8);
	SPI(addr&0xFF);
    addr += len;
	SPI(cb);
	if (cb&_W5500_SPI_WRITE_) {
#ifdef DEBUG_SPI_WRITE
		uart_putc('w');
#endif
		while (len--) {
#ifdef DEBUG_SPI_WRITE
            uart_puthex4(ptr);
#endif
            val = mem==MEM_FLASH? pgm_read_byte_progmem(ptr++): mem==MEM_EEPROM? eeprom_read_byte(ptr++): ptr? *ptr++: 0; 
            SPI(val);
#ifdef DEBUG_SPI_WRITE
			uart_puthex2(val);
#endif
		}
	} else {
#ifdef DEBUG_SPI
		uart_putc('r');
#endif
		while (len--) {
			SPI(0);
            val = SPDR;
            if (ptr) *ptr++ = val;
#ifdef DEBUG_SPI
			uart_puthex2(val);
#endif
		}
	}
	SPIEND(W5500_SS);
#ifdef DEBUG_SPI
	uart_putc('\n');
#endif
    return addr;
}

#define W51_sendmem(addr,cb,ptr,len,mem) W51_xfer(addr,(cb)|_W5500_SPI_WRITE_,(uint8_t*)(ptr),len,mem)
#define W51_send(addr,cb,ptr,len) W51_xfer(addr,(cb)|_W5500_SPI_WRITE_,ptr,len,MEM_SRAM)
#define W51_recv(addr,cb,ptr,len) W51_xfer(addr,(cb)|_W5500_SPI_READ_,ptr,len,MEM_SRAM)

uint8_t W51_read (uint16_t reg) {
	uint8_t data;
	W51_recv(reg>>8,reg&0xFF,&data,1);
	return data;
}

void W51_write (uint16_t reg, uint8_t data) {
	W51_send(reg>>8,reg&0xFF,&data,1);
}

#define W51_writeptr(reg,ptr,len) W51_send(reg>>8,reg&0xFF,ptr,len);


uint16_t htonsval;

uint16_t W51_read16 (uint16_t reg) {
    W51_recv(reg>>8,reg&0xFF,(uint8_t*)&htonsval,2);
	return NTOHS(htonsval);
}

void W51_write16 (uint16_t reg, uint16_t data) {
    htonsval = HTONS(data); // Can't swap data here, leads to strange corruptions tho assembly looks ok
	W51_send(reg>>8,reg&0xFF,(uint8_t*)&htonsval,2);
}

void W51_exec (uint8_t sock, uint8_t cmd) {
	W51_write(Sn_CR(sock),cmd);
	while (W51_read(Sn_CR(sock))) continue;
}

uint16_t W51_read16twice (uint16_t addr) {
	uint16_t val1, val;
#ifdef DEBUG_W5500
	uart_putc('T');
#endif
	do {
		val1 = W51_read16(addr);
#ifdef DEBUG_W5500
		uart_putc('a');
		uart_puthex4(val1);
#endif
		if (val1) {
			val = W51_read16(addr);
#ifdef DEBUG_W5500
			uart_putc('b');
			uart_puthex4(val);
#endif
		} else {
			val = 0;
		}
		if (val!=val1) {
#ifdef DEBUG_W5500
			uart_putp(PSTR("UH-OH-GO"));
#endif
			continue;
		}
	} while (0);
#ifdef DEBUG_W5500
	uart_putc('\n');
#endif
	return val;
}

void dhcplookup ();
void dnslookup (uint8_t* e2name, uint16_t e2len);

const uint8_t GETSTART[4] PROGMEM = "GET ";
const uint8_t GETSWID[10] PROGMEM = "&loaderid=";
const uint8_t GETSPACE[3] PROGMEM = "%20";
const uint8_t GETMCUSR[7] PROGMEM = "&mcusr=";
const uint8_t GETHOST[17] PROGMEM = " HTTP/1.0\r\nHost: ";
const uint8_t GETEND[4] PROGMEM = "\r\n\r\n";

uint8_t mymac[6], myip[4], mymask[4], gwip[4], dnsip[4], serverip[4];
uint8_t rdbuf[64];
uint16_t seed;

uint8_t attempt_bootload () {
	uint8_t* p;
	uint16_t addr;
	uint8_t i, c, ir, sr, sock;
    uint16_t path, pathlen;
	uint16_t avail, len;
     
#ifdef DEBUG_PHASES
	uart_putc('a');
#endif
	// Initialize W5500
	DDRB |= W5500_POWER;
	PORTB &= ~W5500_POWER; // Deactivate W5500 power supply
   	_delay_us(500); // Wait for W5500 to power down completely

#ifdef DEBUG_PHASES
	uart_putc('b');
#endif
	PORTB |= W5500_POWER; // Activate W5500 power supply
	_delay_ms(10); // Wait for W5500 to power up completely
#ifdef DEBUG_PHASES
	uart_putc('c');
#endif
    TCCR1B = 1<<CS10; // No prescaling, count at 8MHz
	// Await PHY LINK. This will take about 1-2 seconds
	while (!(W51_read(PHYCFGR)&PHYCFGR_LNK_ON)) {
#ifdef DEBUG_PROGRESS
		uart_putc('.');
		_delay_ms(10);
#endif
	}
    seed = TCNT1;
    TCCR1B = 0;
    TCNT1 = 0;
#ifdef DEBUG_PHASES
	uart_putc('d');
#endif
	
	for (i=0; i<sizeof(mymac); i++) {
    	mymac[i] = eeprom_read_byte(EEPROM_MAC+i);
	}
	for (i=0; i<sizeof(myip); i++) {
    	myip[i] = eeprom_read_byte(EEPROM_MYIP+i);
    	mymask[i] = eeprom_read_byte(EEPROM_MYMASK+i);
    	gwip[i] = eeprom_read_byte(EEPROM_GWIP+i);
    	dnsip[i] = eeprom_read_byte(EEPROM_DNSIP+i);
	}
    
   	W51_writeptr(SHAR,mymac,sizeof(mymac));

#ifdef DEBUG_PHASES
    uart_putc('e');
    uart_puthex2(myip[0]);
#endif
       
	if (myip[0]==0xFF) {
		dhcplookup();
	}

#ifdef DEBUG_PHASES
    uart_putc('f');
#endif

	// Establish IP, subnet mask and gateway
    W51_writeptr(SIPR,myip,4);
    W51_writeptr(SUBR,mymask,4);
    W51_writeptr(GAR,gwip,4);

#ifdef DEBUG_PHASES
    uart_putc('g');
#endif


	// Parse URL in eeprom to establish server part and path part
	path = 0;
	while (eeprom_read_byte(EEPROM_URL+path)!='/') path++;
	pathlen = 0;
	while (eeprom_read_byte(EEPROM_URL+path+pathlen)) pathlen++;
	// hostname starts at EEPROM_URL, length of hostname is path
	
	// Lookup serverip
	dnslookup(EEPROM_URL,path);

#ifdef DEBUG_PHASES
    uart_putc('h');
#endif
    
    wdt_reset();
	
	// Connect to server
    sock = 0;
	W51_write(Sn_MR(sock),Sn_MR_TCP/*|Sn_MR_ND*/);
	W51_writeptr(Sn_DIPR(sock),serverip,4);
	W51_write16(Sn_PORT(sock),45000);
	W51_write16(Sn_DPORT(sock),80);
#ifdef DEBUG_PHASES
	uart_putc('i');
#endif
	W51_exec(sock,Sn_CR_OPEN);
	while (W51_read(Sn_SR(sock)==SOCK_CLOSED)) {
#ifdef DEBUG_PROGRESS
		uart_putc('.');
#endif
		_delay_ms(10);
	}
#ifdef DEBUG_PHASES
	uart_putc('j');
#endif
	W51_exec(sock,Sn_CR_CONNECT);
	while (W51_read(Sn_SR(sock))!=SOCK_ESTABLISHED) {
#ifdef DEBUG_PROGRESS		
		uart_putc('.');
#endif
		_delay_ms(10);
		if (W51_read(Sn_IR(sock))&Sn_IR_TIMEOUT) {
			uart_putc('t');
			W51_write(Sn_IR(sock),Sn_IR_TIMEOUT);
			break;
		}
	}
	if (W51_read(Sn_SR(sock))==SOCK_ESTABLISHED) {
#ifdef DEBUG_PHASES
		uart_putc('k');
#endif

		addr = W51_read16(Sn_TX_WR(sock));
		addr = W51_sendmem(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,GETSTART,sizeof(GETSTART),MEM_FLASH);
		addr = W51_sendmem(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,EEPROM_URL+path,pathlen,MEM_EEPROM);
		addr = W51_sendmem(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,GETSWID,sizeof(GETSWID),MEM_FLASH);
        ir = 0;
        for (ir=0, i=0; ; i++) {
            c = pgm_read_byte_progmem(swid+i);
            if (c==0 || c==' ') {
                if (i>ir) {
                    addr = W51_sendmem(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,(uint8_t*)swid+ir,i-ir,MEM_FLASH);
                }
                if (c==0) break;
                addr = W51_sendmem(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,GETSPACE,sizeof(GETSPACE),MEM_FLASH);
                ir = i+1;
            }
        }
        addr = W51_sendmem(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,GETMCUSR,sizeof(GETMCUSR),MEM_FLASH);
        rdbuf[0] = hex(mcusr>>4);
        rdbuf[1] = hex(mcusr&0xF);
        addr = W51_send(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,rdbuf,2);
		addr = W51_sendmem(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,GETHOST,sizeof(GETHOST),MEM_FLASH);
		addr = W51_sendmem(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,EEPROM_URL,path,MEM_EEPROM);
		addr = W51_sendmem(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,GETEND,sizeof(GETEND),MEM_FLASH);
		W51_write16(Sn_TX_WR(sock),addr);
#ifdef DEBUG_PHASES
		uart_putc('l');
#endif
		W51_exec(sock,Sn_CR_SEND);
		while (W51_read(Sn_SR(sock))==SOCK_ESTABLISHED) {
			ir = W51_read(Sn_IR(sock));
			if (ir & Sn_IR_SENDOK) {
#ifdef DEBUG_PHASES
				uart_putc('m');
#endif
				W51_write(Sn_IR(sock),Sn_IR_SENDOK);
				break;
			}
			if (ir & Sn_IR_TIMEOUT) {
				uart_putc('t');
				W51_write(Sn_IR(sock),Sn_IR_TIMEOUT);
				break;
			}
#ifdef DEBUG_PROGRESS
			uart_putc('.');
#endif
			_delay_ms(10);
		}
	}
    
    wdt_reset();
    
    mem = MEM_FLASH;
	process_start();
#ifdef DEBUG_PHASES
	uart_putc('n');
#endif
	while (1) {
		// uint16_t availbug = W51_read16twice(Sn_RX_RSR(sock));
		addr = W51_read16(Sn_RX_RD(sock));
		avail = W51_read16(Sn_RX_WR(sock))-addr;
		sr = W51_read(Sn_SR(sock));
		if (sr!=SOCK_ESTABLISHED && sr!=SOCK_CLOSE_WAIT) break;
		if (avail) {
#ifdef DEBUG_W5500
			uart_putc('r');
			uart_puthex2(sr);
			uart_putc(',');
			uart_puthex4(avail);
			// uart_putc(',');
			// uart_puthex4(availbug);
			uart_putc(',');
			uart_puthex4(addr);
			uart_putc(':');
#endif
            while (avail) {
                len = avail>sizeof(rdbuf)?sizeof(rdbuf):avail;
 			    W51_recv(addr,WIZCHIP_RXBUF_BLOCK(sock)<<3,rdbuf,len);
                addr += len;
                avail -= len;
    			p = rdbuf;
    			while (len--) {
#ifdef DEBUG_RECEIVE
        			uart_putc(*p);
#endif
        			process_hex(*p++);
    			}
            }
			W51_write16(Sn_RX_RD(sock),addr);
			W51_exec(sock,Sn_CR_RECV);
		} else if (sr==SOCK_CLOSE_WAIT) {
			break;
		} else {
#ifdef DEBUG_PROGRESS
			uart_putc('.');
#endif
			_delay_ms(10);
		}
	}
	process_end();
#ifdef DEBUG_PHASES
	uart_putc('o');
#endif
	if (sr==SOCK_CLOSE_WAIT) {
#ifdef DEBUG_PHASES
		uart_putc('p');
#endif
		W51_exec(sock,Sn_CR_DISCON);
		while (W51_read(Sn_SR(sock))!=SOCK_CLOSED) {
			if (W51_read(Sn_IR(sock))&Sn_IR_TIMEOUT) {
#ifdef DEBUG_PHASES
				uart_putc('t');
#endif
				W51_write(Sn_IR(sock),Sn_IR_TIMEOUT);
				W51_exec(sock,Sn_CR_CLOSE);
				W51_write(Sn_IR(sock),0xFF);
				break;
			}
#ifdef DEBUG_PROGRESS
			uart_putc('.');
#endif
			_delay_ms(10);
		}
	} else {
#ifdef DEBUG_PHASES
		uart_putc('q');
#endif
		W51_exec(sock,Sn_CR_CLOSE);
		W51_write(Sn_IR(sock),0xFF);
	}
#ifdef DEBUG_PHASES
	uart_putc('z');
#endif
#ifdef DEBUG_PHASES
  	if (state==STATE_END) {
       	uart_putc('@');
   	}
#endif
    return state==STATE_END;
}

struct udp_hdr {
    uint8_t ip[4];
    uint16_t port;
    uint16_t dlen;
};

#define UDP ((struct udp_hdr*)p)

#define DHCP_CLIENT_PORT	68	/* from client to server */
#define	DHCP_SERVER_PORT	67	/* from server to client */

/* DHCP op */
#define DHCP_BOOTREQUEST    1
#define DHCP_BOOTREPLY      2

#define DHCP_HTYPEETHERNET  1
#define DHCP_HLENETHERNET	6
#define DHCP_HOPS           0
#define DHCP_FLAGS1_BCAST	0x80

#define DHCP_MAGIC_COOKIE_0 0x63
#define DHCP_MAGIC_COOKIE_1 0x82
#define DHCP_MAGIC_COOKIE_2 0x53
#define DHCP_MAGIC_COOKIE_3 0x63

#define DHCP_OPTPAD                 0
#define DHCP_OPT_SUBNET             1
#define DHCP_OPT_TIMEROFFSET        2
#define DHCP_OPT_ROUTERSONSUBNET    3
#define DHCP_OPT_DNS                6
#define DHCP_OPT_HOSTNAME           12
#define DHCP_OPT_DOMAINNAME         15
#define DHCP_OPT_REQUESTEDIPADDR    50
#define DHCP_OPT_IPADDRLEASETIME    51
#define DHCP_OPT_MESSAGETYPE        53
#define DHCP_OPT_SERVERIDENTIFIER   54
#define DHCP_OPT_PARAMREQUEST       55
#define DHCP_OPT_T1VALUE            58
#define DHCP_OPT_T2VALUE            59
#define DHCP_OPT_CLIENTIDENTIFIER   61
#define DHCP_OPTEND                 255

/* DHCP OPT MESSAGETYPE */
#define	DHCP_DISCOVER               1
#define DHCP_OFFER                  2
#define DHCP_CLIENTIDENT_TYPE_MAC   1

#define DHCP_RECEIVE_TIMEOUT        1000  // ms

#define DHCP_MYIP_DONE              0x01
#define DHCP_MSGTYP_DONE            0x02
#define DHCP_DHCPIP_DONE            0x04
#define DHCP_MYMASK_DONE            0x08
#define DHCP_GWIP_DONE              0x10
#define DHCP_DNSIP_DONE             0x20
#define DHCP_ALL_DONE               0x3F


struct dhcp_hdr {
    uint8_t op;
    uint8_t htype;
    uint8_t hlen;
    uint8_t hops;
    uint16_t xidh, xidl;
    uint16_t secs;
    uint8_t flags1;
    uint8_t flags2;
    uint8_t ciaddr[4];
    uint8_t yiaddr[4];
    uint8_t siaddr[4];
    uint8_t giaddr[4];
    // uint8_t chaddr[16];
    // uint8_t sname[64];
    // uint8_t file[128];
    // uint8_t options[];
};

struct dhcp_opt {
    uint8_t opt;
    uint8_t len;    // For all options except OPTPAD and OPTEND
    uint8_t data[]; // data[len];
};

void dhcplookup () {
	// Establish IP, subnet mask and gateway via DHCP
	// Create UDP socket
	uint8_t sock = 0, i, done = 0;
    uint8_t *p;
    uint16_t xidh=0, xidl=seed, avail, timeout;
#define cookie mymask
    struct dhcp_opt opt;

#ifdef DEBUG_DHCP_VERBOSE
    uart_putp(PSTR("dhcplookup"));
#endif

    W51_write(Sn_MR(sock),Sn_MR_UDP);
	W51_write16(Sn_PORT(sock),DHCP_CLIENT_PORT);
	W51_exec(sock,Sn_CR_OPEN);

#ifdef DEBUG_DHCP
    uart_putc('a');
#endif
   
    uint8_t dhcpip[4];
    dhcpip[0] = dhcpip[1] = dhcpip[2] = dhcpip[3] = 0xFF;
	W51_writeptr(Sn_DIPR(sock),dhcpip,sizeof(dhcpip));
	W51_write16(Sn_DPORT(sock),DHCP_SERVER_PORT);

    
    while (done!=DHCP_ALL_DONE) {
#ifdef DEBUG_DHCP
        uart_putc('b');
#endif

        p = spmdata;
#define HDR ((struct dhcp_hdr*)p)
	    for (i=0; i<sizeof(*HDR); i++) p[i] = 0;
        HDR->op = DHCP_BOOTREQUEST;
        HDR->htype = DHCP_HTYPEETHERNET;
        HDR->hlen = DHCP_HLENETHERNET;
        HDR->hops = DHCP_HOPS;
        HDR->xidh = xidh; // No HTONS here, no NTOHS later
        HDR->xidl = ++xidl; // No HTONS here, no NTOHS later
        HDR->flags1 = DHCP_FLAGS1_BCAST;
        addr = W51_read16(Sn_TX_WR(sock));
	    addr = W51_send(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,spmdata,sizeof(*HDR));
        addr = W51_send(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,NULL,16+64+128);
        // options
        *p++ = DHCP_MAGIC_COOKIE_0;
        *p++ = DHCP_MAGIC_COOKIE_1;
        *p++ = DHCP_MAGIC_COOKIE_2;
        *p++ = DHCP_MAGIC_COOKIE_3;
#define OPT ((struct dhcp_opt*)p)
        OPT->opt = DHCP_OPT_MESSAGETYPE;
        OPT->len = 1;
        OPT->data[0] = DHCP_DISCOVER;
        p += sizeof(*OPT)+OPT->len;
        OPT->opt = DHCP_OPT_CLIENTIDENTIFIER;
        OPT->len = 1+sizeof(mymac);
        OPT->data[0] = DHCP_CLIENTIDENT_TYPE_MAC;
        for (i=0; i<sizeof(mymac); i++) {
            OPT->data[i+1] = mymac[i];
        }
        p += sizeof(*OPT)+OPT->len;
        OPT->opt = DHCP_OPT_PARAMREQUEST;
        OPT->len = 3;
        OPT->data[0] = DHCP_OPT_SUBNET;
        OPT->data[1] = DHCP_OPT_ROUTERSONSUBNET;
        OPT->data[2] = DHCP_OPT_DNS;
        p += sizeof(*OPT)+OPT->len;
        *p++ = DHCP_OPTEND;
	    addr = W51_send(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,spmdata,p-spmdata);
        W51_write16(Sn_TX_WR(sock),addr);
	    W51_exec(sock,Sn_CR_SEND);

#ifdef DEBUG_DHCP
        uart_putc('c');
#endif
        for (timeout=0; done!=DHCP_ALL_DONE && timeout<DHCP_RECEIVE_TIMEOUT/10; timeout++) {
            _delay_ms(10);
            addr = W51_read16(Sn_RX_RD(sock));
            esar = W51_read16(Sn_RX_WR(sock));
            avail = esar-addr;
            if (!avail) {
#ifdef DEBUG_PROGRESS
                uart_putc('.');
#endif
                continue;
            }
#ifdef DEBUG_DHCP_VERBOSE
            uart_putp(PSTR(" rxrd:"));
            uart_puthex4(addr);
            uart_putp(PSTR(" rxwr:"));
            uart_puthex4(esar);
            uart_putp(PSTR(" avail:"));
            uart_puthex4(avail);
            uart_putc(' ');
#endif
            done = 0;
            if (avail>=sizeof(*UDP)+sizeof(*HDR)+16+64+128+sizeof(cookie)) {
                avail -= sizeof(*UDP)+sizeof(*HDR)+16+64+128+sizeof(cookie);
                addr = W51_recv(addr,WIZCHIP_RXBUF_BLOCK(sock)<<3,p=spmdata,sizeof(*UDP)+sizeof(*HDR));
#ifdef DEBUG_DHCP_VERBOSE
                for (uint16_t i=0; i<sizeof(*UDP)+sizeof(*HDR); i++) uart_puthex2(p[i]);
#endif
                p += sizeof(*UDP);
#ifdef DEBUG_DHCP_VERBOSE
                uart_putp(PSTR(" HDR->op:"));
                uart_puthex2(HDR->op);
                uart_putp(PSTR(" HDR->xidh:"));
                uart_puthex4(HDR->xidh);
                uart_putp(PSTR(" xidh:"));
                uart_puthex4(xidh);
                uart_putp(PSTR(" HDR->xidl:"));
                uart_puthex4(HDR->xidl);
                uart_putp(PSTR(" xidl:"));
                uart_puthex4(xidl);
                uart_putc('\n');
#endif
                if (HDR->op==DHCP_BOOTREPLY && HDR->xidh==xidh && HDR->xidl==xidl) {
#ifdef DEBUG_DHCP
                    uart_putc('h');
#endif
                    addr = W51_recv(addr,WIZCHIP_RXBUF_BLOCK(sock)<<3,NULL,16+64+128);
                    addr = W51_recv(addr,WIZCHIP_RXBUF_BLOCK(sock)<<3,cookie,sizeof(cookie));
                    if (cookie[0]==DHCP_MAGIC_COOKIE_0 && cookie[1]==DHCP_MAGIC_COOKIE_1
                            && cookie[2]==DHCP_MAGIC_COOKIE_2 && cookie[3]==DHCP_MAGIC_COOKIE_3) {
#ifdef DEBUG_DHCP
                        uart_putc('c');
#endif
                        done |= DHCP_MYIP_DONE;
#ifdef DEBUG_DHCP_VERBOSE
                        uart_puthex2(done);
#endif
                    }
                }
            }
            
            if (done) {
                for (i=0; i<sizeof(myip); i++) {
                    myip[i] = HDR->yiaddr[i];
                }
#ifdef DEBUG_DHCP
                uart_putc('i');
#endif
                while (avail) {
#ifdef DEBUG_DHCP_VERBOSE
                    uart_puthex4(avail);
#endif
                    addr = W51_recv(addr,WIZCHIP_RXBUF_BLOCK(sock)<<3,&opt.opt,1);
                    avail--;
#ifdef DEBUG_DHCP_VERBOSE
                    uart_puthex2(opt.opt);
#endif
                    if (opt.opt==DHCP_OPTPAD || opt.opt==DHCP_OPTEND) continue;
                    if (!avail) break; // Error case
                    addr = W51_recv(addr,WIZCHIP_RXBUF_BLOCK(sock)<<3,&opt.len,1);
                    avail--;
#ifdef DEBUG_DHCP_VERBOSE
                    uart_puthex2(opt.len);
#endif
                    if (avail<opt.len) { // Error case
                        avail = 0;
                        break;
                    }
                    addr = W51_recv(addr,WIZCHIP_RXBUF_BLOCK(sock)<<3,p=spmdata,opt.len);
                    avail -= opt.len;
                    switch (opt.opt) {
                    case DHCP_OPT_MESSAGETYPE:
                        if (opt.len==1 && p[0]==DHCP_OFFER) {
                            done |= DHCP_MSGTYP_DONE;
#ifdef DEBUG_DHCP
                            uart_putc('t');
#endif
                        }
                        break;
                    case DHCP_OPT_SERVERIDENTIFIER:
                        if (opt.len==4) {
                            for (i=0; i<sizeof(dhcpip); i++) {
                                dhcpip[i] = p[i];
                            }
                            done |= DHCP_DHCPIP_DONE;
#ifdef DEBUG_DHCP
                            uart_putc('s');
#endif
                        }
                        break;
                    case DHCP_OPT_SUBNET:
                        if (opt.len==4) {
                            for (i=0; i<sizeof(mymask); i++) {
                                mymask[i] = p[i];
                            }
                            done |= DHCP_MYMASK_DONE;
#ifdef DEBUG_DHCP
                            uart_putc('m');
#endif
                        }
                        break;
                    case DHCP_OPT_ROUTERSONSUBNET:
                        if (opt.len==4) {
                            for (i=0; i<sizeof(gwip); i++) {
                                gwip[i] = p[i];
                            }
                            done |= DHCP_GWIP_DONE;
#ifdef DEBUG_DHCP
                            uart_putc('g');
#endif
                        }
                        break;
                    case DHCP_OPT_DNS:
                        if (opt.len==4) {
                            for (i=0; i<sizeof(dnsip); i++) {
                                dnsip[i] = p[i];
                            }
                            done |= DHCP_DNSIP_DONE;
#ifdef DEBUG_DHCP
                            uart_putc('d');
#endif
                        }
                        break;
                    }
                }                
            }
            W51_write16(Sn_RX_RD(sock),esar);
            W51_exec(sock,Sn_CR_RECV);
        }        
    
#ifdef DEBUG_DHCP
    	uart_putc('x');
#endif
		
        // We need our myip only temporarily, so do NOT send DHCP_REQUEST nor wait for DHCP_ACK
    }    

    W51_exec(sock,Sn_CR_CLOSE);
	W51_write(Sn_IR(sock),0xFF);
    
#ifdef DEBUG_DHCP
    uart_putc('\n');
#endif
}
#undef HDR
#undef OPT

#define DNS_RECEIVE_TIMEOUT       1000  // ms

/** DNS field TYPE used for "Resource Records" */
#define DNS_RRTYPE_A              1     /* a host address */

/** DNS field CLASS used for "Resource Records" */
#define DNS_RRCLASS_IN            1     /* the Internet */
#define DNS_RRCLASS_FLUSH         0x800 /* Flush bit */

#define DNS_SERVER_PORT           53
#define DNS_CLIENT_PORT           53

/* DNS protocol flags */
#define DNS_FLAG1_QUERY			  0x00
#define DNS_FLAG1_RESPONSE        0x80
#define DNS_FLAG1_TRUNC           0x02
#define DNS_FLAG1_RD              0x01
#define DNS_FLAG2_ERR_MASK        0x0f

#define DNS_LABELCOMPRESSION      0xC0

/** DNS message header */
struct dns_hdr {
	uint16_t id;
	uint8_t flags1;
	uint8_t flags2;
	uint16_t numquestions;
	uint16_t numanswers;
	uint16_t numauthrr;
	uint16_t numextrarr;
};

/** DNS query message structure */
struct dns_qry {
  /* DNS query record starts with either a domain name or a pointer
     to a name already present somewhere in the packet. */
  uint16_t type;
  uint16_t cls;
};

/** DNS answer message structure */
struct dns_answer {
  /* DNS answer record starts with either a domain name or a pointer
     to a name already present somewhere in the packet. */
  uint16_t type;
  uint16_t cls;
  uint32_t ttl;
  uint16_t len;
};

void dnslookup (uint8_t* e2name, uint16_t e2len) {
	uint8_t sock = 0;
    uint16_t hdrid = 0;
    
	W51_write(Sn_MR(sock),Sn_MR_UDP);
	W51_write16(Sn_PORT(sock),DNS_CLIENT_PORT);
	W51_exec(sock,Sn_CR_OPEN);

	W51_writeptr(Sn_DIPR(sock),dnsip,4);
	W51_write16(Sn_DPORT(sock),DNS_SERVER_PORT);

    uint8_t *p, *pc, *name, c;
    uint16_t len, dlen, alen, timeout, nq, na;
    
    serverip[0] = 0xFF;
    while (serverip[0]==0xFF) {
    	p = spmdata;
#define HDR ((struct dns_hdr*)p)
	    for (c=0; c<sizeof(*HDR); c++) p[c] = 0;
        hdrid++;
	    HDR->id = HTONS(hdrid);
	    HDR->flags1 = DNS_FLAG1_QUERY|DNS_FLAG1_RD;
	    HDR->numquestions = HTONS(1);
	    p += sizeof(*HDR);
	    pc = p;
	    len = e2len+1;
        name = e2name;
	    while (len--) {
		    c = len?eeprom_read_byte(name++):'.';
		    if (c!='.') {
			    *++pc = c;
		    } else if (pc!=p) {
			    *p = pc-p;
			    p = ++pc;
		    }
	    }
	    *p++ = 0;
#define QRY ((struct dns_qry*)p)
	    QRY->type = HTONS(DNS_RRTYPE_A);
	    QRY->cls = HTONS(DNS_RRCLASS_IN);
	    p += sizeof(*QRY);

        addr = W51_read16(Sn_TX_WR(sock));
        addr = W51_send(addr,WIZCHIP_TXBUF_BLOCK(sock)<<3,spmdata,p-spmdata);
        W51_write16(Sn_TX_WR(sock),addr);
       	W51_exec(sock,Sn_CR_SEND);

        // Now wait for answer
        for (timeout=0; serverip[0]==0xFF && timeout<DNS_RECEIVE_TIMEOUT/10; timeout++) {
            _delay_ms(10);
            addr = W51_read16(Sn_RX_RD(sock));
            len  = W51_read16(Sn_RX_WR(sock))-addr;
            if (!len) {
#ifdef DEBUG_PROGRESS
                uart_putc('.');
#endif
                continue;
            }
#ifdef DEBUG_DNS
            uart_putc('u');
            uart_puthex2(len);
            uart_putc(':');
#endif
            if (len>sizeof(spmdata)) len = sizeof(spmdata);
            W51_recv(addr,WIZCHIP_RXBUF_BLOCK(sock)<<3,spmdata,len);
            W51_write16(Sn_RX_RD(sock),W51_read16(Sn_RX_WR(sock)));
            W51_exec(sock,Sn_CR_RECV);
#ifdef DEBUG_DNS
            uart_putc('\n');
            for (dlen=0; dlen<len; dlen++) uart_puthex2(spmdata[dlen]);
            uart_putc('\n');
#endif
            p = spmdata;
            if (len<sizeof(*UDP)) continue;
            if (NTOHS(UDP->port)!=DNS_SERVER_PORT) continue;
            dlen = NTOHS(UDP->dlen);
            // if (dlen>len-8) continue; Even if there is more data than we could read, lets try to find an answer in what we got ...
            p = spmdata+8;
            if (dlen<sizeof(*HDR)) continue;
            if (NTOHS(HDR->id)!=hdrid) continue;
            if (!(HDR->flags1 & DNS_FLAG1_RESPONSE) || (HDR->flags1 & DNS_FLAG1_TRUNC) || (HDR->flags2 & DNS_FLAG2_ERR_MASK)) continue;
            nq = NTOHS(HDR->numquestions);
            na = NTOHS(HDR->numanswers);
            if (!na) continue;
            p += sizeof(*HDR);
#ifdef DEBUG_DNS
            uart_putc('h');
            uart_puthex2((p-spmdata)>>8);
            uart_puthex2((p-spmdata)&0xFF);
#endif
            
            // Skip questions.
            while (nq) {
#ifdef DEBUG_DNS
                uart_putc('Q');
                uart_puthex2(nq);
#endif
                nq--;
                while (p<spmdata+len && (c = *p++)) {
#ifdef DEBUG_DNS
                    uart_putc('L');
                    uart_puthex2(c);
#endif
                    while (c--) {
#ifdef DEBUG_DNS
                        uart_putc(*p);
#endif
                        p++;
                    }                    
                }
                p += sizeof(*QRY);
#ifdef DEBUG_DNS
                uart_puthex2((p-spmdata)>>8);
                uart_puthex2((p-spmdata)&0xFF);
#endif
            }
#ifdef DEBUG_DNS
            uart_putc('q');
#endif
            // Now we're up to the bit we're interested in, the answer
            // There might be more than one answer (although we'll just use the first
            // type A answer) and some authority and additional resource records but
            // we're going to ignore all of them.
            while (na) {
#ifdef DEBUG_DNS
                uart_putc('A');
                uart_puthex2(na);
                uart_putc(':');
                uart_puthex2((p-spmdata)>>8);
                uart_puthex2((p-spmdata)&0xFF);
                uart_putc('-');
#endif
                na--;
                // Skip the label
                c = *p++;
                if (c & DNS_LABELCOMPRESSION) {
                    c = 1;
                }
                while (c--) p++;
#ifdef DEBUG_DNS
                uart_puthex2((p-spmdata)>>8);
                uart_puthex2((p-spmdata)&0xFF);
#endif
#define ANS ((struct dns_answer*)p)
                alen = NTOHS(ANS->len);
                if (p+sizeof(*ANS)+alen>spmdata+len) break;
                if (ANS->type==HTONS(DNS_RRTYPE_A) && ANS->cls==HTONS(DNS_RRCLASS_IN) && alen==4) {
                    p += sizeof(*ANS);
#ifdef DEBUG_DNS
                    uart_puthex2(p[0]);
                    uart_puthex2(p[1]);
                    uart_puthex2(p[2]);
                    uart_puthex2(p[3]);
                    uart_putc('!');
#endif
                    serverip[0] = *p++;
                    serverip[1] = *p++;
                    serverip[2] = *p++;
                    serverip[3] = *p++;
                    break;
                }
                p += len;
            }
        }                    
    }    
	W51_exec(sock,Sn_CR_CLOSE);
	W51_write(Sn_IR(sock),0xFF);
}

