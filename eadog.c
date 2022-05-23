#include <string.h>
#include "qconfig.h"
#include "eadog.h"
#include "ringbufs.h"

volatile struct spi_link_type spi_link;
struct ringBufS_t ring_buf1;

static const spi1_configuration_t spi1_configuration[] = {   
    { 0x2, 0x40, 0x0, 0x2, 0 }
};

static void send_lcd_cmd_long(uint8_t); // for display init only
static void send_lcd_data(uint8_t);
static void send_lcd_cmd(uint8_t);
bool SPI1_Config(void);

bool SPI1_Config(void)
{
	if (!SPI1CON0bits.EN) {
		SPI1CON0 = spi1_configuration[0].con0;
		SPI1CON1 = spi1_configuration[0].con1;
		SPI1CON2 = spi1_configuration[0].con2 | (_SPI1CON2_SPI1RXR_MASK | _SPI1CON2_SPI1TXR_MASK);
		SPI1CLK = 0x00;
		SPI1BAUD = spi1_configuration[0].baud;
		TRISAbits.TRISA4 = spi1_configuration[0].operation;
		SPI1CON0bits.EN = 1;
		return true;
	}
	return false;
}

/*
 * Init the NHD-0420D3Z-NSW-BBW-V3 in 8-bit serial mode
 * channel 1 DMA
 */
bool init_display(void)
{	
	SPI1_Close();
	if (!SPI1_Config()) {
		return false;
	};
	spi_link.tx1a = &ring_buf1;
	ringBufS_init(spi_link.tx1a);
#ifdef DEBUG_DISP2
	DLED2 = true;
#endif
#ifdef NHD
	send_lcd_cmd(0x46); // home cursor
	wdtdelay(800);
	send_lcd_cmd(0x41); // display on
	wdtdelay(80);
	send_lcd_cmd(0x53); // set back-light level
	send_lcd_data(NHD_BL_LOW);
	wdtdelay(80);
	send_lcd_cmd(0x51); // clear screen
	wdtdelay(800);
#else
#endif

#ifdef DEBUG_DISP2
	DLED2 = false;
#endif
#ifdef USE_DMA
	DMA1_SetSCNTIInterruptHandler(clear_lcd_done);
#endif
	return true;
}

#ifdef NHD

/*
 * R2 short on LCD NHD-0420D3Z-NSW-BBW-V3 board
 */

static void send_lcd_data(const uint8_t data)
{
	CS_SetLow();
	SPI1_ExchangeByte(data);
	wdtdelay(8);
}

static void send_lcd_cmd(const uint8_t cmd)
{
	CS_SetLow();
	SPI1_ExchangeByte(NHD_CMD);
	wdtdelay(8);
	SPI1_ExchangeByte(cmd);
	wdtdelay(8);
}

static void send_lcd_cmd_long(const uint8_t cmd)
{
	CS_SetLow();
	SPI1_ExchangeByte(NHD_CMD);
	wdtdelay(8);
	SPI1_ExchangeByte(cmd);
	wdtdelay(800);
}

/*
 * CAN use DMA channel 1 for transfers
 */
void eaDogM_WriteString(char *strPtr)
{
	uint8_t len = (uint8_t) strlen(strPtr);

#ifdef DEBUG_DISP1
	DLED1 = true;
#endif
	wait_lcd_done();
	wait_lcd_set();
	/* reset buffer for DMA */
	ringBufS_flush(spi_link.tx1a, false);
	CS_SetLow(); /* SPI select display */
	if (len > (uint8_t) max_strlen) {
		len = max_strlen;
	}
	ringBufS_put_dma_cpy(spi_link.tx1a, strPtr, len);
#ifdef USE_DMA
	DMA1_SetSourceAddress((uint24_t) spi_link.tx1a);
	DMA1_SetSourceSize(len);
	DMA1_SetDestinationSize(1);
#else
	SPI1_ExchangeBlock(spi_link.tx1a, len);
#endif
	start_lcd(); // start DMA transfer
#ifdef DISPLAY_SLOW
	wdtdelay(9000);
#endif
#ifdef DEBUG_DISP1
	DLED1 = false;
#endif
#ifndef USE_DMA
	wait_lcd_done();
#endif
}

/*
 * uses DMA channel 1 for transfers
 */
void send_lcd_cmd_dma(const uint8_t strPtr)
{
	wait_lcd_done();
	send_lcd_data_dma(NHD_CMD); //prefix
	wait_lcd_done();
	send_lcd_data_dma(strPtr); // cmd code
	wait_lcd_done();
}

/*
 * uses DMA channel 1 for transfers
 */
void send_lcd_data_dma(const uint8_t strPtr)
{
#ifdef DEBUG_DISP2
	DLED2 = true;
#endif
	wait_lcd_set();
	/* reset buffer for DMA */
	ringBufS_flush(spi_link.tx1a, false);
	CS_SetLow(); /* SPI select display */
	ringBufS_put_dma(spi_link.tx1a, strPtr); // don't use printf to send zeros
#ifdef USE_DMA
	DMA1_SetSourceAddress((uint24_t) spi_link.tx1a);
	DMA1_SetSourceSize(1);
	DMA1_SetDestinationSize(1);
#endif
	start_lcd(); // start DMA transfer
#ifdef DEBUG_DISP2
	DLED2 = false;
#endif
}

void eaDogM_WriteStringAtPos(const uint8_t r, const uint8_t c, char *strPtr)
{
	uint8_t row;

	switch (r) {
	case 1:
		row = 0x40;
		break;
	case 2:
		row = 0x14;
		break;
	case 3:
		row = 0x54;
		break;
	case 0:
		row = 0x00;
		break;
	default:
		row = 0x00;
		break;
	}
	
	return;  // DEBUG
	
	send_lcd_cmd(0x45);
	send_lcd_data(row + c);
	wait_lcd_done();
	CS_SetHigh(); /* SPI deselect display */
	eaDogM_WriteString(strPtr);
}

void eaDogM_WriteIntAtPos(uint8_t r, uint8_t c, uint8_t i)
{

}

void eaDogM_SetPos(const uint8_t r, const uint8_t c)
{

}

void eaDogM_ClearRow(const uint8_t r)
{

}

void eaDogM_WriteByteToCGRAM(uint8_t ndx, uint8_t data)
{

}

#else
#endif

void eaDogM_WriteCommand(const uint8_t cmd)
{
	send_lcd_cmd(cmd);
}

void eaDogM_WriteChr(const int8_t value)
{
	send_lcd_data((uint8_t) value);
}

/*
 * Trigger the SPI DMA transfer to the LCD display
 */
void start_lcd(void)
{
#ifdef USE_DMA
	DMA1_StartTransfer();
#endif
}

void wait_lcd_set(void)
{
	spi_link.LCD_DATA = true;
}

bool wait_lcd_check(void)
{
	return spi_link.LCD_DATA;
}

void wait_lcd_done(void)
{
#ifdef USE_DMA
	while ((bool) spi_link.LCD_DATA) {
	};
#endif
	while (!SPI1STATUSbits.TXBE) {
	};
	CS_SetHigh(); /* SPI deselect display */
}

void clear_lcd_done(void)
{
	spi_link.LCD_DATA = false;
}

void spi_rec_done(void)
{
#ifdef USE_DMA
#endif
}