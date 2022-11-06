/*!
 * @file    Adafruit_ILI9341.h
 * @brief   Driver code for Adafruit ILI9341 TFT LCD with Touchscreen Breakout Board
 *          (https://www.adafruit.com/product/1770)
 * @ref     Initialization sequence and large part of defines taken from Adafruits github
 *          repository for this device (https://github.com/adafruit/Adafruit_ILI9341)
 * @note    This code is written with intention of using SPI to communicate to the
 *          STM32 L031K6 board and uses the following pinout on the display:
 *
 *          PINOUT  LABEL           PORT/PIN
 *          --------------------------------
 *          CLK     SPI1_SCLK       PA_5
 *          MISO    SPI1_MISO       PA_6
 *          MOSI    SPI1_MOSI       PA_7
 *          CS      GPIO_OUTPUT     PA_4
 *          D/C     GPIO_OUTPUT     PA_1
 *
 * @author  Miles Hanbury (mhanbury)
 * @author  James Kelly (jkellymi)
 * @author  Joshua Nye (nyej)
 */

/* ----------------------------------------- Includes ------------------------------------------ */
#include "stm32l1xx_hal.h"// provides defintions for SPI/GPIO types
#include <string.h>                                                                                 // provides string functions
#include <stdlib.h>
#include <stdio.h>

/* ------------------------------------- Display Parameters ------------------------------------ */
#define ILI9341_HEIGHT 320
#define ILI9341_WIDTH 480
#define ILI9341_TXTBOX_HEIGHT 320
#define ILI9341_TXTBOX_WIDTH 480

/* ------------------------------------ Level 1 Command Set ------------------------------------ */
// Page 83
#define ILI9341_NOOP                0x00    // no operation
#define ILI9341_SW_RESET            0x01    // software reset
#define ILI9341_RD_ID               0x04    // read display identification information
#define ILI9341_RD_STS              0x09    // read display status
#define ILI9341_RD_PWR_MODE         0x0A    // read display power mode
#define ILI9341_RD_MADCTL           0x0B    // read display MADCTL
#define ILI9341_RD_PX_FORMAT        0x0C    // read display pixel format
#define ILI9341_RD_IMG_FORMAT       0x0D    // read display image format
#define ILI9341_RD_SIG_MODE         0x0E    // read display signal mode
#define ILI9341_RD_DIAG_RES         0x0F    // read display self-diagnostic result
#define ILI9341_SLEEP_IN            0x10    // enter sleep mode
#define ILI9341_SLEEP_OUT           0x11    // sleep out
#define ILI9341_PART_ON             0x12    // partial mode on
#define ILI9341_NORM_ON             0x13    // normal display mode on
#define ILI9341_INV_OFF             0x20    // display inversion off
#define ILI9341_INV_ON              0x21    // display inversion on
#define ILI9341_GAMMA_SET           0x26    // gamma set
#define ILI9341_DISP_OFF            0x28    // display off
#define ILI9341_DISP_ON             0x29    // display on
#define ILI9341_COL_ADDR_SET        0x2A    // column address set
#define ILI9341_PG_ADDR_SET         0x2B    // page address set
// Page 84
#define ILI9341_MEM_W               0x2C    // memory write
#define ILI9341_COLOR_SET           0x2D    // color set
#define ILI9341_MEM_R               0x2E    // memory read
#define ILI9341_PART_AREA           0x30    // partial area
#define ILI9341_VERT_SCRL_DEF       0x33    // vertical scrolling definition
#define ILI9341_TEAR_EFF_OFF        0x34    // tearing effect line off
#define ILI9341_TEAR_EFF_ON         0x35    // tearing effect line on
#define ILI9341_MEM_ACS_CTRL        0x36    // memory access control
#define ILI9341_VERT_SCRL_START     0x37    // vertical scrolling start address
#define ILI9341_IDLE_MODE_OFF       0x38    // idle mode off
#define ILI9341_IDLE_MODE_ON        0x39    // idle mode on
#define ILI9341_PX_FORMAT_SET       0x3A    // pixel format set
#define ILI9341_W_MEM_CONT          0x3C    // write memory continue
#define ILI9341_R_MEM_CONT          0x3E    // read memory continue
#define ILI9341_SET_TEAR_SCANLINE   0x44    // set tear scanline
#define ILI9341_GET_SCANLINE        0x45    // get scanline
#define ILI9341_W_DISP_BRGHT        0x51    // write display brightness
// Page 85
#define ILI9341_R_DISP_BRGHT        0x52    // read display brightness
#define ILI9341_W_CTRL_DISP         0x53    // write CTRL diplay
#define ILI9341_R_CTRL_DISP         0x54    // read CTRL display
#define ILI9341_W_CABC              0x55    // write content adaptive brightness control
#define ILI9341_R_CABC              0x56    // read content adaptive brightness control
#define ILI9341_W_CABC_MIN          0x5E    // write CABC minimum brightness
#define ILI9341_R_CABC_MIN          0x5F    // read CABC minimum brightness
#define ILI9341_R_ID1               0xDA    // read ID1
#define ILI9341_R_ID2               0xDB    // read ID2
#define ILI9341_R_ID3               0xDC    // read ID3

/* ------------------------------------ Level 2 Command Set ------------------------------------ */
// Page 85
#define ILI9341_RGB_ISC             0xB0    // RGB interface signal control
#define ILI9341_FRAME_CTRL_NORM     0xB1    // frame control (in normal mode)
#define ILI9341_FRAME_CTRL_IDLE     0xB2    // frame control (in idle mode)
#define ILI9341_FRAME_CTRL_PART     0xB3    // frame control (in partial mode)
#define ILI9341_DISP_INV_CTRL       0xB4    // display inversion control
#define ILI9341_BLANK_PORCH_CTRL    0xB5    // blanking porch control
// Page 86
#define ILI9341_DISP_FUNC_CTRL      0xB6    // display function control
#define ILI9341_ENTRY_MODE_SET      0xB7    // entry mode set
#define ILI9341_BKLT_CTRL1          0xB8    // backlight control 1
#define ILI9341_BKLT_CTRL2          0xB9    // backlight control 2
#define ILI9341_BKLT_CTRL3          0xBA    // backlight control 3
#define ILI9341_BKLT_CTRL4          0xBB    // backlight control 4
#define ILI9341_BKLT_CTRL5          0xBC    // backlight control 5
#define ILI9341_BKLT_CTRL7          0xBE    // backlight control 7
#define ILI9341_BKLT_CTRL8          0xBF    // backlight control 8
#define ILI9341_PWR_CTRL1           0xC0    // power control 1
#define ILI9341_PWR_CTRL2           0xC1    // power control 2
#define ILI9341_VCOM_CTRL1          0xC5    // VCOM control 1
#define ILI9341_VCOM_CTRL2          0xC7    // VCOM control 2
#define ILI9341_NV_MEM_W            0xD0    // NV memory write
#define ILI9341_NV_MEM_PK           0xD1    // NV memory protection key
#define ILI9341_NV_MEM_STS_R        0xD2    // NV memory status read
// Page 87
#define ILI9341_R_ID4               0xD3    // read ID4
#define ILI9341_POS_GAMMA_COR       0xE0    // positive gamma correction
#define ILI9341_NEG_GAMMA_COR       0xE1    // negative gamma correction
#define ILI9341_DIG_GAMMA_CTRL1     0xE2    // digital gamma control 1
#define ILI9341_DIG_GAMMA_CTRL2     0xE3    // digital gamma control 2
#define ILI9341_INTERFACE_CTRL      0xF6    // interface control

/* -------------------------------- Initialization Command Set --------------------------------- */
static const uint8_t ILI9341_InitCMDs[] =
{
    0xEF,                     3, 0x03, 0x80, 0x02,
    0xCF,                     3, 0x00, 0xC1, 0x30,
    0xED,                     4, 0x64, 0x03, 0x12, 0x81,
    0xE8,                     3, 0x85, 0x00, 0x78,
    0xCB,                     5, 0x39, 0x2C, 0x00, 0x34, 0x02,
    0xF7,                     1, 0x20,
    0xEA,                     2, 0x00, 0x00,
    ILI9341_PWR_CTRL1,        1, 0x23,
    ILI9341_PWR_CTRL2,        1, 0x10,
    ILI9341_VCOM_CTRL1,       2, 0x3e, 0x28,
    ILI9341_VCOM_CTRL2,       1, 0x86,
    ILI9341_RD_MADCTL,        1, 0x48,
    ILI9341_VERT_SCRL_START,  1, 0x00,
    ILI9341_PX_FORMAT_SET,    1, 0x55,
    ILI9341_FRAME_CTRL_NORM,  2, 0x00, 0x18,
    ILI9341_DISP_FUNC_CTRL,   3, 0x08, 0x82, 0x27,
    0xF2,                     1, 0x00,
    ILI9341_GAMMA_SET,        1, 0x01,
    ILI9341_POS_GAMMA_COR,   15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
    ILI9341_NEG_GAMMA_COR,   15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
    ILI9341_SLEEP_OUT,        1, 0x80,
    ILI9341_DISP_ON,          1, 0x80,
    0x00
};

/* -------------------------------------- Font Print Data -------------------------------------- */
#define ILI9341_FONT_BASE_HEIGHT 8
#define ILI9341_FONT_BASE_WIDTH  5

static const uint8_t ILI9341_Font[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, // (space)
    0x00, 0x00, 0x5F, 0x00, 0x00, // !
    0x00, 0x07, 0x00, 0x07, 0x00, // "
    0x14, 0x7F, 0x14, 0x7F, 0x14, // #
    0x24, 0x2A, 0x7F, 0x2A, 0x12, // $
    0x23, 0x13, 0x08, 0x64, 0x62, // %
    0x36, 0x49, 0x55, 0x22, 0x50, // &
    0x00, 0x05, 0x03, 0x00, 0x00, // '
    0x00, 0x1C, 0x22, 0x41, 0x00, // (
    0x00, 0x41, 0x22, 0x1C, 0x00, // )
    0x08, 0x2A, 0x1C, 0x2A, 0x08, // *
    0x08, 0x08, 0x3E, 0x08, 0x08, // +
    0x00, 0x50, 0x30, 0x00, 0x00, // ,
    0x08, 0x08, 0x08, 0x08, 0x08, // -
    0x00, 0x60, 0x60, 0x00, 0x00, // .
    0x20, 0x10, 0x08, 0x04, 0x02, // /
    0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
    0x00, 0x42, 0x7F, 0x40, 0x00, // 1
    0x42, 0x61, 0x51, 0x49, 0x46, // 2
    0x21, 0x41, 0x45, 0x4B, 0x31, // 3
    0x18, 0x14, 0x12, 0x7F, 0x10, // 4
    0x27, 0x45, 0x45, 0x45, 0x39, // 5
    0x3C, 0x4A, 0x49, 0x49, 0x30, // 6
    0x01, 0x71, 0x09, 0x05, 0x03, // 7
    0x36, 0x49, 0x49, 0x49, 0x36, // 8
    0x06, 0x49, 0x49, 0x29, 0x1E, // 9
    0x00, 0x36, 0x36, 0x00, 0x00, // :
    0x00, 0x56, 0x36, 0x00, 0x00, // ;
    0x00, 0x08, 0x14, 0x22, 0x41, // <
    0x14, 0x14, 0x14, 0x14, 0x14, // =
    0x41, 0x22, 0x14, 0x08, 0x00, // >
    0x02, 0x01, 0x51, 0x09, 0x06, // ?
    0x32, 0x49, 0x79, 0x41, 0x3E, // @
    0x7E, 0x11, 0x11, 0x11, 0x7E, // A
    0x7F, 0x49, 0x49, 0x49, 0x36, // B
    0x3E, 0x41, 0x41, 0x41, 0x22, // C
    0x7F, 0x41, 0x41, 0x22, 0x1C, // D
    0x7F, 0x49, 0x49, 0x49, 0x41, // E
    0x7F, 0x09, 0x09, 0x01, 0x01, // F
    0x3E, 0x41, 0x41, 0x51, 0x32, // G
    0x7F, 0x08, 0x08, 0x08, 0x7F, // H
    0x00, 0x41, 0x7F, 0x41, 0x00, // I
    0x20, 0x40, 0x41, 0x3F, 0x01, // J
    0x7F, 0x08, 0x14, 0x22, 0x41, // K
    0x7F, 0x40, 0x40, 0x40, 0x40, // L
    0x7F, 0x02, 0x04, 0x02, 0x7F, // M
    0x7F, 0x04, 0x08, 0x10, 0x7F, // N
    0x3E, 0x41, 0x41, 0x41, 0x3E, // O
    0x7F, 0x09, 0x09, 0x09, 0x06, // P
    0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
    0x7F, 0x09, 0x19, 0x29, 0x46, // R
    0x46, 0x49, 0x49, 0x49, 0x31, // S
    0x01, 0x01, 0x7F, 0x01, 0x01, // T
    0x3F, 0x40, 0x40, 0x40, 0x3F, // U
    0x1F, 0x20, 0x40, 0x20, 0x1F, // V
    0x7F, 0x20, 0x18, 0x20, 0x7F, // W
    0x63, 0x14, 0x08, 0x14, 0x63, // X
    0x03, 0x04, 0x78, 0x04, 0x03, // Y
    0x61, 0x51, 0x49, 0x45, 0x43, // Z
    0x00, 0x00, 0x7F, 0x41, 0x41, // [
    0x02, 0x04, 0x08, 0x10, 0x20, // "\"
    0x41, 0x41, 0x7F, 0x00, 0x00, // ]
    0x04, 0x02, 0x01, 0x02, 0x04, // ^
    0x40, 0x40, 0x40, 0x40, 0x40, // _
    0x00, 0x01, 0x02, 0x04, 0x00, // `
    0x20, 0x54, 0x54, 0x54, 0x78, // a
    0x7F, 0x48, 0x44, 0x44, 0x38, // b
    0x38, 0x44, 0x44, 0x44, 0x20, // c
    0x38, 0x44, 0x44, 0x48, 0x7F, // d
    0x38, 0x54, 0x54, 0x54, 0x18, // e
    0x08, 0x7E, 0x09, 0x01, 0x02, // f
    0x08, 0x14, 0x54, 0x54, 0x3C, // g
    0x7F, 0x08, 0x04, 0x04, 0x78, // h
    0x00, 0x44, 0x7D, 0x40, 0x00, // i
    0x20, 0x40, 0x44, 0x3D, 0x00, // j
    0x00, 0x7F, 0x10, 0x28, 0x44, // k
    0x00, 0x41, 0x7F, 0x40, 0x00, // l
    0x7C, 0x04, 0x18, 0x04, 0x78, // m
    0x7C, 0x08, 0x04, 0x04, 0x78, // n
    0x38, 0x44, 0x44, 0x44, 0x38, // o
    0x7C, 0x14, 0x14, 0x14, 0x08, // p
    0x08, 0x14, 0x14, 0x18, 0x7C, // q
    0x7C, 0x08, 0x04, 0x04, 0x08, // r
    0x48, 0x54, 0x54, 0x54, 0x20, // s
    0x04, 0x3F, 0x44, 0x40, 0x20, // t
    0x3C, 0x40, 0x40, 0x20, 0x7C, // u
    0x1C, 0x20, 0x40, 0x20, 0x1C, // v
    0x3C, 0x40, 0x30, 0x40, 0x3C, // w
    0x44, 0x28, 0x10, 0x28, 0x44, // x
    0x0C, 0x50, 0x50, 0x50, 0x3C, // y
    0x44, 0x64, 0x54, 0x4C, 0x44, // z
    0x20, 0x55, 0x54, 0x55, 0x78, // ä
    0x3A, 0x44, 0x44, 0x3A, 0x00, // ö
    0x3A, 0x40, 0x40, 0x3A, 0x00, // ü
    0x00, 0x08, 0x36, 0x41, 0x00, // {
    0x00, 0x00, 0x7F, 0x00, 0x00, // |
    0x00, 0x41, 0x36, 0x08, 0x00, // }
    0x14, 0x3E, 0x55, 0x41, 0x22, // €
    0x08, 0x08, 0x2A, 0x1C, 0x08, // -> (ALT + 0134) †
    0x08, 0x1C, 0x2A, 0x08, 0x08, // <- (ALT + 01  35) ‡
    0x00, 0x00, 0x07, 0x05, 0x07
};

/* ------------------------------------ Block M Print Data ------------------------------------- */
#define ILI9341_BLOCKM_BASE_HEIGHT 16
#define ILI9341_BLOCKM_BASE_WIDTH 21

static const uint16_t ILI9341_BlockM[] =
{
    0xF03C, 0xF03C, 0xFFFC,
    0xFFFC, 0xFFFC, 0xFFFC,
    0xFFF8, 0xF3F0, 0x07E0,
    0x0FC0, 0x1F80, 0x0FC0,
    0x07E0, 0xF3F0, 0xFFF8,
    0xFFFC, 0xFFFC, 0xFFFC,
    0xFFFC, 0xF03C, 0xF03C
};

/* ------------------------------------- Arrow Print Data -------------------------------------- */
#define ILI9341_ARROW_BASE_HEIGHT 16
#define ILI9341_ARROW_BASE_WIDTH 13

static const uint16_t ILI9341_ARROW_N[] =
{
    0x0000, 0x0100, 0x0180, 0x01C0, 0x3FE0,
    0x3FF0, 0x3FF8, 0x3FF0, 0x3FE0, 0x01C0,
    0x0180, 0x0100, 0x0000
};

static const uint16_t ILI9341_ARROW_NE[] =
{
    0x0000, 0x0000, 0x0600, 0x0F10, 0x1FB0,
    0x1FF0, 0x0FF0, 0x07F0, 0x03F0, 0x07F0,
    0x0FF0, 0x0000, 0x0000
};

static const uint16_t ILI9341_ARROW_E[] =
{
    0x0000, 0x07C0, 0x07C0, 0x07C0, 0x07C0,
    0x07C0, 0x3FF8, 0x1FF0, 0x0FE0, 0x07C0,
    0x0380, 0x0100, 0x0000
};

static const uint16_t ILI9341_ARROW_SE[] =
{
    0x0000, 0x0000, 0x0060, 0x08F0, 0x0DF8,
    0x0FF8, 0x0FF0, 0x0FE0, 0x0FC0, 0x0FE0,
    0x0FF0, 0x0000, 0x0000
};

static const uint16_t ILI9341_ARROW_S[] =
{
    0x0000, 0x0100, 0x0300, 0x0700, 0x0FF8,
    0x1FF8, 0x3FF8, 0x1FF8, 0x0FF8, 0x0700,
    0x0300, 0x0100, 0x0000
};

static const uint16_t ILI9341_ARROW_SW[] =
{
    0x0000, 0x0000, 0x1FE0, 0x1FC0, 0x1F80,
    0x1FC0, 0x1FE0, 0x1FF0, 0x1BF0, 0x11E0,
    0x00C0, 0x0000, 0x0000
};

static const uint16_t ILI9341_ARROW_W[] =
{
    0x0000, 0x0100, 0x0380, 0x07C0, 0x0FE0,
    0x1FF0, 0x3FF8, 0x07C0, 0x07C0, 0x07C0,
    0x07C0, 0x07C0, 0x0000
};

static const uint16_t ILI9341_ARROW_NW[] =
{
    0x0000, 0x0000, 0x0FF0, 0x07F0, 0x03F0,
    0x07F0, 0x0FF0, 0x1FF0, 0x1FB0, 0x0F10,
    0x0600, 0x0000, 0x0000
};

/* --------------------------------------- Settings Icon --------------------------------------- */
#define ILI9341_SETTINGS_BASE_HEIGHT 16
#define ILI9341_SETTINGS_BASE_WIDTH 13

static const uint16_t ILI9341_SETTINGS[] =
{
    0x0000, 0x1998, 0x1998, 0x1998, 0x1998,
    0x1998, 0x1998, 0x1998, 0x1998, 0x1998,
    0x1998, 0x1998, 0x0000
};

/* ----------------------------------------- Structures ---------------------------------------- */
typedef struct CURSOR_STRUCT
{
    uint16_t x, y;
} cursor_t;

typedef enum SCREEN_ENUM
{
    HOMESCREEN,
    SETTINGS
} screen_enum;

/* -------------------------------- Read/Write Cycle Sequences --------------------------------- */
/*!
 * @brief   Writes 1-byte command from Adafruit ILI9341 command set specified on page 83 of
 *          datasheet (https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf)
 * @param   cmd         command to be written
 */
void ILI9341_WriteCommand(uint8_t cmd);

/*!
 * @brief   Writes 1-byte data as a parameter following a command
 * @note    Data to be written specifies provided per command in the reference manual
 *          (https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf)
 * @param   data        data to be written
 */
void ILI9341_WriteData(uint8_t data);

/* -------------------------------- Level 1 Command Operations --------------------------------- */
/*!
 * @brief   This function is used to define an area in memory that the MCU can access
 * @note    x0 must be less than or equal to x1
 * @note    y0 must be less than or equal to y1
 * @param   x0          lower bound row in memory
 * @param   x1          upper bound row in memory
 * @param   y0          lower bound column in memory
 * @param   y1          upper bound column in memory
 */
void ILI9341_SetFrameArea(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1);

/*!
 * @brief   Fills area within specified bounds with specified color
 * @param   color       fill color for frame
 * @param   x0          lower bound row in memory
 * @param   x1          upper bound row in memory
 * @param   y0          lower bound column in memory
 * @param   y1          upper bound column in memory
 */
void ILI9341_FillFrame(uint16_t color, uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1);

/* ------------------------------------- Derived Operations ------------------------------------ */
/*!
 * @brief   Fills the entire screen with specified color
 * @param   color       fill color for screen
 */
void ILI9341_FillScreen(uint16_t color);

/*!
 * @brief   prints an array of pixels with 8-bit height
 * @param   cur         coordinate location of array
 * @param   arr         pointer to array
 * @param   width       width of array
 * @param   scale       scale to print array
 */
void ILI9341_PrintArr8(cursor_t* cur, uint8_t* arr, uint8_t width, uint8_t scale);

/*!
 * @brief   prints an array of pixels with 16-bit height
 * @param   cur         coordinate location of array
 * @param   arr         pointer to array
 * @param   width       width of array
 * @param   scale       scale to print array
 */
void ILI9341_PrintArr16(cursor_t* cur, uint16_t* arr, uint8_t width, uint8_t scale);

/*!
 * @brief   clears the text box and resets the cursor to starting position
 * @param   cur         cursor to get reset position
 */
void ILI9341_ResetTextBox(cursor_t* cur);

/*!
 * @brief   based on y position, returns the availably width for given x
 * @note    assumes that end of arrow is always in line with end of text box
 * @param   y           cursors vertical position
 * @return  int         returns upper limit for x
 */
int LineAvailability(uint16_t y);

/*!
 * @brief   prints a single character to display
 * @note    Able to handle special character functions
 * @param   cur         coordinate location of character
 * @param   c           character
 */
void ILI9341_PrintChar(cursor_t* cur, char c);

/*!
 * @brief   checks if character matches any of the interrupt characters
 * @param   c           character
 * @return  int         return true if character is an interrupt character
 */
int isInterruptChar(char c);

/*!
 * @brief   prints string with multi-line complexity
 * @note    if string is greater than text box width, split between lines with dash
 * @note    if string will overflow but can fit on its own line, move it to the next line
 * @param   cur         coordinate location of character
 * @param   str         string to be printed
 */
void ILI9341_PrintString(cursor_t* cur, char* str);

/* -------------------------------------- Initializations -------------------------------------- */
/*!
 * @brief   writes a series of commands and data to initialize the display
 * @note    must be run before attempting to do any other transactions at the start of a program
 *
 */
void ILI9341_Init(void);

/*!
 * @brief   sets up sppech-to-text user interface
 */
void ILI9341_SetupSTTInterface(void);

/*!
 * @brief   sets up settings user interface
 */
void ILI9341_SetupSettingsInterface(void);

/* ---------------------------------- Parameter Sets/Recieves ---------------------------------- */

/*!
 * @brief   sets the primary and secondary color scheme
 * @param   _clr1       primary color
 * @param   _clr2       secondary color
 */
void ILI9341_SetClrParam(uint16_t _clr1, uint16_t _clr2);

/*!
 * @brief   sets the font size
 * @param   size        font scaler
 */
void ILI9341_SetFontParam(uint8_t size);

/*!
 * @brief   returns the current font size
 * @return  uint8_t     current font size
 */
uint8_t ILI9341_GetFontSize(void);

/*!
 * @brief   returns the current arrow size
 * @return  uint8_t     current arrow size
 */
uint8_t ILI9341_GetArrowSize(void);

/*!
 * @brief   sets the arrow size
 * @param   size        arrow scaler
 */
void ILI9341_SetArrowParam(uint8_t size);

/*!
 * @brief   returns the current brightness of display
 * @return  uint8_t     display brightness
 */
uint8_t ILI9341_GetBrightness();

/*!
 * @brief   sets the brightness of the display
 * @param   val         display brightness
 */
void ILI9341_SetBrightness(uint8_t val);

/*!
 * @brief   sets the secondary color to the temporary color holder
 */
void ILI9341_UpdateColor(void);

/*!
 * @brief   adjust the slider based on the size inputed
 * @param   size        scaler value of slider
 * @param   x           x position of slider
 * @param   dir         '1' if incrementing, '-1' if decrementing
 */
void ILI9341_AdjustSlider(uint8_t size, uint16_t x, int dir);
