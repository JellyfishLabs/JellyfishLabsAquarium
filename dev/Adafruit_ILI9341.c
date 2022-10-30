/*!
 * @file    Adafruit_ILI9341.c
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

#include "Adafruit_ILI9341.h"

/* ------------------------------------- Global Variables -------------------------------------- */
static uint16_t clr1,                                                                               // primary color
                clr2,                                                                               // secondary color
                clrt;                                                                               // temporary color

static uint16_t ILI9341_TXTBOX_X,                                                                   // x position of upper-left corner anchor
                ILI9341_TXTBOX_Y;                                                                   // y position of upper-left corner anchor

static uint8_t  ILI9341_ARROW_SIZE,                                                                 // arrow scaler
                ILI9341_FONT_SIZE,                                                                  // font scaler
                ILI9341_BRIGHTNESS;                                                                 // display brightness


/* --------------------------------------- STM32 L031K6 ---------------------------------------- */
extern SPI_HandleTypeDef* ILI9341_HSPI_INST;                                                        // hspi instance pointer

extern GPIO_TypeDef* ILI9341_CSX_PORT;                                                              // csx port location
extern uint16_t ILI9341_CSX_PIN;                                                                    // csx pin location

extern GPIO_TypeDef* ILI9341_DCX_PORT;                                                              // dcx port location
extern uint16_t ILI9341_DCX_PIN;                                                                    // dcx port location

/* -------------------------------- Read/Write Cycle Sequences --------------------------------- */
/*!
 * @brief   Writes 1-byte command from Adafruit ILI9341 command set specified on page 83 of
 *          datasheet (https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf)
 * @param   cmd         command to be written
 */
void ILI9341_WriteCommand(uint8_t cmd)
{
    HAL_GPIO_WritePin(ILI9341_CSX_PORT, ILI9341_CSX_PIN, GPIO_PIN_RESET);                           // sets chip select low beginning transaction
    HAL_GPIO_WritePin(ILI9341_DCX_PORT, ILI9341_DCX_PIN, GPIO_PIN_RESET);                           // sets D/C low indicating command write
    HAL_SPI_Transmit(ILI9341_HSPI_INST, (uint8_t*)&cmd, 1, 100);                                    // sends command to be written
    HAL_GPIO_WritePin(ILI9341_CSX_PORT, ILI9341_CSX_PIN, GPIO_PIN_SET);                             // sets chip select high ending transaction
}

/*!
 * @brief   Writes 1-byte data as a parameter following a command
 * @note    Data to be written specifies provided per command in the reference manual
 *          (https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf)
 * @param   data        data to be written
 */
void ILI9341_WriteData(uint8_t data)
{
    HAL_GPIO_WritePin(ILI9341_CSX_PORT, ILI9341_CSX_PIN, GPIO_PIN_RESET);                            // sets chip select low beginning transaction
    HAL_GPIO_WritePin(ILI9341_DCX_PORT, ILI9341_DCX_PIN, GPIO_PIN_SET);                              // sets D/C high indicating data write
    HAL_SPI_Transmit(ILI9341_HSPI_INST, (uint8_t*)&data, 1, 100);                                    // sends data to be written
    HAL_GPIO_WritePin(ILI9341_CSX_PORT, ILI9341_CSX_PIN, GPIO_PIN_SET);                              // sets chip select high ending transaction
}

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
void ILI9341_SetFrameArea(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1)
{
    ILI9341_WriteCommand(ILI9341_PG_ADDR_SET);                                                      // sets the frame height
    ILI9341_WriteData((uint8_t)(x0 >> 8));
    ILI9341_WriteData((uint8_t)(x0));
    ILI9341_WriteData((uint8_t)(x1 >> 8));
    ILI9341_WriteData((uint8_t)(x1));

    ILI9341_WriteCommand(ILI9341_COL_ADDR_SET);                                                     // sets the frame width
    ILI9341_WriteData((uint8_t)(y0 >> 8));
    ILI9341_WriteData((uint8_t)(y0));
    ILI9341_WriteData((uint8_t)(y1 >> 8));
    ILI9341_WriteData((uint8_t)(y1));
}

/*!
 * @brief   Fills area within specified bounds with specified color
 * @param   color       fill color for frame
 * @param   x0          lower bound row in memory
 * @param   x1          upper bound row in memory
 * @param   y0          lower bound column in memory
 * @param   y1          upper bound column in memory
 */
void ILI9341_FillFrame(uint16_t color, uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1)
{
    if ((x0 < 0) || (x1 > ILI9341_WIDTH) || (y0 < 0) || (y1 > ILI9341_HEIGHT)) return;              // makes sure frame doesn't go out of scope
    if ((x1-x0) < 0 || (y1-y0) < 0) return;                                                         // makes sure coordinates are sent in correct order

    ILI9341_SetFrameArea(x0, x1, y0, y1);                                                           // selects frame to be filled
    ILI9341_WriteCommand(ILI9341_MEM_W);
    for (int i = 0; i < (x1-x0+1)*(y1-y0+1); ++i)                                                   // writes color to every pixel in that frame
    {
        ILI9341_WriteData((uint8_t)(color >> 8));
        ILI9341_WriteData((uint8_t)(color));
    }
}

/* ------------------------------------- Derived Operations ------------------------------------ */
/*!
 * @brief   Fills the entire screen with specified color
 * @param   color       fill color for screen
 */
void ILI9341_FillScreen(uint16_t color)
{
    ILI9341_FillFrame(color, 0, ILI9341_WIDTH, 0, ILI9341_HEIGHT);
}

/*!
 * @brief   prints an array of pixels with 8-bit height
 * @param   cur         coordinate location of array
 * @param   arr         pointer to array
 * @param   width       width of array
 * @param   scale       scale to print array
 */
void ILI9341_PrintArr8(cursor_t* cur, uint8_t* arr, uint8_t width, uint8_t scale)
{
    ILI9341_SetFrameArea(cur->x, cur->x + width*scale - 1, cur->y, cur->y + 8*scale - 1);

    uint16_t col;                                                                                   // holds current column being printed
    int k = 0,
        z = 0;                                                                                      // holds index of current column

    ILI9341_WriteCommand(ILI9341_MEM_W);
    for (int i = 0; i < width*scale; ++i)                                                           // iterates through character print data and prints scaled character
    {
        col = *(arr + k);                                                                           // col = current column being printed
        for (int j = 0; j < 8*scale; ++j)
        {
            if ((col & (1 << z)) > 0)                                                               // print pixel at z position in column
            {
                ILI9341_WriteData((uint8_t)(clr1 >> 8));
                ILI9341_WriteData((uint8_t)(clr1));
            }
            else
            {
                ILI9341_WriteData((uint8_t)(clr2 >> 8));
                ILI9341_WriteData((uint8_t)(clr2));
            }
            if((j % scale) == scale - 1) z++;                                                       // increments row index after scaled row prints
        }
        if ((i % scale) == scale - 1) k++;                                                          // increments column index after scaled column prints
        z = 0;
    }
}

/*!
 * @brief   prints an array of pixels with 16-bit height
 * @param   cur         coordinate location of array
 * @param   arr         pointer to array
 * @param   width       width of array
 * @param   scale       scale to print array
 */
void ILI9341_PrintArr16(cursor_t* cur, uint16_t* arr, uint8_t width, uint8_t scale)
{
    ILI9341_SetFrameArea(cur->x, cur->x + width*scale - 1, cur->y, cur->y + 16*scale - 1);

    uint16_t col;                                                                                   // holds current column being printed
    int k = 0,
        z = 0;                                                                                      // holds index of current column

    ILI9341_WriteCommand(ILI9341_MEM_W);
    for (int i = 0; i < width*scale; ++i)                                                           // iterates through character print data and prints scaled character
    {
        col = *(arr + k);                                                                           // col = current column being printed
        for (int j = 0; j < 16*scale; ++j)
        {
            if ((col & (1 << z)) > 0)                                                               // print pixel at z position in column
            {
                ILI9341_WriteData((uint8_t)(clr1 >> 8));
                ILI9341_WriteData((uint8_t)(clr1));
            }
            else
            {
                ILI9341_WriteData((uint8_t)(clr2 >> 8));
                ILI9341_WriteData((uint8_t)(clr2));
            }
            if((j % scale) == scale - 1) z++;                                                       // increments row index after scaled row prints
        }
        if ((i % scale) == scale - 1) k++;                                                          // increments column index after scaled column prints
        z = 0;
    }
}

/*!
 * @brief   clears the text box and resets the cursor to starting position
 * @param   cur         cursor to get reset position
 */
void ILI9341_ResetTextBox(cursor_t* cur)
{
    ILI9341_FillFrame(clr2, ILI9341_TXTBOX_X, ILI9341_TXTBOX_X + ILI9341_TXTBOX_WIDTH,
        ILI9341_TXTBOX_Y, ILI9341_TXTBOX_Y + ILI9341_TXTBOX_HEIGHT);

    cur->x = ILI9341_WIDTH - ILI9341_ARROW_BASE_WIDTH*ILI9341_ARROW_SIZE - 10;
    cur->y = 4;
    //ILI9341_PrintArr16(cur, ILI9341_ARROW_N, ILI9341_ARROW_BASE_WIDTH, ILI9341_ARROW_SIZE);

    cur->x = ILI9341_TXTBOX_X;
    cur->y = ILI9341_TXTBOX_Y;
}

/*!
 * @brief   based on y position, returns the availably width for given x
 * @note    assumes that end of arrow is always in line with end of text box
 * @param   y           cursors vertical position
 * @return  int         returns upper limit for x
 */
int LineAvailability(uint16_t y)
{
    if (y <= (ILI9341_ARROW_BASE_HEIGHT + 1)*ILI9341_ARROW_SIZE)
        return ILI9341_TXTBOX_WIDTH - (ILI9341_ARROW_BASE_WIDTH + 1)*ILI9341_ARROW_SIZE;
    else
        return ILI9341_TXTBOX_WIDTH;
}

/*!
 * @brief   prints a single character to display
 * @note    Able to handle special character functions
 * @param   cur         coordinate location of character
 * @param   c           character
 */
void ILI9341_PrintChar(cursor_t* cur, char c)
{
    if(((c < 32) || (c > 132)) && (c != '\n')) return;                                              // checks that character fits in range of printable characters

    ILI9341_SetFrameArea(cur->x, cur->x + ILI9341_FONT_BASE_WIDTH*ILI9341_FONT_SIZE - 1,
                         cur->y, cur->y + ILI9341_FONT_BASE_HEIGHT*ILI9341_FONT_SIZE - 1);          // sets frame for character

    switch (c)
    {
    case '\n':
        if (cur->y + ILI9341_FONT_SIZE*(2*ILI9341_FONT_BASE_HEIGHT + 1)
            > ILI9341_TXTBOX_HEIGHT + ILI9341_TXTBOX_Y)
            ILI9341_ResetTextBox(cur);                                                              // reset text box when new line overflows
        else if (cur->x != ILI9341_TXTBOX_X)
        {
            cur->x  = ILI9341_TXTBOX_X;                                                             // move cursor to new line
            cur->y += ILI9341_FONT_SIZE*(ILI9341_FONT_BASE_HEIGHT + 1);
        }
        break;

    case '\0':
        /* do nothing */
        break;

    default:
        ILI9341_PrintArr8(cur, (ILI9341_Font + (c - 32)*5),
            ILI9341_FONT_BASE_WIDTH, ILI9341_FONT_SIZE);                                            // prints character

        cur->x += ILI9341_FONT_SIZE*(ILI9341_FONT_BASE_WIDTH + 1);                                  // insert seperation after character
        break;
    }
}

/*!
 * @brief   checks if character matches any of the interrupt characters
 * @param   c           character
 * @return  int         return true if character is an interrupt character
 */
int isInterruptChar(char c)
{
    return (c == '\0') || (c == '\n') || (c == ' ');
}

/*!
 * @brief   prints string with multi-line complexity
 * @note    if string is greater than text box width, split between lines with dash
 * @note    if string will overflow but can fit on its own line, move it to the next line
 * @param   cur         coordinate location of character
 * @param   str         string to be printed
 */
void ILI9341_PrintString(cursor_t* cur, char* str)
{
    int pos     = 0,                                                                                // current position in string
        wrd_len = 0,                                                                                // length of current word
        isSplit = 0;                                                                                // true if word needs to be split

    while (*(str + pos) != '\0')
    {
        while (!isInterruptChar(*(str + pos + wrd_len))) wrd_len++;                                 // gets the length of word in string
        if (wrd_len == 0 && isInterruptChar(*(str + pos)) && *(str + pos) != '\0') wrd_len = 1;     // handles if word is a special character

        if (wrd_len*(ILI9341_FONT_BASE_WIDTH + 1)*ILI9341_FONT_SIZE > LineAvailability(cur->y))
        {
            int char_spc = (int)((LineAvailability(cur->y) + ILI9341_TXTBOX_X - cur->x)             // gets remaining character space
                                /((ILI9341_FONT_BASE_WIDTH + 1)*ILI9341_FONT_SIZE));
            char_spc--;                                                                             // accounts for '-' character to be included
            wrd_len = char_spc;                                                                     // set word length to space available on line
            isSplit = 1;
        }
        else if (wrd_len*(ILI9341_FONT_BASE_WIDTH + 1)*ILI9341_FONT_SIZE + cur->x
                > LineAvailability(cur->y) + ILI9341_TXTBOX_X)
            ILI9341_PrintChar(cur, '\n');                                                           // moves word to new line if it overflows current line

        int k = 0;
        while (k < wrd_len) ILI9341_PrintChar(cur, *(str + pos + k++));                             // print word
        if (*(str + pos + wrd_len) == ' ' && cur->x > 0)                                            // print space if not first character on line
        {
            ILI9341_PrintChar(cur, ' ');
            wrd_len++;
        }
        if(isSplit)
        {
            ILI9341_PrintString(cur, "-\n");                                                        // print continuation character
            isSplit = 0;                                                                            // new line
        }
        pos += wrd_len;                                                                             // increment position in string
        wrd_len = 0;                                                                                // reset word length
    }

}

/* -------------------------------------- Initializations -------------------------------------- */
/*!
 * @brief   writes a series of commands and data to initialize the display
 * @note    must be run before attempting to do any other transactions at the start of a program
 *
 */
void ILI9341_Init(void)
{
    ILI9341_WriteCommand(ILI9341_SW_RESET);
    HAL_Delay(150);

    uint8_t cmd_idx = 0;                                                                            // initialization command set index
    while (*(ILI9341_InitCMDs + cmd_idx) != 0x00)                                                   // runs through initialization sequence until terminator is reached
    {
        ILI9341_WriteCommand(*(ILI9341_InitCMDs + cmd_idx++));
        int num_data = *((ILI9341_InitCMDs + cmd_idx++));
        for (size_t i = num_data; i > 0; i--)
            ILI9341_WriteData(*(ILI9341_InitCMDs + cmd_idx++));
    }

    ILI9341_ARROW_SIZE = 4;                                                                         // set default arrow size
    ILI9341_FONT_SIZE  = 2;                                                                         // set default font size
    ILI9341_BRIGHTNESS = 8;                                                                         // set default brightness
    clr1 = 0x0000;
    clr2 = 0xFFFF;
    clrt = 0xFFFF;
}

/*!
 * @brief   sets up sppech-to-text user interface
 */
void ILI9341_SetupSTTInterface(void)
{
    ILI9341_FillScreen(clr2);                                                                       // fill background color

    cursor_t cur;                                                                                   // coordinate location to write icons

    cur.x = 10;
    cur.y = 222;
    ILI9341_PrintArr16(&cur, ILI9341_BlockM, ILI9341_BLOCKM_BASE_WIDTH, 1);                         // print block M in bottom left corner

    cur.x = 20 + ILI9341_BLOCKM_BASE_WIDTH;
    cur.y = 224;
    ILI9341_PrintArr16(&cur, ILI9341_SETTINGS, ILI9341_SETTINGS_BASE_WIDTH, 1);                     // print settings icon to the right of block M

    uint8_t t_size = ILI9341_FONT_SIZE;
    ILI9341_FONT_SIZE = 1;
    cur.x = ILI9341_WIDTH - 5*(ILI9341_FONT_BASE_WIDTH + 1) - 10;
    cur.y = ILI9341_HEIGHT - ILI9341_FONT_BASE_HEIGHT - 4;
    char* str = "clear";
    for (uint8_t i = 0; i < 5; i++) ILI9341_PrintChar(&cur, *(str + i));
    ILI9341_FONT_SIZE = t_size;

    ILI9341_TXTBOX_X = 10;
    ILI9341_TXTBOX_Y = 10;
}

/*!
 * @brief   sets up settings user interface
 */
void ILI9341_SetupSettingsInterface(void)
{
    __disable_irq();                                                                                // disables interrupts

    ILI9341_FillScreen(clr2);                                                                       // fill screen with background color

    cursor_t cur = {0,0};                                                                           // create return button
    uint8_t arrow_size = ILI9341_ARROW_SIZE;
    uint8_t font_size  = ILI9341_FONT_SIZE;
    ILI9341_ARROW_SIZE = 0;
    ILI9341_SetFontParam(1);
    ILI9341_PrintString(&cur, "< return");

    cur.x = 124;                                                                                    // create SETTINGS title
    cur.y = 10;
    ILI9341_SetFontParam(2);
    ILI9341_PrintString(&cur, "SETTINGS");

    ILI9341_SetFontParam(2);                                                                        // draw brightness slider
    char word[20] = "Brightness";                                                                   // print label
    for(uint8_t i = 0; i < 10; ++i)
    {
        cur.x = 10;
        cur.y = 52 + ILI9341_FONT_BASE_HEIGHT*i*2;
        ILI9341_PrintChar(&cur, word[i]);
    }
    printSlider(30, ILI9341_BRIGHTNESS);

    ILI9341_SetFontParam(2);                                                                        // draw font size slider
    strcpy(word, "Font Size");                                                                      // print label
    for(uint8_t i = 0; i < 9; ++i) {
        cur.x = 125;
        cur.y = 60 + ILI9341_FONT_BASE_HEIGHT*i*2;
        ILI9341_PrintChar(&cur, word[i]);
    }
    printSlider(145, font_size);

    ILI9341_SetFontParam(2);                                                                        // draw arrow size slider
    strcpy(word, "Arrow Size");
    for(uint8_t i = 0; i < 10; ++i) {
        cur.x = 240;
        cur.y = 52 + ILI9341_FONT_BASE_HEIGHT*i*2;
        ILI9341_PrintChar(&cur, word[i]);
    }
    printSlider(260, arrow_size);

    ILI9341_ARROW_SIZE = arrow_size;
    ILI9341_FONT_SIZE = font_size;

    __enable_irq();                                                                                 // enables interrupts
}

/* ---------------------------------- Parameter Sets/Recieves ---------------------------------- */

/*!
 * @brief   sets the primary and secondary color scheme
 * @param   _clr1       primary color
 * @param   _clr2       secondary color
 */
void ILI9341_SetClrParam(uint16_t _clr1, uint16_t _clr2)
{
    clr1 = _clr1;
    clr2 = _clr2;
}

/*!
 * @brief   sets the font size
 * @param   size        font scaler
 */
void ILI9341_SetFontParam(uint8_t size)
{
    ILI9341_FONT_SIZE = size;
}

/*!
 * @brief   returns the current font size
 * @return  uint8_t     current font size
 */
uint8_t ILI9341_GetFontSize(void)
{
    return ILI9341_FONT_SIZE;
}

/*!
 * @brief   returns the current arrow size
 * @return  uint8_t     current arrow size
 */
uint8_t ILI9341_GetArrowSize(void)
{
    return ILI9341_ARROW_SIZE;
}

/*!
 * @brief   sets the arrow size
 * @param   size        arrow scaler
 */
void ILI9341_SetArrowParam(uint8_t size)
{
    ILI9341_ARROW_SIZE = size;
}

/*!
 * @brief   returns the current brightness of display
 * @return  uint8_t     display brightness
 */
uint8_t ILI9341_GetBrightness()
{
    return ILI9341_BRIGHTNESS;
}

/*!
 * @brief   sets the brightness of the display
 * @param   val         display brightness
 */
void ILI9341_SetBrightness(uint8_t val)
{
    ILI9341_BRIGHTNESS = val;
    clrt = 0xFFFF - 0x2104*(8-val);
}

/*!
 * @brief   sets the secondary color to the temporary color holder
 */
void ILI9341_UpdateColor(void)
{
    clr2 = clrt;
}

/*!
 * @brief   adjust the slider based on the size inputed
 * @param   size        scaler value of slider
 * @param   x           x position of slider
 * @param   dir         '1' if incrementing, '-1' if decrementing
 */
void ILI9341_AdjustSlider(uint8_t size, uint16_t x, int dir)
{
    if (dir) ILI9341_FillFrame(clr2, x + 2, x + 48, 62 + 17*(8 - size), 62 + 17*(9 - size));
    else     ILI9341_FillFrame(clr1, x + 2, x + 48, 62 + 17*(7 - size), 62 + 17*(8 - size));
}

/*!
 * @brief   prints a rectangle with a frame representing a value
 * @param   x           starting x position of slider
 * @param   val         value in slider
 */
void printSlider(uint16_t x, uint8_t val)
{
    ILI9341_FillFrame(clr1, x, x + 50, 60, 200);                                                        // print slider background
    ILI9341_SetFontParam(3);
    ILI9341_PrintChar(&((cursor_t){x + 18, 26}), '+');
    ILI9341_PrintChar(&((cursor_t){x + 18, 210}), '-');
    ILI9341_FillFrame(clr2, x + 2, x + 48, 62 + 17*(8 - val), 198);
}
