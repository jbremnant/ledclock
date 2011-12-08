#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "ht1632.h"
#include "font.h"


// these variables will be configured during setup func
byte2 N_PANELS;
byte2 MAX_COLUMN; 
byte2 USE_SHADOWRAM = 1;
byte2 TOTAL_CELLS = 0;
byte ht1632_shadowram[TOTAL_CELLS_MAX];  // copy of the display's RAM - all 4 panels
byte HT1632_INITIALIZED = 0;

#define ht1632_selectboard() CS_PORT &= ~(1<<CS_PIN)
#define ht1632_freeboard()   CS_PORT |= (1<<CS_PIN)
#define HT1632_LEFT  0
#define HT1632_RIGHT 1
#define NMAX(x,y) ((x>y) ? x : y )
#define NMIN(x,y) ((x<y) ? x : y )

void ht1632_writebits(byte bits, byte firstbit)
{
  while (firstbit)
  {
    PIN_SET_LOW( WR_PORT, WR_PIN ); 
    if (bits & firstbit) {
      PIN_SET_HIGH( DAT_PORT, DAT_PIN );
    }
    else {
      PIN_SET_LOW( DAT_PORT, DAT_PIN );
    }
    PIN_SET_HIGH( WR_PORT, WR_PIN );
    firstbit >>= 1;
  }
}
/*
 * ht1632_sendcmd
 *  Send a command to the ht1632 chip.
 *  A command consists of a 3-bit "CMD" ID, an 8bit command, and
 *  one "don't care bit".
 *
 *   CS        CMD   Command bits            X  Free
 *   --------------------------------------------------
 *   CS_Select 1 0 0 c7 c6 c5 c4 c3 c2 c1 c0 xx CS_Free
 */ 
void ht1632_sendcmd(byte command)
{
  ht1632_selectboard();                    // Select chip
  ht1632_writebits(HT1632_ID_CMD, HT1632_ID_BITS); // send 3 bits of id: COMMMAND
  ht1632_writebits(command, HT1632_CMD_BITS);      // send the actual command
  ht1632_writebits(0, 1);                          // one extra dont-care bit in cmd 
  ht1632_freeboard();                              //done
}

/*
 * ht1632_senddata
 *  Send a nibble (4 bits) of data to a particular memory location of the
 *  ht1632. The command has 3 bit ID, 7 bits of address, and 4 bits of data.
 *
 *   CS        CMD   Address bits         Data        Free
 *   --------------------------------------------------------
 *   CS_Select 1 0 1 A6 A5 A4 A3 A2 A1 A0 D0 D1 D2 D3 CS_Free
 *
 *  Note that the address is sent MSB first, while the data is sent LSB first!
 *  This means that somewhere a bit reversal will have to be done to get
 *  zero-based addressing of words and dots within words.
 *
 *  Currently, this routine only does single writes. You can do successive
 *  address writing with HT1632.
 */
void ht1632_senddata(byte address, byte data)
{
  ht1632_selectboard();                   // Select chip
  ht1632_writebits(HT1632_ID_WR, HT1632_ID_BITS);  // send ID: WRITE to RAM
  ht1632_writebits(address, 1<<6);                 // Send address (7 bits)
  ht1632_writebits(data,    1<<3);                 // send data (4 bits)
  ht1632_freeboard();                              // done
}

/*
 * ht1632_wipe
 *  Send value to all displays, and the shadow memory, and the snapshot
 *  memory.  This uses the "write multiple words" capability of
 *  the chipset by writing all 96 words of memory without raising
 *  the chipselect signal.
 */
void ht1632_wipe(byte value, byte displayonly)
{
  byte i;
  // PANEL_WIDTH columns *2 bytes per column on each panel
  // wipes with value per column
  for (i=0; i<NUM_CELLS; i++)
    ht1632_senddata(i, value);  // clear the display! 

  if(USE_SHADOWRAM && !displayonly)
    ht1632_clearram();
}

void ht1632_clear()
{
  ht1632_wipe(0, 0);
}



/*
 * ht1632_setup
 */
byte ht1632_setup()
{
  USE_SHADOWRAM = 1;

	PIN_MODE(CS_REG, CS_PIN, OUTPUT);
	PIN_MODE(WR_REG, WR_PIN, OUTPUT);
	PIN_MODE(DAT_REG,DAT_PIN,OUTPUT);

  // we first send it to all boards
  ht1632_selectboard();
  ht1632_sendcmd(HT1632_CMD_SYSDIS);  // Disable system
  ht1632_sendcmd(HT1632_CMD_COMS10);  // xy, PMOS drivers, 32OUT 8COMS, 8x32
  // ht1632_sendcmd(HT1632_CMD_COMS11);  // xy, PMOS drivers, 24OUT 16COMS, 16x24
  ht1632_sendcmd(HT1632_CMD_MSTMD);   // Master Mode
  ht1632_sendcmd(HT1632_CMD_SYSON);   // System on
  ht1632_sendcmd(HT1632_CMD_LEDON);   // LEDs on

  N_PANELS = 1;

  MAX_COLUMN  = (N_PANELS * PANEL_WIDTH) -1;
  TOTAL_CELLS = N_PANELS * NUM_CELLS; // 96 banks (of 4 bits) for 24 x 16 matrix
  
  ht1632_clear();
  
  return 1;
}

byte2 ht1632_xmax()
{
  return MAX_COLUMN+1;
}

byte2 ht1632_ymax()
{
  return MAX_ROW+1;
}


void ht1632_getaddr(byte x, byte y, byte wrap, struct plot_addr* pa)
{
  if(x > MAX_COLUMN)
    x = (wrap) ? x % MAX_COLUMN : MAX_COLUMN;  // wrap : bounds
  if(y > MAX_ROW)
    y = (wrap) ? y % MAX_ROW : MAX_ROW;        // wrap : bounds

  pa->addr   = (x<<1) + (y>>2);               // compute which memory word this is in
  pa->bitval = 8>>(y&3);                      // compute which bit will need to be set
}

void ht1632_setram(byte x, byte y, byte dispbyte)
{
  struct plot_addr pa;
  ht1632_getaddr(x, y, 0, &pa);
  ht1632_shadowram[pa.addr] = dispbyte;
}

void ht1632_sendbyte(byte x, byte y, byte dispbyte, byte clear)
{
  struct plot_addr pa;
  ht1632_getaddr(x, y, 0, &pa);
  if(clear)
    ht1632_senddata(pa.addr, 0x00);
    
  ht1632_senddata(pa.addr, dispbyte);
}
/*
 * plot a point on the display, with the upper left hand corner
 * being (0,0), and the lower right hand corner being (23, 15).
 * Note that Y increases going "downward" in contrast with most
 * mathematical coordiate systems, but in common with many displays
 * No error checking; bad things may happen if arguments are out of
 * bounds!  (The ASSERTS compile to nothing by default
 */
void ht1632_do_plot(byte x, byte y, byte val, byte wrap, byte ramonly)
{
  byte displayByte;
  struct plot_addr pa;

  if(!wrap && x > MAX_COLUMN)
    return;

  ht1632_getaddr(x, y, wrap, &pa);

  displayByte = ht1632_shadowram[pa.addr];

  // modify the display byte
  if (val)
    displayByte |= pa.bitval;
  else
    displayByte &= ~pa.bitval;

  ht1632_shadowram[pa.addr] = displayByte;

  if(!ramonly)
    ht1632_senddata(pa.addr, displayByte);
}


/*
 * ht1632_snapram    
 *  Copy the shadow ram into the snapshot ram (the upper bits)
 *  This gives us a separate copy so we can plot new data while
 *  still having a copy of the old data.  snapshotram is NOT
 *  updated by the plot functions (except "clear")
 */
void ht1632_snapram()
{
  byte i;
  for (i=0; i<TOTAL_CELLS; i++) {
    ht1632_shadowram[i] = (ht1632_shadowram[i] & 0x0F) | ht1632_shadowram[i] << 4;  // Use the upper bits
  }
}

void ht1632_syncram()
{
  byte i;
  ht1632_wipe(0, 1);
  ht1632_snapram(); // save the state.

  for (i=0; i<TOTAL_CELLS; i++)
  {
    ht1632_senddata(i%NUM_CELLS, ht1632_shadowram[i]);
  }
}

/*
 * Clear the lower half of each bank
 */
void ht1632_clearram()
{
  byte i;
  for (i=0; i<TOTAL_CELLS; i++)
    ht1632_shadowram[i] = ht1632_shadowram[i] & 0xF0;
}


/*
 * get_snapshotram
 *  get a pixel value from the snapshot ram (instead of
 *  the actual displayed (shadow) memory
 */
byte ht1632_getram(byte x, byte y)
{
  byte addr, bitval;

  bitval = 128>>(y&3);       // user upper bits!
  addr   = (x<<2) + (y>>2);  // compute which memory word this is in
  if (ht1632_shadowram[addr] & bitval)
    return 1;
  return 0;
}


/*
 * Shift the contents of the ram to the left by 2 banks
 */
void ht1632_shiftram(byte direction, byte endbyte)
{
  byte i;

  if(direction==HT1632_LEFT)
  {
    for (i=0; i<TOTAL_CELLS_MAX-2; i++)
    {
      ht1632_shadowram[i] = ht1632_shadowram[i+2];
    }
    ht1632_shadowram[i]   = 0x0F & endbyte; 
    ht1632_shadowram[i+1] = (0xF0 & endbyte) >> 4; 
  }
  else if(direction==HT1632_RIGHT)
  {
    for (i=TOTAL_CELLS_MAX-1; i>1; i--)
    {
      ht1632_shadowram[i] = ht1632_shadowram[i-2];
    }
    ht1632_shadowram[0] = 0x0F & endbyte; 
    ht1632_shadowram[1] = (0xF0 & endbyte) >> 4; 
  }
}


void ht1632_shiftleft(byte endbyte)
{
  ht1632_wipe(0, 1); // display only
  ht1632_shiftram(HT1632_LEFT, endbyte);
  ht1632_syncram();
}


byte ht1632_getchcol(byte x, char c)
{
  byte col;
  byte dots;
  if ( (c >= 'A' && c <= 'Z') ||
    ( c >= 'a' && c <= 'z') ) {
    c &= 0x1F;   // A-Z maps to 1-26
  }
  else if (c >= '0' && c <= '9') {
    c = (c - '0') + 27; // 37;  // 27;
  }
  else if (c == ' ') {
    c = 0; // space
  }
  for (col=0; col< 5; col++)
  {
    dots = pgm_read_byte_near(&myfont[(byte)c][(byte)col]);
    if(x+col > MAX_COLUMN)
      return dots;
  }
  return dots;
}

byte ht1632_strram(byte x, const char* str, byte n)
{
  char c, i = 0;
  byte endbyte;
  byte pos = x;
  while( (c = *str++) && (i++ < n) )
  {
    endbyte = ht1632_getchcol(pos, c);
    if(pos>MAX_COLUMN)
      return endbyte;
    pos += 6;
  }
  return endbyte;
}

/*
  for colpos in range(ncols+self.x):
      xind  = self.x - min(colpos,self.x)
      start = int(max(0,colpos-self.x)/6) # char we want
      width = min(self.x/6, colpos/6)+1
      offset= max(0,colpos-self.x) % 6

      t = text[start:start+width]
      # print t
      l = self.textcmd(t, xind, y, offset, 0, delay)
*/

void ht1632_putchar2(byte x, byte y, char c, byte offset)
{
  byte col;
  byte row;
  byte dots;
  byte tf;
  c = ht1632_whichch(c);

  for (col=offset; col< 5; col++)
  {
    dots = ht1632_chcol(c, col);
    for (row=0; row < 7; row++)
    {
      tf = (dots & (64>>row)) ? 1 : 0;          // only 7 rows.
      ht1632_do_plot(x+col-offset, y+row, tf, 0, 1);
    }
  }
}

void ht1632_scrollstr(const char* str, byte n, byte msec)
{
  int16_t x, xind, i, o, ii, j,pos;
  int16_t txtw = n * 6;
  int16_t span = txtw + 32;
  char* ptr;
  char ch;

  for(x=0; x<span; x++)
  {
    xind = PANEL_WIDTH - NMIN(x, PANEL_WIDTH); // panel scroll position
    i    = NMAX(0, x - PANEL_WIDTH) / 6;       // str start position
    o    = NMIN(PANEL_WIDTH/6, x/6) + 1;       // length of substr
    o    = NMIN(o, n-i);
    ii   = NMAX(0, x-PANEL_WIDTH) % 6;         // front char offset
    ptr  = &str[i];

    ht1632_clear();

    j = 0;
    pos = xind;
    while( (ch = *ptr++) && (j < o) )
    {
      if(ch=='\n') break;
      if(pos>MAX_COLUMN) break;
      if(j>=n) break;

      ht1632_putchar2(pos, 1, ch, (j==0) ? ii : 0);
      pos += (j==0) ? 6 - ii : 6;
      j++;
    }

    ht1632_syncram();
    _delay_ms(msec);
  } 
}


char ht1632_whichch(char c)
{
  if ( (c >= 'A' && c <= 'Z') ||
    ( c >= 'a' && c <= 'z') ) {
    c &= 0x1F;   // A-Z maps to 1-26
  }
  else if (c >= '0' && c <= '9') {
    c = (c - '0') + 27; // 37;  // 27;
  }
  else if (c == ' ') {
    c = 0; // space
  }
  return c;
}

byte ht1632_chcol(char c, byte j)
{
  if(j > 4)
    j = 0;
  return pgm_read_byte_near(&myfont[(byte)c][j]);
}


/*
 * Copy a character glyph from the myfont data structure to
 *  display memory, with its upper left at the given coordinate
 *  This is unoptimized and simply uses plot() to draw each dot.
 */
void ht1632_putchar(byte x, byte y, char c, byte ramonly)
{
  byte col;
  byte row;
  byte dots;
  c = ht1632_whichch(c);

  for (col=0; col< 5; col++)
  {
    dots = ht1632_chcol(c, col);
    for (row=0; row < 7; row++)
    {
      if (dots & (64>>row))          // only 7 rows.
        ht1632_do_plot(x+col, y+row, 1, 0, ramonly);
      else
        ht1632_do_plot(x+col, y+row, 0, 0, ramonly);
    }
  }
}

void ht1632_print(byte x, byte y, const char* str, byte n, byte ramonly)
{
  char c, i = 0;
  byte endbyte;
  byte pos = x;
  while( (c = *str++) && (i++ < n) )
  {
    if(c=='\n')
      break;
    ht1632_putchar(pos, y, c, ramonly);
    if(pos>MAX_COLUMN)
      return;
    pos += 6;
  }
}


