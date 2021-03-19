#define tft M5.Lcd

#define C64_DARKBLUE      0x4338ccU
#define C64_LIGHTBLUE     0xC9BEFFU

#define bgcolor C64_DARKBLUE
#define fgcolor C64_LIGHTBLUE

static int cursorX, cursorY, fontWidth, fontHeight, screenColumns, screenRows;
static char* screenBuffer; // text only buffer


int getBufferIdx( int x, int y )
{
  return x/fontWidth + (y/fontHeight)*screenColumns;
}

void updateBuffer( uint16_t idx, char c )
{
  screenBuffer[idx] = c;
}

void updateBuffer( int x, int y, char c ) // pixel coords
{
  updateBuffer( getBufferIdx(x, y), c );
}

void updateBuffer( char c )
{
  updateBuffer( cursorX, cursorY, c );
}

static void drawCursor( bool state )
{
  uint32_t color;
  if( state ) color = fgcolor;
  else color = bgcolor;
  tft.fillRect( cursorX, cursorY, fontWidth, fontHeight, color );
}

static void blinkCursor()
{
  static bool cursorState = true;
  static unsigned long lastBlink = millis();
  if( millis() - lastBlink > 500 ) {
    cursorState = ! cursorState;
    drawCursor( cursorState );
    lastBlink = millis();
  }
}
