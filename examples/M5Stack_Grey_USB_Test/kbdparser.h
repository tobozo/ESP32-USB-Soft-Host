
class KbdRptParser : public KeyboardReportParser
{
  void PrintKey(uint8_t mod, uint8_t key);
  char lastkey[2] = {0,0};
  protected:
    void OnControlKeysChanged(uint8_t before, uint8_t after);
    void OnKeyDown	(uint8_t mod, uint8_t key);
    void OnKeyUp	(uint8_t mod, uint8_t key);
    void OnKeyPressed(uint8_t key);
};


void KbdRptParser::PrintKey(uint8_t m, uint8_t key)
{
  MODIFIERKEYS mod;
  *((uint8_t*)&mod) = m;
  Serial.print((mod.bmLeftCtrl   == 1) ? "C" : " ");
  Serial.print((mod.bmLeftShift  == 1) ? "S" : " ");
  Serial.print((mod.bmLeftAlt    == 1) ? "A" : " ");
  Serial.print((mod.bmLeftGUI    == 1) ? "G" : " ");

  Serial.print(" >");
  Serial.printf("0x%02x", key );
  Serial.print("< ");

  Serial.print((mod.bmRightCtrl   == 1) ? "C" : " ");
  Serial.print((mod.bmRightShift  == 1) ? "S" : " ");
  Serial.print((mod.bmRightAlt    == 1) ? "A" : " ");
  Serial.println((mod.bmRightGUI    == 1) ? "G" : " ");
};


void KbdRptParser::OnKeyDown(uint8_t mod, uint8_t key)
{
  Serial.print("DN ");
  PrintKey(mod, key);
  uint8_t c = OemToAscii(mod, key);

  if (c)
    OnKeyPressed(c);

  cursorX = tft.getCursorX();
  cursorY = tft.getCursorY();

  if( key == 0x2a ) { // BACKSPACE
    drawCursor( false );
    if( lastkey[0] == 0 ) return;
    if( cursorX == 0 ) { // need to go line up
      if( cursorY == 0 ) return; // already at the top level
      cursorY -= fontHeight;
      cursorX = tft.width() - ( fontWidth + tft.width()%fontWidth );
      int currendIdx = getBufferIdx( cursorX, cursorY );
      while( currendIdx > 0 ) {
        if( screenBuffer[currendIdx] != 0 ) break;
        if( cursorX == 0 ) break;
        //currendIdx--;
        cursorX -= fontWidth;
        currendIdx = getBufferIdx( cursorX, cursorY );
      }
    } else {
      cursorX -= fontWidth;
    }

    tft.setCursor(cursorX, cursorY);
  }
}

void KbdRptParser::OnControlKeysChanged(uint8_t before, uint8_t after) {

  MODIFIERKEYS beforeMod;
  *((uint8_t*)&beforeMod) = before;

  MODIFIERKEYS afterMod;
  *((uint8_t*)&afterMod) = after;

  if (beforeMod.bmLeftCtrl != afterMod.bmLeftCtrl) {
    Serial.println("LeftCtrl changed");
  }
  if (beforeMod.bmLeftShift != afterMod.bmLeftShift) {
    Serial.println("LeftShift changed");
  }
  if (beforeMod.bmLeftAlt != afterMod.bmLeftAlt) {
    Serial.println("LeftAlt changed");
  }
  if (beforeMod.bmLeftGUI != afterMod.bmLeftGUI) {
    Serial.println("LeftGUI changed");
  }

  if (beforeMod.bmRightCtrl != afterMod.bmRightCtrl) {
    Serial.println("RightCtrl changed");
  }
  if (beforeMod.bmRightShift != afterMod.bmRightShift) {
    Serial.println("RightShift changed");
  }
  if (beforeMod.bmRightAlt != afterMod.bmRightAlt) {
    Serial.println("RightAlt changed");
  }
  if (beforeMod.bmRightGUI != afterMod.bmRightGUI) {
    Serial.println("RightGUI changed");
  }

}

void KbdRptParser::OnKeyUp(uint8_t mod, uint8_t key)
{
  Serial.print("UP ");
  PrintKey(mod, key);
}

void KbdRptParser::OnKeyPressed(uint8_t key)
{
  Serial.printf("ASCII(0x%02x): ", key);
  Serial.println((char)key);
  lastkey[0] = (char)key;
  drawCursor( false ); // clear cursor

  updateBuffer( key );

  switch(key) {
    case 0x0d: // ENTER
      if( cursorY+fontHeight <= tft.height() - ( fontHeight + tft.height()%fontHeight ) ) {
        cursorY += fontHeight;
        cursorX = 0;
      } else {
        cursorY = 0;
        cursorX = 0;
      }
    break;
    default:
      tft.drawString(lastkey, cursorX, cursorY );
      if( cursorX+fontWidth <= tft.width() - ( fontWidth + tft.width()%fontWidth ) ) {
        cursorX += fontWidth;
      } else {
        cursorY += fontHeight;
        cursorX  = 0;
      }

  }
  tft.setCursor(cursorX, cursorY);
};
