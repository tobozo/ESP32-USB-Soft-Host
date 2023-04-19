// reversed from https://github.com/felis/USB_Host_Shield_2.0
// /!\ this is limited to only one device (keyboard)

#define UHS_HID_BOOT_KEY_ZERO           0x27
#define UHS_HID_BOOT_KEY_ENTER          0x28
#define UHS_HID_BOOT_KEY_SPACE          0x2c
#define UHS_HID_BOOT_KEY_CAPS_LOCK      0x39
#define UHS_HID_BOOT_KEY_SCROLL_LOCK    0x47
#define UHS_HID_BOOT_KEY_NUM_LOCK       0x53
#define UHS_HID_BOOT_KEY_ZERO2          0x62
#define UHS_HID_BOOT_KEY_PERIOD         0x63

#define VALUE_BETWEEN(v,l,h) (((v)>(l)) && ((v)<(h)))
#define VALUE_WITHIN(v,l,h) (((v)>=(l)) && ((v)<=(h)))

struct MODIFIERKEYS
{
  uint8_t bmLeftCtrl : 1;
  uint8_t bmLeftShift : 1;
  uint8_t bmLeftAlt : 1;
  uint8_t bmLeftGUI : 1;
  uint8_t bmRightCtrl : 1;
  uint8_t bmRightShift : 1;
  uint8_t bmRightAlt : 1;
  uint8_t bmRightGUI : 1;
};

struct KBDINFO
{
  struct
  {
    uint8_t bmLeftCtrl : 1;
    uint8_t bmLeftShift : 1;
    uint8_t bmLeftAlt : 1;
    uint8_t bmLeftGUI : 1;
    uint8_t bmRightCtrl : 1;
    uint8_t bmRightShift : 1;
    uint8_t bmRightAlt : 1;
    uint8_t bmRightGUI : 1;
  };
  uint8_t bReserved;
  uint8_t Keys[6];
};


struct KBDLEDS
{
  uint8_t bmNumLock : 1;
  uint8_t bmCapsLock : 1;
  uint8_t bmScrollLock : 1;
  uint8_t bmCompose : 1;
  uint8_t bmKana : 1;
  uint8_t bmReserved : 3;
};


class KeyboardReportParser
{
  static const uint8_t numKeys[10];
  static const uint8_t symKeysUp[12];
  static const uint8_t symKeysLo[12];
  static const uint8_t padKeys[5];

  protected:

    union
    {
      KBDINFO kbdInfo;
      uint8_t bInfo[sizeof (KBDINFO)];
    } prevState;

    union
    {
      KBDLEDS kbdLeds;
      uint8_t bLeds;
    } kbdLockingKeys;

  public:

    KeyboardReportParser()
    {
      //bLeds = 0;
      bmCapsLock   = false;
      bmNumLock    = false;
      bmScrollLock = false;
    };

    bool bmCapsLock;
    bool bmNumLock;
    bool bmScrollLock;

    uint8_t OemToAscii(uint8_t mod, uint8_t key);
    void Parse(uint8_t len, uint8_t *buf);


  protected:

    virtual uint8_t HandleLockingKeys(uint8_t key)
    {
      switch(key) {
        case UHS_HID_BOOT_KEY_NUM_LOCK:
          bmNumLock = !bmNumLock;
        break;
        case UHS_HID_BOOT_KEY_CAPS_LOCK:
          bmCapsLock = !bmCapsLock;
        break;
        case UHS_HID_BOOT_KEY_SCROLL_LOCK:
          bmScrollLock = !bmScrollLock;
        break;
      }
      return 0;
    };

    virtual void OnControlKeysChanged(uint8_t before __attribute__((unused)), uint8_t after __attribute__((unused))) { };
    virtual void OnKeyDown(uint8_t mod __attribute__((unused)), uint8_t key __attribute__((unused))) { };
    virtual void OnKeyUp(uint8_t mod __attribute__((unused)), uint8_t key __attribute__((unused))) { };
    virtual const uint8_t *getNumKeys() { return numKeys; };
    virtual const uint8_t *getSymKeysUp() { return symKeysUp; };
    virtual const uint8_t *getSymKeysLo() { return symKeysLo; };
    virtual const uint8_t *getPadKeys() { return padKeys; };

};



const uint8_t KeyboardReportParser::numKeys[10] PROGMEM = {'!', '@', '#', '$', '%', '^', '&', '*', '(', ')'};
const uint8_t KeyboardReportParser::symKeysUp[12] PROGMEM = {'_', '+', '{', '}', '|', '~', ':', '"', '~', '<', '>', '?'};
const uint8_t KeyboardReportParser::symKeysLo[12] PROGMEM = {'-', '=', '[', ']', '\\', ' ', ';', '\'', '`', ',', '.', '/'};
const uint8_t KeyboardReportParser::padKeys[5] PROGMEM = {'/', '*', '-', '+', '\r'};


void KeyboardReportParser::Parse(uint8_t len, uint8_t *buf)
{
  if (buf[2] == 1)
    return;
  // provide event for changed control key state
  if (prevState.bInfo[0x00] != buf[0x00]) {
    OnControlKeysChanged(prevState.bInfo[0x00], buf[0x00]);
  }

  for (uint8_t i = 2; i < 8; i++) {
    bool down = false;
    bool up = false;

    for (uint8_t j = 2; j < 8; j++) {
      if (buf[i] == prevState.bInfo[j] && buf[i] != 1)
        down = true;
      if (buf[j] == prevState.bInfo[i] && prevState.bInfo[i] != 1)
        up = true;
    }
    if (!down) {
      HandleLockingKeys(buf[i]);
      OnKeyDown(*buf, buf[i]);
    }
    if (!up)
      OnKeyUp(prevState.bInfo[0], prevState.bInfo[i]);
  }
  for (uint8_t i = 0; i < 8; i++)
    prevState.bInfo[i] = buf[i];
}


uint8_t KeyboardReportParser::OemToAscii(uint8_t mod, uint8_t key)
{
  uint8_t shift = (mod & 0x22);
  // [a-z]
  if (VALUE_WITHIN(key, 0x04, 0x1d)) {
    // Upper case letters
    if ((bmCapsLock == 0 && shift) ||
      (bmCapsLock == 1 && shift == 0))
      return (key - 4 + 'A');

      // Lower case letters
    else
      return (key - 4 + 'a');
  }// Numbers
  else if (VALUE_WITHIN(key, 0x1e, 0x27)) {
    if (shift)
      return ((uint8_t)pgm_read_byte(&getNumKeys()[key - 0x1e]));
    else
      return ((key == UHS_HID_BOOT_KEY_ZERO) ? '0' : key - 0x1e + '1');
  }// Keypad Numbers
  else if(VALUE_WITHIN(key, 0x59, 0x61)) {
    if(bmNumLock == 1)
      return (key - 0x59 + '1');
  } else if(VALUE_WITHIN(key, 0x2d, 0x38))
    return ((shift) ? (uint8_t)pgm_read_byte(&getSymKeysUp()[key - 0x2d]) : (uint8_t)pgm_read_byte(&getSymKeysLo()[key - 0x2d]));
  else if(VALUE_WITHIN(key, 0x54, 0x58))
    return (uint8_t)pgm_read_byte(&getPadKeys()[key - 0x54]);
  else {
    switch(key) {
      case UHS_HID_BOOT_KEY_SPACE: return (0x20);
      case UHS_HID_BOOT_KEY_ENTER: return ('\r'); // Carriage return (0x0D)
      case UHS_HID_BOOT_KEY_ZERO2: return ((bmNumLock == 1) ? '0': 0);
      case UHS_HID_BOOT_KEY_PERIOD: return ((bmNumLock == 1) ? '.': 0);
    }
  }
  return ( 0);
}
