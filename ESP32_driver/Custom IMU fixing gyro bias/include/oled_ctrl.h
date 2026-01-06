#include <Adafruit_SSD1306.h>

extern unsigned long currentTimeMillis;
extern unsigned long lastTimeMillis;

extern bool screenDefaultMode;
extern unsigned long currentTimeMillis;
// default
extern String screenLine_0;
extern String screenLine_1;
extern String screenLine_2;
extern String screenLine_3;

// custom
extern String customLine_0;
extern String customLine_1;
extern String customLine_2;
extern String customLine_3;

// #include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH   128 // OLED display width, in pixels
#define SCREEN_HEIGHT  32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

extern Adafruit_SSD1306 display;

void init_oled();
void oled_update();
void oledInfoUpdate();
void oledCtrl(byte, String);