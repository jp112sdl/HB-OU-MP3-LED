//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2019-02-15 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#include <AskSinPP.h>
#include <LowPower.h>

#include <Register.h>
#include <MultiChannelDevice.h>
#include <Switch.h>
#include <DFRobotDFPlayerMini.h>
#include <FastLED.h>
#include <SoftwareSerial.h>

#define CONFIG_BUTTON_PIN 6
#define DEVICE_LED_PIN    4
#define WS_DATA_PIN       7
#define DF_BUSY_PIN       5
#define DF_RX_PIN         8
#define DF_TX_PIN         9


#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
struct _LED {
  CRGB     LEDs[32];
  bool     isRunning  = false;
  uint8_t  FadeBPM    = 60;
  uint8_t  Count      = 16;
  uint32_t Color      = CRGB::Black;
  uint8_t  Brightness = 0;
} LED;

struct _DFPlayer {
  bool     isBusy      = false;
  uint8_t  Repeat      = 0;
  uint8_t  PlayNum     = 0;
  DFRobotDFPlayerMini Device;
} DFPlayer;

// number of available peers per channel
#define PEERS_PER_CHANNEL  8
#define CREG_LEDCOUNT      0x7F

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xf3, 0x44, 0x00},          // Device ID
  "JPMP300000",                // Device Serial
  {0xf3, 0x44},                // Device Model
  0x10,                        // Firmware Version
  as::DeviceType::OutputUnit,  // Device Type
  {0x00, 0x01}                 // Info Bytes
};

/**
   Configure the used hardware
*/
typedef AvrSPI<10, 11, 12, 13> RadioSPI;
typedef Radio<RadioSPI, 2> RadioType;
typedef StatusLed<DEVICE_LED_PIN> LedType;
typedef AskSin<LedType, NoBattery, RadioType> BaseHal;
//BaseHal hal;
class Hal: public BaseHal {
  public:
    void init(const HMID& id) {
      BaseHal::init(id);
      // 2165E8 == 868.35 MHz
      radio.initReg(CC1101_FREQ2, 0x21);
      radio.initReg(CC1101_FREQ1, 0x65);
      radio.initReg(CC1101_FREQ0, 0xE8);
    }

    bool runready () {
      return sysclock.runready() || BaseHal::runready();
    }
} hal;

DEFREGISTER(Reg0, MASTERID_REGS, DREG_LOCALRESETDISABLE)
class OUList0 : public RegList0<Reg0> {
  public:
    OUList0(uint16_t addr) : RegList0<Reg0>(addr) {}
    void defaults () {
      clear();
    }
};

DEFREGISTER(Reg1, CREG_LEDCOUNT )
class LEDList1 : public RegList1<Reg1> {
  public:
    LEDList1(uint16_t addr) : RegList1<Reg1>(addr) {}

    bool ledCount (uint8_t value) const {
      return this->writeRegister(CREG_LEDCOUNT, value & 0xff);
    }
    uint8_t ledCount () const {
      return this->readRegister(CREG_LEDCOUNT, 0);
    }

    void defaults () {
      clear();
      //ledCount(16);
    }
};

class LEDChannel : public ActorChannel<Hal, LEDList1, SwitchList3, PEERS_PER_CHANNEL, OUList0, SwitchStateMachine>  {
  private:
    bool first;
  protected:
    typedef ActorChannel<Hal, LEDList1, SwitchList3, PEERS_PER_CHANNEL, OUList0, SwitchStateMachine> BaseChannel;
  public:
    LEDChannel () : BaseChannel(), first(true), ledAlarm(*this)  {}
    virtual ~LEDChannel() {}

    class LEDTimerAlarm : public Alarm {
        LEDChannel& chan;
      public:
        LEDTimerAlarm (LEDChannel& c) : Alarm(0), chan(c) {}
        virtual ~LEDTimerAlarm () {}

        void trigger (__attribute__ ((unused)) AlarmClock& clock)  {
          chan.ledOff();
          chan.BaseChannel::set( 0x00, 0x00, 0xffff );
        }
    } ledAlarm;

    void ledOff() {
      LED.isRunning = false;
      LED.Color = CRGB::Black;
      fill_solid(LED.LEDs, LED.Count, LED.Color);
      FastLED.show();
    }

    void configChanged() {
      LED.Count = max(this->getList1().ledCount(), 1);
    }

    uint8_t flags () const {
      return 0;
    }

    bool process (const ActionSetMsg& msg) {
      BaseChannel::set( msg.value(), msg.ramp(), msg.delay() );
      return true;
    }

    bool process (const ActionCommandMsg& msg) {
      LED.Brightness = msg.value(1);
      LED.FadeBPM = msg.value(2);
      FastLED.setBrightness(LED.Brightness);

      switch (msg.value(3)) {
        case 0:
          LED.Color = CRGB::Black;
          break;
        case 11:
          LED.Color = CRGB::Red;
          break;
        case 21:
          LED.Color = CRGB::Green;
          break;
        case 31:
          LED.Color = CRGB::Yellow;
          break;
        case 41:
          LED.Color = CRGB::Blue;
          break;
        case 51:
          LED.Color = CRGB::Violet;
          break;
        case 61:
          LED.Color = CRGB::Turquoise;
          break;
        case 71:
          LED.Color = CRGB::White;
          break;
      }

      sysclock.cancel(ledAlarm);

      uint16_t t = ((msg.value(msg.len() - 2)) << 8) + (msg.value(msg.len() - 1));
      ledAlarm.set(AskSinBase::intTimeCvt(t));

      sysclock.add(ledAlarm);
      BaseChannel::set( 0xc8, 0x00, 0xffff );

      LED.isRunning = true;
      return true;
    }

    bool process (__attribute__((unused)) const RemoteEventMsg& msg) {
      return false;
    }

    void init() {
      FastLED.addLeds<LED_TYPE, WS_DATA_PIN, COLOR_ORDER>(LED.LEDs, LED.Count).setCorrection(TypicalLEDStrip);
      FastLED.setBrightness(0);
      fill_solid(LED.LEDs, LED.Count, CRGB::Black);
      FastLED.show();
      typename BaseChannel::List1 l1 = BaseChannel::getList1();
      this->set(l1.powerUpAction() == true ? 200 : 0, 0, 0xffff );
      this->changed(true);
      first = false;
    }

    virtual void switchState(__attribute__((unused)) uint8_t oldstate, __attribute__((unused)) uint8_t newstate, __attribute__((unused)) uint32_t delay) {
      if ( newstate == AS_CM_JT_OFF ) {
        if (first == false ) {
          this->ledOff();
        }
      }
      this->changed(true);
    }
};

SoftwareSerial DFSerial(DF_RX_PIN, DF_TX_PIN);
class MP3Channel : public SwitchChannel<Hal, PEERS_PER_CHANNEL, OUList0>  {
  private:
    bool first;
  protected:
    typedef SwitchChannel<Hal, PEERS_PER_CHANNEL, OUList0> BaseChannel;

  public:
    MP3Channel () : BaseChannel(), first(false), mp3Alarm(*this) {}
    virtual ~MP3Channel() {}
    class MP3TimerAlarm : public Alarm {
        MP3Channel& chan;
      public:
        MP3TimerAlarm (MP3Channel& c) : Alarm(0), chan(c) {}
        virtual ~MP3TimerAlarm () {}

        void trigger (__attribute__ ((unused)) AlarmClock& clock)  {
          chan.playOff();
        }
    } mp3Alarm;

    void playOff() {
      DFPlayer.Repeat = 0;
      DFPlayer.Device.stop();
      DFPlayer.Device.volume(0);
    }

    void switchOff() {
      DFPlayer.Repeat = 0;
      BaseChannel::set( 0x00, 0x00, 0xffff );
    }

    void checkBusy() {
      bool _isBusy = (digitalRead(DF_BUSY_PIN) == LOW);
      if (_isBusy == false && DFPlayer.isBusy == true) {
        if (DFPlayer.Repeat > 0) {
          DFPlayer.Repeat--;
          DFPlayer.Device.playMp3Folder(DFPlayer.PlayNum);
        } else {
          switchOff();
        }
      }
      DFPlayer.isBusy = _isBusy;
    }

    void configChanged() {
      DPRINTLN("MP3 List 1 Changed");
    }

    bool process (const ActionSetMsg& msg) {
      BaseChannel::set( msg.value(), msg.ramp(), msg.delay() );
      return true;
    }

    bool process (__attribute__((unused)) const RemoteEventMsg& msg) {
      return false;
    }

    bool process (const ActionCommandMsg& msg) {
      for (int i = 0; i < msg.len(); i++) {
        DHEX(msg.value(i)); DPRINT(" ");
      }
      DPRINTLN("");

      sysclock.cancel(mp3Alarm);

      uint8_t volume = msg.value(0);
      if (volume == 0x00) {
        playOff();
      } else {
        DFPlayer.Device.volume(map(volume, 0, 200, 0, 30));
      }

      uint8_t rept = msg.value(1);
      DFPlayer.Repeat = rept - 1;

      uint16_t t = ((msg.value(msg.len() - 2)) << 8) + (msg.value(msg.len() - 1));
      if (t != 0x83CA) {
        mp3Alarm.set(AskSinBase::intTimeCvt(t));
        sysclock.add(mp3Alarm);
      }

      uint8_t playNum = msg.value(2);
      DFPlayer.PlayNum = playNum;
      DFPlayer.Device.playMp3Folder(playNum);

      BaseChannel::set( 0xc8, 0x00, 0xffff );

      return true;
    }

    void init() {
      DFSerial.begin(9600);
      if (!DFPlayer.Device.begin(DFSerial)) {
        DPRINTLN("DFPlayer Init Error.");
        while (1) {  }
      }
      DPRINTLN("DFPlayer Mini online.");
      DFPlayer.Device.volume(0);
      typename BaseChannel::List1 l1 = BaseChannel::getList1();
      this->set(l1.powerUpAction() == true ? 200 : 0, 0, 0xffff );
      this->changed(true);
      first = false;
    }

    virtual void switchState(__attribute__((unused)) uint8_t oldstate, __attribute__((unused)) uint8_t newstate, __attribute__((unused)) uint32_t delay) {
      if ( newstate == AS_CM_JT_OFF ) {
        if (first == false ) {
          this->playOff();
        }
      }
      this->changed(true);
    }
};

class MP3LEDDevice : public ChannelDevice<Hal, VirtBaseChannel<Hal, OUList0>, 2, OUList0> {
  public:
    VirtChannel<Hal, LEDChannel, OUList0> c1;
    VirtChannel<Hal, MP3Channel, OUList0> c2;
  public:
    typedef ChannelDevice<Hal, VirtBaseChannel<Hal, OUList0>, 2, OUList0> DeviceType;
    MP3LEDDevice (const DeviceInfo& info, uint16_t addr) : DeviceType(info, addr) {
      DeviceType::registerChannel(c1, 1);
      DeviceType::registerChannel(c2, 2);
    }
    virtual ~MP3LEDDevice () {}

    LEDChannel& LedChannel()  {
      return c1;
    }
    MP3Channel& Mp3Channel()  {
      return c2;
    }

    virtual void configChanged () {}
};
MP3LEDDevice sdev(devinfo, 0x20);
ConfigButton<MP3LEDDevice> cfgBtn(sdev);

void setup () {
  pinMode(DF_BUSY_PIN, INPUT);
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  sdev.LedChannel().init();
  sdev.Mp3Channel().init();
  sdev.initDone();
  DDEVINFO(sdev);
}

void loop() {

  //Check if DFPlayer is playing
  EVERY_N_MILLISECONDS(200) {
    sdev.Mp3Channel().checkBusy();
  }

  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false && LED.isRunning == false && DFPlayer.isBusy == false) {
    hal.activity.savePower<Idle<>>(hal);
  } else if (LED.isRunning == true) {
    EVERY_N_MILLISECONDS(LED.FadeBPM / 3) {
      if (LED.Count > 1) {
        fadeToBlackBy(LED.LEDs, LED.Count, 25);
        byte pos = (beat8(LED.FadeBPM) * LED.Count) / 255;
        LED.LEDs[pos] = LED.Color;
      } else {
        fill_solid(LED.LEDs, LED.Count, LED.Color);
      }
    }
  }
  FastLED.show();

}

