//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2019-02-15 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// ci-test=yes board=328p aes=no

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
#define STATUS_LED_PIN    4
#define WS_DATA_PIN       7
#define DF_BUSY_PIN       5
#define DF_RX_PIN         8
#define DF_TX_PIN         9


#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
struct _LED {
  CRGB     Pixels[32];
  uint8_t  PixelCount = 16;
  bool     isRunning  = false;
  uint8_t  FadeBPM    = 60;
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
#define PEERS_PER_CHANNEL     8
#define CREG_INTERRUPTRUNNING 0x7E
#define CREG_LEDCOUNT         0x7F

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

typedef AvrSPI<10, 11, 12, 13> RadioSPI;
typedef Radio<RadioSPI, 2> RadioType;
typedef StatusLed<STATUS_LED_PIN> LedType;
typedef AskSin<LedType, NoBattery, RadioType> Hal;
Hal hal;

DEFREGISTER(OUReg0, MASTERID_REGS, DREG_LEDMODE)
class OUList0 : public RegList0<OUReg0> {
  public:
    OUList0(uint16_t addr) : RegList0<OUReg0>(addr) {}
    void defaults () {
      clear();
      ledMode(1);
    }
};

DEFREGISTER(LEDReg1, CREG_LEDCOUNT, CREG_INTERRUPTRUNNING)
class LEDList1 : public RegList1<LEDReg1> {
  public:
    LEDList1(uint16_t addr) : RegList1<LEDReg1>(addr) {}

    bool interruptRunning (uint8_t value) const {
      return this->writeRegister(CREG_INTERRUPTRUNNING, value);
    }
    bool interruptRunning () const {
      return this->readRegister(CREG_INTERRUPTRUNNING, true);
    }

    bool ledCount (uint8_t value) const {
      return this->writeRegister(CREG_LEDCOUNT, value & 0xff);
    }
    uint8_t ledCount () const {
      return this->readRegister(CREG_LEDCOUNT, 0);
    }

    void defaults () {
      clear();
      ledCount(16);
      interruptRunning(true);
    }
};

DEFREGISTER(MP3Reg1, CREG_INTERRUPTRUNNING)
class MP3List1 : public RegList1<MP3Reg1> {
  public:
    MP3List1(uint16_t addr) : RegList1<MP3Reg1>(addr) {}

    bool interruptRunning (uint8_t value) const {
      return this->writeRegister(CREG_INTERRUPTRUNNING, value);
    }
    bool interruptRunning () const {
      return this->readRegister(CREG_INTERRUPTRUNNING, true);
    }

    void defaults () {
      clear();
      interruptRunning(true);
    }
};

DEFREGISTER(OUReg3, SWITCH_LIST3_STANDARD_REGISTER, PREG_ACTTYPE, PREG_ACTNUM, PREG_ACTINTENS);
typedef RegList3<OUReg3> SwPeerListEx;
class OUList3 : public SwitchList3Tmpl<SwPeerListEx> {
  public:
    OUList3(uint16_t addr) : SwitchList3Tmpl<SwPeerListEx>(addr) {}
    void defaults() {
      SwitchList3Tmpl<SwPeerListEx>::defaults();
    }
    void even () {
      SwitchList3Tmpl<SwPeerListEx>::even();
    }
    void odd () {
      SwitchList3Tmpl<SwPeerListEx>::odd();
    }
    void single () {
      SwitchList3Tmpl<SwPeerListEx>::single();
    }
};

class LEDChannel : public ActorChannel<Hal, LEDList1, OUList3, PEERS_PER_CHANNEL, OUList0, SwitchStateMachine>  {
  private:
    bool first;
  protected:
    typedef ActorChannel<Hal, LEDList1, OUList3, PEERS_PER_CHANNEL, OUList0, SwitchStateMachine> BaseChannel;
  public:
    LEDChannel () : BaseChannel(), first(true), ledAlarm(*this)  {}
    virtual ~LEDChannel() {}

    class LEDTimerAlarm : public Alarm {
        LEDChannel& chan;
      public:
        LEDTimerAlarm (LEDChannel& c) : Alarm(0), chan(c) {}
        virtual ~LEDTimerAlarm () {}

        void trigger (__attribute__ ((unused)) AlarmClock& clock)  {
          chan.ledOff(true);

        }
    } ledAlarm;

    void ledOff(bool setCh) {
      sysclock.cancel(ledAlarm);
      LED.isRunning = false;
      LED.Color = CRGB::Black;
      fill_solid(LED.Pixels, LED.PixelCount, LED.Color);
      FastLED.show();
      if (setCh) BaseChannel::set( 0x00, 0x00, 0xffff );
    }

    void setLedOffDelay(uint16_t dly) {
      sysclock.cancel(ledAlarm);
      ledAlarm.set(dly);
      sysclock.add(ledAlarm);
    }

    void ledOn() {
      FastLED.setBrightness(LED.Brightness);
      BaseChannel::set( 0xc8, 0x00, 0xffff );
      LED.isRunning = true;
    }

    void setLedColor(uint8_t val) {
      switch (val) {
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
        case 81:
          LED.Color = CRGB::Orange;
          break;
      }
    }

    void setLedBrightness(uint8_t val) {
      LED.Brightness = val;
    }

    void setLedBPM(uint8_t val) {
      LED.FadeBPM = val;
    }

    bool process (const ActionSetMsg& msg) {
      BaseChannel::set( msg.value(), msg.ramp(), msg.delay() );
      return true;
    }

    bool process (const ActionCommandMsg& msg) {
      if ( (this->getList1().interruptRunning() == true && LED.isRunning == true) || LED.isRunning == false   ) {
        static uint8_t lastmsgcnt = 0;
        if (msg.count() != lastmsgcnt) {
          lastmsgcnt = msg.count();
          setLedBrightness(msg.value(1));
          setLedBPM(msg.value(2));
          uint8_t color = msg.value(3);

          if (color == 0) {
            ledOff(true);

          } else {
            setLedColor(color);

            uint16_t t = ((msg.value(msg.len() - 2)) << 8) + (msg.value(msg.len() - 1));
            if (t > 0 && t != 0x83CA) {
              setLedOffDelay(AskSinBase::intTimeCvt(t));
            }

            ledOn();
          }
        }
      }
      return true;
    }

    bool process (const RemoteEventMsg& msg) {
      if ( (this->getList1().interruptRunning() == true && LED.isRunning == true) || LED.isRunning == false   ) {
        bool lg = msg.isLong();
        Peer p(msg.peer());
        uint8_t cnt = msg.counter();
        OUList3 l3 = BaseChannel::getList3(p);
        if ( l3.valid() == true ) {
          typename OUList3::PeerList pl = lg ? l3.lg() : l3.sh();
          if ( lg == false || cnt != lastmsgcnt || pl.multiExec() == true ) {
            lastmsgcnt = cnt;
            //DPRINT(F("ACT_TYPE   ")); DHEXLN(pl.actType());  // Farbe
            //DPRINT(F("ACT_NUM    ")); DDECLN(pl.actNum());   // BPM
            //DPRINT(F("ACT_INTENS ")); DDECLN(pl.actIntens());// Helligkeit
            //DPRINT(F("OFFDELAY   ")); DDECLN(pl.offDly());   // Ausschaltverzögerung

            if (pl.actType() == 0) {
              ledOff(true);
            } else {
              setLedColor(pl.actType());
              setLedBrightness(pl.actIntens());
              setLedBPM(pl.actNum());
              if (pl.offDly() > 0)
                setLedOffDelay(AskSinBase::byteTimeCvt(pl.offDly()));
              ledOn();
            }
          }
          return true;
        }
        return false;
      }
      return true;
    }

    void init() {
      FastLED.addLeds<LED_TYPE, WS_DATA_PIN, COLOR_ORDER>(LED.Pixels, LED.PixelCount).setCorrection(TypicalLEDStrip);
      FastLED.setBrightness(64);

      uint32_t bootColors[3] = {CRGB::Red, CRGB::Green, CRGB::Blue };

      for (uint8_t i = 0; i < 3; i++) {
        fill_solid(LED.Pixels, LED.PixelCount, bootColors[i]);
        FastLED.show();
        _delay_ms(400);
      }

      fill_solid(LED.Pixels, LED.PixelCount, CRGB::Black);
      FastLED.show();

      ledOff(true);
      this->changed(true);
      first = false;
    }

    uint8_t flags () const {
      return 0;
    }

    void configChanged() {
      LED.PixelCount = max(this->getList1().ledCount(), 1);
      DPRINT(F("LED Interrupt Running: ")); DDECLN(this->getList1().interruptRunning());
    }

    virtual void switchState(__attribute__((unused)) uint8_t oldstate, __attribute__((unused)) uint8_t newstate, __attribute__((unused)) uint32_t delay) {
      if ( newstate == AS_CM_JT_OFF ) {
        if (first == false ) {
          this->ledOff(false);
        }
      }
      this->changed(true);
    }
};

SoftwareSerial DFSerial(DF_RX_PIN, DF_TX_PIN);
class MP3Channel : public ActorChannel<Hal, MP3List1, OUList3, PEERS_PER_CHANNEL, OUList0, SwitchStateMachine>  {
  private:
    bool first;
  protected:
    typedef ActorChannel<Hal, MP3List1, OUList3, PEERS_PER_CHANNEL, OUList0, SwitchStateMachine> BaseChannel;

  public:
    MP3Channel () : BaseChannel(), first(false), mp3Alarm(*this) {}
    virtual ~MP3Channel() {}
    class MP3TimerAlarm : public Alarm {
        MP3Channel& chan;
      public:
        MP3TimerAlarm (MP3Channel& c) : Alarm(0), chan(c) {}
        virtual ~MP3TimerAlarm () {}

        void trigger (__attribute__ ((unused)) AlarmClock& clock)  {
          // chan.playOff();
          chan.playStop(true);
        }
    } mp3Alarm;

    void playStop(bool setCh) {
      sysclock.cancel(mp3Alarm);
      DFPlayer.Repeat = 0;
      DFPlayer.Device.stop();
      DFPlayer.Device.volume(0);
      if (setCh) BaseChannel::set( 0x00, 0x00, 0xffff );
    }

    void checkBusy() {
      bool _isBusy = (digitalRead(DF_BUSY_PIN) == LOW);
      if (DFPlayer.isBusy != _isBusy) {
        if (_isBusy == false) {
          if (DFPlayer.Repeat > 0) {
            //DPRINT(F("checkBusy(): playStart, Repeat ="));DDECLN(DFPlayer.Repeat);
            DFPlayer.Repeat--;
            playStart(false);
          } else {
            //DPRINTLN(F("checkBusy(): playStop"));
            playStop(true);
          }
        }
      }
      DFPlayer.isBusy = _isBusy;
    }

    void setVolume(uint8_t val) {
      //DPRINT(F("setVolume val = ")); DDECLN(val);
      DFPlayer.Device.volume(map(val, 0, 200, 0, 30));
    }

    void setRepeat(uint8_t val) {
      DFPlayer.Repeat = val - 1;
    }

    void setMP3Num(uint8_t val) {
      DFPlayer.PlayNum = val;
    }

    void setPlayOffDelay(uint16_t val) {
      sysclock.cancel(mp3Alarm);
      mp3Alarm.set(val);
      sysclock.add(mp3Alarm);
    }

    void playStart(bool setChan) {
      //DPRINT(F("playStart ")); DDECLN(setChan);
      DFPlayer.Device.playMp3Folder(DFPlayer.PlayNum);
      if (setChan == true) BaseChannel::set( 0xc8, 0x00, 0xffff );
    }

    void playStart() {
      playStart(true);
    }

    bool process (const ActionSetMsg& msg) {
      BaseChannel::set( msg.value(), msg.ramp(), msg.delay() );
      return true;
    }

    bool process (const RemoteEventMsg& msg) {
      if ( (this->getList1().interruptRunning() == true && DFPlayer.isBusy == true) || DFPlayer.isBusy == false ) {
        bool lg = msg.isLong();
        Peer p(msg.peer());
        uint8_t cnt = msg.counter();
        OUList3 l3 = BaseChannel::getList3(p);
        if ( l3.valid() == true ) {
          typename OUList3::PeerList pl = lg ? l3.lg() : l3.sh();
          if ( lg == false || cnt != lastmsgcnt || pl.multiExec() == true ) {
            lastmsgcnt = cnt;
            //DPRINT(F("ACT_TYPE   ")); DDECLN(pl.actType());  //MP3 Nummer
            //DPRINT(F("ACT_NUM    ")); DDECLN(pl.actNum());   //Anzahl Durchläufe
            //DPRINT(F("ACT_INTENS ")); DDECLN(pl.actIntens());//Volume
            //DPRINT(F("OFFDELAY   ")); DDECLN(pl.offDly());

            if (pl.actIntens() == 0) {
              playStop(true);
            } else {
              setMP3Num(pl.actType());
              setVolume(pl.actIntens());
              setRepeat(pl.actNum());

              if (pl.offDly() > 0)
                setPlayOffDelay(AskSinBase::byteTimeCvt(pl.offDly()));

              playStart();
            }
          }
          return true;
        }
        return false;
      }
      return true;
    }

    bool process (const ActionCommandMsg& msg) {
      static uint8_t lastmsgcnt = 0;
      if (msg.count() != lastmsgcnt) {
        lastmsgcnt = msg.count();
        if ( (this->getList1().interruptRunning() == true && DFPlayer.isBusy == true) || DFPlayer.isBusy == false   ) {
          uint8_t volume = msg.value(0);
          if (volume == 0x00) {
            playStop(true);
            return true;
          } else {
            setVolume(volume);
          }

          uint8_t rept = msg.value(1);
          setRepeat(rept);

          uint8_t playNum = msg.value(2);
          setMP3Num(playNum);

          uint16_t t = ((msg.value(msg.len() - 2)) << 8) + (msg.value(msg.len() - 1));
          if (t != 0x83CA) {
            setPlayOffDelay(AskSinBase::intTimeCvt(t));
          }

          playStart();
        }
      }
      return true;
    }

    void init() {
      DFSerial.begin(9600);
      if (!DFPlayer.Device.begin(DFSerial)) {
        DPRINTLN(F("DFPlayer Init Error."));
        FastLED.setBrightness(64);
        fill_solid(LED.Pixels, LED.PixelCount, CRGB::Red);
        FastLED.show();
        while (1) {}
      } else {
        DPRINTLN(F("DFPlayer Mini online."));
        playStop(true);
      }

      this->changed(true);
      first = false;
    }

    uint8_t flags () const {
      return 0;
    }

    void configChanged() {
      DPRINT(F("MP3 Interrupt Running: ")); DDECLN(this->getList1().interruptRunning());
    }

    virtual void switchState(__attribute__((unused)) uint8_t oldstate, __attribute__((unused)) uint8_t newstate, __attribute__((unused)) uint32_t delay) {
      if ( newstate == AS_CM_JT_OFF ) {
        if (first == false ) {
          this->playStop(false);
        }
      }
      this->changed(true);
    }
};

class OUDevice : public ChannelDevice<Hal, VirtBaseChannel<Hal, OUList0>, 2, OUList0> {
  public:
    VirtChannel<Hal, LEDChannel, OUList0> c1;
    VirtChannel<Hal, MP3Channel, OUList0> c2;
  public:
    typedef ChannelDevice<Hal, VirtBaseChannel<Hal, OUList0>, 2, OUList0> DeviceType;
    OUDevice (const DeviceInfo& info, uint16_t addr) : DeviceType(info, addr) {
      DeviceType::registerChannel(c1, 1);
      DeviceType::registerChannel(c2, 2);
    }
    virtual ~OUDevice () {}

    LEDChannel& LedChannel()  {
      return c1;
    }
    MP3Channel& Mp3Channel()  {
      return c2;
    }

    virtual void configChanged () {}
};
OUDevice sdev(devinfo, 0x20);
ConfigButton<OUDevice> cfgBtn(sdev);

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

  //Check if DFPlayer is still playing
  EVERY_N_MILLISECONDS(200) {
    sdev.Mp3Channel().checkBusy();
  }

  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false && LED.isRunning == false && DFPlayer.isBusy == false) {
    hal.activity.savePower<Idle<>>(hal);
  } else if (LED.isRunning == true) {
    EVERY_N_MILLISECONDS(LED.FadeBPM / 3) {
      if (LED.PixelCount > 1) {
        fadeToBlackBy(LED.Pixels, LED.PixelCount, 25);
        byte pos = (beat8(LED.FadeBPM) * LED.PixelCount) / 255;
        LED.Pixels[pos] = LED.Color;
      } else {
        fill_solid(LED.Pixels, LED.PixelCount, LED.Color);
      }
    }
  }
  FastLED.show();
}

