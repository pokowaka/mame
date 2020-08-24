// license:BSD-3-Clause
// copyright-holders:Paul Daniels
/**********************************************************************

  machine.c

  Functions to emulate general aspects of the machine (RAM, ROM, interrups,
  I/O ports)

**********************************************************************/

#include "includes/p2000t.h"
#include "emu.h"
#include <bitset>
#include <chrono>
#include <iostream>

#define P2000M_101F_CASDAT 0x01
#define P2000M_101F_CASCMD 0x02
#define P2000M_101F_CASREW 0x04
#define P2000M_101F_CASFOR 0x08
#define P2000M_101F_KEYINT 0x40
#define P2000M_101F_PRNOUT 0x80

#define P2000M_202F_PINPUT 0x01
#define P2000M_202F_PREADY 0x02
#define P2000M_202F_STRAPN 0x04
#define P2000M_202F_CASENB 0x08
#define P2000M_202F_CASPOS 0x10
#define P2000M_202F_CASEND 0x20
#define P2000M_202F_CASCLK 0x40
#define P2000M_202F_CASDAT 0x80

#define P2000M_303F_VIDEO 0x01

#define P2000M_707F_DISA 0x01

struct MiniCas {
  MiniCas() { data.resize(4 * 1024 * 8); }
  enum Direction { Stop, Rev };
  Direction dir = Direction::Stop;
  std::vector<uint8_t> data;
  uint32_t pos = 512;

  bool valid() { return pos > 0 && pos < data.size(); }

  void fwd() {
    // TODO mutex.
    dir = Direction::Stop;
    if (pos < data.size()) {
      pos++;
    }
    std::cout << ">" << pos << std::endl;
  }

  void rev() { std::cout << "B" << std::endl; dir = Direction::Rev; }

  void move() {
    if (dir == Direction::Rev && pos > 0) {
      pos--;
      std::cout << "<" << pos << std::endl;
    }
  }

  void stop() {
	  // a stopped tape is valid.
    dir = Direction::Stop;
    if (pos == 0)
      pos = 1;
	if (pos == data.size())
	  pos = data.size() - 1;
  }

  void write(bool bit) {
    if (valid())
      data[pos - 1] = bit;
  }

  bool read() { return valid() && data[pos - 1] == 1; }
};

static MiniCas cas;

TIMER_DEVICE_CALLBACK_MEMBER(p2000t_state::rdc_1) {
  m_rdc_1 = !m_rdc_1;
  cas.move();
}

/*
    Keyboard port 0x0x

    If the keyboard interrupt is enabled, all keyboard matrix rows are
    connected and reading from either of these ports will give the
    keyboard status (FF=no key pressed)

    If the keyboard interrupt is disabled, reading one of these ports
    will read the corresponding keyboard matrix row
*/
uint8_t p2000t_state::p2000t_port_000f_r(offs_t offset) {
  if (m_port_101f & P2000M_101F_KEYINT) {
    return (m_keyboard[0]->read() & m_keyboard[1]->read() &
            m_keyboard[2]->read() & m_keyboard[3]->read() &
            m_keyboard[4]->read() & m_keyboard[5]->read() &
            m_keyboard[6]->read() & m_keyboard[7]->read() &
            m_keyboard[8]->read() & m_keyboard[9]->read());
  } else if (offset < 10) {
    return m_keyboard[offset]->read();
  } else
    return 0xff;
}

/*
    Input port 0x2x

    bit 0 - Printer input
    bit 1 - Printer ready
    bit 2 - Strap N (daisy/matrix)
    bit 3 - Cassette write enabled (WEN)
    bit 4 - Cassette in position   (CIP)
    bit 5 - Begin/end of tape      (BET)
    bit 6 - Cassette read clock    (RDC)
    bit 7 - Cassette read data     (RDA)

    It looks like we need to flip the bits upon return.
*/
uint8_t p2000t_state::p2000t_port_202f_r() {
  /*
The CASSETTE INPUT consists of three status signals, a data signal and a clock
signal. The Write Enable (WEN) signal is coming from the switch on the drive
which is activated when a write enable plug is added to the cassette medium.
Cassette In Position (CIP) is active when a cassette is loaded on the drive and
the door is closed. Begin and End of Tape (BET) is signalled by the drive as a
condition to stop the tape transport.


The Read Data (ROA) from the cassette is a serial bit pattern in Phase Encoded
format. The Read Clock (ROC) is active at start of every new bit. This
clockpulse is triggering the CASSETTE TIMING flip-flop. The flip flop toggles on
every clockpulse thus offering a timing signal (ROC 1) in phase with received
data. The monitor program checks during a cassette read operation a change of
the ROC 1 signal and then loads the value on RDA as a next bit.

          1   0   1   1   0   0
   RDA:  _----____--__----__--__--
   RDC:  _-___-___-___-___-___-___

   A phase is 166 us. (W00t 6024 bits/s!)
  */

  // std::cout << m_cassette->get_position();
  std::bitset<8> state;
  // Cassette is available, and always writeable.
  state.set(3, 1);
  state.set(4, 1);
  state.set(5, !cas.valid());
  state.set(6, m_rdc_1);
  state.set(7, cas.read());
  // It looks like we need to flip the bits..
  if (!cas.valid()) {
    std::cout << state << " " << (m_rdc_1 ? "H" : "L") << "  "
              << (cas.valid() ? "" : "E") << std::endl;
	// cas.stop();
  }
  uint8_t status = (~(state.to_ulong())) & 0xFF;
  return status;
}

/*
    Output Port 0x1x

    bit 0 - Cassette write data    (WDA)
    bit 1 - Cassette write command (WCD)
    bit 2 - Cassette rewind        (RWD)
    bit 3 - Cassette forward       (FWD)
    bit 4 - Unused
    bit 5 - Unused
    bit 6 - Keyboard interrupt enable
    bit 7 - Printer output
*/
void p2000t_state::p2000t_port_101f_w(uint8_t data) {
  // 08BD calls this.
  m_port_101f = data;
  std::bitset<8> state(data);

  /*
        The CASSETTE is controlled by 4 output lines. Forward (FWD) and rewind
        (RWD) are two motor control signals to activate the motor in either
     forward or reverse direction. Data is written to the cassette via the Write
     Data (WDA) line, which is on the drive enabled when the Write Command (WCD)
     line is also active. The control of the motor and translation of data to
        a--serial bitpattern in Phase Encoded (PE) format is controlled via
        routines in the Monitor -Rom.
  */

  if (state.test(1)) {
    std::cout << "W: " << state.test(0);
    cas.write(state.test(0));
  }

  if (state.test(2)) {
    // std::cout << "B" << std::endl;
    cas.rev();
  }

  if (state.test(3)) {
    // std::cout << "F" << std::endl;
    cas.fwd();
  }

  if (!state.test(2) && !state.test(3)) {
    cas.stop();
  }
}

/*
    Scroll Register 0x3x (P2000T only)

    bit 0 - /
    bit 1 - |
    bit 2 - | Index of the first character
    bit 3 - | to be displayed
    bit 4 - |
    bit 5 - |
    bit 6 - \
    bit 7 - Video disable (0 = enabled)
*/
void p2000t_state::p2000t_port_303f_w(uint8_t data) { m_port_303f = data; }

/*
    Beeper 0x5x

    bit 0 - Beeper
    bit 1 - Unused
    bit 2 - Unused
    bit 3 - Unused
    bit 4 - Unused
    bit 5 - Unused
    bit 6 - Unused
    bit 7 - Unused
*/
void p2000t_state::p2000t_port_505f_w(uint8_t data) {
  m_speaker->level_w(BIT(data, 0));
}

/*
    DISAS 0x7x (P2000M only)

    bit 0 - Unused
    bit 1 - DISAS enable
    bit 2 - Unused
    bit 3 - Unused
    bit 4 - Unused
    bit 5 - Unused
    bit 6 - Unused
    bit 7 - Unused

    When the DISAS is active, the CPU has the highest priority and
    video refresh is disabled when the CPU accesses video memory

*/
void p2000t_state::p2000t_port_707f_w(uint8_t data) { m_port_707f = data; }

void p2000t_state::p2000t_port_888b_w(uint8_t data) {}
void p2000t_state::p2000t_port_8c90_w(uint8_t data) {}
void p2000t_state::p2000t_port_9494_w(uint8_t data) {}
