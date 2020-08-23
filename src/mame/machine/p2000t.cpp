// license:BSD-3-Clause
// copyright-holders:Paul Daniels
/**********************************************************************

  machine.c

  Functions to emulate general aspects of the machine (RAM, ROM, interrups,
  I/O ports)

**********************************************************************/

#include "emu.h"
#include "includes/p2000t.h"
#include <chrono>
#include <iostream>
#include <bitset>

#define P2000M_101F_CASDAT  0x01
#define P2000M_101F_CASCMD  0x02
#define P2000M_101F_CASREW  0x04
#define P2000M_101F_CASFOR  0x08
#define P2000M_101F_KEYINT  0x40
#define P2000M_101F_PRNOUT  0x80

#define P2000M_202F_PINPUT  0x01
#define P2000M_202F_PREADY  0x02
#define P2000M_202F_STRAPN  0x04
#define P2000M_202F_CASENB  0x08
#define P2000M_202F_CASPOS  0x10
#define P2000M_202F_CASEND  0x20
#define P2000M_202F_CASCLK  0x40
#define P2000M_202F_CASDAT  0x80

#define P2000M_303F_VIDEO   0x01

#define P2000M_707F_DISA    0x01



struct FakeCas {
  enum Direction { Stop, Rev, Fwd };

  Direction move{Stop};
  uint64_t counter{5000};
  std::vector<uint8_t> data;
  int write{true};
  int inpos{true};
};

static FakeCas fcas{};

TIMER_DEVICE_CALLBACK_MEMBER(p2000t_state::rdc_1)
{
	m_rdc_1 = true;

	if (fcas.move == FakeCas::Direction::Rev && fcas.counter > 0) {
		fcas.counter--;
	}
	if (fcas.move == FakeCas::Direction::Fwd) {
		fcas.counter++;
	}
}

/*
    Keyboard port 0x0x

    If the keyboard interrupt is enabled, all keyboard matrix rows are
    connected and reading from either of these ports will give the
    keyboard status (FF=no key pressed)

    If the keyboard interrupt is disabled, reading one of these ports
    will read the corresponding keyboard matrix row
*/
uint8_t p2000t_state::p2000t_port_000f_r(offs_t offset)
{
	if (m_port_101f & P2000M_101F_KEYINT)
	{
		return (
		m_keyboard[0]->read() & m_keyboard[1]->read() &
		m_keyboard[2]->read() & m_keyboard[3]->read() &
		m_keyboard[4]->read() & m_keyboard[5]->read() &
		m_keyboard[6]->read() & m_keyboard[7]->read() &
		m_keyboard[8]->read() & m_keyboard[9]->read());
	}
	else
	if (offset < 10)
	{
		return m_keyboard[offset]->read();
	}
	else
		return 0xff;
}

#define set_bit(l, x) l |= (1 << x);
/*
    Input port 0x2x

    bit 0 - Printer input
    bit 1 - Printer ready
    bit 2 - Strap N (daisy/matrix)
    bit 3 - Cassette write enabled
    bit 4 - Cassette in position
    bit 5 - Begin/end of tape
    bit 6 - Cassette read clock
    bit 7 - Cassette read data

	It looks like we need to flip the bits..
*/
uint8_t p2000t_state::p2000t_port_202f_r()
{
	/*
	Cassette input consists of three status signals, a data signal and a clock
	signal. The Write Enabe (WEN) signal is coming from the switch on the drive
	which is activated when a write enabled plug is added to the cassette
	medium. Cassette in Position (CIP) is active when a casette is loaded on the
	drive and the door is closed. Begin and End of Tape (BET) is signalled by
	the drive as a condition to stop the tape transport.

	The Read Data (RDA) from the casette is a serial bit pattern in Phase
	Encoded format. The Read Clock (RDC) is active at now of every new bit
	This clock-pulse is triggering the cassette timing flip-flop. The flip
	flop toggles on every clock pulse thus offering a timing signal (RDC 1)
	in phase with received data. The monitor program checks during a cassette
	read operation a change of the RDC 1 signal and then loads the value on
	RAD as a next bit.

	        1   0   1   1   0   0
	 RDA:  _----____--__----__--__--
     RDC:  _-___-___-___-___-___-___

	 A phase is 166 us. (W00t 6024 bits/s!)
	*/

	// std::cout << m_cassette->get_position();
	std::bitset<8> state;
	// cass available..
	state.set(4, 1);
	if (fcas.write) {
		state.set(3, 1);
	}
	// if (fcas.inpos)
	// 	set_bit(state, 4); // Cassette available?
	if (fcas.move == FakeCas::Direction::Rev) {
		fcas.counter--;
	}
	if (fcas.move == FakeCas::Direction::Fwd) {
		fcas.counter++;
	}

	if (fcas.counter == 0 || fcas.counter == fcas.data.size()) {
		if (fcas.move != FakeCas::Direction::Stop) {
			state.set(5, 1);
			std::cout << "Stop" << std::endl;
		}
	}
	if (m_rdc_1)
	{
		//set_bit(state, 6);
		m_rdc_1 = false;
	}
	if (fcas.counter < fcas.data.size() && fcas.data[fcas.counter]) {
		//set_bit(state, 7); // 0 bit?
	}
	// It looks like we need to flip the bits..
	uint8_t status = (~(state.to_ulong())) & 0xFF;
	return status;
}


/*
    Output Port 0x1x

    bit 0 - Cassette write data
    bit 1 - Cassette write command
    bit 2 - Cassette rewind
    bit 3 - Cassette forward
    bit 4 - Unused
    bit 5 - Unused
    bit 6 - Keyboard interrupt enable
    bit 7 - Printer output
*/
void p2000t_state::p2000t_port_101f_w(uint8_t data)
{
	m_port_101f = data;
	/*

	The CASSETTE is controlled by 4 output lines. Forward (FWD) and rewind
	(RWD) are two motorcontrol signals to activate the motor in either
	forward or reverse driection. Data is written via dt WDA line which is on
	the drive enabled when the WCD line is also active. The control of the
	motor and translation of data to a serial bitpattern in Phase Encoded (PE)
	format is controlled via routines in the Monitor - Rom.
	*/
	// auto now = std::chrono::microseconds(std::chrono::system_clock::now().time_since_epoch()).count();
	// if (data != 0 && data != 0x40)
	// 	std::cout << now << ":W 0x" << std::hex << (int) data << std::dec << std::endl;
	if (BIT(data, 0) || BIT(data, 1)) {
		std::cout << "W: " << data << "CWD|CWC" << std::endl;
	}
	if (BIT(data, 2)) {
		std::cout << "W: Rewind" << std::endl;
		fcas.data.resize(64 * 1024);
		for(int i = 0; i < fcas.data.size(); i++) {
			fcas.data[i] = i % 2;
		}
		fcas.move = FakeCas::Direction::Rev;
	}

	if (BIT(data, 3)) {
		std::cout <<"W: Forward" << std::endl;
		fcas.move = FakeCas::Direction::Fwd;
	}

	if (!BIT(data, 3) && !BIT(data, 2)) {
		fcas.move = FakeCas::Direction::Stop;
		std::cout <<  "W: STOP" << std::endl;
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
void p2000t_state::p2000t_port_303f_w(uint8_t data)
{
	m_port_303f = data;
}

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
void p2000t_state::p2000t_port_505f_w(uint8_t data)
{
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
void p2000t_state::p2000t_port_707f_w(uint8_t data)
{
	m_port_707f = data;
}

void p2000t_state::p2000t_port_888b_w(uint8_t data) {}
void p2000t_state::p2000t_port_8c90_w(uint8_t data) {}
void p2000t_state::p2000t_port_9494_w(uint8_t data) {}
