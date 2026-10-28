package spc

import "errors"

// The SMP's registers at $F0-$FF.
const (
	smpTest       = 0x0
	smpControl    = 0x1
	smpDSPAddr    = 0x2
	smpDSPData    = 0x3
	smpPort0      = 0x4
	smpPort1      = 0x5
	smpPort2      = 0x6
	smpPort3      = 0x7
	smpRAM8       = 0x8
	smpRAM9       = 0x9
	smpTimer0     = 0xA // targets $FA-$FC
	smpCounter0   = 0xD // counters $FD-$FF
	smpRegCount   = 0x10
	smpRegBase    = 0xF0
	timerCount    = 3
	PortCount     = 4
	ramSize       = 0x10000
	ramPadding    = 0x100 // past the end, filled with STOP so a runaway PC stops
	romAddr       = 0xFFC0
	romSize       = 0x40
	padFill       = 0xFF
	controlROM    = 0x80
	controlPort01 = 0x10 // clear ports 0 and 1
	controlPort23 = 0x20

	noReadBeforeWrite = 0x2000 // added to written data when the write had no read before it
	tempoUnit         = 0x100
	timer2Shift       = 4  // 64 kHz
	otherTimerShift   = 3  // 8 kHz
	extraSamples      = 16 // the output starts with half of this in silence
)

// ipl is the SPC700's boot ROM, which receives uploads from the main CPU.
var ipl = [romSize]byte{
	0xCD, 0xEF, 0xBD, 0xE8, 0x00, 0xC6, 0x1D, 0xD0, 0xFC, 0x8F, 0xAA, 0xF4, 0x8F, 0xBB, 0xF5, 0x78,
	0xCC, 0xF4, 0xD0, 0xFB, 0x2F, 0x19, 0xEB, 0xF4, 0xD0, 0xFC, 0x7E, 0xF4, 0xD0, 0x0B, 0xE4, 0xF5,
	0xCB, 0xF4, 0xD7, 0x00, 0xFC, 0xD0, 0xF3, 0xAB, 0x01, 0x10, 0xEF, 0x7E, 0xF4, 0x10, 0xEB, 0xBA,
	0xF6, 0xDA, 0x00, 0xBA, 0xF4, 0xC4, 0xF4, 0xDD, 0x5D, 0xD0, 0xDB, 0x1F, 0x00, 0x00, 0xC0, 0xFF,
}

type timer struct {
	nextTime  int // time of the next tick
	prescaler int
	period    int
	divider   int
	enabled   bool
	counter   int
}

// SPC is the sound unit: processor, timers, DSP and their 64K RAM. Time is in clocks
// (clockRate per second) within the current frame.
type SPC struct {
	ram    []byte // ramSize, then ramPadding
	dsp    dsp
	timers [timerCount]timer
	regs   [smpRegCount]uint8 // as last written
	regsIn [smpRegCount]uint8 // as read: ports from the main CPU, counters

	pc, a, x, y, psw, sp int
	// while running: c carries in bit 8, nz holds N and Z, dp is the direct page (0 or 0x100)
	c, nz, dp int
	relTime   int

	dspTime     int
	spcTime     int
	tempo       int
	extraClocks int
	romEnabled  bool
	rom         [romSize]uint8
	hiRAM       [romSize]uint8 // the RAM under the ROM
	err         error
}

// ErrStopped is returned when the processor ran into STOP or SLEEP.
var ErrStopped = errors.New("spc: processor stopped")

// New returns a sound unit after power-on: the IPL ROM runs, waiting for an upload.
func New() *SPC {
	s := &SPC{ram: make([]byte, ramSize+ramPadding), tempo: tempoUnit, rom: ipl}
	s.dsp.init(s.ram) // with the padding: echo writes near the end spill into it
	s.Reset()
	return s
}

// Reset is power-on.
func (s *SPC) Reset() {
	for i := range s.ram[:ramSize] {
		s.ram[i] = padFill
	}
	s.ramLoaded()
	s.resetCommon(0x0F)
	s.dsp.reset()
}

// The file layout of an .spc snapshot.
const (
	spcSignature  = "SNES-SPC700 Sound File Data v0.30"
	spcPC         = 0x25
	spcA          = 0x27
	spcX          = 0x28
	spcY          = 0x29
	spcPSW        = 0x2A
	spcSP         = 0x2B
	spcRAM        = 0x100
	spcDSP        = spcRAM + ramSize
	spcMinSize    = spcDSP + dspRegisterCount
	spcSigCompare = 27
)

// LoadSPC sets the unit to the state in an .spc snapshot.
func (s *SPC) LoadSPC(b []byte) error {
	if len(b) < spcMinSize || string(b[:spcSigCompare]) != spcSignature[:spcSigCompare] {
		return errors.New("spc: not an SPC file")
	}
	s.pc = int(b[spcPC]) | int(b[spcPC+1])<<8
	s.a, s.x, s.y = int(b[spcA]), int(b[spcX]), int(b[spcY])
	s.psw, s.sp = int(b[spcPSW]), int(b[spcSP])
	copy(s.ram[:ramSize], b[spcRAM:spcRAM+ramSize])
	s.ramLoaded()
	s.dsp.load(b[spcDSP : spcDSP+dspRegisterCount])
	s.resetTimeRegs()
	return nil
}

func (s *SPC) ramLoaded() {
	s.romEnabled = false
	s.loadRegs(s.ram[smpRegBase : smpRegBase+smpRegCount])
	for i := ramSize; i < len(s.ram); i++ {
		s.ram[i] = padFill
	}
}

func (s *SPC) loadRegs(in []byte) {
	copy(s.regs[:], in)
	s.regsIn = s.regs
	// these always read back as 0
	s.regsIn[smpTest] = 0
	s.regsIn[smpControl] = 0
	for i := range timerCount {
		s.regsIn[smpTimer0+i] = 0
	}
}

func (s *SPC) resetCommon(counterInit uint8) {
	for i := range timerCount {
		s.regsIn[smpCounter0+i] = counterInit
	}
	s.pc, s.a, s.x, s.y, s.psw, s.sp = romAddr, 0, 0, 0, 0, 0
	s.regs[smpTest] = 0x0A
	s.regs[smpControl] = controlROM | controlPort01 | controlPort23
	for i := range PortCount {
		s.regsIn[smpPort0+i] = 0
	}
	s.resetTimeRegs()
}

func (s *SPC) resetTimeRegs() {
	s.err = nil
	s.spcTime = 0
	s.dspTime = 0
	for i := range s.timers {
		s.timers[i].nextTime = 1
		s.timers[i].divider = 0
	}
	s.enableROM(s.regs[smpControl]&controlROM != 0)
	s.timersLoaded()
	s.extraClocks = 0
	s.dsp.out = append(s.dsp.out[:0], make([]int16, extraSamples/2)...)
}

func (s *SPC) timersLoaded() {
	for i := range s.timers {
		t := &s.timers[i]
		t.period = ifZero256(int(s.regs[smpTimer0+i]))
		t.enabled = s.regs[smpControl]>>i&1 != 0
		t.counter = int(s.regsIn[smpCounter0+i]) & 0x0F
	}
	s.setTempo(s.tempo)
}

func (s *SPC) setTempo(t int) {
	s.tempo = t
	if t == 0 {
		t = 1
	}
	const timer2Rate = 1 << timer2Shift
	rate := (timer2Rate*tempoUnit + (t >> 1)) / t
	rate = max(rate, timer2Rate/4) // at most 4x tempo
	s.timers[2].prescaler = rate
	s.timers[1].prescaler = rate << otherTimerShift
	s.timers[0].prescaler = rate << otherTimerShift
}

// ifZero256 is n, or 256 for 0 (n is a byte).
func ifZero256(n int) int { return int(uint8(n-1)) + 1 }

func (s *SPC) enableROM(enable bool) {
	if s.romEnabled == enable {
		return
	}
	s.romEnabled = enable
	if enable {
		copy(s.hiRAM[:], s.ram[romAddr:romAddr+romSize])
		copy(s.ram[romAddr:romAddr+romSize], s.rom[:])
	} else {
		copy(s.ram[romAddr:romAddr+romSize], s.hiRAM[:])
	}
}

func (s *SPC) runTimerNow(t *timer, time int) {
	elapsed := (time-t.nextTime)/t.prescaler + 1
	t.nextTime += t.prescaler * elapsed
	if !t.enabled {
		return
	}
	remain := ifZero256(t.period - t.divider)
	divider := t.divider + elapsed
	if over := elapsed - remain; over >= 0 {
		n := over / t.period
		t.counter = (t.counter + 1 + n) & 0x0F
		divider = over - n*t.period
	}
	t.divider = int(uint8(divider))
}

func (s *SPC) runTimer(t *timer, time int) *timer {
	if time >= t.nextTime {
		s.runTimerNow(t, time)
	}
	return t
}

func (s *SPC) runDSP(time int) {
	if count := time - s.dspTime; count > 0 {
		s.dspTime = time
		s.dsp.run(count)
	}
}

func (s *SPC) dspRead(time int) int {
	s.runDSP(time)
	return s.dsp.read(int(s.regs[smpDSPAddr]) & (dspRegisterCount - 1))
}

func (s *SPC) dspWrite(data, time int) {
	s.runDSP(time)
	if r := int(s.regs[smpDSPAddr]); r < dspRegisterCount {
		s.dsp.write(r, data)
	}
}

func (s *SPC) writeSMPReg(data, time, reg int) {
	switch {
	case reg >= smpTimer0 && reg < smpTimer0+timerCount:
		t := &s.timers[reg-smpTimer0]
		if period := ifZero256(data); t.period != period {
			s.runTimer(t, time).period = period
		}
	case reg >= smpCounter0:
		if data < noReadBeforeWrite/2 {
			s.runTimer(&s.timers[reg-smpCounter0], time-1).counter = 0
		}
	case reg == smpRAM8 || reg == smpRAM9:
		s.regsIn[reg] = uint8(data)
	case reg == smpControl:
		if data&controlPort01 != 0 {
			s.regsIn[smpPort0] = 0
			s.regsIn[smpPort1] = 0
		}
		if data&controlPort23 != 0 {
			s.regsIn[smpPort2] = 0
			s.regsIn[smpPort3] = 0
		}
		for i := range s.timers {
			t := &s.timers[i]
			if enabled := data>>i&1 != 0; t.enabled != enabled {
				t = s.runTimer(t, time)
				t.enabled = enabled
				if enabled {
					t.divider = 0
					t.counter = 0
				}
			}
		}
		s.enableROM(data&controlROM != 0)
	}
}

// write is a processor write of data (a byte, maybe plus noReadBeforeWrite) to addr at time.
func (s *SPC) write(addr, data, time int) {
	s.ram[addr] = uint8(data)
	reg := addr - smpRegBase
	if reg < 0 {
		return
	}
	if reg < smpRegCount {
		s.regs[reg] = uint8(data)
		switch {
		case reg == smpDSPData:
			s.dspWrite(data, time)
		case reg == smpDSPAddr || reg >= smpPort0 && reg <= smpPort3:
		default:
			s.writeSMPReg(data, time, reg)
		}
		return
	}
	if i := addr - romAddr; i >= 0 {
		if i < romSize {
			s.hiRAM[i] = uint8(data)
			if s.romEnabled {
				s.ram[addr] = s.rom[i] // the ROM stays
			}
		} else { // wrapped past the end
			s.ram[addr] = padFill
			s.write(addr-ramSize, data, time)
		}
	}
}

// read is a processor read of addr at time.
func (s *SPC) read(addr, time int) int {
	switch {
	case addr >= smpRegBase+smpCounter0 && addr < smpRegBase+smpRegCount:
		t := &s.timers[addr-smpRegBase-smpCounter0]
		if time >= t.nextTime {
			s.runTimerNow(t, time)
		}
		result := t.counter
		t.counter = 0
		return result
	case addr >= smpRegBase && addr < smpRegBase+smpRegCount:
		reg := addr - smpRegBase
		switch reg {
		case smpDSPAddr:
			return int(s.regs[smpDSPAddr])
		case smpDSPData:
			return s.dspRead(time)
		}
		return int(s.regsIn[reg])
	case addr >= ramSize:
		return s.read(addr-ramSize, time)
	}
	return int(s.ram[addr])
}

// runUntil runs the processor to endTime.
func (s *SPC) runUntil(endTime int) {
	rel := s.spcTime - endTime
	s.spcTime = endTime
	s.dspTime += rel
	for i := range s.timers {
		s.timers[i].nextTime += rel
	}
	s.relTime = rel
	s.runCPU()
	rel = s.relTime
	s.spcTime += rel
	s.dspTime -= rel
	for i := range s.timers {
		s.timers[i].nextTime -= rel
	}
}

// ReadPort is what the processor last wrote to port (0..3), at time t of the frame.
func (s *SPC) ReadPort(t, port int) int {
	s.runUntil(t)
	return int(s.regs[smpPort0+port])
}

// WritePort puts data on port (0..3) for the processor, at time t of the frame.
func (s *SPC) WritePort(t, port, data int) {
	s.runUntil(t)
	s.regsIn[smpPort0+port] = uint8(data)
}

// endFrame ends a frame of endTime clocks.
func (s *SPC) endFrame(endTime int) {
	if endTime > s.spcTime {
		s.runUntil(endTime)
	}
	s.spcTime -= endTime
	s.extraClocks += endTime
	for i := range s.timers {
		s.runTimer(&s.timers[i], 0)
	}
	if s.dspTime < 0 {
		s.runDSP(0)
	}
}

// Play runs one frame of len(out)/2 stereo samples and writes them to out (left, right,
// ...). len(out) must be even.
func (s *SPC) Play(out []int16) error {
	if len(out) > 0 {
		s.extraClocks &= clocksPerSample - 1
		s.endFrame(len(out) * clocksPerSample / 2)
		n := copy(out, s.dsp.out)
		clear(out[n:])
		s.dsp.out = append(s.dsp.out[:0], s.dsp.out[n:]...)
	}
	err := s.err
	s.err = nil
	return err
}

// RAM is the unit's 64K memory, for the main CPU's side to fill as an upload would.
func (s *SPC) RAM() []byte { return s.ram[:ramSize] }
