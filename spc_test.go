package spc

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"flag"
	"os"
	"path/filepath"
	"testing"
)

var update = flag.Bool("update", false, "rewrite testdata/tone.spc from toneSPC")

const tonePath = "testdata/tone.spc"

// toneSum is the SHA-256 of toneSeconds of tone.spc rendered by blargg's snes_spc 0.9.0, 32 stereo
// samples per spc_play call, as little-endian int16.
const (
	toneSum     = "d514b39f2cedfd06c9c5368854cf86eff5da77fe341245b85e211b5fe7c97445"
	toneSeconds = 10
	frameShorts = 64
)

// The test program, at programAddr. It keys on voices 0 and 1, then at every tick of timer 0
// (250 Hz) adds one to voice 0's pitch.
var program = []byte{
	0x8F, 0x4C, 0xF2, // mov $F2, #KON
	0x8F, 0x03, 0xF3, // mov $F3, #voices 0 and 1
	0x8F, 0x20, 0xFA, // mov $FA, #32: 8 kHz / 32
	0x8F, 0x01, 0xF1, // mov $F1, #timer 0 on
	0xE4, 0xFD, // wait: mov a, $FD
	0xF0, 0xFC, //       beq wait
	0x8F, 0x02, 0xF2, // mov $F2, #V0 PITCHL
	0xAB, 0x10, //       inc $10
	0xE4, 0x10, //       mov a, $10
	0xC4, 0xF3, //       mov $F3, a
	0x2F, 0xF1, //       bra wait
}

const (
	programAddr = 0x0400
	dirAddr     = 0x0200
	sampleAddr  = 0x1000
	echoPage    = 0x60
)

// toneSPC builds an .spc snapshot that runs program with a looping saw sample in all four BRR
// filters, an ADSR voice, a GAIN noise voice and echo through the FIR filter.
func toneSPC() []byte {
	b := make([]byte, spcMinSize)
	copy(b, spcSignature)
	b[0x21], b[0x22], b[0x23], b[0x24] = 26, 26, 27, 30
	binary.LittleEndian.PutUint16(b[spcPC:], programAddr)
	b[spcSP] = 0xEF
	ram := b[spcRAM : spcRAM+ramSize]
	copy(ram[programAddr:], program)
	binary.LittleEndian.PutUint16(ram[dirAddr:], sampleAddr)
	binary.LittleEndian.PutUint16(ram[dirAddr+2:], sampleAddr)
	for i, header := range []byte{0xB0, 0xB4, 0xB8, 0xBF} { // range 11, filters 0-3, last ends and loops
		block := ram[sampleAddr+i*brrBlockSize:]
		block[0] = header
		for j := 1; j < brrBlockSize; j++ {
			block[j] = byte(j*2-2)<<4 | byte(j*2-1)
		}
	}
	dsp := b[spcDSP:]
	const voice1 = voiceRegStride
	fir := func(i int) int { return regFIR + i*voiceRegStride }
	for reg, v := range map[int]byte{
		regMainVolL: 0x7F, regMainVolR: 0x60,
		regEchoVolL: 0x30, regEchoVolR: 0xD0,
		regEchoFeed:  0x40,
		regNoiseOn:   0x02, // voice 1
		regEchoOn:    0x01, // voice 0
		regSampleDir: dirAddr >> 8,
		regFlags:     0x1C, // noise clock
		regEchoStart: echoPage,
		regEchoDelay: 0x02,
		fir(0):       0x40, fir(1): 0x20, fir(2): 0x10, fir(6): 0xF0, fir(7): 0x08,
		vVolL: 0x60, vVolR: 0x40, vPitchH: 0x10, vADSR0: 0xCF, vADSR1: 0xB5,
		voice1 + vVolL: 0x20, voice1 + vVolR: 0xE0, voice1 + vPitchH: 0x08,
		voice1 + vGain: 0xDF, // bent increase
	} {
		dsp[reg] = v
	}
	return b
}

func render(t *testing.T, s *SPC, seconds int) []byte {
	t.Helper()
	out := make([]byte, 0, seconds*SampleRate*4)
	buf := make([]int16, frameShorts)
	for len(out) < cap(out) {
		if err := s.Play(buf); err != nil {
			t.Fatal(err)
		}
		for _, v := range buf {
			out = binary.LittleEndian.AppendUint16(out, uint16(v))
		}
	}
	return out
}

func TestMatchesC(t *testing.T) {
	if *update {
		if err := os.MkdirAll(filepath.Dir(tonePath), 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(tonePath, toneSPC(), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	b, err := os.ReadFile(tonePath)
	if err != nil {
		t.Fatal(err)
	}
	s := New()
	if err := s.LoadSPC(b); err != nil {
		t.Fatal(err)
	}
	sum := sha256.Sum256(render(t, s, toneSeconds))
	if got := hex.EncodeToString(sum[:]); got != toneSum {
		t.Errorf("output sum %s, want %s", got, toneSum)
	}
}

// TestIPLUpload sends a program through the boot ROM's handshake and checks that it runs.
func TestIPLUpload(t *testing.T) {
	const (
		ready, ready1 = 0xAA, 0xBB
		start         = 0xCC
		dest          = 0x0200
		marker        = 0x5A
	)
	prog := []byte{0x8F, marker, 0xF7, 0x2F, 0xFE} // mov $F7, #marker; bra *
	s := New()
	buf := make([]int16, frameShorts)
	wait := func(port, v int) {
		t.Helper()
		for range 100 {
			if s.ReadPort(0, port) == v {
				return
			}
			s.Play(buf)
		}
		t.Fatalf("port %d never read %#x", port, v)
	}
	wait(0, ready)
	wait(1, ready1)
	s.WritePort(0, 2, dest&0xFF)
	s.WritePort(0, 3, dest>>8)
	s.WritePort(0, 1, 1)
	s.WritePort(0, 0, start)
	wait(0, start)
	for i, v := range prog {
		s.WritePort(0, 1, int(v))
		s.WritePort(0, 0, i)
		wait(0, i)
	}
	s.WritePort(0, 2, dest&0xFF)
	s.WritePort(0, 3, dest>>8)
	s.WritePort(0, 1, 0)
	s.WritePort(0, 0, len(prog)+1)
	wait(3, marker)
	if got := s.RAM()[dest : dest+len(prog)]; string(got) != string(prog) {
		t.Errorf("RAM at %#x is % x, want % x", dest, got, prog)
	}
}
