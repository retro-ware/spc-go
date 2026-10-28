// Package spc emulates the SNES sound unit: the SPC700 processor, its timers and the DSP that mixes
// eight BRR sample voices with echo. It is a Go port of blargg's snes_spc 0.9.0 (Shay Green, LGPL 2.1),
// kept step for step so its output matches the C library sample for sample.
package spc

// SampleRate is the DSP's output rate: one stereo pair every clocksPerSample clocks.
const (
	SampleRate      = 32000
	clockRate       = 1024000
	clocksPerSample = clockRate / SampleRate
)

const (
	dspRegisterCount = 128
	voiceCount       = 8
	voiceRegStride   = 16 // a voice's registers are at voice*16 + the v* offset
	brrBufSize       = 12 // decoded samples kept per voice (stored twice for wrap-around)
	brrBlockSize     = 9  // a BRR block: a header byte and 16 four-bit samples
	echoHistSize     = 8
	dspPhases        = 32 // DSP clocks per output sample
)

// Global DSP registers.
const (
	regMainVolL   = 0x0C
	regMainVolR   = 0x1C
	regEchoVolL   = 0x2C
	regEchoVolR   = 0x3C
	regKeyOn      = 0x4C
	regKeyOff     = 0x5C
	regFlags      = 0x6C
	regEndX       = 0x7C
	regEchoFeed   = 0x0D
	regPitchMod   = 0x2D
	regNoiseOn    = 0x3D
	regEchoOn     = 0x4D
	regSampleDir  = 0x5D
	regEchoStart  = 0x6D
	regEchoDelay  = 0x7D
	regFIR        = 0x0F // eight coefficients at 0x0F, 0x1F .. 0x7F
	globalRegMask = 0x0F // the low nibble that picks a register within a voice's 16
)

// Voice registers, offsets within a voice's 16.
const (
	vVolL   = 0x0
	vVolR   = 0x1
	vPitchL = 0x2
	vPitchH = 0x3
	vSource = 0x4
	vADSR0  = 0x5
	vADSR1  = 0x6
	vGain   = 0x7
	vEnvX   = 0x8
	vOutX   = 0x9
)

// FLG register bits.
const (
	flgSoftReset   = 0x80
	flgMute        = 0x40
	flgEchoDisable = 0x20
	flgNoiseRate   = 0x1F
)

type envMode int

const (
	envRelease envMode = iota
	envAttack
	envDecay
	envSustain
)

// Envelope constants (11-bit level).
const (
	envMax          = 0x7FF
	envReleaseStep  = 0x8
	envLinearStep   = 0x20
	envAttackFast   = 0x400
	envBentLimit    = 0x600 // GAIN mode 7 slows above this level
	envBentStep     = 0x8
	adsrEnable      = 0x80
	gainDirectModes = 4
)

type voice struct {
	buf       [brrBufSize * 2]int // decoded samples, twice for wrap handling
	bufPos    int                 // where the next four samples are decoded
	interpPos int                 // fractional position in the sample (0x1000 = 1.0)
	brrAddr   int                 // current BRR block
	brrOffset int                 // decoding offset in the block
	reg       int                 // index of the voice's first DSP register
	bit       int                 // 1 << voice number
	konDelay  int                 // KON setup phase
	envMode   envMode
	env       int // envelope level
	hiddenEnv int // GAIN mode 7 quirk
	envXOut   uint8
}

// dsp is the S-DSP. It reads samples from and writes echo into the shared 64K RAM.
type dsp struct {
	regs [dspRegisterCount]uint8

	echoHist    [echoHistSize * 2][2]int // newest 8 echo samples, twice for wrap handling
	echoHistPos int
	everyOther  int // toggles every sample
	kon         int // KON when last checked
	noise       int
	counter     int
	echoOffset  int // offset from ESA in the echo buffer
	echoLength  int
	phase       int // next clock to run, 0..31
	konCheck    bool

	newKon  int
	endxBuf uint8
	envxBuf uint8
	outxBuf uint8

	// latched once per sample or a few clocks ahead
	tPitchMod    int
	tNoiseOn     int
	tEchoOn      int
	tDir         int
	tKeyOff      int
	tBrrNextAddr int
	tADSR0       int
	tBrrHeader   int
	tBrrByte     int
	tSource      int
	tEchoStart   int
	tEchoEnabled int
	tDirAddr     int
	tPitch       int
	tOutput      int
	tLooped      int
	tEchoPtr     int
	tMainOut     [2]int
	tEchoOut     [2]int
	tEchoIn      [2]int

	voices [voiceCount]voice

	ram      []byte // 64K shared with the processor
	muteMask int
	out      []int16 // samples made and not yet taken, left and right
}

var initialDSPRegs = [dspRegisterCount]uint8{
	0x45, 0x8B, 0x5A, 0x9A, 0xE4, 0x82, 0x1B, 0x78, 0x00, 0x00, 0xAA, 0x96, 0x89, 0x0E, 0xE0, 0x80,
	0x2A, 0x49, 0x3D, 0xBA, 0x14, 0xA0, 0xAC, 0xC5, 0x00, 0x00, 0x51, 0xBB, 0x9C, 0x4E, 0x7B, 0xFF,
	0xF4, 0xFD, 0x57, 0x32, 0x37, 0xD9, 0x42, 0x22, 0x00, 0x00, 0x5B, 0x3C, 0x9F, 0x1B, 0x87, 0x9A,
	0x6F, 0x27, 0xAF, 0x7B, 0xE5, 0x68, 0x0A, 0xD9, 0x00, 0x00, 0x9A, 0xC5, 0x9C, 0x4E, 0x7B, 0xFF,
	0xEA, 0x21, 0x78, 0x4F, 0xDD, 0xED, 0x24, 0x14, 0x00, 0x00, 0x77, 0xB1, 0xD1, 0x36, 0xC1, 0x67,
	0x52, 0x57, 0x46, 0x3D, 0x59, 0xF4, 0x87, 0xA4, 0x00, 0x00, 0x7E, 0x44, 0x9C, 0x4E, 0x7B, 0xFF,
	0x75, 0xF5, 0x06, 0x97, 0x10, 0xC3, 0x24, 0xBB, 0x00, 0x00, 0x7B, 0x7A, 0xE0, 0x60, 0x12, 0x0F,
	0xF7, 0x74, 0x1C, 0xE5, 0x39, 0x3D, 0x73, 0xC1, 0x00, 0x00, 0x7A, 0xB3, 0xFF, 0x4E, 0x7B, 0xFF,
}

// clamp16 saturates v to the int16 range.
func clamp16(v int) int {
	if int(int16(v)) != v {
		return (v >> 31) ^ 0x7FFF
	}
	return v
}

func (d *dsp) init(ram []byte) {
	d.ram = ram
	d.muteMask = 0
	d.reset()
}

func (d *dsp) read(addr int) int { return int(d.regs[addr]) }

func (d *dsp) write(addr, data int) {
	d.regs[addr] = uint8(data)
	switch addr & globalRegMask {
	case vEnvX:
		d.envxBuf = uint8(data)
	case vOutX:
		d.outxBuf = uint8(data)
	case regKeyOn & globalRegMask:
		if addr == regKeyOn {
			d.newKon = int(uint8(data))
		}
		if addr == regEndX { // always cleared, whatever is written
			d.endxBuf = 0
			d.regs[regEndX] = 0
		}
	}
}

// Gaussian interpolation table.
var gauss = [512]int{
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2,
	2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5,
	6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10,
	11, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 17, 17,
	18, 19, 19, 20, 20, 21, 21, 22, 23, 23, 24, 24, 25, 26, 27, 27,
	28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 36, 36, 37, 38, 39, 40,
	41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56,
	58, 59, 60, 61, 62, 64, 65, 66, 67, 69, 70, 71, 73, 74, 76, 77,
	78, 80, 81, 83, 84, 86, 87, 89, 90, 92, 94, 95, 97, 99, 100, 102,
	104, 106, 107, 109, 111, 113, 115, 117, 118, 120, 122, 124, 126, 128, 130, 132,
	134, 137, 139, 141, 143, 145, 147, 150, 152, 154, 156, 159, 161, 163, 166, 168,
	171, 173, 175, 178, 180, 183, 186, 188, 191, 193, 196, 199, 201, 204, 207, 210,
	212, 215, 218, 221, 224, 227, 230, 233, 236, 239, 242, 245, 248, 251, 254, 257,
	260, 263, 267, 270, 273, 276, 280, 283, 286, 290, 293, 297, 300, 304, 307, 311,
	314, 318, 321, 325, 328, 332, 336, 339, 343, 347, 351, 354, 358, 362, 366, 370,
	374, 378, 381, 385, 389, 393, 397, 401, 405, 410, 414, 418, 422, 426, 430, 434,
	439, 443, 447, 451, 456, 460, 464, 469, 473, 477, 482, 486, 491, 495, 499, 504,
	508, 513, 517, 522, 527, 531, 536, 540, 545, 550, 554, 559, 563, 568, 573, 577,
	582, 587, 592, 596, 601, 606, 611, 615, 620, 625, 630, 635, 640, 644, 649, 654,
	659, 664, 669, 674, 678, 683, 688, 693, 698, 703, 708, 713, 718, 723, 728, 732,
	737, 742, 747, 752, 757, 762, 767, 772, 777, 782, 787, 792, 797, 802, 806, 811,
	816, 821, 826, 831, 836, 841, 846, 851, 855, 860, 865, 870, 875, 880, 884, 889,
	894, 899, 904, 908, 913, 918, 923, 927, 932, 937, 941, 946, 951, 955, 960, 965,
	969, 974, 978, 983, 988, 992, 997, 1001, 1005, 1010, 1014, 1019, 1023, 1027, 1032, 1036,
	1040, 1045, 1049, 1053, 1057, 1061, 1066, 1070, 1074, 1078, 1082, 1086, 1090, 1094, 1098, 1102,
	1106, 1109, 1113, 1117, 1121, 1125, 1128, 1132, 1136, 1139, 1143, 1146, 1150, 1153, 1157, 1160,
	1164, 1167, 1170, 1174, 1177, 1180, 1183, 1186, 1190, 1193, 1196, 1199, 1202, 1205, 1207, 1210,
	1213, 1216, 1219, 1221, 1224, 1227, 1229, 1232, 1234, 1237, 1239, 1241, 1244, 1246, 1248, 1251,
	1253, 1255, 1257, 1259, 1261, 1263, 1265, 1267, 1269, 1270, 1272, 1274, 1275, 1277, 1279, 1280,
	1282, 1283, 1284, 1286, 1287, 1288, 1290, 1291, 1292, 1293, 1294, 1295, 1296, 1297, 1297, 1298,
	1299, 1300, 1300, 1301, 1302, 1302, 1303, 1303, 1303, 1304, 1304, 1304, 1304, 1304, 1305, 1305,
}

func (d *dsp) interpolate(v *voice) int {
	offset := v.interpPos >> 4 & 0xFF
	fwd := 255 - offset
	rev := offset // mirror of the left half
	in := v.buf[(v.interpPos>>12)+v.bufPos:]
	out := (gauss[fwd] * in[0]) >> 11
	out += (gauss[fwd+256] * in[1]) >> 11
	out += (gauss[rev+256] * in[2]) >> 11
	out = int(int16(out))
	out += (gauss[rev] * in[3]) >> 11
	return clamp16(out) &^ 1
}

// Rate counters: rate r fires every counterRates[r] samples, offset by counterOffsets[r].
const simpleCounterRange = 2048 * 5 * 3

var counterRates = [32]int{
	simpleCounterRange + 1, // never fires
	2048, 1536,
	1280, 1024, 768,
	640, 512, 384,
	320, 256, 192,
	160, 128, 96,
	80, 64, 48,
	40, 32, 24,
	20, 16, 12,
	10, 8, 6,
	5, 4, 3,
	2,
	1,
}

var counterOffsets = [32]int{
	1, 0, 1040,
	536, 0, 1040,
	536, 0, 1040,
	536, 0, 1040,
	536, 0, 1040,
	536, 0, 1040,
	536, 0, 1040,
	536, 0, 1040,
	536, 0, 1040,
	536, 0, 1040,
	0,
	0,
}

func (d *dsp) runCounters() {
	if d.counter--; d.counter < 0 {
		d.counter = simpleCounterRange - 1
	}
}

func (d *dsp) readCounter(rate int) int {
	return (d.counter + counterOffsets[rate]) % counterRates[rate]
}

func (d *dsp) runEnvelope(v *voice) {
	env := v.env
	if v.envMode == envRelease {
		if env -= envReleaseStep; env < 0 {
			env = 0
		}
		v.env = env
		return
	}
	var rate int
	envData := int(d.regs[v.reg+vADSR1])
	if d.tADSR0&adsrEnable != 0 {
		if v.envMode >= envDecay {
			env--
			env -= env >> 8
			rate = envData & 0x1F
			if v.envMode == envDecay {
				rate = (d.tADSR0 >> 3 & 0x0E) + 0x10
			}
		} else { // attack
			rate = (d.tADSR0&0x0F)*2 + 1
			if rate < 31 {
				env += envLinearStep
			} else {
				env += envAttackFast
			}
		}
	} else { // GAIN
		envData = int(d.regs[v.reg+vGain])
		mode := envData >> 5
		if mode < gainDirectModes {
			env = envData * 0x10
			rate = 31
		} else {
			rate = envData & 0x1F
			switch {
			case mode == 4: // linear decrease
				env -= envLinearStep
			case mode < 6: // exponential decrease
				env--
				env -= env >> 8
			default: // linear increase, mode 7 bent
				env += envLinearStep
				if mode > 6 && uint(v.hiddenEnv) >= envBentLimit {
					env += envBentStep - envLinearStep
				}
			}
		}
	}
	// sustain level
	if env>>8 == envData>>5 && v.envMode == envDecay {
		v.envMode = envSustain
	}
	v.hiddenEnv = env
	// unsigned: a linear decrease going negative also triggers this
	if uint(env) > envMax {
		if env < 0 {
			env = 0
		} else {
			env = envMax
		}
		if v.envMode == envAttack {
			v.envMode = envDecay
		}
	}
	if d.readCounter(rate) == 0 {
		v.env = env
	}
}

func (d *dsp) decodeBRR(v *voice) {
	nybbles := d.tBrrByte*0x100 + int(d.ram[(v.brrAddr+v.brrOffset+1)&0xFFFF])
	header := d.tBrrHeader
	pos := v.bufPos
	if v.bufPos += 4; v.bufPos >= brrBufSize {
		v.bufPos = 0
	}
	for end := pos + 4; pos < end; pos, nybbles = pos+1, nybbles<<4 {
		s := int(int16(nybbles)) >> 12 // sign-extended nybble
		shift := header >> 4
		s = (s << shift) >> 1
		if shift >= 0xD { // invalid range
			s = (s >> 25) << 11
		}
		filter := header & 0x0C
		p1 := v.buf[pos+brrBufSize-1]
		p2 := v.buf[pos+brrBufSize-2] >> 1
		if filter >= 8 {
			s += p1
			s -= p2
			if filter == 8 { // s += p1 * 0.953125 - p2 * 0.46875
				s += p2 >> 4
				s += (p1 * -3) >> 6
			} else { // s += p1 * 0.8984375 - p2 * 0.40625
				s += (p1 * -13) >> 7
				s += (p2 * 3) >> 4
			}
		} else if filter != 0 { // s += p1 * 0.46875
			s += p1 >> 1
			s += (-p1) >> 5
		}
		s = int(int16(clamp16(s) * 2))
		v.buf[pos+brrBufSize] = s
		v.buf[pos] = s
	}
}

func (d *dsp) misc27() { d.tPitchMod = int(d.regs[regPitchMod]) &^ 1 } // voice 0 has no PMON

func (d *dsp) misc28() {
	d.tNoiseOn = int(d.regs[regNoiseOn])
	d.tEchoOn = int(d.regs[regEchoOn])
	d.tDir = int(d.regs[regSampleDir])
}

func (d *dsp) misc29() {
	if d.everyOther ^= 1; d.everyOther != 0 {
		d.newKon &^= d.kon // clears KON 63 clocks after it was last read
	}
}

func (d *dsp) misc30() {
	if d.everyOther != 0 {
		d.kon = d.newKon
		d.tKeyOff = int(d.regs[regKeyOff]) | d.muteMask
	}
	d.runCounters()
	if d.readCounter(int(d.regs[regFlags])&flgNoiseRate) == 0 {
		feedback := (d.noise << 13) ^ (d.noise << 14)
		d.noise = (feedback & 0x4000) ^ (d.noise >> 1)
	}
}

func (d *dsp) voiceV1(v *voice) {
	d.tDirAddr = d.tDir*0x100 + d.tSource*4
	d.tSource = int(d.regs[v.reg+vSource])
}

func (d *dsp) voiceV2(v *voice) {
	entry := d.tDirAddr
	if v.konDelay == 0 {
		entry += 2 // the loop address
	}
	d.tBrrNextAddr = int(d.ram[entry]) | int(d.ram[entry+1])<<8
	d.tADSR0 = int(d.regs[v.reg+vADSR0])
	d.tPitch = int(d.regs[v.reg+vPitchL]) // pitch is read over two clocks
}

func (d *dsp) voiceV3a(v *voice) { d.tPitch += (int(d.regs[v.reg+vPitchH]) & 0x3F) << 8 }

func (d *dsp) voiceV3b(v *voice) {
	d.tBrrByte = int(d.ram[(v.brrAddr+v.brrOffset)&0xFFFF])
	d.tBrrHeader = int(d.ram[v.brrAddr])
}

func (d *dsp) voiceV3c(v *voice) {
	if d.tPitchMod&v.bit != 0 { // pitch modulation by the previous voice's output
		d.tPitch += ((d.tOutput >> 5) * d.tPitch) >> 10
	}
	if v.konDelay != 0 {
		if v.konDelay == 5 { // start BRR decoding on the next sample
			v.brrAddr = d.tBrrNextAddr
			v.brrOffset = 1
			v.bufPos = 0
			d.tBrrHeader = 0 // ignored on this sample
			d.konCheck = true
		}
		v.env = 0 // the envelope never runs during KON
		v.hiddenEnv = 0
		v.interpPos = 0 // no decoding until the last three samples
		if v.konDelay--; v.konDelay&3 != 0 {
			v.interpPos = 0x4000
		}
		d.tPitch = 0
	}
	output := d.interpolate(v)
	if d.tNoiseOn&v.bit != 0 {
		output = int(int16(d.noise * 2))
	}
	d.tOutput = (output * v.env) >> 11 &^ 1
	v.envXOut = uint8(v.env >> 4)

	// silence at the end of a sample or on soft reset
	if int(d.regs[regFlags])&flgSoftReset != 0 || d.tBrrHeader&3 == 1 {
		v.envMode = envRelease
		v.env = 0
	}
	if d.everyOther != 0 {
		if d.tKeyOff&v.bit != 0 {
			v.envMode = envRelease
		}
		if d.kon&v.bit != 0 {
			v.konDelay = 5
			v.envMode = envAttack
		}
	}
	if v.konDelay == 0 {
		d.runEnvelope(v)
	}
}

func (d *dsp) voiceOutput(v *voice, ch int) {
	amp := (d.tOutput * int(int8(d.regs[v.reg+vVolL+ch]))) >> 7
	d.tMainOut[ch] = clamp16(d.tMainOut[ch] + amp)
	if d.tEchoOn&v.bit != 0 {
		d.tEchoOut[ch] = clamp16(d.tEchoOut[ch] + amp)
	}
}

func (d *dsp) voiceV4(v *voice) {
	d.tLooped = 0
	if v.interpPos >= 0x4000 {
		d.decodeBRR(v)
		if v.brrOffset += 2; v.brrOffset >= brrBlockSize {
			v.brrAddr = (v.brrAddr + brrBlockSize) & 0xFFFF
			if d.tBrrHeader&1 != 0 {
				v.brrAddr = d.tBrrNextAddr
				d.tLooped = v.bit
			}
			v.brrOffset = 1
		}
	}
	v.interpPos = (v.interpPos & 0x3FFF) + d.tPitch
	if v.interpPos > 0x7FFF { // keep from running ahead under pitch modulation
		v.interpPos = 0x7FFF
	}
	d.voiceOutput(v, 0)
}

func (d *dsp) voiceV5(v *voice) {
	d.voiceOutput(v, 1)
	endx := int(d.regs[regEndX]) | d.tLooped
	if v.konDelay == 5 {
		endx &^= v.bit
	}
	d.endxBuf = uint8(endx)
}

func (d *dsp) voiceV6(*voice) { d.outxBuf = uint8(d.tOutput >> 8) }

func (d *dsp) voiceV7(v *voice) {
	d.regs[regEndX] = d.endxBuf
	d.envxBuf = v.envXOut
}

func (d *dsp) voiceV8(v *voice) { d.regs[v.reg+vOutX] = d.outxBuf }

func (d *dsp) voiceV9(v *voice) { d.regs[v.reg+vEnvX] = d.envxBuf }

func (d *dsp) voiceV3(v *voice) {
	d.voiceV3a(v)
	d.voiceV3b(v)
	d.voiceV3c(v)
}

func (d *dsp) voiceV7V4V1(i int) {
	d.voiceV7(&d.voices[i])
	d.voiceV1(&d.voices[i+3])
	d.voiceV4(&d.voices[i+1])
}

func (d *dsp) voiceV8V5V2(i int) {
	d.voiceV8(&d.voices[i])
	d.voiceV5(&d.voices[i+1])
	d.voiceV2(&d.voices[i+2])
}

func (d *dsp) voiceV9V6V3(i int) {
	d.voiceV9(&d.voices[i])
	d.voiceV6(&d.voices[i+1])
	d.voiceV3(&d.voices[i+2])
}

// echoFIR is history sample i, 0 the oldest.
func (d *dsp) echoFIR(i int) *[2]int { return &d.echoHist[d.echoHistPos+i] }

func (d *dsp) calcFIR(i, ch int) int {
	return (d.echoFIR(i + 1)[ch] * int(int8(d.regs[regFIR+i*0x10]))) >> 6
}

func (d *dsp) echoRead(ch int) {
	p := d.tEchoPtr + ch*2
	s := int(int16(uint16(d.ram[p]) | uint16(d.ram[p+1])<<8))
	d.echoFIR(0)[ch] = s >> 1
	d.echoFIR(8)[ch] = s >> 1
}

func (d *dsp) echo22() {
	if d.echoHistPos++; d.echoHistPos >= echoHistSize {
		d.echoHistPos = 0
	}
	d.tEchoPtr = (d.tEchoStart*0x100 + d.echoOffset) & 0xFFFF
	d.echoRead(0)
	d.tEchoIn[0] = d.calcFIR(0, 0)
	d.tEchoIn[1] = d.calcFIR(0, 1)
}

func (d *dsp) echo23() {
	l := d.calcFIR(1, 0) + d.calcFIR(2, 0)
	r := d.calcFIR(1, 1) + d.calcFIR(2, 1)
	d.tEchoIn[0] += l
	d.tEchoIn[1] += r
	d.echoRead(1)
}

func (d *dsp) echo24() {
	d.tEchoIn[0] += d.calcFIR(3, 0) + d.calcFIR(4, 0) + d.calcFIR(5, 0)
	d.tEchoIn[1] += d.calcFIR(3, 1) + d.calcFIR(4, 1) + d.calcFIR(5, 1)
}

func (d *dsp) echo25() {
	l := int(int16(d.tEchoIn[0] + d.calcFIR(6, 0)))
	r := int(int16(d.tEchoIn[1] + d.calcFIR(6, 1)))
	l += int(int16(d.calcFIR(7, 0)))
	r += int(int16(d.calcFIR(7, 1)))
	d.tEchoIn[0] = clamp16(l) &^ 1
	d.tEchoIn[1] = clamp16(r) &^ 1
}

func (d *dsp) echoOutput(ch int) int {
	out := int(int16((d.tMainOut[ch]*int(int8(d.regs[regMainVolL+ch*0x10])))>>7)) +
		int(int16((d.tEchoIn[ch]*int(int8(d.regs[regEchoVolL+ch*0x10])))>>7))
	return clamp16(out)
}

func (d *dsp) echo26() {
	d.tMainOut[0] = d.echoOutput(0) // kept for the next clock, to output both together
	feedback := int(int8(d.regs[regEchoFeed]))
	l := d.tEchoOut[0] + int(int16((d.tEchoIn[0]*feedback)>>7))
	r := d.tEchoOut[1] + int(int16((d.tEchoIn[1]*feedback)>>7))
	d.tEchoOut[0] = clamp16(l) &^ 1
	d.tEchoOut[1] = clamp16(r) &^ 1
}

func (d *dsp) echo27() {
	l := d.tMainOut[0]
	r := d.echoOutput(1)
	d.tMainOut[0] = 0
	d.tMainOut[1] = 0
	if int(d.regs[regFlags])&flgMute != 0 {
		l, r = 0, 0
	}
	d.out = append(d.out, int16(l), int16(r))
}

func (d *dsp) echo28() { d.tEchoEnabled = int(d.regs[regFlags]) }

func (d *dsp) echoWrite(ch int) {
	if d.tEchoEnabled&flgEchoDisable == 0 {
		p := d.tEchoPtr + ch*2
		d.ram[p] = uint8(d.tEchoOut[ch])
		d.ram[p+1] = uint8(d.tEchoOut[ch] >> 8)
	}
	d.tEchoOut[ch] = 0
}

func (d *dsp) echo29() {
	d.tEchoStart = int(d.regs[regEchoStart])
	if d.echoOffset == 0 {
		d.echoLength = (int(d.regs[regEchoDelay]) & 0x0F) * 0x800
	}
	if d.echoOffset += 4; d.echoOffset >= d.echoLength {
		d.echoOffset = 0
	}
	d.echoWrite(0)
	d.tEchoEnabled = int(d.regs[regFlags])
}

func (d *dsp) echo30() { d.echoWrite(1) }

// clock runs DSP clock (phase) p of a sample.
func (d *dsp) clock(p int) {
	v := &d.voices
	switch p {
	case 0:
		d.voiceV5(&v[0])
		d.voiceV2(&v[1])
	case 1:
		d.voiceV6(&v[0])
		d.voiceV3(&v[1])
	case 2:
		d.voiceV7V4V1(0)
	case 3:
		d.voiceV8V5V2(0)
	case 4:
		d.voiceV9V6V3(0)
	case 5:
		d.voiceV7V4V1(1)
	case 6:
		d.voiceV8V5V2(1)
	case 7:
		d.voiceV9V6V3(1)
	case 8:
		d.voiceV7V4V1(2)
	case 9:
		d.voiceV8V5V2(2)
	case 10:
		d.voiceV9V6V3(2)
	case 11:
		d.voiceV7V4V1(3)
	case 12:
		d.voiceV8V5V2(3)
	case 13:
		d.voiceV9V6V3(3)
	case 14:
		d.voiceV7V4V1(4)
	case 15:
		d.voiceV8V5V2(4)
	case 16:
		d.voiceV9V6V3(4)
	case 17:
		d.voiceV1(&v[0])
		d.voiceV7(&v[5])
		d.voiceV4(&v[6])
	case 18:
		d.voiceV8V5V2(5)
	case 19:
		d.voiceV9V6V3(5)
	case 20:
		d.voiceV1(&v[1])
		d.voiceV7(&v[6])
		d.voiceV4(&v[7])
	case 21:
		d.voiceV8(&v[6])
		d.voiceV5(&v[7])
		d.voiceV2(&v[0]) // t_brr_next_addr order dependency
	case 22:
		d.voiceV3a(&v[0])
		d.voiceV9(&v[6])
		d.voiceV6(&v[7])
		d.echo22()
	case 23:
		d.voiceV7(&v[7])
		d.echo23()
	case 24:
		d.voiceV8(&v[7])
		d.echo24()
	case 25:
		d.voiceV3b(&v[0])
		d.voiceV9(&v[7])
		d.echo25()
	case 26:
		d.echo26()
	case 27:
		d.misc27()
		d.echo27()
	case 28:
		d.misc28()
		d.echo28()
	case 29:
		d.misc29()
		d.echo29()
	case 30:
		d.misc30()
		d.voiceV3c(&v[0])
		d.echo30()
	case 31:
		d.voiceV4(&v[0])
		d.voiceV1(&v[2])
	}
}

// run runs the DSP for clocks clocks (32 per output sample).
func (d *dsp) run(clocks int) {
	for ; clocks > 0; clocks-- {
		d.clock(d.phase)
		d.phase = (d.phase + 1) % dspPhases
	}
}

func (d *dsp) softResetCommon() {
	d.noise = 0x4000
	d.echoHistPos = 0
	d.everyOther = 1
	d.echoOffset = 0
	d.phase = 0
	d.counter = 0
}

// load resets the DSP with regs as its registers.
func (d *dsp) load(regs []byte) {
	ram, mute, out := d.ram, d.muteMask, d.out
	*d = dsp{ram: ram, muteMask: mute, out: out}
	copy(d.regs[:], regs)
	for i := range d.voices {
		v := &d.voices[i]
		v.brrOffset = 1
		v.bit = 1 << i
		v.reg = i * voiceRegStride
	}
	d.newKon = int(d.regs[regKeyOn])
	d.tDir = int(d.regs[regSampleDir])
	d.tEchoStart = int(d.regs[regEchoStart])
	d.softResetCommon()
}

func (d *dsp) reset() { d.load(initialDSPRegs[:]) }
