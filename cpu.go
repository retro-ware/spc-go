package spc

// The SPC700 processor. Each instruction is charged its cycles up front; memory accesses
// happen at their clock within the instruction (the offset passed to read and write,
// relative to the instruction's end), which matters for timers and DSP registers.

// PSW flags; N and Z live in nz, C in bit 8 of c and P in dp while running.
const (
	flagN = 0x80
	flagV = 0x40
	flagP = 0x20
	flagB = 0x10
	flagH = 0x08
	flagI = 0x04
	flagZ = 0x02
	flagC = 0x01

	nzNegMask = 0x880 // either bit set: N
	carryBit  = 0x100
	stackPage = 0x100
	pcallPage = 0xFF00
	brkVector = 0xFFDE // TCALL n's vector is brkVector - 2n
)

// cycles per opcode, two per byte.
var cycleTable = func() (t [256]int) {
	packed := [128]uint8{
		0x28, 0x47, 0x34, 0x36, 0x26, 0x54, 0x54, 0x68,
		0x48, 0x47, 0x45, 0x56, 0x55, 0x65, 0x22, 0x46,
		0x28, 0x47, 0x34, 0x36, 0x26, 0x54, 0x54, 0x74,
		0x48, 0x47, 0x45, 0x56, 0x55, 0x65, 0x22, 0x38,
		0x28, 0x47, 0x34, 0x36, 0x26, 0x44, 0x54, 0x66,
		0x48, 0x47, 0x45, 0x56, 0x55, 0x45, 0x22, 0x43,
		0x28, 0x47, 0x34, 0x36, 0x26, 0x44, 0x54, 0x75,
		0x48, 0x47, 0x45, 0x56, 0x55, 0x55, 0x22, 0x36,
		0x28, 0x47, 0x34, 0x36, 0x26, 0x54, 0x52, 0x45,
		0x48, 0x47, 0x45, 0x56, 0x55, 0x55, 0x22, 0xC5,
		0x38, 0x47, 0x34, 0x36, 0x26, 0x44, 0x52, 0x44,
		0x48, 0x47, 0x45, 0x56, 0x55, 0x55, 0x22, 0x34,
		0x38, 0x47, 0x45, 0x47, 0x25, 0x64, 0x52, 0x49,
		0x48, 0x47, 0x56, 0x67, 0x45, 0x55, 0x22, 0x83,
		0x28, 0x47, 0x34, 0x36, 0x24, 0x53, 0x43, 0x40,
		0x48, 0x47, 0x45, 0x56, 0x34, 0x54, 0x22, 0x60,
	}
	for i, n := range packed {
		t[i*2] = int(n >> 4)
		t[i*2+1] = int(n & 0x0F)
	}
	return t
}()

func (s *SPC) getPSW() int {
	out := s.psw &^ (flagN | flagP | flagZ | flagC)
	out |= s.c >> 8 & flagC
	out |= s.dp >> 3 & flagP
	out |= ((s.nz >> 4) | s.nz) & flagN
	if uint8(s.nz) == 0 {
		out |= flagZ
	}
	return out
}

func (s *SPC) setPSW(in int) {
	s.psw = in
	s.c = in << 8
	s.dp = in << 3 & stackPage
	s.nz = (in << 4 & 0x800) | (^in & flagZ)
}

func (s *SPC) push(v int) {
	s.ram[stackPage+s.sp] = uint8(v)
	s.sp = (s.sp - 1) & 0xFF
}

func (s *SPC) pop() int {
	s.sp = (s.sp + 1) & 0xFF
	return int(s.ram[stackPage+s.sp])
}

func (s *SPC) push16(v int) {
	s.push(v >> 8)
	s.push(v)
}

func (s *SPC) pop16() int {
	lo := s.pop()
	return lo | s.pop()<<8
}

// word reads a little-endian word straight from RAM (no I/O), as program fetches do.
func (s *SPC) word(addr int) int { return int(s.ram[addr]) | int(s.ram[addr+1])<<8 }

// rd and wr access memory at offset clocks from the instruction's end.
func (s *SPC) rd(offset, addr int) int     { return s.read(addr, s.relTime+offset) }
func (s *SPC) wr(offset, addr, data int)   { s.write(addr, data, s.relTime+offset) }
func (s *SPC) rdDP(offset, addr int) int   { return s.rd(offset, s.dp+addr) }
func (s *SPC) wrDP(offset, addr, data int) { s.wr(offset, s.dp+addr, data) }
func (s *SPC) memBit(offset int) int {
	addr := s.word(s.pc)
	t := s.rd(offset, addr&0x1FFF) >> (addr >> 13)
	return t << 8 & carryBit
}

// Addressing modes of the ALU and MOV families, as offsets from the family's immediate opcode.
const (
	modeIndirectX = -0x02 // (X)
	modeIndirectY = 0x0F  // (dp)+Y
	modeIndexedIn = -0x01 // (dp+X)
	modeAbsY      = 0x0E
	modeAbsX      = 0x0D
	modeAbs       = -0x03
	modeDPX       = 0x0C
	modeDP        = -0x04
)

// operand returns the address an addressing mode names; data is the byte after the opcode
// and s.pc points at it. It leaves s.pc on the instruction's last byte.
func (s *SPC) operand(mode, data int) int {
	switch mode {
	case modeIndirectX:
		s.pc--
		return s.x + s.dp
	case modeIndirectY:
		return s.word(data+s.dp) + s.y
	case modeIndexedIn:
		return s.word(int(uint8(data+s.x)) + s.dp)
	case modeAbsY:
		s.pc++
		return data + s.y + 0x100*int(s.ram[s.pc])
	case modeAbsX:
		s.pc++
		return data + s.x + 0x100*int(s.ram[s.pc])
	case modeAbs:
		s.pc++
		return data + 0x100*int(s.ram[s.pc])
	case modeDPX:
		return int(uint8(data+s.x)) + s.dp
	}
	return data + s.dp
}

// family is the immediate opcode of an ALU or MOV family and the mode of op in it.
func family(op, base int) (int, bool) {
	switch m := op - base; m {
	case modeIndirectX, modeIndirectY, modeIndexedIn, modeAbsY, modeAbsX, modeAbs, modeDPX, modeDP:
		return m, true
	}
	return 0, false
}

func (s *SPC) branch(cond bool, data int) {
	s.pc++
	if cond {
		s.pc += int(int8(data))
		return
	}
	s.relTime -= 2
}

// cbranch is a compare-and-branch: s.pc is on the byte before the displacement.
func (s *SPC) cbranch(cond bool) {
	s.pc++
	if cond {
		s.pc += int(int8(s.ram[s.pc]))
	} else {
		s.relTime -= 2
	}
	s.pc++
}

func (s *SPC) adc(opcode, data, nz, addr int) {
	if opcode >= 0xA0 { // SBC
		data ^= 0xFF
	}
	flags := data ^ nz
	nz += data + (s.c >> 8 & 1)
	flags ^= nz
	s.psw = (s.psw &^ (flagV | flagH)) | (flags >> 1 & flagH) | ((flags + 0x80) >> 2 & flagV)
	s.c = nz
	s.nz = nz
	if addr < 0 {
		s.a = int(uint8(nz))
	} else {
		s.wr(0, addr, nz)
	}
	s.pc++
}

func (s *SPC) compare(reg, data int) {
	s.nz = reg - data
	s.c = ^s.nz
	s.nz &= 0xFF
}

// logic is AND (0x28), OR (0x08) or EOR (0x48) on a and b.
func logic(base, a, b int) int {
	switch base {
	case 0x28:
		return a & b
	case 0x08:
		return a | b
	}
	return a ^ b
}

func (s *SPC) runCPU() {
	s.setPSW(s.psw)
	for {
		if uint(s.pc) >= ramSize {
			s.pc &= 0xFFFF
		}
		opcode := int(s.ram[s.pc])
		if s.relTime += cycleTable[opcode]; s.relTime > 0 {
			s.relTime -= cycleTable[opcode] // not run: it would go past the end
			break
		}
		s.pc++
		data := int(s.ram[s.pc])
		if !s.step(opcode, data) {
			break
		}
	}
	s.psw = s.getPSW()
	s.pc &= 0xFFFF
	s.a, s.x, s.y = int(uint8(s.a)), int(uint8(s.x)), int(uint8(s.y))
}

// step runs one instruction with s.pc on the byte after the opcode; false when the
// processor stopped.
func (s *SPC) step(opcode, data int) bool {
	// the ALU and MOV families
	if m, ok := family(opcode, 0xE8); ok && opcode != 0xE4 { // MOV A,addr
		addr := s.operand(m, data)
		s.a = s.rd(0, addr)
		s.nz = s.a
		s.pc++
		return true
	}
	if m, ok := family(opcode, 0xC8); ok && opcode != 0xC4 { // MOV addr,A
		s.wr(0, s.operand(m, data), s.a)
		s.pc++
		return true
	}
	for _, base := range [...]int{0x28, 0x08, 0x48} {
		if m, ok := family(opcode, base); ok {
			s.a = logic(base, s.a, s.rd(0, s.operand(m, data)))
			s.nz = s.a
			s.pc++
			return true
		}
		switch opcode - base {
		case 0: // imm
			s.a = logic(base, s.a, data)
			s.nz = s.a
			s.pc++
			return true
		case 0x11: // (X),(Y)
			v := s.rdDP(-2, s.y)
			addr := s.x + s.dp
			s.nz = logic(base, v, s.rd(-1, addr))
			s.wr(0, addr, s.nz)
			return true
		case 0x01, 0x10: // dp,dp and dp,imm
			if opcode-base == 0x01 {
				data = s.rdDP(-3, data)
			}
			addr := int(s.ram[s.pc+1]) + s.dp
			s.pc += 2
			s.nz = logic(base, data, s.rd(-1, addr))
			s.wr(0, addr, s.nz)
			return true
		}
	}
	if m, ok := family(opcode, 0x68); ok { // CMP A,addr
		s.compare(s.a, s.rd(0, s.operand(m, data)))
		s.pc++
		return true
	}
	if m, ok := family(opcode&^0x20, 0x88); ok { // ADC/SBC A,addr
		v := s.rd(0, s.operand(m, data))
		s.adc(opcode, v, s.a, -1)
		return true
	}

	switch opcode {
	case 0xF0: // BEQ
		s.branch(uint8(s.nz) == 0, data)
	case 0xD0: // BNE
		s.branch(uint8(s.nz) != 0, data)
	case 0x3F: // CALL
		ret := s.pc + 2
		s.pc = s.word(s.pc)
		s.push16(ret)
	case 0x6F: // RET
		s.pc = s.pop16()
	case 0xE4: // MOV A,dp
		s.pc++
		s.a = s.rdDP(0, data)
		s.nz = s.a
	case 0xFA, 0x8F: // MOV dp,dp / MOV dp,#imm
		if opcode == 0xFA {
			data = s.rdDP(-2, data) + noReadBeforeWrite
		}
		dst := int(s.ram[s.pc+1])
		s.pc += 2
		s.wrDP(0, dst, data)
	case 0xC4: // MOV dp,A
		s.pc++
		s.wrDP(0, data, s.a)
	case 0xBF: // MOV A,(X)+
		addr := s.x + s.dp
		s.x = int(uint8(s.x + 1))
		s.a = s.rd(-1, addr)
		s.nz = s.a
	case 0xE8: // MOV A,imm
		s.a, s.nz = data, data
		s.pc++
	case 0xF9, 0xF8: // MOV X,dp+Y / dp
		if opcode == 0xF9 {
			data = int(uint8(data + s.y))
		}
		s.x = s.rdDP(0, data)
		s.nz = s.x
		s.pc++
	case 0xE9, 0xCD: // MOV X,abs / imm
		if opcode == 0xE9 {
			data = s.rd(0, s.word(s.pc))
			s.pc++
		}
		s.x, s.nz = data, data
		s.pc++
	case 0xFB, 0xEB: // MOV Y,dp+X / dp
		if opcode == 0xFB {
			data = int(uint8(data + s.x))
		}
		s.pc++
		s.y = s.rdDP(0, data)
		s.nz = s.y
	case 0xEC: // MOV Y,abs
		addr := s.word(s.pc)
		s.pc += 2
		s.y = s.rd(0, addr)
		s.nz = s.y
	case 0x8D: // MOV Y,imm
		s.y, s.nz = data, data
		s.pc++
	case 0xCC, 0xC9: // MOV abs,Y / X
		v := s.y
		if opcode == 0xC9 {
			v = s.x
		}
		s.wr(0, s.word(s.pc), v)
		s.pc += 2
	case 0xD9, 0xD8: // MOV dp+Y,X / dp,X
		if opcode == 0xD9 {
			data = int(uint8(data + s.y))
		}
		s.wr(0, data+s.dp, s.x)
		s.pc++
	case 0xDB, 0xCB: // MOV dp+X,Y / dp,Y
		if opcode == 0xDB {
			data = int(uint8(data + s.x))
		}
		s.wr(0, data+s.dp, s.y)
		s.pc++
	case 0x7D: // MOV A,X
		s.a, s.nz = s.x, s.x
	case 0xDD: // MOV A,Y
		s.a, s.nz = s.y, s.y
	case 0x5D: // MOV X,A
		s.x, s.nz = s.a, s.a
	case 0xFD: // MOV Y,A
		s.y, s.nz = s.a, s.a
	case 0x9D: // MOV X,SP
		s.x, s.nz = s.sp, s.sp
	case 0xBD: // MOV SP,X
		s.sp = s.x
	case 0xAF: // MOV (X)+,A
		s.wrDP(0, s.x, s.a+noReadBeforeWrite)
		s.x = int(uint8(s.x + 1))

	case 0x68: // CMP A,imm
		s.compare(s.a, data)
		s.pc++
	case 0x79: // CMP (X),(Y)
		v := s.rdDP(-2, s.y)
		s.compare(s.rdDP(-1, s.x), v)
	case 0x69, 0x78: // CMP dp,dp / dp,imm
		if opcode == 0x69 {
			data = s.rdDP(-3, data)
		}
		s.pc++
		s.compare(s.rdDP(-1, int(s.ram[s.pc])), data)
		s.pc++
	case 0x3E, 0x1E, 0xC8: // CMP X,dp / abs / imm
		switch opcode {
		case 0x3E:
			data = s.rd(0, data+s.dp)
		case 0x1E:
			data = s.rd(0, s.word(s.pc))
			s.pc++
		}
		s.compare(s.x, data)
		s.pc++
	case 0x7E, 0x5E, 0xAD: // CMP Y,dp / abs / imm
		switch opcode {
		case 0x7E:
			data = s.rd(0, data+s.dp)
		case 0x5E:
			data = s.rd(0, s.word(s.pc))
			s.pc++
		}
		s.compare(s.y, data)
		s.pc++

	case 0xB9, 0x99: // SBC/ADC (X),(Y)
		s.pc-- // the adc's increment
		v := s.rdDP(-2, s.y)
		addr := s.x + s.dp
		s.adc(opcode, v, s.rd(-1, addr), addr)
	case 0xA9, 0x89, 0xB8, 0x98: // SBC/ADC dp,dp / dp,imm
		if opcode == 0xA9 || opcode == 0x89 {
			data = s.rdDP(-3, data)
		}
		s.pc++
		addr := int(s.ram[s.pc]) + s.dp
		s.adc(opcode, data, s.rd(-1, addr), addr)
	case 0xA8, 0x88: // SBC/ADC imm
		s.adc(opcode, data, s.a, -1)

	case 0xBC: // INC A
		s.nz = s.a + 1
		s.a = int(uint8(s.nz))
	case 0x3D: // INC X
		s.nz = s.x + 1
		s.x = int(uint8(s.nz))
	case 0xFC: // INC Y
		s.nz = s.y + 1
		s.y = int(uint8(s.nz))
	case 0x9C: // DEC A
		s.nz = s.a - 1
		s.a = int(uint8(s.nz))
	case 0x1D: // DEC X
		s.nz = s.x - 1
		s.x = int(uint8(s.nz))
	case 0xDC: // DEC Y
		s.nz = s.y - 1
		s.y = int(uint8(s.nz))
	case 0x9B, 0xBB, 0x8B, 0xAB, 0x8C, 0xAC: // DEC/INC dp+X, dp, abs
		switch opcode {
		case 0x9B, 0xBB:
			data = int(uint8(data+s.x)) + s.dp
		case 0x8B, 0xAB:
			data += s.dp
		default:
			data = s.word(s.pc)
			s.pc++
		}
		s.nz = (opcode >> 4 & 2) - 1
		s.nz += s.rd(-1, data)
		s.wr(0, data, s.nz)
		s.pc++

	case 0x5C, 0x7C: // LSR A / ROR A
		if opcode == 0x5C {
			s.c = 0
		}
		s.nz = (s.c >> 1 & 0x80) | (s.a >> 1)
		s.c = s.a << 8
		s.a = s.nz
	case 0x1C, 0x3C: // ASL A / ROL A
		if opcode == 0x1C {
			s.c = 0
		}
		carry := s.c >> 8 & 1
		s.c = s.a << 1
		s.nz = s.c | carry
		s.a = int(uint8(s.nz))
	case 0x0B, 0x1B, 0x3B, 0x2B, 0x0C, 0x2C: // ASL/ROL dp, dp+X, abs
		switch opcode {
		case 0x0B:
			s.c = 0
			data += s.dp
		case 0x1B, 0x3B:
			if opcode == 0x1B {
				s.c = 0
			}
			data = int(uint8(data+s.x)) + s.dp
		case 0x2B:
			data += s.dp
		default:
			if opcode == 0x0C {
				s.c = 0
			}
			data = s.word(s.pc)
			s.pc++
		}
		s.nz = s.c >> 8 & 1
		s.c = s.rd(-1, data) << 1
		s.nz |= s.c
		s.wr(0, data, s.nz)
		s.pc++
	case 0x4B, 0x5B, 0x7B, 0x6B, 0x4C, 0x6C: // LSR/ROR dp, dp+X, abs
		switch opcode {
		case 0x4B:
			s.c = 0
			data += s.dp
		case 0x5B, 0x7B:
			if opcode == 0x5B {
				s.c = 0
			}
			data = int(uint8(data+s.x)) + s.dp
		case 0x6B:
			data += s.dp
		default:
			if opcode == 0x4C {
				s.c = 0
			}
			data = s.word(s.pc)
			s.pc++
		}
		v := s.rd(-1, data)
		s.nz = (s.c >> 1 & 0x80) | (v >> 1)
		s.c = v << 8
		s.wr(0, data, s.nz)
		s.pc++
	case 0x9F: // XCN
		s.a = (s.a >> 4) | int(uint8(s.a<<4))
		s.nz = s.a

	case 0xBA: // MOVW YA,dp
		s.a = s.rdDP(-2, data)
		s.nz = (s.a & 0x7F) | (s.a >> 1)
		s.y = s.rdDP(0, int(uint8(data+1)))
		s.nz |= s.y
		s.pc++
	case 0xDA: // MOVW dp,YA
		s.wrDP(-1, data, s.a)
		s.wrDP(0, int(uint8(data+1)), s.y+noReadBeforeWrite)
		s.pc++
	case 0x3A, 0x1A: // INCW / DECW dp
		data += s.dp
		v := s.rd(-3, data) + (opcode >> 4 & 2) - 1
		s.nz = ((v >> 1) | v) & 0x7F
		s.wr(-2, data, v)
		data = int(uint8(data+1)) + s.dp
		v = int(uint8((v >> 8) + s.rd(-1, data)))
		s.nz |= v
		s.wr(0, data, v)
		s.pc++
	case 0x7A, 0x9A: // ADDW / SUBW YA,dp
		lo := s.rdDP(-2, data)
		hi := s.rdDP(0, int(uint8(data+1)))
		if opcode == 0x9A {
			lo = (lo ^ 0xFF) + 1
			hi ^= 0xFF
		}
		lo += s.a
		result := s.y + hi + (lo >> 8)
		flags := hi ^ s.y ^ result
		s.psw = (s.psw &^ (flagV | flagH)) | (flags >> 1 & flagH) | ((flags + 0x80) >> 2 & flagV)
		s.c = result
		s.a = int(uint8(lo))
		s.y = int(uint8(result))
		s.nz = (((lo >> 1) | lo) & 0x7F) | s.y
		s.pc++
	case 0x5A: // CMPW YA,dp
		v := s.a - s.rdDP(-1, data)
		s.nz = ((v >> 1) | v) & 0x7F
		v = s.y + (v >> 8)
		v -= s.rdDP(0, int(uint8(data+1)))
		s.nz |= v
		s.c = ^v
		s.nz &= 0xFF
		s.pc++

	case 0xCF: // MUL YA
		v := s.y * s.a
		s.a = int(uint8(v))
		s.nz = ((v >> 1) | v) & 0x7F
		s.y = v >> 8
		s.nz |= s.y
	case 0x9E: // DIV YA,X
		ya := s.y*0x100 + s.a
		s.psw &^= flagH | flagV
		if s.y >= s.x {
			s.psw |= flagV
		}
		if s.y&15 >= s.x&15 {
			s.psw |= flagH
		}
		if s.y < s.x*2 {
			s.a = ya / s.x
			s.y = ya - s.a*s.x
		} else {
			s.a = 255 - (ya-s.x*0x200)/(256-s.x)
			s.y = s.x + (ya-s.x*0x200)%(256-s.x)
		}
		s.a = int(uint8(s.a))
		s.nz = s.a
	case 0xDF: // DAA
		if s.a > 0x99 || s.c&carryBit != 0 {
			s.a += 0x60
			s.c = carryBit
		}
		if s.a&0x0F > 9 || s.psw&flagH != 0 {
			s.a += 0x06
		}
		s.nz = s.a
		s.a = int(uint8(s.a))
	case 0xBE: // DAS
		if s.a > 0x99 || s.c&carryBit == 0 {
			s.a -= 0x60
			s.c = 0
		}
		if s.a&0x0F > 9 || s.psw&flagH == 0 {
			s.a -= 0x06
		}
		s.nz = s.a
		s.a = int(uint8(s.a))

	case 0x2F: // BRA
		s.pc += int(int8(data)) + 1
	case 0x30: // BMI
		s.branch(s.nz&nzNegMask != 0, data)
	case 0x10: // BPL
		s.branch(s.nz&nzNegMask == 0, data)
	case 0xB0: // BCS
		s.branch(s.c&carryBit != 0, data)
	case 0x90: // BCC
		s.branch(s.c&carryBit == 0, data)
	case 0x70: // BVS
		s.branch(s.psw&flagV != 0, data)
	case 0x50: // BVC
		s.branch(s.psw&flagV == 0, data)
	case 0x03, 0x23, 0x43, 0x63, 0x83, 0xA3, 0xC3, 0xE3: // BBS dp.bit,rel
		s.cbranch(s.rdDP(-4, data)>>(opcode>>5)&1 != 0)
	case 0x13, 0x33, 0x53, 0x73, 0x93, 0xB3, 0xD3, 0xF3: // BBC dp.bit,rel
		s.cbranch(s.rdDP(-4, data)>>(opcode>>5)&1 == 0)
	case 0xDE, 0x2E: // CBNE dp+X,rel / dp,rel
		if opcode == 0xDE {
			data = int(uint8(data + s.x))
		}
		s.cbranch(s.rdDP(-4, data) != s.a)
	case 0x6E: // DBNZ dp,rel
		v := s.rdDP(-4, data) - 1
		s.wrDP(-3, data, v+noReadBeforeWrite)
		s.cbranch(v != 0)
	case 0xFE: // DBNZ Y,rel
		s.y = int(uint8(s.y - 1))
		s.branch(s.y != 0, data)
	case 0x1F: // JMP [abs+X]
		s.pc = s.word(s.word(s.pc) + s.x)
	case 0x5F: // JMP abs
		s.pc = s.word(s.pc)

	case 0x0F: // BRK
		ret := s.pc
		s.pc = s.word(brkVector)
		s.push16(ret)
		p := s.getPSW()
		s.psw = (s.psw | flagB) &^ flagI
		s.push(p)
	case 0x4F: // PCALL
		ret := s.pc + 1
		s.pc = pcallPage | data
		s.push16(ret)
	case 0x01, 0x11, 0x21, 0x31, 0x41, 0x51, 0x61, 0x71, 0x81, 0x91, 0xA1, 0xB1, 0xC1, 0xD1, 0xE1, 0xF1: // TCALL n
		ret := s.pc
		s.pc = s.word(brkVector - opcode>>3)
		s.push16(ret)

	case 0x7F: // RET1
		s.setPSW(s.pop())
		s.pc = s.pop16()
	case 0x8E: // POP PSW
		s.setPSW(s.pop())
	case 0x0D: // PUSH PSW
		s.push(s.getPSW())
	case 0x2D: // PUSH A
		s.push(s.a)
	case 0x4D: // PUSH X
		s.push(s.x)
	case 0x6D: // PUSH Y
		s.push(s.y)
	case 0xAE: // POP A
		s.a = s.pop()
	case 0xCE: // POP X
		s.x = s.pop()
	case 0xEE: // POP Y
		s.y = s.pop()

	case 0x02, 0x22, 0x42, 0x62, 0x82, 0xA2, 0xC2, 0xE2, // SET1
		0x12, 0x32, 0x52, 0x72, 0x92, 0xB2, 0xD2, 0xF2: // CLR1
		bit := 1 << (opcode >> 5)
		mask := ^bit
		if opcode&0x10 != 0 {
			bit = 0
		}
		data += s.dp
		s.wr(0, data, s.rd(-1, data)&mask|bit)
		s.pc++
	case 0x0E, 0x4E: // TSET1 / TCLR1 abs
		addr := s.word(s.pc)
		s.pc += 2
		v := s.rd(-2, addr)
		s.nz = int(uint8(s.a - v))
		v &^= s.a
		if opcode == 0x0E {
			v |= s.a
		}
		s.wr(0, addr, v)
	case 0x4A: // AND1 C,mem.bit
		s.c &= s.memBit(0)
		s.pc += 2
	case 0x6A: // AND1 C,/mem.bit
		s.c &= ^s.memBit(0)
		s.pc += 2
	case 0x0A: // OR1 C,mem.bit
		s.c |= s.memBit(-1)
		s.pc += 2
	case 0x2A: // OR1 C,/mem.bit
		s.c |= ^s.memBit(-1)
		s.pc += 2
	case 0x8A: // EOR1 C,mem.bit
		s.c ^= s.memBit(-1)
		s.pc += 2
	case 0xEA: // NOT1 mem.bit
		addr := s.word(s.pc)
		s.pc += 2
		v := s.rd(-1, addr&0x1FFF) ^ 1<<(addr>>13)
		s.wr(0, addr&0x1FFF, v)
	case 0xCA: // MOV1 mem.bit,C
		addr := s.word(s.pc)
		s.pc += 2
		v := s.rd(-2, addr&0x1FFF)
		bit := addr >> 13
		v = v&^(1<<bit) | (s.c>>8&1)<<bit
		s.wr(0, addr&0x1FFF, v+noReadBeforeWrite)
	case 0xAA: // MOV1 C,mem.bit
		s.c = s.memBit(0)
		s.pc += 2

	case 0x60: // CLRC
		s.c = 0
	case 0x80: // SETC
		s.c = ^0
	case 0xED: // NOTC
		s.c ^= carryBit
	case 0xE0: // CLRV
		s.psw &^= flagV | flagH
	case 0x20: // CLRP
		s.dp = 0
	case 0x40: // SETP
		s.dp = stackPage
	case 0xA0: // EI
		s.psw |= flagI
	case 0xC0: // DI
		s.psw &^= flagI
	case 0x00: // NOP
	case 0xFF, 0xEF: // STOP, SLEEP
		s.pc--
		s.relTime = 0
		s.err = ErrStopped
		return false
	default:
		panic("spc: unhandled opcode")
	}
	return true
}
