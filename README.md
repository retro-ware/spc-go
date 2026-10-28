# spc-go

A pure-Go emulator of the SNES sound unit (the SPC700 processor, its timers and the S-DSP),
for playing SNES music and sound from the sound program itself rather than from recorded PCM.

It is a port of blargg's [snes_spc](https://github.com/elizagamedev/snes_spc) 0.9.0 (Shay Green),
kept step for step: its output matches the C library sample for sample. No cgo.

```go
s := spc.New()
if err := s.LoadSPC(data); err != nil { // an .spc snapshot
	return err
}
buf := make([]int16, 2*512) // stereo, 32 kHz (spc.SampleRate)
for {
	if err := s.Play(buf); err != nil {
		return err
	}
	// play buf
}
```

A game can also drive the unit as the SNES CPU would: start from `New()` (the boot ROM waits for
an upload), and talk to the sound program through `WritePort` and `ReadPort` between calls to `Play`.

## License

LGPL 2.1, as snes_spc. See LICENSE.
