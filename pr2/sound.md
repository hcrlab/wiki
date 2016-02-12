# Sound and TTS on the PR2

## Adjusting sound output
Use `alsamixer` to adjust the sound output from the terminal.

## Troubleshooting
The robot's sound output can sometimes be glitchy.
This often occurs as a result of launching sound_play multiple times.
Note that on the PR2, sound_play is launched by default, so no package should ever launch sound_play itself, even under a different name.
So far, the only solution is the restart the robot.

## Festival
You can experiment with new voices using festival directly:
```
festival
festival> (voice.list)
festival> (voice_cmu_us_slt_arctic_hts)
festival> (SayText "I am about to plug myself in.")
```

The names of the voices for sound_play's SoundClient are without the `voice_` prefix, e.g., `cmu_us_slt_arctic_hls`.

## SoX
`sox` is a useful command for doing all sorts of processing on audio files.
