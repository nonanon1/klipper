This is a fork of [Klipper](https://www.klipper3d.org/)
([GitHub](https://github.com/KevinOConnor/klipper)), a 3d-Printer firmware.
The code in this repo adds experimental features to reduce resonances and
ghosting during printing, like vibration suppression via special motion
smoothing, input shaping, S-curve acceleration, etc. The current focus is to
extensively test these features, stabilize the configuration API and tuning
instructions, collect the feedback and fix any outstanding bugs - so that some
(or all) of these features can be eventually integrated into the mainline
Klipper codebase.

The current branch recommended for the broad use is **`scurve-shaping`**
([installation and tuning instructions](https://github.com/dmbutyugin/klipper/blob/scurve-shaping/docs/S-Curve.md#switch-to-s-curve-acceleration-branch)).
All other branches are either deprecated in favor of this one, or have been
published for convenience and contain more experimental features and lack
proper documentation. Please update your installation to the suggested branch
if you are using one of the older branches.

The feedback is welcome at the original
[S-Curve acceleration issue](https://github.com/KevinOConnor/klipper/issues/57)
in the main Klipper repo. If you have issues specifically with input shaping,
please use [this ticket](https://github.com/KevinOConnor/klipper/issues/3025)
instead. If you report bugs, please double-check that the issue
cannot be reproduced on the mainline Klipper code, and attach the **full**
`klippy.log` of the failed print attempt to your bug report. Typically the
problematic GCode is also required to debug the issue. However, if the issue
can be reproduced on the mainline Klipper code, please create a separate issue
[here](https://github.com/KevinOConnor/klipper/issues) instead, and attach
`klippy.log` from the attempt on the **mainline** code.

Updates:
  * 2020-07-06: `scurve-shaping` branch is now recommended for broad use.
