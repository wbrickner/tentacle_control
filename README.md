# Infrared Tentacle Firmware

This firmware controls an animatronic tentacle I built.
It takes Infrared Image data from any of the `AMG88xx` sensor family over I2C.
It uses a crude heuristic model to control a set of `A4988` motor drivers (which in turn drive `NEMA 17` motors to actuate the stages).

The tentacle will track hot objects, so in a cool dim room will track human faces which was the original goal.