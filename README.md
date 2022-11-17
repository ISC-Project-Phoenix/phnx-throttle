# Phoenix Throttle ECU

This is the codebase for Phoenix's throttle ECU, part of the drive by wire subsystem.

See [the design doc](https://github.com/ISC-Project-Phoenix/design/blob/main/software/Throttle.md) for more info.

## Pinout

- Can rx: pb8
- Can tx: pb9
- Dac out: pa4
- ADC in: pa0

Transceiver is powered with 3v3 and Gnd.

Pedal is connected across the pot as the second resistor in a voltage divider, with a 7.5K resistor as the other end. This means
that its connected as 5v -> 7.5K resistor -> ADC -> pot in -> pot out (not wiper out) -> Gnd. 

The blue LED will blink with each Can message received.

# Features

- `vol_out`: Has this ecu echo voltages in its throttle message rather than percents while in training mode. This is useful
for gathering noise data and calibrating the kalman filter.
- `kalman(default)`: Enables kalman smoothing. This has a minimal performance impact but great smoothing. Consider disabling
when sampling noise.

## Building

This codebase is designed to be run on an ST Nucleo-f767zi.

1. Install the [Rust toolchain](https://www.rust-lang.org/learn/get-started).
2. `cargo install probe-run` 
3. `cargo install flip-link`
4. `rustup target install thumbv7em-none-eabihf`
5. build the repo with `cargo build` or `cargo build --release` and flash manually, or debug with
`cargo run`.
6. If testing noise, compile with `cargo run --no-default-features --features vol_out`
