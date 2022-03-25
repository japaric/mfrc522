This repository is no longer maintained! Last release of this repository on crates.io was v0.2.0.
Ownership over the `mfrc522` crate on crates.io has been transferred to [`@jspngh`](https://github.com/jspngh).
`@jspngh` maintains a MFRC522 driver crate at https://gitlab.com/jspngh/rfid-rs

---

# `mfrc522`

> A platform agnostic driver to interface the MFRC522 (RFID reader/writer)

<p align="center">
  <img alt="MFRC522" src="https://i.imgur.com/yI4qaTO.jpg">
</p>

## What works

- REQuest A
- SELECT for single size UIDs

## TODO

- [x] Make sure this works with the `spidev` crate (i.e. with the Raspberry Pi)
- [ ] Anticollision loop -- SELECT works when there's only one nearby tag
- [ ] Authentication (`MFAuthent`)
- [ ] Reading / writing data into the tag
- [ ] Configurable timeout
- [ ] Make the API non-blocking and compatible with the interrupt pin
- [ ] Support double and triple size UIDs -- I don't have hardware (tags) to test this.
- ???

## Examples

There's an example for the Raspberry Pi (3) in the examples directory. The example turns an LED on
when a card is nearby and turns it off when a tag in nearby. There's a video of the example [here].
To reproduce the example you'll have to tweak the values of the card and tag UIDs (Unique
IDentifiers).

[here]: https://mobile.twitter.com/japaricious/status/936385342579539969

The same example has been implemented for the [Blue Pill] development board. You'll find the code
for that example in [this branch] of the blue-pill repository. If that branch is gone, check the
master branch.

[Blue Pill]: http://wiki.stm32duino.com/index.php?title=Blue_Pill
[this branch]: https://github.com/japaric/blue-pill/tree/singletons/examples

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the
work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
