# `mfrc522`

> A WIP, no_std, generic driver for the MFRC522 (RFID reader / writer)

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

You should find at least one example in the [blue-pill] repository. If that branch is gone, check
the master branch.

[blue-pill]: https://github.com/japaric/blue-pill/tree/singletons/examples

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
