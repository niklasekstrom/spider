# SPIder

<img alt="SPIder logo" src="Docs/spider-logo.png" width="250px"/>

## Introduction

SPIder is an adapter that connects to the clockport on an Amiga, and provides
two SPI controllers and additional GPIO pins.

SPIder is built around the RP2040 microcontroller. GPIO pins 0..15 on the RP2040
are connected to the clockport, and pins 16..29 are available on a pin header.
GPIO pins 16..29 have multiple alternative functions. A table with the possible
functions for the GPIO pins are available in
[section 1.4.3](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf#page=13)
of the RP2040 datasheet.

## Architecture

This section contains a high level overview of the architecture of the firmware.
The RP2040 has two Cortex-M0+ ARM cores. One core is dedicated to responding to
clockport accesses (reads and writes). This has to be done in a time sensitive
manner as a clockport access lasts for less than a microsecond. The other core
communicates with the peripherals on the RP2040, such as the two SPI
controllers.

![Architecture](Docs/architecture.drawio.svg)

## Bill of Materials (BOM)

| References | Value | Footprint | Quantity | JLCPCB |
|------------|-------|-----------|----------|--------|
| C5, C6, C7, C9, C11, C12, C13, C14, C15, C16, C18, C19, C20 | 100n | C_0402_1005Metric | 13 | [C1525](https://jlcpcb.com/partdetail/1877-CL05B104KO5NNNC/C1525) |
| C1, C4 | 10u | C_0805_2012Metric | 2 | [C15850](https://jlcpcb.com/partdetail/16532-CL21A106KAYNNNE/C15850) |
| C2, C3 | 15p | C_0402_1005Metric | 2 | [C1548](https://jlcpcb.com/partdetail/1900-0402CG150J500NT/C1548) |
| C8, C10 | 1u | C_0402_1005Metric | 2 | [C52923](https://jlcpcb.com/partdetail/53938-CL05A105KA5NQNC/C52923) |
| R2, R3 | 27 | R_0402_1005Metric | 2 | [C25105](https://jlcpcb.com/partdetail/25848-0402WGF330JTCE/C25105) |
| R1 | 10k | R_0402_1005Metric | 1 | [C25744](https://jlcpcb.com/partdetail/26487-0402WGF1002TCE/C25744) |
| R5 | 1k | R_0402_1005Metric | 1 | [C11702](https://jlcpcb.com/partdetail/12256-0402WGF1001TCE/C11702) |
| U1 | RP2040 | QFN-56-1EP_7x7mm_P0.4mm_EP3.2x3.2mm | 1 | [C2040](https://jlcpcb.com/partdetail/RaspberryPi-RP2040/C2040) |
| Y1 | ABM8-272-T3 | Crystal_SMD_3225-4Pin_3.2x2.5mm | 1 | [C20625731](https://jlcpcb.com/partdetail/AbraconLlc-ABM8_272T3/C20625731) |
| IC1 | SN74CBTD3384CPWR | TSSOP-24_4.4x7.8mm_P0.65mm | 1 | [C484747](https://jlcpcb.com/partdetail/TexasInstruments-SN74CBTD3384CPWR/C484747) |
| IC2 | SN74LVC125AIPWREP | TSSOP-14_4.4x5mm_P0.65mm | 1 | [C7813](https://jlcpcb.com/partdetail/TexasInstruments-SN74LVC125APWR/C7813) |
| IC3 | SN74LVC2G32DCUR | VSSOP-8_2.3x2mm_P0.5mm | 1 | [C91874](https://jlcpcb.com/partdetail/TexasInstruments-SN74LVC2G32DCUR/C91874) |
| IC4 | W25Q32RVXHJQ_TR | Winbond_USON-8-1EP_3x2mm_P0.5mm_EP0.2x1.6mm | 1 | [C5258281](https://jlcpcb.com/partdetail/Zetta-ZD25WQ32CEIGR/C5258281) |
| IC5 | TLV1117LV33DCYR | SOT-223 | 1 | [C15578](https://jlcpcb.com/partdetail/TexasInstruments-TLV1117LV33DCYR/C15578) |
| J1 | Conn_02x11_Odd_Even | PinSocket_2x11_P2.00mm_Vertical | 1 | |
| J2 | Conn_02x10_Odd_Even | PinHeader_2x10_P2.54mm_Horizontal | 1 | |
| J3 | Conn_01x03 | PinHeader_1x03_P2.54mm_Vertical | 1 | |
| J4 | Conn_01x05 | PinHeader_1x05_P2.54mm_Vertical | 1 | |
