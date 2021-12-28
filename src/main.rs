//! # Pico USB 'Twitchy' Mouse Example
//!
//! Creates a USB HID Class Poiting device (i.e. a virtual mouse) on a Pico
//! board, with the USB driver running in the main thread.
//!
//! It generates movement reports which will twitch the cursor up and down by a
//! few pixels, several times a second.
//!
//! See the `Cargo.toml` file for Copyright and licence details.
//!
//! This is a port of
//! https://github.com/atsamd-rs/atsamd/blob/master/boards/itsybitsy_m0/examples/twitching_usb_mouse.rs

#![no_std]
#![no_main]
#![feature(exclusive_range_pattern)]
#![feature(try_blocks)]

pub mod pmw3360;

use crate::pmw3360::*;

use core::cell::RefCell;
use core::cmp::min;
use core::convert::TryInto;
use core::fmt;
use core::str::from_utf8;
use core::str::Utf8Error;
use cortex_m::interrupt::CriticalSection;
use cortex_m::interrupt::Mutex;
use cortex_m::prelude::_embedded_hal_spi_FullDuplex;
use cortex_m_rt::entry;
use display_interface::WriteOnlyDataCommand;
use embedded_graphics::mono_font::{ascii::FONT_6X10, MonoTextStyle};
use embedded_graphics::primitives::PrimitiveStyleBuilder;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::{Alignment, Text};
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use once_cell::unsync::OnceCell;
use panic_halt as _;
use pro_micro_rp2040::hal;
use pro_micro_rp2040::hal::pac;
use pro_micro_rp2040::hal::pac::interrupt;
use pro_micro_rp2040::hal::prelude::*;
use pro_micro_rp2040::hal::{Sio, I2C};
use pro_micro_rp2040::XOSC_CRYSTAL_FREQ;
use ssd1306::I2CDisplayInterface;
use usb_device::{class_prelude::*, prelude::*};
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::MouseReport;
use usbd_hid::hid_class::HIDClass;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID_KEYBOARD: Option<HIDClass<hal::usb::UsbBus>> = None;

pub const KEY_A: u8 = 0x04;
pub const KEY_B: u8 = 0x05;
pub const KEY_C: u8 = 0x06;
pub const KEY_D: u8 = 0x07;
pub const KEY_E: u8 = 0x08;
pub const KEY_F: u8 = 0x09;
pub const KEY_G: u8 = 0x0a;
pub const KEY_H: u8 = 0x0b;
pub const KEY_I: u8 = 0x0c;
pub const KEY_J: u8 = 0x0d;
pub const KEY_K: u8 = 0x0e;
pub const KEY_L: u8 = 0x0f;
pub const KEY_M: u8 = 0x10;
pub const KEY_N: u8 = 0x11;
pub const KEY_O: u8 = 0x12;
pub const KEY_P: u8 = 0x13;
pub const KEY_Q: u8 = 0x14;
pub const KEY_R: u8 = 0x15;
pub const KEY_S: u8 = 0x16;
pub const KEY_T: u8 = 0x17;
pub const KEY_U: u8 = 0x18;
pub const KEY_V: u8 = 0x19;
pub const KEY_W: u8 = 0x1a;
pub const KEY_X: u8 = 0x1b;
pub const KEY_Y: u8 = 0x1c;
pub const KEY_Z: u8 = 0x1d;
pub const KEY_1: u8 = 0x1e;
pub const KEY_2: u8 = 0x1f;
pub const KEY_3: u8 = 0x20;
pub const KEY_4: u8 = 0x21;
pub const KEY_5: u8 = 0x22;
pub const KEY_6: u8 = 0x23;
pub const KEY_7: u8 = 0x24;
pub const KEY_8: u8 = 0x25;
pub const KEY_9: u8 = 0x26;
pub const KEY_0: u8 = 0x27;

pub const KEY_ENTER: u8 = 40;

pub const M_L: u8 = 0x01;
pub const M_R: u8 = 0x02;

enum KeyMapping {
    Normal(u8),
    Mouse(u8),
    Empty,
}
use KeyMapping::*;
/*
#[rustfmt::skip]
const KEYMAP: [[KeyMapping; 6]; 4] = [
    [Normal(KEY_Y), Normal(KEY_U),  Normal(KEY_I),Normal(KEY_O),Normal(KEY_P),Normal(KEY_I),],
    [Normal(KEY_H), Normal(KEY_J),  Normal(KEY_K),Normal(KEY_L),Normal(KEY_P),Normal(KEY_I),],
    [Normal(KEY_N), Normal(KEY_M),  Normal(KEY_K),Normal(KEY_A),Normal(KEY_L),Normal(KEY_I),],
    [Empty,         Mouse(M_L),     Mouse(M_R),   Normal(KEY_ENTER),Normal(KEY_L),Normal(KEY_I),],
];
*/

#[rustfmt::skip]
const KEYMAP: [[KeyMapping; 6]; 4] = [
    [Normal(KEY_A), Normal(KEY_B),  Normal(KEY_C),Normal(KEY_D),Normal(KEY_E),Normal(KEY_F),],
    [Normal(KEY_G), Normal(KEY_H),  Normal(KEY_I),Normal(KEY_J),Normal(KEY_K),Normal(KEY_L),],
    [Normal(KEY_M), Normal(KEY_N),  Normal(KEY_O),Normal(KEY_P),Normal(KEY_Q),Normal(KEY_R),],
    [Normal(KEY_S), Normal(KEY_T),  Normal(KEY_U),Normal(KEY_V),Normal(KEY_W),Normal(KEY_X),],
];

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = KEYBOARD) = {
        (usage_page = KEYBOARD, usage_min = 0xE0, usage_max = 0xE7) = {
            #[packed_bits 8] #[item_settings data,variable,absolute] modifier=input;
        };
        (usage_min = 0x00, usage_max = 0xFF) = {
            #[item_settings constant,variable,absolute] reserved=input;
        };
        (usage_page = KEYBOARD, usage_min = 0x00, usage_max = 0xDD) = {
            #[item_settings data,array,absolute] keycodes=input;
        };
}
)]
#[allow(dead_code)]
pub struct KeyboardReport2 {
    pub modifier: u8,
    pub reserved: u8,
    pub keycodes: [u8; 6],
}

#[derive(Debug)]
struct CommError;

const DISPLAY_WIDTH: usize = 128;
const DISPLAY_HEIGHT: usize = 32;
const DISPLAY_WIDTH_I32: i32 = DISPLAY_WIDTH as i32;
const DISPLAY_HEIGHT_I32: i32 = DISPLAY_HEIGHT as i32;

struct ExampleDisplay {
    /// The framebuffer with one `u8` value per pixel.
    framebuffer: [u8; DISPLAY_WIDTH * DISPLAY_HEIGHT],
}

impl ExampleDisplay {
    pub fn new() -> Self {
        use ssd1306::command::AddrMode;
        use ssd1306::command::Command;
        use ssd1306::command::VcomhLevel;
        let cs = unsafe { CriticalSection::new() };
        let interface = &mut *I2C_INTERFACE.borrow(&cs).get().unwrap().borrow_mut();
        Command::DisplayOn(false).send(interface).unwrap();
        Command::DisplayClockDiv(0x8, 0x0).send(interface).unwrap();
        Command::Multiplex(64 - 1).send(interface).unwrap();
        Command::DisplayOffset(0).send(interface).unwrap();
        Command::StartLine(0).send(interface).unwrap();
        Command::ChargePump(true).send(interface).unwrap();
        Command::AddressMode(AddrMode::Horizontal)
            .send(interface)
            .unwrap();

        Command::VcomhDeselect(VcomhLevel::Auto)
            .send(interface)
            .unwrap();
        Command::AllOn(false).send(interface).unwrap();
        Command::Invert(false).send(interface).unwrap();
        Command::EnableScroll(false).send(interface).unwrap();

        // Set page address to (0, 0)
        interface
            .send_commands(display_interface::DataFormat::U8(&[0x22, 0, 7]))
            .unwrap();
        // Set column address to (0, 0)
        interface
            .send_commands(display_interface::DataFormat::U8(&[0x21, 0, 127]))
            .unwrap();

        Command::DisplayOn(true).send(interface).unwrap();
        Command::AllOn(true).send(interface).unwrap();
        //delay.delay_ms(1000);
        Command::AllOn(false).send(interface).unwrap();

        ExampleDisplay {
            framebuffer: [0; 64 * 64],
        }
    }
    /// Updates the display from the framebuffer.
    pub fn flush(&mut self) -> Result<(), CommError> {
        let cs = unsafe { CriticalSection::new() };
        let interface = &mut *I2C_INTERFACE.borrow(&cs).get().unwrap().borrow_mut();
        // Set page address to (0, 0)
        interface
            .send_commands(display_interface::DataFormat::U8(&[0x22, 0, 7]))
            .unwrap();
        // Set column address to (0, 0)
        interface
            .send_commands(display_interface::DataFormat::U8(&[0x21, 0, 127]))
            .unwrap();
        // x, y in display coordinates
        for y in 0..DISPLAY_HEIGHT / 4 {
            for x in 0..DISPLAY_WIDTH {
                let d = (self.framebuffer[(y * 4 + 3) * DISPLAY_WIDTH + x] << 6)
                    + (self.framebuffer[(y * 4 + 2) * DISPLAY_WIDTH + x] << 4)
                    + (self.framebuffer[(y * 4 + 1) * DISPLAY_WIDTH + x] << 2)
                    + self.framebuffer[(y * 4) * DISPLAY_WIDTH + x];
                interface
                    .send_data(display_interface::DataFormat::U8(&[d]))
                    .unwrap();
            }
        }
        Ok(())
    }
}

impl DrawTarget for ExampleDisplay {
    type Color = BinaryColor;
    // `ExampleDisplay` uses a framebuffer and doesn't need to communicate with the display
    // controller to draw pixel, which means that drawing operations can never fail. To reflect
    // this the type `Infallible` was chosen as the `Error` type.
    type Error = core::convert::Infallible;

    fn draw_iter<IT>(&mut self, pixels: IT) -> Result<(), Self::Error>
    where
        IT: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            // Check if the pixel coordinates are out of bounds (negative or greater than
            // (63,63)). `DrawTarget` implementation are required to discard any out of bounds
            // pixels without returning an error or causing a panic.
            if let Ok((x @ 0..DISPLAY_HEIGHT_I32, y @ 0..DISPLAY_WIDTH_I32)) = coord.try_into() {
                // Calculate the index in the framebuffer.
                // convert corrdinates
                let disp_x = y;
                let disp_y = DISPLAY_HEIGHT_I32 - 1 - x;
                let index = disp_x + disp_y * DISPLAY_WIDTH_I32;
                self.framebuffer[index as usize] = if color.is_on() { 1 } else { 0 };
            }
        }

        Ok(())
    }
}

impl OriginDimensions for ExampleDisplay {
    fn size(&self) -> Size {
        // report rotated corrdinates
        Size::new(DISPLAY_HEIGHT as u32, DISPLAY_WIDTH as u32)
    }
}

struct KeyBall46 {}

impl KeyBall46 {
    fn new() -> Self {
        KeyBall46 {}
    }
    fn panic(&mut self, ecode: u32) -> ! {
        let mut print_buf = PrintBuf::new();
        self.print_string(2, print_buf.format(format_args!("E{}!", ecode)).unwrap());
        panic!();
    }
    fn print_num(&mut self, row: i8, ecode: u32) {
        let mut print_buf = PrintBuf::new();
        self.print_string(row, print_buf.format(format_args!("N{}~", ecode)).unwrap());
    }
    fn print_string(&mut self, row: i8, s: &str) {
        const ROW_HEIGHT: i32 = 11;
        let cs = unsafe { CriticalSection::new() };
        let display = &mut *DISPLAY.borrow(&cs).get().unwrap().borrow_mut();
        let style = PrimitiveStyleBuilder::new()
            .fill_color(BinaryColor::Off)
            .build();
        // Fill background
        Rectangle::new(
            Point::new(0, ROW_HEIGHT * row as i32),
            Size::new(display.bounding_box().size.width, ROW_HEIGHT as u32),
        )
        .into_styled(style)
        .draw(display)
        .unwrap();
        let character_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        Text::with_alignment(
            s,
            Point::new(0, ROW_HEIGHT * row as i32 + 6),
            character_style,
            Alignment::Left,
        )
        .draw(display)
        .unwrap();
        display.flush().unwrap();
    }
}

struct PrintBuf {
    buf: [u8; 32],
    used: usize,
}

impl<'a> PrintBuf {
    fn new() -> Self {
        PrintBuf {
            buf: [0; 32],
            used: 0,
        }
    }
    fn as_str(&mut self) -> Result<&str, Utf8Error> {
        let len = self.used;
        self.used = 0;
        from_utf8(&self.buf[..len])
    }
    fn format(&'a mut self, args: fmt::Arguments) -> Result<&'a str, fmt::Error> {
        fmt::write(self, args)?;
        self.as_str().or(Err(fmt::Error))
    }
}

impl fmt::Write for PrintBuf {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        if self.used + s.len() > self.buf.len() {
            // Not enough space
            return Err(fmt::Error);
        }
        let dst = &mut self.buf[self.used..(self.used + s.len())];
        let s = s.as_bytes();
        let write_num = min(s.len(), dst.len());
        dst[..write_num].copy_from_slice(&s[..write_num]);
        self.used += s.len();
        Ok(())
    }
}

static DISPLAY: Mutex<OnceCell<RefCell<ExampleDisplay>>> = Mutex::new(OnceCell::new());

type DisplayI2CSdaPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio2, hal::gpio::Function<hal::gpio::I2C>>;
type DisplayI2CSclPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio3, hal::gpio::Function<hal::gpio::I2C>>;
type DisplayI2C = ssd1306::prelude::I2CInterface<
    hal::I2C<crate::pac::I2C1, (DisplayI2CSdaPin, DisplayI2CSclPin)>,
>;

static I2C_INTERFACE: Mutex<OnceCell<RefCell<DisplayI2C>>> = Mutex::new(OnceCell::new());

#[doc(hidden)]
pub fn _print(row: usize, args: fmt::Arguments) {
    let mut print_buf = PrintBuf::new();
    let s = print_buf.format(args).unwrap();
    const ROW_HEIGHT: i32 = 11;
    let cs = unsafe { CriticalSection::new() };
    let display = &mut *DISPLAY.borrow(&cs).get().unwrap().borrow_mut();
    let style = PrimitiveStyleBuilder::new()
        .fill_color(BinaryColor::Off)
        .build();
    // Fill background
    Rectangle::new(
        Point::new(0, ROW_HEIGHT * row as i32),
        Size::new(display.bounding_box().size.width, ROW_HEIGHT as u32),
    )
    .into_styled(style)
    .draw(display)
    .unwrap();
    let character_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    Text::with_alignment(
        s,
        Point::new(0, ROW_HEIGHT * row as i32 + 6),
        character_style,
        Alignment::Left,
    )
    .draw(display)
    .unwrap();
    display.flush().unwrap();
}

macro_rules! print {
            ($row:expr, $($arg:expr),+) => ($crate::_print($row, format_args!($($arg),+)));
}

#[entry]
fn main() -> ! {
    match body() {
        Ok(_) => {}
        Err(e) => {
            print!(10, "P!{}", e);
        }
    }
    loop {
        cortex_m::asm::wfi();
    }
}
fn body() -> Result<(), i32> {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB HID Class Device driver, providing Mouse Reports
    let usb_hid = HIDClass::new(bus_ref, MouseReport::desc(), 10);
    let usb_hid_keyboard = HIDClass::new(bus_ref, KeyboardReport2::desc(), 10);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
        USB_HID_KEYBOARD = Some(usb_hid_keyboard);
    }

    // Create a USB device with a fake VID and PID ()
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0xfeed, 0x1307))
        .manufacturer("Fake company")
        .product("Twitchy Mousey")
        .serial_number("TEST")
        .device_class(0x00) // HID
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };
    let core = pac::CorePeripherals::take().unwrap();
    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio2.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio3.into_mode::<hal::gpio::FunctionI2C>();
    let oled_i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin, // sda
        scl_pin, // scl
        1.MHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );
    let interface = I2CDisplayInterface::new(oled_i2c);
    unsafe {
        let cs = CriticalSection::new();
        let global_interface = I2C_INTERFACE.borrow(&cs);
        global_interface
            .set(RefCell::new(interface))
            .map_err(|_| 0)?;
    }
    let display = ExampleDisplay::new();
    unsafe {
        let cs = CriticalSection::new();
        let global_display = DISPLAY.borrow(&cs);
        global_display.set(RefCell::new(display)).map_err(|_| 0)?;
    }
    let mut keyball46 = KeyBall46::new();
    let mut print_buf = PrintBuf::new();
    keyball46.print_string(0, "Key");
    keyball46.print_string(1, " Ball");
    keyball46.print_string(2, print_buf.format(format_args!("={}", 46)).unwrap());
    /*
    #define MATRIX_ROW_PINS \
        { F4, F5, F6, F7 }
    #define MATRIX_COL_PINS \
        { D4, C6, D7, E6, B4, B5 }

    ---------+-----------------+-------------------
    NAME     | Pro Micro (AVR) | Pro Micro (RP2040)
             |                 | Pin    | GPIO
    ---------+-----------------+-------------------
    ROW0     | F4                A3       GPIO29
    ROW1     | F5                A2       GPIO28
    ROW2     | F6                A1       GPIO27
    ROW3     | F7                A0       GPIO26
             |
    COL0     | D4                4        GPIO4
    COL1     | C6                5        GPIO5
    COL2     | D7                6        GPIO6
    COL3     | E6                7        GPIO7
    COL4     | B4                8        GPIO8
    COL5     | B5                9        GPIO9

    OLED_SDA | D1                2        GPIO2
    OLED_SCK | D0                3        GPIO3

    BALL_SCLK| B1                SCK      GPIO22
    BALL_MISO| B3                CI       GPIO20
    BALL_MOSI| B2                CO       GPIO23
    BALL_NCS | B6                21       GPIO21

    TOP VIEW, RIGHT
        5 4 3 2 1 0
    0
    1
    2
    3

    */

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio22.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio20.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio23.into_mode::<hal::gpio::FunctionSpi>();
    let mut spi_ncs = pins.gpio21.into_push_pull_output();
    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        2_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );
    // Reset PMW3360
    keyball46.print_string(6, "--");
    spi_ncs.set_high().unwrap();

    let spi = RefCell::new(spi);
    let spi_ncs = RefCell::new(spi_ncs);
    let delay = RefCell::new(delay);

    let read_pmw3360_reg = |reg: u8| -> Result<u8, u8> {
        spi_ncs.borrow_mut().set_low().unwrap();
        spi.borrow_mut().send(reg).unwrap();
        loop {
            let sspsr = spi.borrow_mut().device().sspsr.read();
            if sspsr.tfe().bit_is_set() && sspsr.bsy().bit_is_clear() {
                break;
            }
            delay.borrow_mut().delay_us(10);
        }
        delay.borrow_mut().delay_us(160);
        let send_success = spi.borrow_mut().send(0x00); // fake write to read
                                                        // wait to finish sending the fake data
        loop {
            let sspsr = spi.borrow_mut().device().sspsr.read();
            if sspsr.tfe().bit_is_set() && sspsr.bsy().bit_is_clear() {
                break;
            }
            delay.borrow_mut().delay_us(10);
        }
        spi.borrow_mut().read().unwrap(); // fake read
        let ret = match send_success {
            Ok(_) => {
                if let Ok(value) = spi.borrow_mut().read() {
                    Ok(value)
                } else {
                    Err(3)
                }
            }
            Err(_) => Err(2),
        };
        spi_ncs.borrow_mut().set_high().unwrap();
        delay.borrow_mut().delay_us(20);
        ret
    };
    let write_pmw3360_reg = |reg: u8, data: u8| -> Result<(), u8> {
        spi_ncs.borrow_mut().set_low().unwrap();
        let send_success = spi.borrow_mut().send(reg | (1 << 7));
        let ret = match send_success {
            Ok(_) => {
                if spi.borrow_mut().send(data).is_ok() {
                    Ok(())
                } else {
                    Err(4)
                }
            }
            Err(_) => Err(2),
        };
        loop {
            let sspsr = spi.borrow_mut().device().sspsr.read();
            if sspsr.tfe().bit_is_set() && sspsr.bsy().bit_is_clear() {
                break;
            }
            delay.borrow_mut().delay_us(10);
        }
        spi.borrow_mut().read().unwrap(); // fake read
        spi.borrow_mut().read().unwrap(); // fake read
        spi_ncs.borrow_mut().set_high().unwrap();
        delay.borrow_mut().delay_us(180);
        ret
    };
    let write_pmw3360_srom_burst = |data: &[u8]| -> Result<(), u8> {
        delay.borrow_mut().delay_ms(50);
        spi_ncs.borrow_mut().set_low().unwrap();
        let send_success = spi.borrow_mut().send(SROM_LOAD_BURST | (1 << 7));
        delay.borrow_mut().delay_us(20);
        let ret = match send_success {
            Ok(_) => {
                for v in data.iter() {
                    loop {
                        delay.borrow_mut().delay_us(15);
                        match spi.borrow_mut().send(*v) {
                            Ok(_) => break,
                            Err(nb::Error::WouldBlock) => {
                                // tx buffer full: wait a bit and try again
                                delay.borrow_mut().delay_us(20);
                            }
                            Err(_) => return Err(5),
                        }
                    }
                }
                Ok(())
            }
            Err(_) => Err(2),
        };
        loop {
            let sspsr = spi.borrow_mut().device().sspsr.read();
            if sspsr.tfe().bit_is_set() && sspsr.bsy().bit_is_clear() {
                break;
            }
            delay.borrow_mut().delay_us(10);
        }
        loop {
            // read out all rx data
            let sspsr = spi.borrow_mut().device().sspsr.read();
            if sspsr.rne().bit_is_clear() {
                break;
            }
            spi.borrow_mut().read().unwrap(); // fake read
        }
        spi_ncs.borrow_mut().set_high().unwrap();
        delay.borrow_mut().delay_us(180);
        ret
    };

    let read_pmw3360_motion_burst = || -> Result<MotionReport, u8> {
        spi_ncs.borrow_mut().set_low().unwrap();
        spi.borrow_mut().send(MOTION_BURST).unwrap();
        loop {
            let sspsr = spi.borrow_mut().device().sspsr.read();
            if sspsr.tfe().bit_is_set() && sspsr.bsy().bit_is_clear() {
                break;
            }
            delay.borrow_mut().delay_us(10);
        }
        delay.borrow_mut().delay_us(35 /* t_SRAD_MOTBR */);
        let mut report = [0; 12];
        let mut ret = Err(6);
        spi.borrow_mut().read().unwrap(); // fake read
        for report_byte in report.iter_mut() {
            let send_success = spi.borrow_mut().send(0x00); // fake write to read
                                                            // wait to finish sending the fake data
            loop {
                let sspsr = spi.borrow_mut().device().sspsr.read();
                if sspsr.rne().bit_is_set() {
                    break;
                }
                delay.borrow_mut().delay_us(10);
            }
            ret = match send_success {
                Ok(_) => {
                    if let Ok(value) = spi.borrow_mut().read() {
                        *report_byte = value;
                        Ok(())
                    } else {
                        Err(3)
                    }
                }
                Err(_) => Err(2),
            };
        }
        spi_ncs.borrow_mut().set_high().unwrap();
        delay.borrow_mut().delay_us(20);
        if let Err(e) = ret {
            Err(e)
        } else {
            Ok(MotionReport::new(report))
        }
    };

    keyball46.print_num(5, line!());

    write_pmw3360_reg(POWER_UP_RESET, 0x5A)?;
    delay.borrow_mut().delay_ms(50);

    keyball46.print_num(5, line!());

    /*
    loop {
        for i in 0..2 {
            let id = read_pmw3360_reg(i);
            match id {
                Ok(v) => {
                    keyball46.print_num_hex(5, i as u32);
                    keyball46.print_num_hex(6, v as u32);
                }
                Err(e) => keyball46.panic(e as u32),
            }
            delay.borrow_mut().delay_ms(1000);
        }
    }
    */

    read_pmw3360_reg(0x02)?;
    read_pmw3360_reg(0x03)?;
    read_pmw3360_reg(0x04)?;
    read_pmw3360_reg(0x05)?;
    read_pmw3360_reg(0x06)?;

    write_pmw3360_reg(CONFIG2, 0x20)?;
    write_pmw3360_reg(SROM_ENABLE, 0x1d)?;
    delay.borrow_mut().delay_ms(10);
    write_pmw3360_reg(SROM_ENABLE, 0x18)?;

    write_pmw3360_srom_burst(&FIRMWARE)?;

    let id = read_pmw3360_reg(SROM_ID);
    match id {
        Ok(v) => keyball46.print_num(6, v as u32),
        Err(e) => keyball46.panic(e as u32),
    }

    write_pmw3360_reg(CONFIG2, 0x00)?;

    let id = read_pmw3360_reg(PRODUCT_ID);
    let is_ball_enabled = match id {
        Ok(v) => {
            if v == 0x42 {
                true
            } else {
                false
            }
        }
        Err(e) => keyball46.panic(e as u32),
    };
    if is_ball_enabled {
        write_pmw3360_reg(MOTION_BURST, 0x00)?; // Begin motion burst
    }

    use hal::gpio::DynPin;
    let row0 = pins.gpio29.into_push_pull_output();
    let row1 = pins.gpio28.into_push_pull_output();
    let row2 = pins.gpio27.into_push_pull_output();
    let row3 = pins.gpio26.into_push_pull_output();

    // Top to bottom, Top view
    let mut rows: [&mut DynPin; 4] = [
        &mut row0.into(),
        &mut row1.into(),
        &mut row2.into(),
        &mut row3.into(),
    ];

    let col0 = pins.gpio4.into_pull_up_input();
    let col1 = pins.gpio5.into_pull_up_input();
    let col2 = pins.gpio6.into_pull_up_input();
    let col3 = pins.gpio7.into_pull_up_input();
    let col4 = pins.gpio8.into_pull_up_input();
    let col5 = pins.gpio9.into_pull_up_input();

    // Left to Right, Top view
    let cols: [&mut DynPin; 6] = [
        &mut col5.into(),
        &mut col4.into(),
        &mut col3.into(),
        &mut col2.into(),
        &mut col1.into(),
        &mut col0.into(),
    ];

    rows[0].set_low().unwrap();
    rows[1].set_high().unwrap();
    rows[2].set_high().unwrap();
    rows[3].set_high().unwrap();

    // Move the cursor up and down every 200ms
    let mut last_keycodes = [0u8; 6];
    loop {
        let mut mouse_buttons = 0;
        let mut keycodes = [0u8; 6];
        let mut next_keycode_index = 0;

        for (y, keymap_row) in KEYMAP.iter().enumerate() {
            for (i, row) in rows.iter_mut().enumerate() {
                if i == y {
                    row.set_low().unwrap();
                } else {
                    row.set_high().unwrap();
                }
            }
            delay.borrow_mut().delay_ms(20); // Wait a bit to propagete the voltage
            for (x, col) in cols.iter().enumerate() {
                if col.is_low().unwrap() && next_keycode_index < keycodes.len() {
                    match keymap_row[x] {
                        Normal(keycode) => {
                            keycodes[next_keycode_index] = keycode;
                            next_keycode_index += 1;
                        }
                        Mouse(button) => {
                            mouse_buttons |= button;
                        }
                        Empty => {}
                    }
                }
            }
        }

        if last_keycodes != keycodes {
            let rep_up = KeyboardReport2 {
                modifier: 0,
                reserved: 0,
                keycodes,
            };
            push_key_event(rep_up).ok().unwrap_or(0);
            last_keycodes = keycodes;
        }

        if is_ball_enabled {
            let report = read_pmw3360_motion_burst().unwrap();
            let dx = report.dx() / 4;
            let dy = -report.dy() / 4;

            let rep_down = MouseReport {
                x: dx.clamp(i8::MIN as i16, i8::MAX as i16) as i8,
                y: dy.clamp(i8::MIN as i16, i8::MAX as i16) as i8,
                buttons: mouse_buttons,
                wheel: 0,
                pan: 0,
            };
            push_mouse_movement(rep_down).ok().unwrap_or(0);
        }
    }
}

/// Submit a new mouse movement report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
pub fn push_mouse_movement(report: MouseReport) -> Result<usize, usb_device::UsbError> {
    cortex_m::interrupt::free(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

pub fn push_key_event(report: KeyboardReport2) -> Result<usize, usb_device::UsbError> {
    cortex_m::interrupt::free(|_| unsafe {
        USB_HID_KEYBOARD.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid_mouse = USB_HID.as_mut().unwrap();
    let usb_hid_keyboard = USB_HID_KEYBOARD.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid_mouse, usb_hid_keyboard]);
}

// End of file
