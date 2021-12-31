use cortex_m::interrupt::CriticalSection;
use display_interface::WriteOnlyDataCommand;
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};

const DISPLAY_WIDTH: usize = 128;
const DISPLAY_HEIGHT: usize = 32;
const DISPLAY_WIDTH_I32: i32 = DISPLAY_WIDTH as i32;
const DISPLAY_HEIGHT_I32: i32 = DISPLAY_HEIGHT as i32;

pub struct ExampleDisplay {
    /// The framebuffer with one `u8` value per pixel.
    framebuffer: [u8; DISPLAY_WIDTH * DISPLAY_HEIGHT],
}

impl ExampleDisplay {
    pub fn new() -> Self {
        use ssd1306::command::AddrMode;
        use ssd1306::command::Command;
        use ssd1306::command::VcomhLevel;
        let cs = unsafe { CriticalSection::new() };
        let interface = &mut *crate::I2C_INTERFACE.borrow(&cs).get().unwrap().borrow_mut();
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
    pub fn flush(&mut self) -> Result<(), u8> {
        let cs = unsafe { CriticalSection::new() };
        let interface = &mut *crate::I2C_INTERFACE.borrow(&cs).get().unwrap().borrow_mut();
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
