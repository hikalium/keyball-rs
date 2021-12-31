// https://wiki.osdev.org/USB_Human_Interface_Devices
// https://www.win.tue.nl/~aeb/linux/kbd/scancodes-14.html

pub const LCTRL: u8 = 0b00000001;
pub const LSHIFT: u8 = 0b00000010;
pub const LALT: u8 = 0b00000100;
pub const LGUI: u8 = 0b00001000;
pub const RCTRL: u8 = 0b00010000;
pub const RSHIFT: u8 = 0b00100000;
pub const RALT: u8 = 0b01000000;
pub const RGUI: u8 = 0b10000000;

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

pub const ENTER: u8 = 40;
pub const BS: u8 = 42;
pub const TAB: u8 = 43;
pub const SP: u8 = 44;

pub const RIGHT: u8 = 79;
pub const LEFT: u8 = 80;
pub const DOWN: u8 = 81;
pub const UP: u8 = 82;
