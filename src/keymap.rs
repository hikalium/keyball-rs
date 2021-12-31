use crate::keycode::*;

pub const M_L: u8 = 0x01;
pub const M_R: u8 = 0x02;

#[allow(dead_code)]
pub enum KeyMapping {
    K(u8),  // Key(keycode)
    KM(u8), // Key(modifier)
    M(u8),  // Mouse(button)
    Empty,
}
use KeyMapping::*;

pub type KeyMap = [[KeyMapping; 6]; 4];

// All key maps are in top view

#[allow(dead_code)]
#[rustfmt::skip]
pub const KEYMAP_LEFT: KeyMap = [
    [K(KEY_Q),  K(KEY_W),  K(KEY_E),K(KEY_R),K(KEY_T),K(KEY_I),],
    [K(KEY_A),  K(KEY_S),  K(KEY_D),K(KEY_F),K(KEY_G),K(KEY_I),],
    [K(KEY_Z),  K(KEY_X),  K(KEY_C),K(KEY_V),K(KEY_B),K(KEY_I),],
    [KM(LSHIFT),KM(LCTRL),K(LEFT), K(RIGHT),K(SP),   K(BS),],
];

#[allow(dead_code)]
#[rustfmt::skip]
pub const KEYMAP_RIGHT: KeyMap = [
    [K(KEY_Y), K(KEY_U),  K(KEY_I),K(KEY_O),K(KEY_P),K(KEY_I),],
    [K(KEY_H), K(KEY_J),  K(KEY_K),K(KEY_L),K(KEY_P),K(KEY_I),],
    [K(KEY_N), K(KEY_M),  K(KEY_K),K(KEY_A),K(KEY_L),K(KEY_I),],
    [Empty,    M(M_L),    M(M_R),  K(ENTER),K(KEY_L),K(KEY_I),],
];

#[allow(dead_code)]
#[rustfmt::skip]
pub const KEYMAP_TEST: KeyMap = [
    [K(KEY_A), K(KEY_B),  K(KEY_C),K(KEY_D),K(KEY_E),K(KEY_F),],
    [K(KEY_G), K(KEY_H),  K(KEY_I),K(KEY_J),K(KEY_K),K(KEY_L),],
    [K(KEY_M), K(KEY_N),  K(KEY_O),K(KEY_P),K(KEY_Q),K(KEY_R),],
    [K(KEY_S), K(KEY_T),  K(KEY_U),K(KEY_V),K(KEY_W),K(KEY_X),],
];
