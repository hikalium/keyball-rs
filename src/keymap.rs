use crate::keycode::*;

pub const M_L: u8 = 0x01;
pub const M_R: u8 = 0x02;

#[allow(dead_code)]
pub enum KeyMapping {
    Normal(u8),
    Mouse(u8),
    Empty,
}
use KeyMapping::*;

#[allow(dead_code)]
#[rustfmt::skip]
pub const KEYMAP_LEFT: [[KeyMapping; 6]; 4] = [
    [Normal(KEY_Y), Normal(KEY_U),  Normal(KEY_I),Normal(KEY_O),Normal(KEY_P),Normal(KEY_I),],
    [Normal(KEY_H), Normal(KEY_J),  Normal(KEY_K),Normal(KEY_L),Normal(KEY_P),Normal(KEY_I),],
    [Normal(KEY_N), Normal(KEY_M),  Normal(KEY_K),Normal(KEY_A),Normal(KEY_L),Normal(KEY_I),],
    [Empty,         Mouse(M_L),     Mouse(M_R),   Normal(KEY_ENTER),Normal(KEY_L),Normal(KEY_I),],
];

#[allow(dead_code)]
#[rustfmt::skip]
pub const KEYMAP_RIGHT: [[KeyMapping; 6]; 4] = [
    [Normal(KEY_Y), Normal(KEY_U),  Normal(KEY_I),Normal(KEY_O),Normal(KEY_P),Normal(KEY_I),],
    [Normal(KEY_H), Normal(KEY_J),  Normal(KEY_K),Normal(KEY_L),Normal(KEY_P),Normal(KEY_I),],
    [Normal(KEY_N), Normal(KEY_M),  Normal(KEY_K),Normal(KEY_A),Normal(KEY_L),Normal(KEY_I),],
    [Empty,         Mouse(M_L),     Mouse(M_R),   Normal(KEY_ENTER),Normal(KEY_L),Normal(KEY_I),],
];

#[allow(dead_code)]
#[rustfmt::skip]
pub const KEYMAP_TEST: [[KeyMapping; 6]; 4] = [
    [Normal(KEY_A), Normal(KEY_B),  Normal(KEY_C),Normal(KEY_D),Normal(KEY_E),Normal(KEY_F),],
    [Normal(KEY_G), Normal(KEY_H),  Normal(KEY_I),Normal(KEY_J),Normal(KEY_K),Normal(KEY_L),],
    [Normal(KEY_M), Normal(KEY_N),  Normal(KEY_O),Normal(KEY_P),Normal(KEY_Q),Normal(KEY_R),],
    [Normal(KEY_S), Normal(KEY_T),  Normal(KEY_U),Normal(KEY_V),Normal(KEY_W),Normal(KEY_X),],
];
