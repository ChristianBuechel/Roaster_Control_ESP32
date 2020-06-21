// This comes with no warranty, implied or otherwise

// This data structure was designed to support Proportional fonts
// on Arduinos. It can however handle any ttf font that has been converted
// using the conversion program. These could be fixed width or proportional 
// fonts. Individual characters do not have to be multiples of 8 bits wide. 
// Any width is fine and does not need to be fixed.

// The data bits are packed to minimize data requirements, but the tradeoff
// is that a header is required per character.

// lucon.c
// Point Size   : 12
// Memory usage : 1053 bytes
// # characters : 95

// Header Format (to make Arduino UTFT Compatible):
// ------------------------------------------------
// Character Width (Used as a marker to indicate use this format. i.e.: = 0x00)
// Character Height
// First Character (Reserved. 0x00)
// Number Of Characters (Reserved. 0x00)


const unsigned char tft_lucon12[] =
{

0x00, 0x0B, 0x00, 0x00,

// Individual Character Format:
// ----------------------------
// Character Code
// Adjusted Y Offset
// Width
// Height
// xOffset
// xDelta (the distance to move the cursor. Effective width of the character.)
// Data[n]

// NOTE: You can remove any of these characters if they are not needed in
// your application. The first character number in each Glyph indicates
// the ASCII character code. Therefore, these do not have to be sequential.
// Just remove all the content for a particular character to save space.

// ' '
0x20,0x09,0x00,0x00,0x00,0x07,

// '!'
0x21,0x01,0x01,0x08,0x03,0x07,
0xFD,
// '"'
0x22,0x00,0x04,0x03,0x01,0x07,
0x99,0x90,
// '#'
0x23,0x01,0x07,0x08,0x00,0x07,
0x14,0x29,0xF9,0x42,0x9F,0x94,0x28,
// '$'
0x24,0x00,0x05,0x0A,0x01,0x07,
0x23,0xE9,0x46,0x18,0xA5,0xF1,0x00,
// '%'
0x25,0x01,0x07,0x08,0x00,0x07,
0x63,0x2A,0x63,0x81,0xC6,0x54,0xC6,
// '&'
0x26,0x01,0x07,0x08,0x00,0x07,
0x18,0x48,0xA1,0x8D,0x31,0x73,0x3F,
// '''
0x27,0x00,0x01,0x03,0x03,0x07,
0xE0,
// '('
0x28,0x00,0x04,0x0B,0x02,0x07,
0x16,0x48,0x88,0x88,0x46,0x10,
// ')'
0x29,0x00,0x04,0x0B,0x01,0x07,
0x86,0x21,0x11,0x11,0x26,0x80,
// '*'
0x2A,0x01,0x05,0x04,0x01,0x07,
0x26,0xC0,0xA0,
// '+'
0x2B,0x03,0x07,0x06,0x00,0x07,
0x10,0x23,0xF8,0x81,0x02,0x00,
// ','
0x2C,0x07,0x02,0x04,0x03,0x07,
0xF6,
// '-'
0x2D,0x05,0x05,0x01,0x01,0x07,
0xF8,
// '.'
0x2E,0x07,0x02,0x02,0x02,0x07,
0xF0,
// '/'
0x2F,0x00,0x07,0x0B,0x00,0x07,
0x02,0x08,0x10,0x40,0x82,0x08,0x10,0x40,0x82,0x00,
// '0'
0x30,0x01,0x06,0x08,0x00,0x07,
0x31,0x28,0x61,0x86,0x14,0x9E,
// '1'
0x31,0x01,0x05,0x08,0x01,0x07,
0x25,0x08,0x42,0x10,0x9F,
// '2'
0x32,0x01,0x05,0x08,0x01,0x07,
0xF0,0x42,0x22,0x22,0x1F,
// '3'
0x33,0x01,0x04,0x08,0x01,0x07,
0xE1,0x16,0x11,0x1E,
// '4'
0x34,0x01,0x06,0x08,0x00,0x07,
0x08,0x62,0x92,0x8B,0xF0,0x82,
// '5'
0x35,0x01,0x04,0x08,0x01,0x07,
0xF8,0x8E,0x31,0x1E,
// '6'
0x36,0x01,0x05,0x08,0x01,0x07,
0x3A,0x21,0x68,0xC6,0x2E,
// '7'
0x37,0x01,0x05,0x08,0x01,0x07,
0xF8,0x44,0x42,0x22,0x10,
// '8'
0x38,0x01,0x05,0x08,0x01,0x07,
0x74,0x62,0xE9,0x46,0x2E,
// '9'
0x39,0x01,0x05,0x08,0x01,0x07,
0x74,0x63,0x16,0x84,0x5C,
// ':'
0x3A,0x03,0x02,0x06,0x03,0x07,
0xF0,0xF0,
// ';'
0x3B,0x03,0x02,0x08,0x02,0x07,
0xF0,0xF6,
// '<'
0x3C,0x03,0x05,0x06,0x01,0x07,
0x09,0x90,0x83,0x04,
// '='
0x3D,0x04,0x05,0x03,0x01,0x07,
0xF8,0x3E,
// '>'
0x3E,0x03,0x05,0x06,0x01,0x07,
0x82,0x04,0x26,0x40,
// '?'
0x3F,0x01,0x05,0x08,0x01,0x07,
0xF4,0x42,0x22,0x10,0x04,
// '@'
0x40,0x01,0x07,0x08,0x00,0x07,
0x38,0x8A,0x75,0x2A,0x56,0xD0,0x1C,
// 'A'
0x41,0x01,0x07,0x08,0x00,0x07,
0x10,0x70,0xA1,0x44,0x4F,0x91,0x41,
// 'B'
0x42,0x01,0x05,0x08,0x01,0x07,
0xF4,0x63,0xE8,0xC6,0x3E,
// 'C'
0x43,0x01,0x06,0x08,0x00,0x07,
0x3D,0x08,0x20,0x82,0x04,0x0F,
// 'D'
0x44,0x01,0x06,0x08,0x00,0x07,
0xF2,0x28,0x61,0x86,0x18,0xBC,
// 'E'
0x45,0x01,0x05,0x08,0x01,0x07,
0xFC,0x21,0x0F,0x42,0x1F,
// 'F'
0x46,0x01,0x05,0x08,0x01,0x07,
0xFC,0x21,0x0F,0x42,0x10,
// 'G'
0x47,0x01,0x06,0x08,0x00,0x07,
0x3D,0x08,0x20,0x9E,0x14,0x4F,
// 'H'
0x48,0x01,0x05,0x08,0x01,0x07,
0x8C,0x63,0xF8,0xC6,0x31,
// 'I'
0x49,0x01,0x05,0x08,0x01,0x07,
0xF9,0x08,0x42,0x10,0x9F,
// 'J'
0x4A,0x01,0x05,0x08,0x00,0x07,
0x78,0x42,0x10,0x84,0x3E,
// 'K'
0x4B,0x01,0x06,0x08,0x01,0x07,
0x8A,0x4A,0x30,0x82,0x89,0x22,
// 'L'
0x4C,0x01,0x05,0x08,0x01,0x07,
0x84,0x21,0x08,0x42,0x1F,
// 'M'
0x4D,0x01,0x06,0x08,0x00,0x07,
0xCF,0x3C,0x6D,0xB6,0x98,0x61,
// 'N'
0x4E,0x01,0x05,0x08,0x01,0x07,
0x8E,0x73,0x5A,0xCE,0x71,
// 'O'
0x4F,0x01,0x06,0x08,0x00,0x07,
0x79,0x28,0x61,0x86,0x14,0x9E,
// 'P'
0x50,0x01,0x05,0x08,0x01,0x07,
0xF4,0x63,0x1F,0x42,0x10,
// 'Q'
0x51,0x01,0x07,0x0A,0x00,0x07,
0x78,0x92,0x14,0x28,0x50,0x92,0x1C,0x0C,0x04,
// 'R'
0x52,0x01,0x06,0x08,0x01,0x07,
0xF2,0x28,0xA4,0xE2,0x89,0x22,
// 'S'
0x53,0x01,0x05,0x08,0x01,0x07,
0x7C,0x20,0xC1,0x04,0x3E,
// 'T'
0x54,0x01,0x07,0x08,0x00,0x07,
0xFE,0x20,0x40,0x81,0x02,0x04,0x08,
// 'U'
0x55,0x01,0x05,0x08,0x01,0x07,
0x8C,0x63,0x18,0xC6,0x6E,
// 'V'
0x56,0x01,0x07,0x08,0x00,0x07,
0x82,0x81,0x12,0x22,0x85,0x0E,0x08,
// 'W'
0x57,0x01,0x07,0x08,0x00,0x07,
0x83,0x06,0x4C,0x94,0xA9,0x13,0x22,
// 'X'
0x58,0x01,0x07,0x08,0x00,0x07,
0x82,0x88,0xA0,0x81,0x05,0x11,0x41,
// 'Y'
0x59,0x01,0x07,0x08,0x00,0x07,
0x82,0x88,0xA1,0xC1,0x02,0x04,0x08,
// 'Z'
0x5A,0x01,0x06,0x08,0x00,0x07,
0xFC,0x10,0x84,0x21,0x08,0x3F,
// '['
0x5B,0x00,0x04,0x0B,0x03,0x07,
0xF8,0x88,0x88,0x88,0x88,0xF0,
// '\'
0x5C,0x00,0x07,0x0B,0x00,0x07,
0x80,0x81,0x01,0x02,0x02,0x02,0x04,0x04,0x08,0x08,
// ']'
0x5D,0x00,0x04,0x0B,0x01,0x07,
0xF1,0x11,0x11,0x11,0x11,0xF0,
// '^'
0x5E,0x00,0x05,0x07,0x01,0x07,
0x01,0x08,0xA5,0x02,0x20,
// '_'
0x5F,0x09,0x07,0x01,0x00,0x07,
0xFE,
// '`'
0x60,0x00,0x02,0x02,0x02,0x07,
0x90,
// 'a'
0x61,0x03,0x06,0x06,0x01,0x07,
0x70,0x27,0xA2,0x89,0xF0,
// 'b'
0x62,0x00,0x05,0x09,0x01,0x07,
0x84,0x21,0x6C,0xC6,0x33,0xB0,
// 'c'
0x63,0x03,0x05,0x06,0x01,0x07,
0x7C,0x21,0x08,0x3C,
// 'd'
0x64,0x00,0x05,0x09,0x01,0x07,
0x08,0x42,0xFC,0xC6,0x33,0x68,
// 'e'
0x65,0x03,0x05,0x06,0x01,0x07,
0x74,0x7F,0x08,0x3C,
// 'f'
0x66,0x00,0x06,0x09,0x00,0x07,
0x1C,0x82,0x3F,0x20,0x82,0x08,0x20,
// 'g'
0x67,0x03,0x05,0x08,0x01,0x07,
0x7E,0x63,0x19,0xB4,0x2E,
// 'h'
0x68,0x00,0x05,0x09,0x01,0x07,
0x84,0x21,0x6C,0xC6,0x31,0x88,
// 'i'
0x69,0x00,0x03,0x09,0x01,0x07,
0x20,0x72,0x49,0x20,
// 'j'
0x6A,0x00,0x04,0x0B,0x01,0x07,
0x10,0x0F,0x11,0x11,0x11,0xE0,
// 'k'
0x6B,0x00,0x06,0x09,0x01,0x07,
0x82,0x08,0x22,0x93,0x8A,0x24,0x88,
// 'l'
0x6C,0x00,0x03,0x09,0x01,0x07,
0xE4,0x92,0x49,0x20,
// 'm'
0x6D,0x03,0x07,0x06,0x00,0x07,
0xB7,0xB6,0x4C,0x99,0x32,0x40,
// 'n'
0x6E,0x03,0x05,0x06,0x01,0x07,
0xB6,0x63,0x18,0xC4,
// 'o'
0x6F,0x03,0x06,0x06,0x00,0x07,
0x7A,0x18,0x61,0x85,0xE0,
// 'p'
0x70,0x03,0x05,0x08,0x01,0x07,
0xB6,0x63,0x19,0xFA,0x10,
// 'q'
0x71,0x03,0x05,0x08,0x01,0x07,
0x7E,0x63,0x18,0xB4,0x21,
// 'r'
0x72,0x03,0x05,0x06,0x01,0x07,
0xBE,0x61,0x08,0x40,
// 's'
0x73,0x03,0x05,0x06,0x01,0x07,
0x7C,0x38,0x30,0xF8,
// 't'
0x74,0x01,0x06,0x08,0x00,0x07,
0x20,0x8F,0xC8,0x20,0x82,0x07,
// 'u'
0x75,0x03,0x05,0x06,0x01,0x07,
0x8C,0x63,0x19,0xB4,
// 'v'
0x76,0x03,0x07,0x06,0x00,0x07,
0x82,0x89,0x11,0x43,0x82,0x00,
// 'w'
0x77,0x03,0x07,0x06,0x00,0x07,
0x83,0x26,0x6D,0x56,0xC8,0x80,
// 'x'
0x78,0x03,0x06,0x06,0x00,0x07,
0x85,0x23,0x0C,0x4A,0x10,
// 'y'
0x79,0x03,0x06,0x08,0x00,0x07,
0x85,0x24,0x8C,0x30,0x82,0x30,
// 'z'
0x7A,0x03,0x06,0x06,0x00,0x07,
0xFC,0x21,0x08,0x43,0xF0,
// '{'
0x7B,0x00,0x05,0x0B,0x01,0x07,
0x19,0x08,0x42,0x60,0x84,0x21,0x06,
// '|'
0x7C,0x00,0x01,0x0B,0x03,0x07,
0xFF,0xE0,
// '}'
0x7D,0x00,0x05,0x0B,0x01,0x07,
0xC1,0x08,0x42,0x0C,0x84,0x21,0x30,
// '~'
0x7E,0x05,0x07,0x02,0x00,0x07,
0x73,0x38,

// Terminator
0xFF
};