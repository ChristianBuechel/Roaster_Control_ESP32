// This comes with no warranty, implied or otherwise

// This data structure was designed to support Proportional fonts
// on Arduinos. It can however handle any ttf font that has been converted
// using the conversion program. These could be fixed width or proportional 
// fonts. Individual characters do not have to be multiples of 8 bits wide. 
// Any width is fine and does not need to be fixed.

// The data bits are packed to minimize data requirements, but the tradeoff
// is that a header is required per character.

// consola12.c
// Point Size   : 12
// Memory usage : 1077 bytes
// # characters : 95

// Header Format (to make Arduino UTFT Compatible):
// ------------------------------------------------
// Character Width (Used as a marker to indicate use this format. i.e.: = 0x00)
// Character Height
// First Character (Reserved. 0x00)
// Number Of Characters (Reserved. 0x00)

const unsigned char tft_consola12[] =
{

0x00, 0x0C, 0x00, 0x00,

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
0x20,0x0A,0x00,0x00,0x00,0x07,

// '!'
0x21,0x02,0x03,0x08,0x03,0x07,
0xDB,0x68,0x36,
// '"'
0x22,0x02,0x05,0x03,0x01,0x07,
0xDE,0xF6,
// '#'
0x23,0x02,0x06,0x08,0x00,0x07,
0x38,0xEF,0xCE,0x73,0xF7,0x1C,
// '$'
0x24,0x01,0x06,0x0A,0x01,0x07,
0x11,0xED,0x30,0x60,0xE2,0xCA,0xF8,0x80,
// '%'
0x25,0x02,0x07,0x08,0x00,0x07,
0x63,0xCB,0xA3,0x81,0xC7,0x97,0x46,
// '&'
0x26,0x02,0x08,0x08,0x00,0x07,
0x38,0x68,0x68,0x30,0x74,0xCC,0xCC,0x7A,
// '''
0x27,0x02,0x02,0x03,0x03,0x07,
0xFC,
// '('
0x28,0x01,0x04,0x0B,0x01,0x07,
0x12,0x64,0x44,0x44,0x62,0x10,
// ')'
0x29,0x01,0x04,0x0B,0x02,0x07,
0x84,0x62,0x22,0x22,0x64,0x80,
// '*'
0x2A,0x02,0x06,0x05,0x01,0x07,
0x22,0xA7,0x2A,0x20,
// '+'
0x2B,0x04,0x06,0x05,0x01,0x07,
0x20,0x8F,0xCC,0x30,
// ','
0x2C,0x08,0x04,0x04,0x01,0x07,
0x66,0x6C,
// '-'
0x2D,0x06,0x04,0x01,0x02,0x07,
0xF0,
// '.'
0x2E,0x08,0x03,0x02,0x02,0x07,
0xD8,
// '/'
0x2F,0x02,0x05,0x09,0x01,0x07,
0x08,0x84,0x42,0x11,0x08,0x80,
// '0'
0x30,0x02,0x07,0x08,0x01,0x07,
0x38,0x9B,0x36,0xAE,0x58,0x93,0x3C,
// '1'
0x31,0x02,0x05,0x08,0x01,0x07,
0x75,0x8C,0x63,0x18,0xDF,
// '2'
0x32,0x02,0x05,0x08,0x01,0x07,
0x74,0xC6,0x33,0x31,0x1F,
// '3'
0x33,0x02,0x06,0x08,0x01,0x07,
0xF0,0x61,0x9C,0x18,0x21,0xBC,
// '4'
0x34,0x02,0x06,0x08,0x00,0x07,
0x1C,0x72,0xD3,0x8F,0xF0,0xC3,
// '5'
0x35,0x02,0x06,0x08,0x01,0x07,
0xFA,0x08,0x3C,0x08,0x31,0xBC,
// '6'
0x36,0x02,0x06,0x08,0x01,0x07,
0x39,0x8C,0x3E,0xCB,0x2C,0x9C,
// '7'
0x37,0x02,0x05,0x08,0x01,0x07,
0xF8,0x44,0x22,0x11,0x88,
// '8'
0x38,0x02,0x06,0x08,0x01,0x07,
0x73,0x6D,0x9C,0xDB,0x3C,0x9E,
// '9'
0x39,0x02,0x06,0x08,0x01,0x07,
0x73,0x2C,0xB2,0x68,0x21,0xB8,
// ':'
0x3A,0x04,0x03,0x06,0x02,0x07,
0xD8,0x0D,0x80,
// ';'
0x3B,0x04,0x04,0x08,0x01,0x07,
0x66,0x00,0x66,0x6C,
// '<'
0x3C,0x03,0x05,0x07,0x01,0x07,
0x09,0x99,0x86,0x18,0x20,
// '='
0x3D,0x05,0x05,0x03,0x01,0x07,
0xF8,0x3E,
// '>'
0x3E,0x03,0x05,0x07,0x01,0x07,
0x83,0x0C,0x33,0x32,0x00,
// '?'
0x3F,0x02,0x05,0x08,0x02,0x07,
0xE1,0x86,0x2F,0x03,0x18,
// '@'
0x40,0x02,0x09,0x0A,0xFF,0x07,
0x0E,0x19,0x88,0x4D,0xA5,0xD2,0xE9,0x38,0xC0,0x20,0x0E,0x00,
// 'A'
0x41,0x02,0x07,0x08,0x00,0x07,
0x10,0x50,0xA1,0x40,0x4F,0x91,0x22,
// 'B'
0x42,0x02,0x06,0x08,0x01,0x07,
0xF3,0x6D,0xBC,0xCB,0x2C,0xBC,
// 'C'
0x43,0x02,0x05,0x08,0x01,0x07,
0x3A,0x31,0x8C,0x61,0x0F,
// 'D'
0x44,0x02,0x06,0x08,0x01,0x07,
0xF3,0x6C,0xB3,0xCF,0x2D,0xBC,
// 'E'
0x45,0x02,0x04,0x08,0x01,0x07,
0xFC,0xCF,0xCC,0xCF,
// 'F'
0x46,0x02,0x04,0x08,0x01,0x07,
0xFC,0xCF,0xCC,0xCC,
// 'G'
0x47,0x02,0x07,0x08,0x00,0x07,
0x3E,0xC3,0x06,0xFC,0x78,0xD9,0x9E,
// 'H'
0x48,0x02,0x05,0x08,0x01,0x07,
0xDE,0xF7,0xFD,0xEF,0x7B,
// 'I'
0x49,0x02,0x06,0x08,0x01,0x07,
0xFC,0xC3,0x0C,0x30,0xC3,0x3F,
// 'J'
0x4A,0x02,0x05,0x08,0x01,0x07,
0xF8,0xC6,0x31,0x8A,0x4C,
// 'K'
0x4B,0x02,0x06,0x08,0x01,0x07,
0xCB,0x4D,0x38,0xE3,0xCD,0x32,
// 'L'
0x4C,0x02,0x04,0x08,0x01,0x07,
0xCC,0xCC,0xCC,0xCF,
// 'M'
0x4D,0x02,0x07,0x08,0x00,0x07,
0x44,0x89,0xB2,0x25,0x48,0x91,0x23,
// 'N'
0x4E,0x02,0x05,0x08,0x01,0x07,
0xDE,0xF7,0xFF,0xEF,0x7B,
// 'O'
0x4F,0x02,0x06,0x08,0x00,0x07,
0x79,0x2C,0xF3,0xCF,0x34,0x9E,
// 'P'
0x50,0x02,0x06,0x08,0x01,0x07,
0xF3,0x2C,0xF2,0xF3,0x0C,0x30,
// 'Q'
0x51,0x02,0x07,0x0A,0x00,0x07,
0x38,0x9B,0x16,0x3C,0x78,0x93,0x3C,0x18,0x18,
// 'R'
0x52,0x02,0x07,0x08,0x01,0x07,
0xF9,0x93,0x36,0x4F,0x1B,0x33,0x62,
// 'S'
0x53,0x02,0x06,0x08,0x01,0x07,
0x7B,0x0C,0x18,0x18,0x30,0xBE,
// 'T'
0x54,0x02,0x06,0x08,0x01,0x07,
0xFC,0xC3,0x0C,0x30,0xC3,0x0C,
// 'U'
0x55,0x02,0x06,0x08,0x01,0x07,
0xCF,0x3C,0xF3,0xCF,0x2C,0x9C,
// 'V'
0x56,0x02,0x07,0x08,0x00,0x07,
0x44,0x89,0x13,0x42,0x85,0x0E,0x08,
// 'W'
0x57,0x02,0x07,0x08,0x00,0x07,
0x44,0x89,0x12,0xA5,0x4A,0x9B,0x36,
// 'X'
0x58,0x02,0x07,0x08,0x00,0x07,
0xCC,0x90,0xE1,0x83,0x0D,0x11,0x63,
// 'Y'
0x59,0x02,0x08,0x08,0x00,0x07,
0xC2,0x66,0x64,0x38,0x18,0x18,0x18,0x18,
// 'Z'
0x5A,0x02,0x05,0x08,0x01,0x07,
0xF8,0x84,0x42,0x21,0x1F,
// '['
0x5B,0x02,0x04,0x0A,0x01,0x07,
0x74,0x44,0x44,0x44,0x47,
// '\'
0x5C,0x02,0x05,0x09,0x01,0x07,
0x82,0x10,0x42,0x10,0x42,0x08,
// ']'
0x5D,0x02,0x04,0x0A,0x02,0x07,
0xF3,0x33,0x33,0x33,0x3F,
// '^'
0x5E,0x02,0x07,0x04,0x00,0x07,
0x10,0x51,0xB2,0x20,
// '_'
0x5F,0x0B,0x07,0x01,0x00,0x07,
0xFE,
// '`'
0x60,0x01,0x05,0x03,0x00,0x07,
0x61,0x80,
// 'a'
0x61,0x04,0x06,0x06,0x01,0x07,
0x78,0x27,0xF3,0xCD,0xF0,
// 'b'
0x62,0x02,0x06,0x08,0x01,0x07,
0xC3,0x0F,0xB2,0xCB,0x2C,0xBC,
// 'c'
0x63,0x04,0x05,0x06,0x01,0x07,
0x7E,0x31,0x8C,0x3C,
// 'd'
0x64,0x02,0x06,0x08,0x01,0x07,
0x0C,0x37,0xF3,0xCF,0x3D,0xDB,
// 'e'
0x65,0x04,0x06,0x06,0x01,0x07,
0x7B,0x2F,0xB0,0xC1,0xE0,
// 'f'
0x66,0x02,0x07,0x08,0x00,0x07,
0x1E,0x60,0xC7,0xE3,0x06,0x0C,0x18,
// 'g'
0x67,0x04,0x07,0x08,0x01,0x07,
0x7F,0x93,0x27,0xCC,0x1F,0x33,0x3C,
// 'h'
0x68,0x02,0x06,0x08,0x01,0x07,
0xC3,0x0F,0xB2,0xCF,0x3C,0xF3,
// 'i'
0x69,0x01,0x05,0x09,0x01,0x07,
0x31,0x81,0xE3,0x18,0xC6,0xF8,
// 'j'
0x6A,0x01,0x04,0x0B,0x01,0x07,
0x33,0x0F,0x33,0x33,0x33,0xE0,
// 'k'
0x6B,0x02,0x06,0x08,0x01,0x07,
0xC3,0x0C,0xF6,0xF3,0xCD,0xB2,
// 'l'
0x6C,0x02,0x05,0x08,0x01,0x07,
0xF1,0x8C,0x63,0x18,0xDF,
// 'm'
0x6D,0x04,0x06,0x06,0x01,0x07,
0xEB,0xEF,0xFF,0xFF,0xF0,
// 'n'
0x6E,0x04,0x06,0x06,0x01,0x07,
0xFB,0x2C,0xF3,0xCF,0x30,
// 'o'
0x6F,0x04,0x06,0x06,0x01,0x07,
0x73,0x2C,0xB2,0xC9,0xC0,
// 'p'
0x70,0x04,0x06,0x08,0x01,0x07,
0xFB,0x2C,0xB2,0xDB,0xCC,0x30,
// 'q'
0x71,0x04,0x06,0x08,0x01,0x07,
0x7B,0x3C,0xF3,0xDD,0xB0,0xC3,
// 'r'
0x72,0x04,0x06,0x06,0x01,0x07,
0xFB,0x2C,0x30,0xC3,0x00,
// 's'
0x73,0x04,0x05,0x06,0x01,0x07,
0x76,0x38,0x61,0x78,
// 't'
0x74,0x02,0x05,0x08,0x00,0x07,
0x21,0x3E,0x42,0x10,0x87,
// 'u'
0x75,0x04,0x06,0x06,0x01,0x07,
0xCF,0x3C,0xF3,0xDD,0xF0,
// 'v'
0x76,0x04,0x07,0x06,0x00,0x07,
0x44,0x89,0x01,0x42,0x82,0x00,
// 'w'
0x77,0x04,0x07,0x06,0x00,0x07,
0x44,0xA1,0x42,0x46,0xC4,0x80,
// 'x'
0x78,0x04,0x07,0x06,0x00,0x07,
0x6C,0x50,0x41,0x44,0x58,0xC0,
// 'y'
0x79,0x04,0x07,0x08,0x00,0x07,
0x44,0x88,0xA1,0x43,0x02,0x08,0x70,
// 'z'
0x7A,0x04,0x05,0x06,0x01,0x07,
0xF8,0x80,0x04,0x7C,
// '{'
0x7B,0x02,0x05,0x0A,0x01,0x07,
0x19,0x88,0x4E,0x10,0x84,0x30,0xC0,
// '|'
0x7C,0x00,0x02,0x0C,0x03,0x07,
0xAA,0xAA,0xAA,
// '}'
0x7D,0x02,0x05,0x0A,0x01,0x07,
0xC3,0x08,0x43,0x90,0x84,0x66,0x00,
// '~'
0x7E,0x06,0x08,0x02,0xFF,0x07,
0x32,0x6E,

// Terminator
0xFF
};