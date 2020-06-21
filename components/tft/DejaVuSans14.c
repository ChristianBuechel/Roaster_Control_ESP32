// This comes with no warranty, implied or otherwise

// This data structure was designed to support Proportional fonts
// on Arduinos. It can however handle any ttf font that has been converted
// using the conversion program. These could be fixed width or proportional 
// fonts. Individual characters do not have to be multiples of 8 bits wide. 
// Any width is fine and does not need to be fixed.

// The data bits are packed to minimize data requirements, but the tradeoff
// is that a header is required per character.

// dejavue_14.c
// Point Size   : 14
// Memory usage : 1334 bytes
// # characters : 95

// Header Format (to make Arduino UTFT Compatible):
// ------------------------------------------------
// Character Width (Used as a marker to indicate use this format. i.e.: = 0x00)
// Character Height
// First Character (Reserved. 0x00)
// Number Of Characters (Reserved. 0x00)

const unsigned char tft_Dejavu14[] =
{

0x00, 0x0D, 0x00, 0x00,

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
0x20,0x0C,0x00,0x00,0x00,0x04,

// '!'
0x21,0x02,0x01,0x0A,0x02,0x05,
0xFC,0xC0,
// '"'
0x22,0x02,0x03,0x04,0x01,0x05,
0xB6,0xD0,
// '#'
0x23,0x02,0x0A,0x0A,0x01,0x0C,
0x09,0x02,0x40,0x91,0xFF,0x12,0x04,0x8F,0xF8,0x98,0x24,0x09,0x00,
// '$'
0x24,0x01,0x07,0x0D,0x01,0x09,
0x10,0x21,0xF6,0x99,0x1A,0x1E,0x0B,0x13,0x2D,0xF0,0x81,0x00,
// '%'
0x25,0x02,0x0C,0x0A,0x00,0x0D,
0x70,0x8D,0x88,0x89,0x08,0xB0,0xDA,0xE7,0x5B,0x0D,0x10,0x91,0x11,0xB1,0x0E,
// '&'
0x26,0x02,0x0B,0x0A,0x01,0x0C,
0x38,0x08,0x81,0x00,0x10,0x05,0x01,0x10,0xA1,0x14,0x1C,0x43,0x87,0x98,
// '''
0x27,0x02,0x01,0x04,0x01,0x03,
0xF0,
// '('
0x28,0x01,0x03,0x0C,0x01,0x05,
0x29,0x49,0x24,0x89,0x10,
// ')'
0x29,0x01,0x03,0x0C,0x01,0x05,
0x89,0x12,0x49,0x29,0x40,
// '*'
0x2A,0x02,0x07,0x06,0x00,0x07,
0x11,0x24,0xE1,0xC9,0x22,0x00,
// '+'
0x2B,0x03,0x09,0x09,0x01,0x0C,
0x08,0x04,0x02,0x01,0x0F,0xF8,0x40,0x20,0x10,0x08,0x00,
// ','
0x2C,0x0A,0x02,0x03,0x01,0x04,
0x58,
// '-'
0x2D,0x08,0x04,0x01,0x01,0x05,
0xF0,
// '.'
0x2E,0x0A,0x01,0x02,0x02,0x04,
0xC0,
// '/'
0x2F,0x02,0x05,0x0C,0x00,0x05,
0x08,0xC4,0x23,0x10,0x8C,0x42,0x31,0x00,
// '0'
0x30,0x02,0x07,0x0A,0x01,0x09,
0x38,0x8A,0x0C,0x18,0x30,0x60,0xC1,0x44,0x70,
// '1'
0x31,0x02,0x05,0x0A,0x02,0x09,
0x65,0x08,0x42,0x10,0x84,0x27,0xC0,
// '2'
0x32,0x02,0x07,0x0A,0x01,0x09,
0x79,0x18,0x10,0x20,0x81,0x04,0x10,0x41,0xF8,
// '3'
0x33,0x02,0x07,0x0A,0x01,0x09,
0x7D,0x0C,0x08,0x33,0xC0,0xC0,0x81,0x86,0xF0,
// '4'
0x34,0x02,0x07,0x0A,0x01,0x09,
0x0C,0x28,0x91,0x24,0x50,0xBF,0x82,0x04,0x08,
// '5'
0x35,0x02,0x07,0x0A,0x01,0x09,
0xFD,0x02,0x07,0xC8,0x40,0x40,0x81,0x84,0xF0,
// '6'
0x36,0x02,0x07,0x0A,0x01,0x09,
0x3C,0xC5,0x04,0x0B,0x98,0xE0,0xC1,0x46,0x78,
// '7'
0x37,0x02,0x07,0x0A,0x01,0x09,
0xFE,0x08,0x10,0x40,0x82,0x04,0x10,0x20,0x80,
// '8'
0x38,0x02,0x07,0x0A,0x01,0x09,
0x7D,0x8E,0x0E,0x37,0xD8,0xE0,0xC1,0xC6,0xF8,
// '9'
0x39,0x02,0x07,0x0A,0x01,0x09,
0x79,0x8A,0x0C,0x1C,0x6F,0x40,0x82,0x8C,0xF0,
// ':'
0x3A,0x05,0x01,0x07,0x02,0x05,
0xC6,
// ';'
0x3B,0x05,0x02,0x08,0x01,0x05,
0x50,0x16,
// '<'
0x3C,0x04,0x09,0x08,0x01,0x0C,
0x00,0x83,0x8E,0x1C,0x0E,0x01,0xC0,0x1C,0x01,
// '='
0x3D,0x05,0x09,0x04,0x01,0x0C,
0xFF,0x80,0x00,0x1F,0xF0,
// '>'
0x3E,0x04,0x09,0x08,0x01,0x0C,
0x80,0x38,0x03,0x80,0x70,0x38,0x71,0xC1,0x00,
// '?'
0x3F,0x02,0x05,0x0A,0x01,0x07,
0x74,0x42,0x33,0x10,0x80,0x21,0x00,
// '@'
0x40,0x02,0x0C,0x0C,0x01,0x0E,
0x0F,0x83,0x0C,0x60,0x24,0xE9,0x99,0x99,0x09,0x90,0x99,0x9A,0x4E,0xC4,0x00,0x30,0xC0,0xF8,
// 'A'
0x41,0x02,0x09,0x0A,0x00,0x09,
0x08,0x0A,0x05,0x04,0x42,0x21,0x11,0xFC,0x82,0x41,0x40,0x40,
// 'B'
0x42,0x02,0x08,0x0A,0x01,0x0A,
0xFE,0x83,0x81,0x83,0xFE,0x83,0x81,0x81,0x83,0xFE,
// 'C'
0x43,0x02,0x08,0x0A,0x01,0x0A,
0x1E,0x61,0x40,0x80,0x80,0x80,0x80,0x40,0x61,0x1E,
// 'D'
0x44,0x02,0x09,0x0A,0x01,0x0B,
0xFC,0x41,0xA0,0x50,0x18,0x0C,0x06,0x03,0x02,0x83,0x7E,0x00,
// 'E'
0x45,0x02,0x07,0x0A,0x01,0x09,
0xFF,0x02,0x04,0x0F,0xF0,0x20,0x40,0x81,0xFC,
// 'F'
0x46,0x02,0x06,0x0A,0x01,0x08,
0xFE,0x08,0x20,0xFE,0x08,0x20,0x82,0x00,
// 'G'
0x47,0x02,0x09,0x0A,0x01,0x0B,
0x1F,0x30,0x50,0x10,0x08,0x04,0x1E,0x02,0x81,0x60,0x8F,0x80,
// 'H'
0x48,0x02,0x08,0x0A,0x01,0x0A,
0x81,0x81,0x81,0x81,0xFF,0x81,0x81,0x81,0x81,0x81,
// 'I'
0x49,0x02,0x01,0x0A,0x01,0x03,
0xFF,0xC0,
// 'J'
0x4A,0x02,0x03,0x0D,0xFF,0x03,
0x24,0x92,0x49,0x24,0x9C,
// 'K'
0x4B,0x02,0x08,0x0A,0x01,0x09,
0x82,0x84,0x88,0x90,0xA0,0xA0,0x90,0x88,0x84,0x82,
// 'L'
0x4C,0x02,0x06,0x0A,0x01,0x07,
0x82,0x08,0x20,0x82,0x08,0x20,0x83,0xF0,
// 'M'
0x4D,0x02,0x0A,0x0A,0x01,0x0C,
0xC0,0xF0,0x3A,0x16,0x85,0x92,0x64,0x98,0xC6,0x31,0x80,0x60,0x10,
// 'N'
0x4E,0x02,0x08,0x0A,0x01,0x0A,
0xC1,0xC1,0xA1,0x91,0x91,0x89,0x89,0x85,0x83,0x83,
// 'O'
0x4F,0x02,0x09,0x0A,0x01,0x0B,
0x3E,0x31,0x90,0x50,0x18,0x0C,0x06,0x02,0x82,0x63,0x1F,0x00,
// 'P'
0x50,0x02,0x07,0x0A,0x01,0x09,
0xFD,0x0E,0x0C,0x18,0x7F,0xA0,0x40,0x81,0x00,
// 'Q'
0x51,0x02,0x09,0x0C,0x01,0x0B,
0x3E,0x31,0x90,0x50,0x18,0x0C,0x06,0x02,0x83,0x63,0x1F,0x00,0x80,0x20,
// 'R'
0x52,0x02,0x08,0x0A,0x01,0x0A,
0xFC,0x86,0x82,0x82,0x86,0xFC,0x84,0x82,0x82,0x81,
// 'S'
0x53,0x02,0x07,0x0A,0x01,0x09,
0x3D,0x86,0x04,0x06,0x03,0x80,0x81,0x86,0xF8,
// 'T'
0x54,0x02,0x09,0x0A,0x00,0x09,
0xFF,0x84,0x02,0x01,0x00,0x80,0x40,0x20,0x10,0x08,0x04,0x00,
// 'U'
0x55,0x02,0x08,0x0A,0x01,0x0A,
0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x42,0x3C,
// 'V'
0x56,0x02,0x09,0x0A,0x00,0x09,
0x80,0xA0,0x90,0x48,0x22,0x21,0x10,0x50,0x28,0x1C,0x04,0x00,
// 'W'
0x57,0x02,0x0D,0x0A,0x00,0x0D,
0x82,0x0C,0x38,0x51,0x44,0x8A,0x24,0x51,0x14,0x40,0xA2,0x85,0x14,0x38,0xE0,0x82,0x00,
// 'X'
0x58,0x02,0x09,0x0A,0x00,0x09,
0xC1,0xA0,0x88,0x82,0x80,0x80,0xC0,0x50,0x44,0x41,0x60,0xC0,
// 'Y'
0x59,0x02,0x09,0x0A,0x00,0x09,
0xC1,0xA0,0x88,0x84,0x41,0x40,0x40,0x20,0x10,0x08,0x04,0x00,
// 'Z'
0x5A,0x02,0x08,0x0A,0x01,0x0A,
0xFF,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0xFF,
// '['
0x5B,0x01,0x03,0x0C,0x01,0x05,
0xF2,0x49,0x24,0x92,0x70,
// '\'
0x5C,0x02,0x05,0x0C,0x00,0x05,
0x86,0x10,0x86,0x10,0x86,0x10,0x86,0x10,
// ']'
0x5D,0x01,0x03,0x0C,0x01,0x05,
0xE4,0x92,0x49,0x24,0xF0,
// '^'
0x5E,0x02,0x09,0x04,0x01,0x0C,
0x1C,0x1B,0x18,0xD8,0x30,
// '_'
0x5F,0x0E,0x07,0x01,0x00,0x07,
0xFE,
// '`'
0x60,0x00,0x03,0x03,0x01,0x07,
0xC8,0x80,
// 'a'
0x61,0x04,0x06,0x08,0x01,0x08,
0x7A,0x30,0x5F,0xC6,0x18,0xDD,
// 'b'
0x62,0x01,0x07,0x0B,0x01,0x09,
0x81,0x02,0x05,0xCC,0x50,0x60,0xC1,0x83,0x8A,0xE0,
// 'c'
0x63,0x04,0x06,0x08,0x01,0x08,
0x39,0x18,0x20,0x82,0x04,0x4E,
// 'd'
0x64,0x01,0x07,0x0B,0x01,0x09,
0x02,0x04,0x09,0xD4,0x70,0x60,0xC1,0x82,0x8C,0xE8,
// 'e'
0x65,0x04,0x07,0x08,0x01,0x09,
0x38,0x8A,0x0F,0xF8,0x10,0x10,0x9E,
// 'f'
0x66,0x01,0x04,0x0B,0x00,0x04,
0x34,0x4F,0x44,0x44,0x44,0x40,
// 'g'
0x67,0x04,0x07,0x0B,0x01,0x09,
0x3A,0x8E,0x0C,0x18,0x30,0x51,0x9D,0x02,0x88,0xE0,
// 'h'
0x68,0x01,0x07,0x0B,0x01,0x09,
0x81,0x02,0x05,0xEC,0x70,0x60,0xC1,0x83,0x06,0x08,
// 'i'
0x69,0x01,0x01,0x0B,0x01,0x03,
0xDF,0xE0,
// 'j'
0x6A,0x01,0x03,0x0E,0xFF,0x03,
0x24,0x12,0x49,0x24,0x93,0x80,
// 'k'
0x6B,0x01,0x07,0x0B,0x01,0x08,
0x81,0x02,0x04,0x28,0x92,0x38,0x50,0x91,0x12,0x10,
// 'l'
0x6C,0x01,0x01,0x0B,0x01,0x03,
0xFF,0xE0,
// 'm'
0x6D,0x04,0x0B,0x08,0x01,0x0D,
0xB9,0xD9,0xCE,0x10,0xC2,0x18,0x43,0x08,0x61,0x0C,0x21,
// 'n'
0x6E,0x04,0x07,0x08,0x01,0x09,
0xBD,0x8E,0x0C,0x18,0x30,0x60,0xC1,
// 'o'
0x6F,0x04,0x07,0x08,0x01,0x09,
0x38,0x8A,0x0C,0x18,0x30,0x51,0x1C,
// 'p'
0x70,0x04,0x07,0x0B,0x01,0x09,
0xB9,0x8A,0x0C,0x18,0x30,0x71,0x5C,0x81,0x02,0x00,
// 'q'
0x71,0x04,0x07,0x0B,0x01,0x09,
0x3A,0x8E,0x0C,0x18,0x30,0x51,0x9D,0x02,0x04,0x08,
// 'r'
0x72,0x04,0x04,0x08,0x01,0x05,
0xBC,0x88,0x88,0x88,
// 's'
0x73,0x04,0x06,0x08,0x01,0x08,
0x7A,0x18,0x38,0x18,0x18,0x5E,
// 't'
0x74,0x02,0x05,0x0A,0x00,0x05,
0x42,0x3E,0x84,0x21,0x08,0x41,0xC0,
// 'u'
0x75,0x04,0x07,0x08,0x01,0x09,
0x83,0x06,0x0C,0x18,0x30,0x71,0xBD,
// 'v'
0x76,0x04,0x09,0x08,0xFF,0x07,
0x41,0x20,0x88,0x84,0x42,0x20,0xA0,0x50,0x10,
// 'w'
0x77,0x04,0x0B,0x08,0x00,0x0B,
0x44,0x48,0x89,0x11,0x17,0x42,0xA8,0x77,0x04,0x40,0x88,
// 'x'
0x78,0x04,0x09,0x08,0xFF,0x07,
0x41,0x11,0x05,0x01,0x00,0x80,0xA0,0x88,0x82,
// 'y'
0x79,0x04,0x09,0x0B,0xFF,0x07,
0x41,0x20,0x88,0x84,0x41,0x40,0xA0,0x20,0x10,0x08,0x08,0x18,0x00,
// 'z'
0x7A,0x04,0x06,0x08,0x01,0x08,
0xFC,0x10,0x84,0x21,0x08,0x3F,
// '{'
0x7B,0x01,0x05,0x0D,0x02,0x09,
0x19,0x08,0x42,0x13,0x04,0x21,0x08,0x41,0x80,
// '|'
0x7C,0x01,0x01,0x0E,0x02,0x05,
0xFF,0xFC,
// '}'
0x7D,0x01,0x05,0x0D,0x03,0x09,
0xC1,0x08,0x42,0x10,0x64,0x21,0x08,0x4C,0x00,
// '~'
0x7E,0x06,0x09,0x04,0x01,0x0C,
0x00,0x3C,0x63,0xC0,0x00,

// Terminator
0xFF
};