// This comes with no warranty, implied or otherwise

// This data structure was designed to support Proportional fonts
// on Arduinos. It can however handle any ttf font that has been converted
// using the conversion program. These could be fixed width or proportional 
// fonts. Individual characters do not have to be multiples of 8 bits wide. 
// Any width is fine and does not need to be fixed.

// The data bits are packed to minimize data requirements, but the tradeoff
// is that a header is required per character.

// consola20.c
// Point Size   : 20
// Memory usage : 1975 bytes
// # characters : 95

// Header Format (to make Arduino UTFT Compatible):
// ------------------------------------------------
// Character Width (Used as a marker to indicate use this format. i.e.: = 0x00)
// Character Height
// First Character (Reserved. 0x00)
// Number Of Characters (Reserved. 0x00)

const unsigned char tft_Consola_20[] = 
{
0x00, 0x14, 0x00, 0x00,

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
0x20,0x10,0x00,0x00,0x00,0x0B,

// '!'
0x21,0x02,0x03,0x0F,0x04,0x0B,
0x59,0x24,0x92,0x48,0x0F,0xC0,
// '"'
0x22,0x02,0x07,0x05,0x02,0x0B,
0x6C,0xD9,0xB2,0x24,0x40,
// '#'
0x23,0x03,0x0B,0x0D,0x00,0x0B,
0x09,0x83,0x30,0x66,0x3F,0xE7,0xFC,0x22,0x04,0x40,0x88,0x7F,0xC6,0x60,0xCC,0x19,0x83,0x30,
// '$'
0x24,0x01,0x09,0x12,0x01,0x0B,
0x04,0x02,0x07,0x8F,0xEC,0xC6,0x63,0x20,0xF0,0x3E,0x07,0x86,0xE3,0x31,0x9F,0xFB,0xF8,0x20,0x10,0x00,0x00,
// '%'
0x25,0x02,0x0B,0x0F,0x00,0x0B,
0x30,0x4F,0x1B,0x32,0x66,0xC7,0xB0,0x64,0x01,0x80,0x60,0x09,0x83,0x78,0xC9,0x93,0x36,0x24,0x87,0x80,0x00,
// '&'
0x26,0x02,0x0B,0x0F,0x00,0x0B,
0x00,0x03,0xE0,0xCC,0x11,0x83,0x30,0x6C,0x07,0x01,0xE2,0x6E,0xC8,0xD9,0x0E,0x30,0xC7,0x3C,0x7D,0xC0,0x00,
// '''
0x27,0x02,0x03,0x05,0x04,0x0B,
0x5D,0x24,
// '('
0x28,0x01,0x07,0x14,0x02,0x0B,
0x00,0x18,0x30,0xC3,0x06,0x18,0x30,0x60,0xC1,0x83,0x06,0x04,0x0C,0x18,0x18,0x18,0x10,0x00,
// ')'
0x29,0x01,0x07,0x14,0x02,0x0B,
0x00,0xC1,0x81,0x81,0x83,0x03,0x06,0x0C,0x18,0x30,0x60,0xC1,0x06,0x08,0x30,0xC1,0x00,0x00,
// '*'
0x2A,0x02,0x09,0x09,0x01,0x0B,
0x08,0x04,0x12,0xC7,0xC1,0xC3,0x59,0x24,0x10,0x00,0x00,
// '+'
0x2B,0x05,0x0B,0x0B,0x00,0x0B,
0x00,0x00,0x80,0x10,0x02,0x00,0x40,0xFF,0x83,0x80,0x20,0x04,0x00,0x80,0x00,0x00,
// ','
0x2C,0x0C,0x06,0x08,0x02,0x0B,
0x00,0xC3,0x86,0x18,0xCE,0x00,
// '-'
0x2D,0x09,0x07,0x03,0x02,0x0B,
0x00,0xF8,0x00,
// '.'
0x2E,0x0C,0x05,0x05,0x03,0x0B,
0x03,0x9C,0xE0,0x00,
// '/'
0x2F,0x02,0x09,0x11,0x01,0x0B,
0x01,0x01,0x80,0x80,0xC0,0x60,0x20,0x30,0x10,0x18,0x0C,0x0C,0x06,0x02,0x03,0x01,0x01,0x80,0x00,0x00,
// '0'
0x30,0x03,0x0B,0x0E,0x00,0x0B,
0x0E,0x07,0xF0,0xC6,0x30,0x66,0x1C,0xC7,0x9B,0xB3,0xC6,0x70,0xCC,0x18,0x82,0x1D,0xC1,0xF0,0x00,0x00,
// '1'
0x31,0x03,0x09,0x0D,0x01,0x0B,
0x0C,0x1E,0x1B,0x01,0x80,0xC0,0x60,0x30,0x18,0x0C,0x06,0x03,0x0F,0xE7,0xF8,
// '2'
0x32,0x03,0x09,0x0D,0x01,0x0B,
0x3C,0x3F,0x10,0xC0,0x60,0x30,0x18,0x18,0x18,0x1C,0x1C,0x1C,0x0F,0xFF,0xF8,
// '3'
0x33,0x03,0x09,0x0E,0x01,0x0B,
0x7C,0x3F,0x00,0xC0,0x60,0x30,0x30,0xF8,0x06,0x01,0x00,0xC0,0xC0,0xE7,0xC0,0x00,
// '4'
0x34,0x03,0x0B,0x0D,0x00,0x0B,
0x03,0x00,0xE0,0x3C,0x06,0x81,0x90,0x62,0x0C,0x43,0x08,0x63,0x9F,0xF8,0x0C,0x00,0x80,0x10,
// '5'
0x35,0x03,0x09,0x0E,0x01,0x0B,
0x7F,0x3F,0x98,0x0C,0x06,0x03,0xC1,0xFC,0x06,0x01,0x00,0x80,0xC0,0xC7,0xC0,0x00,
// '6'
0x36,0x03,0x09,0x0E,0x01,0x0B,
0x07,0x1F,0x8C,0x0C,0x04,0x06,0xF3,0xFD,0x83,0xC1,0xE0,0xD0,0x6C,0x63,0xE0,0x00,
// '7'
0x37,0x03,0x09,0x0D,0x01,0x0B,
0xFF,0xFF,0xC0,0x40,0x60,0x20,0x30,0x18,0x18,0x0C,0x0C,0x06,0x06,0x03,0x00,
// '8'
0x38,0x03,0x09,0x0E,0x01,0x0B,
0x3E,0x3B,0x90,0x58,0x26,0x33,0xF0,0xF8,0xEE,0x63,0xE0,0xF0,0x6C,0x63,0xE0,0x00,
// '9'
0x39,0x03,0x0A,0x0D,0x00,0x0B,
0x1E,0x0F,0xE6,0x19,0x82,0x60,0xD8,0x33,0x1C,0x7F,0x00,0x80,0x60,0x18,0x7C,0x3C,0x00,
// ':'
0x3A,0x06,0x03,0x0B,0x04,0x0B,
0x5F,0x80,0x02,0xFC,0x00,
// ';'
0x3B,0x06,0x06,0x0E,0x02,0x0B,
0x10,0xE3,0x80,0x00,0x00,0x0E,0x38,0x61,0x8C,0xE0,0x00,
// '<'
0x3C,0x05,0x08,0x0C,0x01,0x0B,
0x00,0x07,0x0C,0x18,0x30,0x60,0x70,0x18,0x0C,0x06,0x03,0x00,
// '='
0x3D,0x08,0x09,0x06,0x01,0x0B,
0xFF,0xBF,0x80,0x00,0x0F,0xF8,0x00,
// '>'
0x3E,0x05,0x08,0x0C,0x02,0x0B,
0x00,0xE0,0x30,0x18,0x0C,0x06,0x0E,0x18,0x30,0x60,0xC0,0x00,
// '?'
0x3F,0x02,0x07,0x0F,0x02,0x0B,
0x60,0xF0,0x30,0x30,0x60,0xC7,0x1C,0x30,0x60,0x00,0x03,0x06,0x00,0x00,
// '@'
0x40,0x02,0x0B,0x13,0x00,0x0B,
0x07,0x03,0x30,0xC1,0x10,0x26,0x06,0x8E,0xD2,0xDE,0xDB,0xD3,0x7A,0x6F,0x4D,0xE9,0xAD,0xFD,0x80,0x10,0x03,0x00,0x31,0x03,0xE0,0x00,0x00,
// 'A'
0x41,0x03,0x0B,0x0D,0x00,0x0B,
0x0E,0x01,0xC0,0x38,0x0D,0x81,0xB0,0x22,0x0C,0x61,0x8C,0x31,0x8F,0xF9,0x83,0x20,0x2C,0x06,
// 'B'
0x42,0x03,0x09,0x0D,0x01,0x0B,
0x7C,0x3F,0x90,0xC8,0x24,0x33,0x39,0xF8,0x86,0x41,0xA0,0xD0,0x6D,0xE7,0xE0,
// 'C'
0x43,0x03,0x0A,0x0E,0x00,0x0B,
0x0F,0x87,0xF3,0x00,0x80,0x60,0x18,0x06,0x01,0x80,0x60,0x18,0x03,0x00,0xF3,0x0F,0x80,0x00,
// 'D'
0x44,0x03,0x0A,0x0D,0x01,0x0B,
0xF8,0x3F,0x8C,0x33,0x06,0xC1,0xB0,0x6C,0x1B,0x06,0xC1,0xB0,0x6C,0x33,0xF8,0xFC,0x00,
// 'E'
0x45,0x03,0x09,0x0D,0x01,0x0B,
0x7F,0x3F,0x98,0x0C,0x06,0x03,0x01,0xFC,0xC0,0x60,0x30,0x18,0x0C,0x07,0xF0,
// 'F'
0x46,0x03,0x08,0x0D,0x02,0x0B,
0xFE,0xFE,0xC0,0xC0,0xC0,0xC0,0xFE,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,
// 'G'
0x47,0x03,0x0A,0x0E,0x00,0x0B,
0x0F,0x87,0xF3,0x01,0x80,0x60,0x18,0x04,0x3D,0x83,0x60,0xD8,0x37,0x0C,0xE3,0x1F,0xC0,0x00,
// 'H'
0x48,0x03,0x09,0x0D,0x01,0x0B,
0xC1,0xE0,0xF0,0x78,0x3C,0x1E,0x0F,0xFF,0x83,0xC1,0xE0,0xF0,0x78,0x3C,0x18,
// 'I'
0x49,0x03,0x09,0x0D,0x01,0x0B,
0x7F,0x3F,0x82,0x01,0x00,0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x03,0x87,0xF0,
// 'J'
0x4A,0x03,0x08,0x0E,0x01,0x0B,
0x7E,0x7F,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x06,0x6E,0x7C,0x00,
// 'K'
0x4B,0x03,0x0A,0x0D,0x01,0x0B,
0x41,0x90,0xC4,0x61,0x30,0x48,0x1E,0x07,0x01,0x60,0x4C,0x13,0x04,0x61,0x0C,0x41,0x80,
// 'L'
0x4C,0x03,0x08,0x0D,0x02,0x0B,
0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xE0,0xFF,
// 'M'
0x4D,0x03,0x0B,0x0D,0x00,0x0B,
0x60,0xCE,0x39,0xC7,0x38,0xE5,0xB4,0x94,0x93,0x92,0x62,0x44,0x48,0x09,0x01,0x20,0x24,0x04,
// 'N'
0x4E,0x03,0x09,0x0D,0x01,0x0B,
0xC1,0xF0,0xF8,0x7E,0x3D,0x1E,0xCF,0x27,0x9B,0xCD,0xE3,0xF1,0xF8,0x7C,0x38,
// 'O'
0x4F,0x03,0x0B,0x0E,0x00,0x0B,
0x0F,0x07,0xF0,0xC3,0x30,0x66,0x0C,0x80,0x90,0x12,0x02,0x60,0xCC,0x19,0x83,0x1D,0xC1,0xF0,0x00,0x00,
// 'P'
0x50,0x03,0x09,0x0D,0x01,0x0B,
0x7C,0x3F,0x90,0xE8,0x34,0x1A,0x0D,0x0C,0xFC,0x60,0x20,0x10,0x08,0x04,0x00,
// 'Q'
0x51,0x03,0x0B,0x11,0x00,0x0B,
0x0F,0x07,0xF0,0xC7,0x30,0x66,0x0C,0x80,0x90,0x12,0x02,0x60,0xCC,0x19,0x83,0x1D,0xC1,0xF0,0x08,0x01,0x80,0x1F,0x00,0x00,
// 'R'
0x52,0x03,0x0A,0x0D,0x01,0x0B,
0x7C,0x1F,0xC4,0x31,0x0C,0x43,0x11,0xC7,0xE1,0xB0,0x46,0x10,0xC4,0x31,0x06,0x41,0x80,
// 'S'
0x53,0x03,0x09,0x0E,0x01,0x0B,
0x1F,0x3F,0xB0,0x18,0x0E,0x03,0x80,0xF0,0x0E,0x03,0x80,0xC0,0x78,0xEF,0xE0,0x00,
// 'T'
0x54,0x03,0x0B,0x0D,0x00,0x0B,
0x7F,0xCF,0xF8,0x10,0x02,0x00,0x40,0x08,0x01,0x00,0x20,0x04,0x00,0x80,0x10,0x02,0x00,0x40,
// 'U'
0x55,0x03,0x09,0x0E,0x01,0x0B,
0xC1,0xE0,0xF0,0x78,0x3C,0x1E,0x0F,0x07,0x83,0xC1,0xE0,0xF0,0x6C,0x63,0xE0,0x00,
// 'V'
0x56,0x03,0x0B,0x0D,0x00,0x0B,
0xC0,0x78,0x0D,0x83,0x30,0x66,0x0C,0x63,0x0C,0x61,0x88,0x1B,0x03,0x60,0x68,0x07,0x00,0xE0,
// 'W'
0x57,0x03,0x0B,0x0D,0x00,0x0B,
0x40,0x48,0x09,0x01,0x20,0x24,0x04,0x88,0x93,0x92,0x72,0x6A,0x4F,0x69,0xC7,0x38,0xE7,0x1C,
// 'X'
0x58,0x03,0x0B,0x0D,0x00,0x0B,
0x60,0xCC,0x18,0xC6,0x0D,0x81,0xB0,0x1C,0x03,0x80,0x70,0x1B,0x06,0x70,0xC6,0x30,0x6E,0x0C,
// 'Y'
0x59,0x03,0x0B,0x0D,0x00,0x0B,
0xC0,0x6C,0x19,0x83,0x18,0xC3,0x18,0x36,0x03,0x80,0x70,0x04,0x00,0x80,0x10,0x02,0x00,0x40,
// 'Z'
0x5A,0x03,0x09,0x0D,0x01,0x0B,
0xFF,0xFF,0xC0,0xC0,0x60,0x60,0x60,0x30,0x30,0x30,0x18,0x18,0x1F,0xFF,0xF8,
// '['
0x5B,0x01,0x06,0x13,0x03,0x0B,
0x03,0xEC,0x30,0xC3,0x0C,0x30,0xC3,0x0C,0x30,0xC3,0x0C,0x30,0xC3,0x0F,0x80,
// '\'
0x5C,0x02,0x09,0x11,0x01,0x0B,
0x40,0x30,0x08,0x06,0x03,0x00,0x80,0x60,0x10,0x0C,0x06,0x01,0x80,0xC0,0x20,0x18,0x04,0x03,0x00,0x00,
// ']'
0x5D,0x01,0x06,0x13,0x02,0x0B,
0x01,0xF0,0xC3,0x0C,0x30,0xC3,0x0C,0x30,0xC3,0x0C,0x30,0xC3,0x0C,0x37,0xC0,
// '^'
0x5E,0x03,0x09,0x07,0x01,0x0B,
0x08,0x0E,0x0D,0x84,0xC6,0x32,0x08,0x00,
// '_'
0x5F,0x12,0x0B,0x02,0x00,0x0B,
0x00,0x1F,0xFC,
// '`'
0x60,0x02,0x07,0x05,0x00,0x0B,
0x30,0x30,0x10,0x00,0x00,
// 'a'
0x61,0x06,0x09,0x0B,0x01,0x0B,
0x3E,0x3F,0x80,0xC0,0x21,0xF3,0xFB,0x05,0x86,0x67,0x3E,0x80,0x00,
// 'b'
0x62,0x02,0x09,0x0F,0x01,0x0B,
0x40,0x20,0x10,0x08,0x04,0xE3,0xF9,0x86,0x83,0x41,0xA0,0xD0,0x68,0x26,0x73,0xF0,0x00,
// 'c'
0x63,0x06,0x09,0x0B,0x01,0x0B,
0x1F,0x1F,0x98,0x0C,0x04,0x02,0x01,0x80,0xC0,0x31,0x0F,0x80,0x00,
// 'd'
0x64,0x02,0x09,0x0F,0x01,0x0B,
0x01,0x00,0x80,0x40,0x21,0xF3,0xF9,0x85,0x82,0xC1,0x60,0xB0,0x58,0x66,0x71,0xC8,0x00,
// 'e'
0x65,0x06,0x09,0x0B,0x01,0x0B,
0x1E,0x3B,0x98,0x58,0x3F,0xFF,0xFF,0x00,0x80,0x70,0x1F,0x80,0x00,
// 'f'
0x66,0x02,0x0B,0x0E,0x00,0x0B,
0x03,0xC0,0xF8,0x30,0x06,0x00,0xC0,0x18,0x1F,0xF0,0x60,0x0C,0x01,0x80,0x30,0x06,0x00,0xC0,0x18,0x00,
// 'g'
0x67,0x06,0x0B,0x0F,0x00,0x0B,
0x1F,0xC7,0x78,0x86,0x10,0xC3,0x18,0x7E,0x0F,0x83,0x00,0x3F,0x07,0xF9,0x83,0x30,0x67,0x1C,0x7E,0x00,0x00,
// 'h'
0x68,0x02,0x09,0x0E,0x01,0x0B,
0x40,0x20,0x10,0x08,0x04,0xE3,0xF9,0x84,0x82,0x41,0x20,0x90,0x48,0x24,0x12,0x08,
// 'i'
0x69,0x02,0x09,0x0E,0x01,0x0B,
0x08,0x0E,0x02,0x00,0x07,0xC3,0xE0,0x30,0x18,0x0C,0x06,0x03,0x01,0x80,0xC3,0xFC,
// 'j'
0x6A,0x02,0x08,0x13,0x01,0x0B,
0x06,0x06,0x06,0x00,0x7E,0x7E,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0xCC,0xF8,0x00,
// 'k'
0x6B,0x02,0x0A,0x0E,0x01,0x0B,
0x40,0x10,0x04,0x01,0x00,0x41,0x90,0xC4,0x61,0x30,0x78,0x1E,0x04,0xC1,0x18,0x43,0x10,0x60,
// 'l'
0x6C,0x02,0x09,0x0E,0x01,0x0B,
0x78,0x3E,0x03,0x01,0x80,0xC0,0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x80,0xC3,0xFC,
// 'm'
0x6D,0x06,0x0B,0x0A,0x00,0x0B,
0x59,0x8F,0xF9,0x99,0x32,0x24,0x44,0x88,0x91,0x12,0x22,0x44,0x48,0x88,
// 'n'
0x6E,0x06,0x09,0x0A,0x01,0x0B,
0x4E,0x3F,0x98,0x48,0x24,0x12,0x09,0x04,0x82,0x41,0x20,0x80,
// 'o'
0x6F,0x06,0x0B,0x0B,0x00,0x0B,
0x1F,0x07,0xF1,0x83,0x30,0x66,0x0C,0xC1,0x98,0x33,0x06,0x31,0x83,0xE0,0x00,0x00,
// 'p'
0x70,0x06,0x09,0x0E,0x01,0x0B,
0x4E,0x3F,0x98,0x68,0x34,0x1A,0x0D,0x06,0x82,0x67,0x3F,0x10,0x08,0x04,0x02,0x00,
// 'q'
0x71,0x06,0x09,0x0E,0x01,0x0B,
0x1F,0x3F,0x98,0x58,0x2C,0x16,0x0B,0x05,0x86,0x67,0x1C,0x80,0x40,0x20,0x10,0x08,
// 'r'
0x72,0x06,0x0A,0x0A,0x01,0x0B,
0x4F,0x1F,0xE7,0x19,0x86,0x60,0x18,0x06,0x01,0x80,0x60,0x18,0x00,
// 's'
0x73,0x06,0x09,0x0B,0x01,0x0B,
0x1F,0x39,0x98,0x0C,0x03,0x80,0xF0,0x0C,0x02,0x43,0x3F,0x00,0x00,
// 't'
0x74,0x03,0x0A,0x0E,0x00,0x0B,
0x08,0x06,0x01,0x81,0xFE,0x7F,0x86,0x01,0x80,0x60,0x18,0x06,0x01,0x80,0x30,0x07,0x80,0x00,
// 'u'
0x75,0x06,0x09,0x0B,0x01,0x0B,
0x41,0x20,0x90,0x48,0x24,0x12,0x09,0x04,0x86,0x67,0x1E,0x80,0x00,
// 'v'
0x76,0x06,0x0B,0x0A,0x00,0x0B,
0x60,0xCC,0x18,0x82,0x18,0xC3,0x18,0x36,0x06,0xC0,0x70,0x0E,0x01,0xC0,
// 'w'
0x77,0x06,0x0B,0x0A,0x00,0x0B,
0x40,0x48,0x09,0x01,0x22,0x26,0xEC,0xDD,0x9A,0xB3,0xDE,0x31,0x86,0x30,
// 'x'
0x78,0x06,0x0B,0x0A,0x00,0x0B,
0x60,0xC6,0x30,0x66,0x0D,0x80,0xE0,0x1C,0x07,0xC0,0xD8,0x31,0x8C,0x18,
// 'y'
0x79,0x06,0x0B,0x0F,0x00,0x0B,
0x60,0xCC,0x18,0x82,0x18,0xC3,0x18,0x32,0x06,0xC0,0x70,0x0E,0x01,0xC0,0x30,0x06,0x03,0x80,0xE0,0x00,0x00,
// 'z'
0x7A,0x06,0x09,0x0A,0x01,0x0B,
0x7F,0x3F,0x81,0x80,0xC0,0xC0,0xC0,0xC0,0x60,0x60,0x3F,0xC0,
// '{'
0x7B,0x01,0x08,0x13,0x01,0x0B,
0x00,0x07,0x0C,0x18,0x18,0x18,0x18,0x18,0x30,0x70,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x0C,0x07,
// '|'
0x7C,0x00,0x03,0x14,0x04,0x0B,
0x49,0x24,0x92,0x49,0x24,0x92,0x49,0x20,
// '}'
0x7D,0x01,0x08,0x13,0x02,0x0B,
0x00,0xE0,0x30,0x18,0x18,0x18,0x18,0x18,0x0C,0x0E,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x30,0xE0,
// '~'
0x7E,0x08,0x0B,0x05,0x00,0x0B,
0x00,0x07,0x09,0xB9,0x23,0xE0,0x10,

// Terminator
0xFF
};