//This data structure was designed to support Proportional fonts
//on Arduinos. It can however handle any ttf font that has been converted
//using the conversion program. These could be fixed width or proportional
//fonts. Individual characters do not have to be multiples of 8 bits wide.
//Any width is fine and does not need to be fixed.

//The data bits are packed to minimize data requirements, but the tradeoff
//is that a header is required per character.
// c:// Header Format (to make Arduino UTFT Compatible)
// ------------------------------------------------
// Character Width (Used as a marker to indicate use this format. i.e.: = 0x00)
// Character Height
// First Character (Reserved. 0x00)
// Number Of Characters (Reserved. 0x00)

const unsigned char tft_Bahn_Cond_24[] =
{
0x00,0x18,0x00,0x00,
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
0x20,0x00,0x00,0x00,0x00,0x06,

// '-' 
0x2d,0x0d,0x08,0x03,0x01,0x0b,
0xff,0xff,0xff,
// '.' 
0x2e,0x14,0x03,0x03,0x01,0x05,
0xff,0x80,
// '0' 
0x30,0x00,0x0a,0x17,0x01,0x0d,
0x3e,0x1f,0xef,0xfb,0x8f,0xe1,0xf8,0x7e,0x1f,0x87,0xe1,0xf8,0x7e,0x1f,0x87,0xe1,0xf8,0x7e,0x1f,0x87,0xe1,0xf8,0x7e,0x1f,0x8f,0xff,0x9f,0xe3,0xe0,
// '1' 
0x31,0x00,0x06,0x17,0x00,0x09,
0x3d,0xff,0xf7,0x9c,0x71,0xc7,0x1c,0x71,0xc7,0x1c,0x71,0xc7,0x1c,0x71,0xc7,0x1c,0x71,0xc0,
// '2' 
0x32,0x00,0x0b,0x17,0x01,0x0d,
0x1f,0x07,0xf1,0xff,0x38,0xee,0x1f,0xc1,0xc0,0x70,0x0e,0x01,0xc0,0x78,0x0e,0x03,0xc0,0x70,0x1e,0x03,0x80,0xf0,0x1c,0x07,0x01,0xe0,0x38,0x0f,0xff,0xff,0xff,0xf8,
// '3' 
0x33,0x00,0x0a,0x17,0x01,0x0d,
0x1f,0x1f,0xe7,0xff,0xc7,0xe1,0xf8,0x7e,0x1c,0x07,0x01,0xc3,0xe0,0xf0,0x3e,0x01,0xc0,0x70,0x1c,0x07,0xe1,0xf8,0x7e,0x1f,0xc7,0xff,0xdf,0xe1,0xf0,
// '4' 
0x34,0x00,0x0c,0x17,0x01,0x0e,
0x07,0x00,0x70,0x0f,0x00,0xe0,0x0e,0x00,0xe0,0x1c,0x01,0xc0,0x1c,0x03,0x9c,0x39,0xc3,0x9c,0x79,0xc7,0x1c,0x71,0xc7,0x1c,0xff,0xff,0xff,0xff,0xf0,0x1c,0x01,0xc0,0x1c,0x01,0xc0,
// '5' 
0x35,0x00,0x09,0x17,0x01,0x0d,
0xff,0xff,0xff,0xfc,0x0e,0x07,0x03,0x81,0xde,0xff,0xff,0xf8,0xfc,0x70,0x18,0x0c,0x06,0x03,0x01,0xf0,0xf8,0xfc,0x7f,0xfb,0xf8,0xf8,
// '6' 
0x36,0x00,0x0a,0x17,0x01,0x0c,
0x07,0x01,0xc0,0xe0,0x38,0x1e,0x07,0x01,0xc0,0xe0,0x38,0x1f,0xc7,0xf9,0xff,0xf1,0xf8,0x7e,0x1f,0x87,0xe1,0xf8,0x7e,0x1f,0xc7,0x7f,0xdf,0xe1,0xf0,
// '7' 
0x37,0x00,0x0a,0x17,0x01,0x0c,
0xff,0xff,0xff,0xff,0x87,0xe1,0xf8,0xe0,0x38,0x0e,0x03,0x81,0xc0,0x70,0x1c,0x07,0x03,0x80,0xe0,0x38,0x1e,0x07,0x01,0xc0,0x70,0x3c,0x0e,0x03,0x80,
// '8' 
0x38,0x00,0x0b,0x17,0x01,0x0d,
0x1f,0x07,0xf1,0xff,0x38,0xe7,0x1c,0xe3,0x9c,0x73,0x8e,0x71,0xc7,0xf0,0x7c,0x3f,0xe7,0x1c,0xc1,0xf8,0x3f,0x07,0xe0,0xfc,0x1d,0xc3,0xb8,0xf7,0xfc,0x7f,0x07,0xc0,
// '9' 
0x39,0x00,0x0a,0x17,0x01,0x0c,
0x1f,0x1f,0xe7,0xff,0xc7,0xe1,0xf8,0x7e,0x1f,0x87,0xe1,0xf8,0x7f,0x1d,0xff,0x7f,0x8f,0xe0,0x38,0x1c,0x07,0x03,0x80,0xe0,0x70,0x1c,0x0f,0x03,0x80,
// ':' 
0x3a,0x07,0x03,0x10,0x01,0x05,
0xff,0x80,0x00,0x00,0x01,0xff,
// 'A' 
0x41,0x00,0x0d,0x17,0x00,0x0e,
0x07,0x00,0x78,0x03,0xc0,0x1e,0x00,0xf0,0x0f,0xc0,0x7e,0x03,0xb0,0x19,0x80,0xce,0x0e,0x70,0x73,0x83,0x8c,0x18,0x71,0xc3,0x8f,0xfc,0x7f,0xe3,0xff,0x38,0x1d,0xc0,0xee,0x07,0x70,0x3b,0x81,0xe0,
// 'B' 
0x42,0x00,0x0a,0x17,0x01,0x0d,
0xff,0x3f,0xef,0xff,0x87,0xe1,0xf8,0x7e,0x1f,0x87,0xe3,0xff,0xef,0xe3,0xfe,0xe3,0xf8,0x7e,0x1f,0x87,0xe1,0xf8,0x7e,0x1f,0x87,0xff,0xff,0xef,0xf0,
// 'C' 
0x43,0x00,0x0a,0x17,0x01,0x0d,
0x3e,0x1f,0xef,0xfb,0x8f,0xe1,0xf8,0x7e,0x1f,0x80,0xe0,0x38,0x0e,0x03,0x80,0xe0,0x38,0x0e,0x03,0x80,0xe1,0xf8,0x7e,0x1f,0x8f,0xff,0x9f,0xe3,0xe0,
// 'D' 
0x44,0x00,0x0a,0x17,0x01,0x0e,
0xff,0x3f,0xef,0xff,0x87,0xe1,0xf8,0x7e,0x1f,0x87,0xe1,0xf8,0x7e,0x1f,0x87,0xe1,0xf8,0x7e,0x1f,0x87,0xe1,0xf8,0x7e,0x1f,0x87,0xff,0xff,0xef,0xf0,
// 'E' 
0x45,0x00,0x09,0x17,0x01,0x0c,
0xff,0xff,0xff,0xfc,0x0e,0x07,0x03,0x81,0xc0,0xe0,0x70,0x3f,0xff,0xff,0xff,0x03,0x81,0xc0,0xe0,0x70,0x38,0x1c,0x0f,0xff,0xff,0xfe,
// 'F' 
0x46,0x00,0x09,0x17,0x01,0x0c,
0xff,0xff,0xff,0xfc,0x0e,0x07,0x03,0x81,0xc0,0xe0,0x70,0x3f,0xff,0xff,0xff,0x03,0x81,0xc0,0xe0,0x70,0x38,0x1c,0x0e,0x07,0x03,0x80,
// 'G' 
0x47,0x00,0x0b,0x17,0x01,0x0e,
0x3f,0x0f,0xf3,0xff,0x71,0xee,0x1d,0xc3,0xb8,0x77,0x00,0xe0,0x1c,0x03,0x9f,0xf3,0xfe,0x7f,0xc1,0xf8,0x3f,0x07,0xe0,0xfc,0x1f,0x87,0x70,0xef,0xfc,0xff,0x0f,0xc0,
// 'H' 
0x48,0x00,0x0a,0x17,0x01,0x0e,
0xe1,0xf8,0x7e,0x1f,0x87,0xe1,0xf8,0x7e,0x1f,0x87,0xe1,0xf8,0x7f,0xff,0xff,0xff,0xf8,0x7e,0x1f,0x87,0xe1,0xf8,0x7e,0x1f,0x87,0xe1,0xf8,0x7e,0x1c,
// 'I' 
0x49,0x00,0x03,0x17,0x01,0x06,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf8,
// 'J' 
0x4a,0x00,0x0a,0x17,0x00,0x0b,
0x01,0xc0,0x70,0x1c,0x07,0x01,0xc0,0x70,0x1c,0x07,0x01,0xc0,0x70,0x1c,0x07,0x01,0xc0,0x70,0x1c,0x07,0x01,0xc0,0x70,0x1d,0x8f,0xff,0x9f,0xe3,0xe0,
// 'K' 
0x4b,0x00,0x0c,0x17,0x01,0x0e,
0xe0,0xee,0x1c,0xe1,0xce,0x38,0xe3,0x8e,0x70,0xe7,0x0e,0xe0,0xee,0x0f,0xe0,0xfe,0x0f,0xe0,0xff,0x0f,0x70,0xf7,0x8e,0x38,0xe3,0x8e,0x1c,0xe1,0xce,0x1c,0xe0,0xee,0x0e,0xe0,0xf0,
// 'L' 
0x4c,0x00,0x09,0x17,0x01,0x0c,
0xe0,0x70,0x38,0x1c,0x0e,0x07,0x03,0x81,0xc0,0xe0,0x70,0x38,0x1c,0x0e,0x07,0x03,0x81,0xc0,0xe0,0x70,0x38,0x1c,0x0f,0xff,0xff,0xfe,
// 'M' 
0x4d,0x00,0x0e,0x17,0x01,0x12,
0xc0,0x1f,0x80,0x7e,0x01,0xf8,0x0f,0xf0,0x3f,0xc0,0xff,0x07,0xfe,0x1f,0xd8,0x7f,0x63,0xfd,0xcf,0xf7,0x37,0xcc,0xdf,0x3f,0x7c,0xf9,0xf1,0xe7,0xc7,0x9f,0x1c,0x7c,0x31,0xf0,0x07,0xc0,0x1f,0x00,0x7c,0x01,0xc0,
// 'N' 
0x4e,0x00,0x0b,0x17,0x01,0x0e,
0xc0,0xfc,0x1f,0x83,0xf0,0x7f,0x0f,0xe1,0xfc,0x3f,0xc7,0xd8,0xfb,0x9f,0x33,0xe6,0x7c,0xef,0x8d,0xf1,0xbe,0x3f,0xc3,0xf8,0x7f,0x0f,0xe0,0xfc,0x1f,0x81,0xf0,0x38,
// 'O' 
0x4f,0x00,0x0b,0x17,0x01,0x0e,
0x3f,0x0f,0xf3,0xff,0x70,0xfe,0x0f,0xc1,0xf8,0x3f,0x07,0xe0,0xfc,0x1f,0x83,0xf0,0x7e,0x0f,0xc1,0xf8,0x3f,0x07,0xe0,0xfc,0x1f,0x83,0xf0,0xff,0xfc,0xff,0x07,0xc0,
// 'P' 
0x50,0x00,0x0b,0x17,0x01,0x0e,
0xff,0x1f,0xf3,0xff,0x70,0xee,0x1d,0xc3,0xf8,0x7f,0x0f,0xe1,0xfc,0x3b,0x87,0x7f,0xef,0xf9,0xfe,0x38,0x07,0x00,0xe0,0x1c,0x03,0x80,0x70,0x0e,0x01,0xc0,0x38,0x00,
// 'Q' 
0x51,0x00,0x0d,0x18,0x01,0x10,
0x3f,0x03,0xfc,0x3f,0xf1,0xc3,0xce,0x0e,0x70,0x73,0x83,0x9c,0x1c,0xe0,0xe7,0x07,0x38,0x39,0xc1,0xce,0x0e,0x70,0x73,0x83,0x9c,0x1c,0xe0,0xe7,0x17,0x39,0xf9,0xc7,0xcf,0xff,0x3f,0xfc,0x7c,0xc0,0x02,
// 'R' 
0x52,0x00,0x0b,0x17,0x01,0x0e,
0xff,0x1f,0xf3,0xff,0x70,0xee,0x0f,0xc1,0xf8,0x3f,0x07,0xe0,0xfc,0x3b,0xff,0x7f,0xcf,0xf1,0xce,0x38,0xe7,0x1c,0xe3,0x9c,0x7b,0x87,0x70,0xee,0x1d,0xc1,0xf8,0x38,
// 'S' 
0x53,0x00,0x0b,0x17,0x01,0x0e,
0x1f,0x87,0xf9,0xff,0xb8,0xfe,0x0f,0xc1,0xf8,0x3f,0x80,0x70,0x0f,0x80,0xfc,0x0f,0xe0,0x7e,0x03,0xc0,0x38,0x07,0x00,0xfc,0x1f,0x83,0xb8,0x77,0xfe,0x7f,0x87,0xe0,
// 'T' 
0x54,0x00,0x0b,0x17,0x00,0x0b,
0xff,0xff,0xff,0xff,0x87,0x00,0xe0,0x1c,0x03,0x80,0x70,0x0e,0x01,0xc0,0x38,0x07,0x00,0xe0,0x1c,0x03,0x80,0x70,0x0e,0x01,0xc0,0x38,0x07,0x00,0xe0,0x1c,0x03,0x80,
// 'U' 
0x55,0x00,0x0b,0x17,0x01,0x0d,
0xe0,0xfc,0x1f,0x83,0xf0,0x7e,0x0f,0xc1,0xf8,0x3f,0x07,0xe0,0xfc,0x1f,0x83,0xf0,0x7e,0x0f,0xc1,0xf8,0x3f,0x07,0xe0,0xfc,0x1f,0x83,0xb8,0xf7,0xfc,0x7f,0x87,0xc0,
// 'V' 
0x56,0x00,0x0c,0x17,0x00,0x0d,
0xe0,0x7f,0x07,0x70,0x77,0x07,0x70,0x67,0x0e,0x70,0xe3,0x8e,0x38,0xe3,0x8c,0x39,0xc3,0x9c,0x19,0xc1,0xdc,0x1d,0x81,0xd8,0x1f,0x80,0xf8,0x0f,0x80,0xf8,0x0f,0x00,0xf0,0x07,0x00,
// 'W' 
0x57,0x00,0x12,0x17,0x00,0x12,
0xe0,0xc1,0xf8,0x30,0x7e,0x0c,0x1d,0xc7,0x86,0x71,0xe3,0x9c,0x78,0xe7,0x1e,0x39,0xc7,0x8e,0x71,0xe3,0x9c,0xfc,0xe3,0x3b,0x30,0xcc,0xcc,0x3b,0x33,0x0e,0xcd,0xc3,0xb3,0x70,0xfc,0xfc,0x3f,0x1f,0x07,0x87,0x81,0xe1,0xe0,0x78,0x78,0x1e,0x1e,0x07,0x87,0x81,0xe1,0xe0,
// 'X' 
0x58,0x00,0x0c,0x17,0x00,0x0d,
0xf0,0x77,0x0f,0x70,0xe3,0x8e,0x39,0xc3,0x9c,0x1d,0xc1,0xf8,0x0f,0x80,0xf0,0x0f,0x00,0x70,0x0f,0x00,0xf8,0x0f,0x81,0xd8,0x1d,0xc3,0x9c,0x38,0xe7,0x0e,0x70,0xe7,0x07,0xe0,0x70,
// 'Y' 
0x59,0x00,0x0b,0x17,0x00,0x0d,
0xe0,0xfc,0x1d,0x83,0xb8,0x67,0x1c,0x63,0x8e,0x61,0xdc,0x1b,0x83,0xe0,0x7c,0x07,0x80,0xe0,0x1c,0x03,0x80,0x70,0x0e,0x01,0xc0,0x38,0x07,0x00,0xe0,0x1c,0x03,0x80,
// 'Z' 
0x5a,0x00,0x0a,0x17,0x01,0x0c,
0x7f,0xdf,0xf7,0xfc,0x0e,0x03,0x80,0xc0,0x70,0x1c,0x0e,0x03,0x80,0xe0,0x70,0x1c,0x07,0x03,0x80,0xe0,0x38,0x1c,0x07,0x01,0xc0,0xff,0xff,0xff,0xfc,
// '�' 
0xb0,0x00,0x07,0x09,0x00,0x09,
0x7d,0xfb,0x3e,0x3c,0x78,0xf3,0xfe,0x7c,
// Terminator
0xFF
};
