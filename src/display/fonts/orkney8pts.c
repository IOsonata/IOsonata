/* Font copyrights:
 *
 * Copyright (c) 2015,
 * Alfredo Marco Pradil (https://behance.net/pradil),
 * Samuel Oakes (http://oakes.co/),
 * Cristiano Sobral (https://www.behance.net/cssobral20f492),
 * with Reserved Font Name Orkney.
 *
 * Code was generated by The Dot Factory (https://github.com/pavius/the-dot-factory)
 *
 * This Font Software is licensed under the SIL Open Font License, Version 1.1. See SIL-License.txt
 * for more informations.
 *
 */

#include "display/ifont.h"

// Character bitmaps for Orkney 8pt
static const uint8_t orkney_8ptBitmaps[] =
{
    // @0 '!' (2 pixels wide)
    0x00, //
    0x00, //
    0xC0, // ##
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x00, //
    0x00, //
    0xC0, // ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @16 '"' (3 pixels wide)
    0x00, //
    0x00, //
    0xA0, // # #
    0xA0, // # #
    0xA0, // # #
    0xA0, // # #
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @32 '#' (8 pixels wide)
    0x00, //
    0x00, //
    0x22, //   #   #
    0x22, //   #   #
    0x22, //   #   #
    0xFF, // ########
    0x24, //   #  #
    0x24, //   #  #
    0x24, //   #  #
    0xFF, // ########
    0x44, //  #   #
    0x44, //  #   #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @48 '$' (6 pixels wide)
    0x10, //    #
    0x38, //   ###
    0x44, //  #   #
    0x80, // #
    0xC0, // ##
    0x60, //  ##
    0x38, //   ###
    0x0C, //     ##
    0x04, //      #
    0x84, // #    #
    0xC4, // ##   #
    0x78, //  ####
    0x10, //    #
    0x00, //
    0x00, //
    0x00, //

    // @64 '%' (8 pixels wide)
    0x00, //
    0x00, //
    0xE3, // ###   ##
    0xA2, // # #   #
    0xA4, // # #  #
    0xE8, // ### #
    0x08, //     #
    0x10, //    #
    0x26, //   #  ##
    0x29, //   # #  #
    0x49, //  #  #  #
    0x86, // #    ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @80 '&' (8 pixels wide)
    0x00, //
    0x00, //
    0x38, //   ###
    0x48, //  #  #
    0x40, //  #
    0x40, //  #
    0x20, //   #
    0x50, //  # #
    0x89, // #   #  #
    0x86, // #    ##
    0x46, //  #   ##
    0x39, //   ###  #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @96 ''' (1 pixels wide)
    0x00, //
    0x00, //
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @112 '(' (3 pixels wide)
    0x20, //   #
    0x20, //   #
    0x40, //  #
    0x40, //  #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x40, //  #
    0x40, //  #
    0x20, //   #

    // @128 ')' (3 pixels wide)
    0x80, // #
    0xC0, // ##
    0x40, //  #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x40, //  #
    0x80, // #

    // @144 '*' (3 pixels wide)
    0x00, //
    0x00, //
    0xC0, // ##
    0xE0, // ###
    0xC0, // ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @160 '+' (8 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x08, //     #
    0x08, //     #
    0x08, //     #
    0x08, //     #
    0xFF, // ########
    0x08, //     #
    0x08, //     #
    0x08, //     #
    0x08, //     #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @176 ',' (1 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x00, //

    // @192 '-' (5 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xF8, // #####
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @208 '.' (2 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xC0, // ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @224 '/' (8 pixels wide)
    0x00, //
    0x00, //
    0x03, //       ##
    0x02, //       #
    0x04, //      #
    0x04, //      #
    0x08, //     #
    0x08, //     #
    0x10, //    #
    0x10, //    #
    0x20, //   #
    0x20, //   #
    0x40, //  #
    0x40, //  #
    0x80, // #
    0x00, //

    // @240 '0' (7 pixels wide)
    0x00, //
    0x00, //
    0x38, //   ###
    0x44, //  #   #
    0xC6, // ##   ##
    0x82, // #     #
    0x82, // #     #
    0x82, // #     #
    0x82, // #     #
    0xC6, // ##   ##
    0x44, //  #   #
    0x38, //   ###
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @256 '1' (4 pixels wide)
    0x00, //
    0x00, //
    0x30, //   ##
    0x70, //  ###
    0xB0, // # ##
    0x30, //   ##
    0x30, //   ##
    0x30, //   ##
    0x30, //   ##
    0x30, //   ##
    0x30, //   ##
    0x30, //   ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @272 '2' (6 pixels wide)
    0x00, //
    0x00, //
    0x78, //  ####
    0xC8, // ##  #
    0x84, // #    #
    0x04, //      #
    0x08, //     #
    0x10, //    #
    0x30, //   ##
    0x60, //  ##
    0xC0, // ##
    0xFC, // ######
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @288 '3' (6 pixels wide)
    0x00, //
    0x00, //
    0x70, //  ###
    0xC8, // ##  #
    0x08, //     #
    0x08, //     #
    0x30, //   ##
    0x08, //     #
    0x04, //      #
    0x84, // #    #
    0xC8, // ##  #
    0x78, //  ####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @304 '4' (7 pixels wide)
    0x00, //
    0x00, //
    0x0C, //     ##
    0x0C, //     ##
    0x14, //    # #
    0x14, //    # #
    0x24, //   #  #
    0x44, //  #   #
    0x44, //  #   #
    0xFE, // #######
    0x04, //      #
    0x04, //      #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @320 '5' (6 pixels wide)
    0x00, //
    0x00, //
    0x7C, //  #####
    0x80, // #
    0x80, // #
    0xF8, // #####
    0x8C, // #   ##
    0x04, //      #
    0x04, //      #
    0x04, //      #
    0x8C, // #   ##
    0x78, //  ####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @336 '6' (7 pixels wide)
    0x00, //
    0x00, //
    0x08, //     #
    0x10, //    #
    0x20, //   #
    0x78, //  ####
    0x44, //  #   #
    0x86, // #    ##
    0x82, // #     #
    0x86, // #    ##
    0xC4, // ##   #
    0x78, //  ####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @352 '7' (7 pixels wide)
    0x00, //
    0x00, //
    0xFE, // #######
    0x06, //      ##
    0x04, //      #
    0x0C, //     ##
    0x08, //     #
    0x10, //    #
    0x10, //    #
    0x20, //   #
    0x20, //   #
    0x40, //  #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @368 '8' (6 pixels wide)
    0x00, //
    0x00, //
    0x70, //  ###
    0xC8, // ##  #
    0x88, // #   #
    0xC8, // ##  #
    0x70, //  ###
    0x88, // #   #
    0x84, // #    #
    0x84, // #    #
    0x88, // #   #
    0x78, //  ####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @384 '9' (7 pixels wide)
    0x00, //
    0x00, //
    0x78, //  ####
    0xC4, // ##   #
    0x86, // #    ##
    0x82, // #     #
    0x84, // #    #
    0xC4, // ##   #
    0x78, //  ####
    0x10, //    #
    0x30, //   ##
    0x60, //  ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @400 ':' (2 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xC0, // ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xC0, // ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @416 ';' (2 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xC0, // ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xC0, // ##
    0x40, //  #
    0x40, //  #
    0x80, // #
    0x00, //

    // @432 '<' (7 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x02, //       #
    0x0C, //     ##
    0x30, //   ##
    0x60, //  ##
    0xC0, // ##
    0x60, //  ##
    0x18, //    ##
    0x06, //      ##
    0x02, //       #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @448 '=' (8 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xFF, // ########
    0x00, //
    0x00, //
    0xFF, // ########
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @464 '>' (7 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x80, // #
    0x60, //  ##
    0x18, //    ##
    0x06, //      ##
    0x02, //       #
    0x0C, //     ##
    0x30, //   ##
    0xC0, // ##
    0x80, // #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @480 '?' (5 pixels wide)
    0x00, //
    0x00, //
    0x70, //  ###
    0x88, // #   #
    0x88, // #   #
    0x08, //     #
    0x18, //    ##
    0x30, //   ##
    0x20, //   #
    0x20, //   #
    0x00, //
    0x20, //   #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @496 '@' (6 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x78, //  ####
    0x48, //  #  #
    0x9C, // #  ###
    0xA4, // # #  #
    0xA4, // # #  #
    0x9C, // #  ###
    0x40, //  #
    0x3C, //   ####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @512 'A' (9 pixels wide)
    0x00, 0x00, //
    0x00, 0x00, //
    0x0C, 0x00, //     ##
    0x0C, 0x00, //     ##
    0x14, 0x00, //    # #
    0x12, 0x00, //    #  #
    0x32, 0x00, //   ##  #
    0x23, 0x00, //   #   ##
    0x21, 0x00, //   #    #
    0x7F, 0x00, //  #######
    0x40, 0x80, //  #      #
    0xC0, 0x80, // ##      #
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //

    // @544 'B' (7 pixels wide)
    0x00, //
    0x00, //
    0xF8, // #####
    0x8C, // #   ##
    0x84, // #    #
    0x8C, // #   ##
    0xF8, // #####
    0x84, // #    #
    0x82, // #     #
    0x82, // #     #
    0x86, // #    ##
    0xFC, // ######
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @560 'C' (7 pixels wide)
    0x00, //
    0x00, //
    0x3E, //   #####
    0x62, //  ##   #
    0xC0, // ##
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0xC0, // ##
    0x62, //  ##   #
    0x3E, //   #####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @576 'D' (8 pixels wide)
    0x00, //
    0x00, //
    0xF8, // #####
    0x84, // #    #
    0x82, // #     #
    0x82, // #     #
    0x83, // #     ##
    0x83, // #     ##
    0x82, // #     #
    0x82, // #     #
    0x84, // #    #
    0xF8, // #####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @592 'E' (5 pixels wide)
    0x00, //
    0x00, //
    0xF8, // #####
    0x80, // #
    0x80, // #
    0x80, // #
    0xF8, // #####
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0xF8, // #####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @608 'F' (5 pixels wide)
    0x00, //
    0x00, //
    0xF8, // #####
    0x80, // #
    0x80, // #
    0x80, // #
    0xF8, // #####
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @624 'G' (8 pixels wide)
    0x00, //
    0x00, //
    0x1E, //    ####
    0x61, //  ##    #
    0x40, //  #
    0x80, // #
    0x8F, // #   ####
    0x81, // #      #
    0x81, // #      #
    0x41, //  #     #
    0x61, //  ##    #
    0x1E, //    ####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @640 'H' (8 pixels wide)
    0x00, //
    0x00, //
    0x83, // #     ##
    0x83, // #     ##
    0x83, // #     ##
    0x83, // #     ##
    0xFF, // ########
    0x83, // #     ##
    0x83, // #     ##
    0x83, // #     ##
    0x83, // #     ##
    0x83, // #     ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @656 'I' (1 pixels wide)
    0x00, //
    0x00, //
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @672 'J' (3 pixels wide)
    0x00, //
    0x00, //
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0xC0, // ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @688 'K' (7 pixels wide)
    0x00, //
    0x00, //
    0x86, // #    ##
    0x88, // #   #
    0x90, // #  #
    0xA0, // # #
    0xC0, // ##
    0xA0, // # #
    0x90, // #  #
    0x88, // #   #
    0x84, // #    #
    0x82, // #     #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @704 'L' (5 pixels wide)
    0x00, //
    0x00, //
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0xF8, // #####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @720 'M' (10 pixels wide)
    0x00, 0x00, //
    0x00, 0x00, //
    0xC0, 0xC0, // ##      ##
    0xC0, 0xC0, // ##      ##
    0xA1, 0x40, // # #    # #
    0xB3, 0x40, // # ##  ## #
    0x92, 0x40, // #  #  #  #
    0x8C, 0x40, // #   ##   #
    0x8C, 0x40, // #   ##   #
    0x88, 0x40, // #   #    #
    0x80, 0x40, // #        #
    0x80, 0x40, // #        #
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //

    // @752 'N' (8 pixels wide)
    0x00, //
    0x00, //
    0x81, // #      #
    0xC1, // ##     #
    0xA1, // # #    #
    0xB1, // # ##   #
    0x91, // #  #   #
    0x89, // #   #  #
    0x8D, // #   ## #
    0x87, // #    ###
    0x83, // #     ##
    0x81, // #      #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @768 'O' (9 pixels wide)
    0x00, 0x00, //
    0x00, 0x00, //
    0x3E, 0x00, //   #####
    0x63, 0x00, //  ##   ##
    0xC1, 0x80, // ##     ##
    0x80, 0x80, // #       #
    0x80, 0x80, // #       #
    0x80, 0x80, // #       #
    0x80, 0x80, // #       #
    0xC1, 0x80, // ##     ##
    0x63, 0x00, //  ##   ##
    0x3E, 0x00, //   #####
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //

    // @800 'P' (6 pixels wide)
    0x00, //
    0x00, //
    0xF8, // #####
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0xF8, // #####
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @816 'Q' (10 pixels wide)
    0x00, 0x00, //
    0x00, 0x00, //
    0x3E, 0x00, //   #####
    0x63, 0x00, //  ##   ##
    0xC1, 0x80, // ##     ##
    0x80, 0x80, // #       #
    0x80, 0x80, // #       #
    0x80, 0x80, // #       #
    0x84, 0x80, // #    #  #
    0xC3, 0x80, // ##    ###
    0x63, 0x00, //  ##   ##
    0x3E, 0xC0, //   ##### ##
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //

    // @848 'R' (7 pixels wide)
    0x00, //
    0x00, //
    0xF8, // #####
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0xFC, // ######
    0x90, // #  #
    0x88, // #   #
    0x88, // #   #
    0x84, // #    #
    0x86, // #    ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @864 'S' (7 pixels wide)
    0x00, //
    0x00, //
    0x78, //  ####
    0xC4, // ##   #
    0x80, // #
    0xC0, // ##
    0x70, //  ###
    0x0C, //     ##
    0x04, //      #
    0x86, // #    ##
    0x84, // #    #
    0x78, //  ####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @880 'T' (7 pixels wide)
    0x00, //
    0x00, //
    0xFE, // #######
    0x10, //    #
    0x10, //    #
    0x10, //    #
    0x10, //    #
    0x10, //    #
    0x10, //    #
    0x10, //    #
    0x10, //    #
    0x10, //    #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @896 'U' (7 pixels wide)
    0x00, //
    0x00, //
    0x82, // #     #
    0x82, // #     #
    0x82, // #     #
    0x82, // #     #
    0x82, // #     #
    0x82, // #     #
    0x82, // #     #
    0x82, // #     #
    0x46, //  #   ##
    0x3C, //   ####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @912 'V' (9 pixels wide)
    0x00, 0x00, //
    0x00, 0x00, //
    0xC0, 0x80, // ##      #
    0x41, 0x80, //  #     ##
    0x41, 0x00, //  #     #
    0x61, 0x00, //  ##    #
    0x22, 0x00, //   #   #
    0x22, 0x00, //   #   #
    0x16, 0x00, //    # ##
    0x14, 0x00, //    # #
    0x1C, 0x00, //    ###
    0x0C, 0x00, //     ##
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //

    // @944 'W' (14 pixels wide)
    0x00, 0x00, //
    0x00, 0x00, //
    0x84, 0x04, // #    #       #
    0xC6, 0x0C, // ##   ##     ##
    0x42, 0x08, //  #    #     #
    0x62, 0x18, //  ##   #    ##
    0x23, 0x10, //   #   ##   #
    0x23, 0x10, //   #   ##   #
    0x13, 0xB0, //    #  ### ##
    0x14, 0xA0, //    # #  # #
    0x1C, 0xE0, //    ###  ###
    0x08, 0x40, //     #    #
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //

    // @976 'X' (8 pixels wide)
    0x00, //
    0x00, //
    0xC3, // ##    ##
    0x62, //  ##   #
    0x24, //   #  #
    0x1C, //    ###
    0x18, //    ##
    0x18, //    ##
    0x14, //    # #
    0x26, //   #  ##
    0x62, //  ##   #
    0xC1, // ##     #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @992 'Y' (8 pixels wide)
    0x00, //
    0x00, //
    0xC1, // ##     #
    0x62, //  ##   #
    0x26, //   #  ##
    0x14, //    # #
    0x18, //    ##
    0x08, //     #
    0x08, //     #
    0x08, //     #
    0x08, //     #
    0x08, //     #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1008 'Z' (7 pixels wide)
    0x00, //
    0x00, //
    0xFE, // #######
    0x04, //      #
    0x0C, //     ##
    0x08, //     #
    0x10, //    #
    0x30, //   ##
    0x20, //   #
    0x40, //  #
    0xC0, // ##
    0xFE, // #######
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1024 '[' (3 pixels wide)
    0xE0, // ###
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0xE0, // ###

    // @1040 '\' (8 pixels wide)
    0x00, //
    0x00, //
    0x80, // #
    0x40, //  #
    0x40, //  #
    0x20, //   #
    0x20, //   #
    0x10, //    #
    0x10, //    #
    0x08, //     #
    0x08, //     #
    0x04, //      #
    0x04, //      #
    0x02, //       #
    0x03, //       ##
    0x00, //

    // @1056 ']' (3 pixels wide)
    0xE0, // ###
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0x20, //   #
    0xE0, // ###

    // @1072 '^' (9 pixels wide)
    0x00, 0x00, //
    0x08, 0x00, //     #
    0x0C, 0x00, //     ##
    0x14, 0x00, //    # #
    0x16, 0x00, //    # ##
    0x22, 0x00, //   #   #
    0x22, 0x00, //   #   #
    0x41, 0x00, //  #     #
    0xC1, 0x80, // ##     ##
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //

    // @1104 '_' (7 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xFE, // #######
    0x00, //
    0x00, //
    0x00, //

    // @1120 '`' (3 pixels wide)
    0x00, //
    0x00, //
    0xC0, // ##
    0x60, //  ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1136 'a' (5 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x70, //  ###
    0x88, // #   #
    0x08, //     #
    0xF8, // #####
    0x88, // #   #
    0x98, // #  ##
    0xE8, // ### #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1152 'b' (6 pixels wide)
    0x00, //
    0x00, //
    0x80, // #
    0x80, // #
    0x80, // #
    0xB8, // # ###
    0xCC, // ##  ##
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0xCC, // ##  ##
    0xB8, // # ###
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1168 'c' (4 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x70, //  ###
    0x90, // #  #
    0x80, // #
    0x80, // #
    0x80, // #
    0x90, // #  #
    0x70, //  ###
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1184 'd' (6 pixels wide)
    0x00, //
    0x00, //
    0x04, //      #
    0x04, //      #
    0x04, //      #
    0x74, //  ### #
    0x8C, // #   ##
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x8C, // #   ##
    0x74, //  ### #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1200 'e' (6 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x70, //  ###
    0x88, // #   #
    0x88, // #   #
    0xFC, // ######
    0x80, // #
    0x88, // #   #
    0x70, //  ###
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1216 'f' (4 pixels wide)
    0x00, //
    0x00, //
    0x70, //  ###
    0x40, //  #
    0x40, //  #
    0xE0, // ###
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1232 'g' (6 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x74, //  ### #
    0x8C, // #   ##
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x8C, // #   ##
    0x74, //  ### #
    0x04, //      #
    0x88, // #   #
    0x70, //  ###
    0x00, //

    // @1248 'h' (6 pixels wide)
    0x00, //
    0x00, //
    0x80, // #
    0x80, // #
    0x80, // #
    0xB8, // # ###
    0xC8, // ##  #
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1264 'i' (2 pixels wide)
    0x00, //
    0x00, //
    0xC0, // ##
    0x00, //
    0x00, //
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1280 'j' (3 pixels wide)
    0x00, //
    0x00, //
    0x60, //  ##
    0x00, //
    0x00, //
    0x60, //  ##
    0x60, //  ##
    0x60, //  ##
    0x60, //  ##
    0x60, //  ##
    0x60, //  ##
    0x60, //  ##
    0x60, //  ##
    0x40, //  #
    0xC0, // ##
    0x00, //

    // @1296 'k' (5 pixels wide)
    0x00, //
    0x00, //
    0x80, // #
    0x80, // #
    0x80, // #
    0x98, // #  ##
    0xB0, // # ##
    0xE0, // ###
    0xE0, // ###
    0xB0, // # ##
    0x98, // #  ##
    0x88, // #   #
    0x08, //     #
    0x00, //
    0x00, //
    0x00, //

    // @1312 'l' (3 pixels wide)
    0x00, //
    0x00, //
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x60, //  ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1328 'm' (10 pixels wide)
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0xFB, 0x80, // ##### ###
    0xCC, 0xC0, // ##  ##  ##
    0x88, 0x40, // #   #    #
    0x88, 0x40, // #   #    #
    0x88, 0x40, // #   #    #
    0x88, 0x40, // #   #    #
    0x88, 0x40, // #   #    #
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //

    // @1360 'n' (6 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xB8, // # ###
    0xC8, // ##  #
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1376 'o' (6 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x70, //  ###
    0x88, // #   #
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x88, // #   #
    0x70, //  ###
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1392 'p' (6 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xB8, // # ###
    0xCC, // ##  ##
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0xCC, // ##  ##
    0xB8, // # ###
    0x80, // #
    0x80, // #
    0x80, // #
    0x00, //

    // @1408 'q' (6 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x74, //  ### #
    0x8C, // #   ##
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x8C, // #   ##
    0x74, //  ### #
    0x04, //      #
    0x04, //      #
    0x04, //      #
    0x00, //

    // @1424 'r' (4 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xB0, // # ##
    0xC0, // ##
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1440 's' (6 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x38, //   ###
    0x48, //  #  #
    0x40, //  #
    0x38, //   ###
    0x0C, //     ##
    0xCC, // ##  ##
    0x78, //  ####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1456 't' (5 pixels wide)
    0x00, //
    0x00, //
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0xF0, // ####
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x38, //   ###
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1472 'u' (6 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0xCC, // ##  ##
    0x74, //  ### #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1488 'v' (6 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xC4, // ##   #
    0x44, //  #   #
    0x48, //  #  #
    0x68, //  ## #
    0x28, //   # #
    0x30, //   ##
    0x10, //    #
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1504 'w' (9 pixels wide)
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0xD8, 0x80, // ## ##   #
    0x49, 0x80, //  #  #  ##
    0x49, 0x00, //  #  #  #
    0x4D, 0x00, //  #  ## #
    0x35, 0x00, //   ## # #
    0x36, 0x00, //   ## ##
    0x32, 0x00, //   ##  #
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //
    0x00, 0x00, //

    // @1536 'x' (7 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xC6, // ##   ##
    0x4C, //  #  ##
    0x38, //   ###
    0x10, //    #
    0x38, //   ###
    0x4C, //  #  ##
    0xC6, // ##   ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1552 'y' (6 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0x84, // #    #
    0xCC, // ##  ##
    0x74, //  ### #
    0x04, //      #
    0x88, // #   #
    0x70, //  ###
    0x00, //

    // @1568 'z' (5 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0xF8, // #####
    0x18, //    ##
    0x30, //   ##
    0x20, //   #
    0x40, //  #
    0xC0, // ##
    0xF8, // #####
    0x00, //
    0x00, //
    0x00, //
    0x00, //

    // @1584 '{' (3 pixels wide)
    0x60, //  ##
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x60, //  ##

    // @1600 '|' (1 pixels wide)
    0x00, //
    0x00, //
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x80, // #
    0x00, //
    0x00, //

    // @1616 '}' (3 pixels wide)
    0x80, // #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x60, //  ##
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x40, //  #
    0x80, // #

    // @1632 '~' (7 pixels wide)
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x62, //  ##   #
    0x52, //  # #  #
    0x8C, // #   ##
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
};

static const CharDesc_t s_Orkney8ptChar[] = {
    {2, orkney_8ptBitmaps + 0UL},    // !
    {3, orkney_8ptBitmaps + 16},      // "
    {8, orkney_8ptBitmaps + 32},      // #
    {6, orkney_8ptBitmaps + 48},      // $
    {8, orkney_8ptBitmaps + 64},      // %
    {8, orkney_8ptBitmaps + 80},      // &
    {1, orkney_8ptBitmaps + 96},      // '
    {3, orkney_8ptBitmaps + 112},     // (
    {3, orkney_8ptBitmaps + 128},     // )
    {3, orkney_8ptBitmaps + 144},     // *
    {8, orkney_8ptBitmaps + 160},     // +
    {1, orkney_8ptBitmaps + 176},     // ,
    {5, orkney_8ptBitmaps + 192},     // -
    {2, orkney_8ptBitmaps + 208},     // .
    {8, orkney_8ptBitmaps + 224},     // /
    {7, orkney_8ptBitmaps + 240},     // 0
    {4, orkney_8ptBitmaps + 256},     // 1
    {6, orkney_8ptBitmaps + 272},     // 2
    {6, orkney_8ptBitmaps + 288},     // 3
    {7, orkney_8ptBitmaps + 304},     // 4
    {6, orkney_8ptBitmaps + 320},     // 5
    {7, orkney_8ptBitmaps + 336},     // 6
    {7, orkney_8ptBitmaps + 352},     // 7
    {6, orkney_8ptBitmaps + 368},     // 8
    {7, orkney_8ptBitmaps + 384},     // 9
    {2, orkney_8ptBitmaps + 400},     // :
    {2, orkney_8ptBitmaps + 416},     // ;
    {7, orkney_8ptBitmaps + 432},     // <
    {8, orkney_8ptBitmaps + 448},     // =
    {7, orkney_8ptBitmaps + 464},     // >
    {5, orkney_8ptBitmaps + 480},     // ?
    {6, orkney_8ptBitmaps + 496},     // @
    {9, orkney_8ptBitmaps + 512},     // A
    {7, orkney_8ptBitmaps + 544},     // B
    {7, orkney_8ptBitmaps + 560},     // C
    {8, orkney_8ptBitmaps + 576},     // D
    {5, orkney_8ptBitmaps + 592},     // E
    {5, orkney_8ptBitmaps + 608},     // F
    {8, orkney_8ptBitmaps + 624},     // G
    {8, orkney_8ptBitmaps + 640},     // H
    {1, orkney_8ptBitmaps + 656},     // I
    {3, orkney_8ptBitmaps + 672},     // J
    {7, orkney_8ptBitmaps + 688},     // K
    {5, orkney_8ptBitmaps + 704},     // L
    {10,orkney_8ptBitmaps + 720},    // M
    {8, orkney_8ptBitmaps + 752},     // N
    {9, orkney_8ptBitmaps + 768},     // O
    {6, orkney_8ptBitmaps + 800},     // P
    {10,orkney_8ptBitmaps + 816},    // Q
    {7, orkney_8ptBitmaps + 848},     // R
    {7, orkney_8ptBitmaps + 864},     // S
    {7, orkney_8ptBitmaps + 880},     // T
    {7, orkney_8ptBitmaps + 896},     // U
    {9, orkney_8ptBitmaps + 912},     // V
    {14,orkney_8ptBitmaps + 944},    // W
    {8, orkney_8ptBitmaps + 976},     // X
    {8, orkney_8ptBitmaps + 992},     // Y
    {7, orkney_8ptBitmaps + 1008},    // Z
    {3, orkney_8ptBitmaps + 1024},    // [
    {8, orkney_8ptBitmaps + 1040},    // '\'
    {3, orkney_8ptBitmaps + 1056},    // ]
    {9, orkney_8ptBitmaps + 1072},    // ^
    {7, orkney_8ptBitmaps + 1104},    // _
    {3, orkney_8ptBitmaps + 1120},    // `
    {5, orkney_8ptBitmaps + 1136},    // a
    {6, orkney_8ptBitmaps + 1152},    // b
    {4, orkney_8ptBitmaps + 1168},    // c
    {6, orkney_8ptBitmaps + 1184},    // d
    {6, orkney_8ptBitmaps + 1200},    // e
    {4, orkney_8ptBitmaps + 1216},    // f
    {6, orkney_8ptBitmaps + 1232},    // g
    {6, orkney_8ptBitmaps + 1248},    // h
    {2, orkney_8ptBitmaps + 1264},    // i
    {3, orkney_8ptBitmaps + 1280},    // j
    {5, orkney_8ptBitmaps + 1296},    // k
    {3, orkney_8ptBitmaps + 1312},    // l
    {10,orkney_8ptBitmaps + 1328},   // m
    {6, orkney_8ptBitmaps + 1360},    // n
    {6, orkney_8ptBitmaps + 1376},    // o
    {6, orkney_8ptBitmaps + 1392},    // p
    {6, orkney_8ptBitmaps + 1408},    // q
    {4, orkney_8ptBitmaps + 1424},    // r
    {6, orkney_8ptBitmaps + 1440},    // s
    {5, orkney_8ptBitmaps + 1456},    // t
    {6, orkney_8ptBitmaps + 1472},    // u
    {6, orkney_8ptBitmaps + 1488},    // v
    {9, orkney_8ptBitmaps + 1504},    // w
    {7, orkney_8ptBitmaps + 1536},    // x
    {6, orkney_8ptBitmaps + 1552},    // y
    {5, orkney_8ptBitmaps + 1568},    // z
    {3, orkney_8ptBitmaps + 1584},    // {
    {1, orkney_8ptBitmaps + 1600},    // |
    {3, orkney_8ptBitmaps + 1616},    // }
    {7, orkney_8ptBitmaps + 1632},    // ~
};

const FontDesc_t g_Orkney8ptDesc = {
	.Flag = 0,	// Variable length, horizontal
	.Width = 14,
	.Height = 15,
	.pCharDesc = s_Orkney8ptChar
};
