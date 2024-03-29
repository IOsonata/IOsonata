// 
//  Font data for FreeMono 10pt
// https://savannah.gnu.org/projects/freefont/
// 
// Font bitmap generated by
// http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator

#include "display/ifont.h"

// Character bitmaps for FreeMono 10pt
static const uint8_t s_FreeMonoBold10ptBitmaps[] = {
	// @0 '!' (1 pixels wide)
	0x00, //  
	0x80, // #
	0x80, // #
	0x80, // #
	0x80, // #
	0x80, // #
	0x80, // #
	0x00, //  
	0x80, // #
	0x00, //  
	0x00, //  
	0x00, //  

	// @12 '"' (4 pixels wide)
	0x00, //     
	0x90, // #  #
	0x90, // #  #
	0x90, // #  #
	0x90, // #  #
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     

	// @24 '#' (6 pixels wide)
	0x48, //  #  # 
	0x48, //  #  # 
	0x48, //  #  # 
	0xFC, // ######
	0x50, //  # #  
	0x50, //  # #  
	0x7C, //  #####
	0x48, //  #  # 
	0x50, //  # #  
	0x50, //  # #  
	0x00, //       
	0x00, //       

	// @36 '$' (5 pixels wide)
	0x00, //      
	0x20, //   #  
	0x78, //  ####
	0x88, // #   #
	0xE0, // ###  
	0x18, //    ##
	0x88, // #   #
	0x88, // #   #
	0xF0, // #### 
	0x20, //   #  
	0x20, //   #  
	0x00, //      

	// @48 '%' (6 pixels wide)
	0x00, //       
	0x60, //  ##   
	0x90, // #  #  
	0x90, // #  #  
	0x7C, //  #####
	0xF8, // ##### 
	0x24, //   #  #
	0x24, //   #  #
	0x18, //    ## 
	0x00, //       
	0x00, //       
	0x00, //       

	// @60 '&' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x78, //  #### 
	0x40, //  #    
	0x40, //  #    
	0x60, //  ##   
	0xBC, // # ####
	0x98, // #  ## 
	0x7C, //  #####
	0x00, //       
	0x00, //       
	0x00, //       

	// @72 ''' (1 pixels wide)
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

	// @84 '(' (2 pixels wide)
	0x00, //   
	0x40, //  #
	0x40, //  #
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0xC0, // ##
	0x40, //  #
	0x40, //  #
	0x00, //   

	// @96 ')' (2 pixels wide)
	0x00, //   
	0x80, // # 
	0x80, // # 
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0xC0, // ##
	0x80, // # 
	0x80, // # 
	0x00, //   

	// @108 '*' (5 pixels wide)
	0x00, //      
	0x20, //   #  
	0xA8, // # # #
	0xF8, // #####
	0x70, //  ### 
	0x50, //  # # 
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @120 '+' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x10, //    #   
	0x10, //    #   
	0x10, //    #   
	0xFE, // #######
	0x10, //    #   
	0x10, //    #   
	0x10, //    #   
	0x00, //        
	0x00, //        
	0x00, //        

	// @132 ',' (2 pixels wide)
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x40, //  #
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x00, //   

	// @144 '-' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @156 '.' (1 pixels wide)
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  
	0x80, // #
	0x00, //  
	0x00, //  
	0x00, //  

	// @168 '/' (6 pixels wide)
	0x0C, //     ##
	0x08, //     # 
	0x18, //    ## 
	0x10, //    #  
	0x10, //    #  
	0x20, //   #   
	0x20, //   #   
	0x60, //  ##   
	0x40, //  #    
	0xC0, // ##    
	0x00, //       
	0x00, //       

	// @180 '0' (5 pixels wide)
	0x00, //      
	0x70, //  ### 
	0xC8, // ##  #
	0x88, // #   #
	0x88, // #   #
	0x88, // #   #
	0x88, // #   #
	0xC8, // ##  #
	0x70, //  ### 
	0x00, //      
	0x00, //      
	0x00, //      

	// @192 '1' (5 pixels wide)
	0x00, //      
	0x60, //  ##  
	0xE0, // ###  
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0xF8, // #####
	0x00, //      
	0x00, //      
	0x00, //      

	// @204 '2' (5 pixels wide)
	0x00, //      
	0x70, //  ### 
	0x88, // #   #
	0x08, //     #
	0x18, //    ##
	0x30, //   ## 
	0x60, //  ##  
	0xC8, // ##  #
	0xF8, // #####
	0x00, //      
	0x00, //      
	0x00, //      

	// @216 '3' (6 pixels wide)
	0x00, //       
	0x38, //   ### 
	0x44, //  #   #
	0x04, //      #
	0x18, //    ## 
	0x04, //      #
	0x04, //      #
	0x04, //      #
	0xF8, // ##### 
	0x00, //       
	0x00, //       
	0x00, //       

	// @228 '4' (5 pixels wide)
	0x00, //      
	0x30, //   ## 
	0x30, //   ## 
	0x70, //  ### 
	0x50, //  # # 
	0x90, // #  # 
	0xF8, // #####
	0x10, //    # 
	0x38, //   ###
	0x00, //      
	0x00, //      
	0x00, //      

	// @240 '5' (6 pixels wide)
	0x00, //       
	0x78, //  #### 
	0x40, //  #    
	0x40, //  #    
	0x78, //  #### 
	0x04, //      #
	0x04, //      #
	0x84, // #    #
	0xF8, // ##### 
	0x00, //       
	0x00, //       
	0x00, //       

	// @252 '6' (5 pixels wide)
	0x00, //      
	0x38, //   ###
	0x40, //  #   
	0x80, // #    
	0xF0, // #### 
	0xC8, // ##  #
	0x88, // #   #
	0xC8, // ##  #
	0x70, //  ### 
	0x00, //      
	0x00, //      
	0x00, //      

	// @264 '7' (6 pixels wide)
	0x00, //       
	0xFC, // ######
	0x88, // #   # 
	0x08, //     # 
	0x18, //    ## 
	0x10, //    #  
	0x10, //    #  
	0x30, //   ##  
	0x20, //   #   
	0x00, //       
	0x00, //       
	0x00, //       

	// @276 '8' (5 pixels wide)
	0x00, //      
	0x70, //  ### 
	0x88, // #   #
	0x88, // #   #
	0x88, // #   #
	0x70, //  ### 
	0x88, // #   #
	0x88, // #   #
	0x70, //  ### 
	0x00, //      
	0x00, //      
	0x00, //      

	// @288 '9' (5 pixels wide)
	0x00, //      
	0x70, //  ### 
	0x98, // #  ##
	0x88, // #   #
	0x98, // #  ##
	0x78, //  ####
	0x08, //     #
	0x10, //    # 
	0xE0, // ###  
	0x00, //      
	0x00, //      
	0x00, //      

	// @300 ':' (1 pixels wide)
	0x00, //  
	0x00, //  
	0x00, //  
	0x80, // #
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  
	0x80, // #
	0x00, //  
	0x00, //  
	0x00, //  

	// @312 ';' (2 pixels wide)
	0x00, //   
	0x00, //   
	0x00, //   
	0x40, //  #
	0x00, //   
	0x00, //   
	0x00, //   
	0x40, //  #
	0x40, //  #
	0x80, // # 
	0x80, // # 
	0x00, //   

	// @324 '<' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x04, //      #
	0x1C, //    ###
	0x70, //  ###  
	0xE0, // ###   
	0x30, //   ##  
	0x0C, //     ##
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @336 '=' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xFC, // ######
	0x00, //       
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @348 '>' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0xC0, // ##    
	0x30, //   ##  
	0x1C, //    ###
	0x70, //  ###  
	0xC0, // ##    
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @360 '?' (5 pixels wide)
	0x00, //      
	0x00, //      
	0xF0, // #### 
	0x88, // #   #
	0x08, //     #
	0x30, //   ## 
	0x20, //   #  
	0x00, //      
	0x20, //   #  
	0x00, //      
	0x00, //      
	0x00, //      

	// @372 '@' (6 pixels wide)
	0x00, //       
	0x70, //  ###  
	0x48, //  #  # 
	0x88, // #   # 
	0x98, // #  ## 
	0xA8, // # # # 
	0xA8, // # # # 
	0x9C, // #  ###
	0x80, // #     
	0x48, //  #  # 
	0x78, //  #### 
	0x00, //       

	// @384 'A' (8 pixels wide)
	0x00, //         
	0x78, //  ####   
	0x3C, //   ####  
	0x24, //   #  #  
	0x24, //   #  #  
	0x7C, //  #####  
	0x42, //  #    # 
	0x42, //  #    # 
	0xE7, // ###  ###
	0x00, //         
	0x00, //         
	0x00, //         

	// @396 'B' (8 pixels wide)
	0x00, //         
	0xFC, // ######  
	0x42, //  #    # 
	0x42, //  #    # 
	0x46, //  #   ## 
	0x7E, //  ###### 
	0x41, //  #     #
	0x41, //  #     #
	0xFE, // ####### 
	0x00, //         
	0x00, //         
	0x00, //         

	// @408 'C' (7 pixels wide)
	0x00, //        
	0x3E, //   #####
	0x46, //  #   ##
	0x82, // #     #
	0x80, // #      
	0x80, // #      
	0x80, // #      
	0x42, //  #    #
	0x3C, //   #### 
	0x00, //        
	0x00, //        
	0x00, //        

	// @420 'D' (7 pixels wide)
	0x00, //        
	0xF8, // #####  
	0x44, //  #   # 
	0x42, //  #    #
	0x42, //  #    #
	0x42, //  #    #
	0x42, //  #    #
	0x44, //  #   # 
	0xF8, // #####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @432 'E' (7 pixels wide)
	0x00, //        
	0xFE, // #######
	0x42, //  #    #
	0x42, //  #    #
	0x48, //  #  #  
	0x78, //  ####  
	0x4A, //  #  # #
	0x42, //  #    #
	0xFE, // #######
	0x00, //        
	0x00, //        
	0x00, //        

	// @444 'F' (7 pixels wide)
	0x00, //        
	0xFE, // #######
	0x42, //  #    #
	0x42, //  #    #
	0x48, //  #  #  
	0x78, //  ####  
	0x48, //  #  #  
	0x40, //  #     
	0xF0, // ####   
	0x00, //        
	0x00, //        
	0x00, //        

	// @456 'G' (8 pixels wide)
	0x00, //         
	0x3E, //   ##### 
	0x42, //  #    # 
	0x82, // #     # 
	0x80, // #       
	0x8F, // #   ####
	0x82, // #     # 
	0x42, //  #    # 
	0x3E, //   ##### 
	0x00, //         
	0x00, //         
	0x00, //         

	// @468 'H' (7 pixels wide)
	0x00, //        
	0xEE, // ### ###
	0x44, //  #   # 
	0x44, //  #   # 
	0x7C, //  ##### 
	0x44, //  #   # 
	0x44, //  #   # 
	0x44, //  #   # 
	0xEE, // ### ###
	0x00, //        
	0x00, //        
	0x00, //        

	// @480 'I' (5 pixels wide)
	0x00, //      
	0xF8, // #####
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0xF8, // #####
	0x00, //      
	0x00, //      
	0x00, //      

	// @492 'J' (6 pixels wide)
	0x00, //       
	0x3C, //   ####
	0x08, //     # 
	0x08, //     # 
	0x08, //     # 
	0x08, //     # 
	0x88, // #   # 
	0x88, // #   # 
	0xF0, // ####  
	0x00, //       
	0x00, //       
	0x00, //       

	// @504 'K' (8 pixels wide)
	0x00, //         
	0xEE, // ### ### 
	0x4C, //  #  ##  
	0x58, //  # ##   
	0x70, //  ###    
	0x78, //  ####   
	0x4C, //  #  ##  
	0x44, //  #   #  
	0xE7, // ###  ###
	0x00, //         
	0x00, //         
	0x00, //         

	// @516 'L' (6 pixels wide)
	0x00, //       
	0xE0, // ###   
	0x40, //  #    
	0x40, //  #    
	0x40, //  #    
	0x40, //  #    
	0x44, //  #   #
	0x44, //  #   #
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       

	// @528 'M' (8 pixels wide)
	0x00, //         
	0xC3, // ##    ##
	0x66, //  ##  ## 
	0x66, //  ##  ## 
	0x7E, //  ###### 
	0x5A, //  # ## # 
	0x5A, //  # ## # 
	0x42, //  #    # 
	0xE7, // ###  ###
	0x00, //         
	0x00, //         
	0x00, //         

	// @540 'N' (8 pixels wide)
	0x00, //         
	0xC7, // ##   ###
	0x62, //  ##   # 
	0x72, //  ###  # 
	0x5A, //  # ## # 
	0x4A, //  #  # # 
	0x4E, //  #  ### 
	0x46, //  #   ## 
	0xE2, // ###   # 
	0x00, //         
	0x00, //         
	0x00, //         

	// @552 'O' (7 pixels wide)
	0x00, //        
	0x38, //   ###  
	0x44, //  #   # 
	0x82, // #     #
	0x82, // #     #
	0x82, // #     #
	0x82, // #     #
	0x44, //  #   # 
	0x38, //   ###  
	0x00, //        
	0x00, //        
	0x00, //        

	// @564 'P' (7 pixels wide)
	0x00, //        
	0xFC, // ###### 
	0x42, //  #    #
	0x42, //  #    #
	0x42, //  #    #
	0x7C, //  ##### 
	0x40, //  #     
	0x40, //  #     
	0xF0, // ####   
	0x00, //        
	0x00, //        
	0x00, //        

	// @576 'Q' (7 pixels wide)
	0x00, //        
	0x38, //   ###  
	0x44, //  #   # 
	0x82, // #     #
	0x82, // #     #
	0x82, // #     #
	0x82, // #     #
	0x44, //  #   # 
	0x38, //   ###  
	0x22, //   #   #
	0x7E, //  ######
	0x00, //        

	// @588 'R' (7 pixels wide)
	0x00, //        
	0xF8, // #####  
	0x44, //  #   # 
	0x44, //  #   # 
	0x4C, //  #  ## 
	0x78, //  ####  
	0x48, //  #  #  
	0x44, //  #   # 
	0xE6, // ###  ##
	0x00, //        
	0x00, //        
	0x00, //        

	// @600 'S' (6 pixels wide)
	0x00, //       
	0x7C, //  #####
	0x84, // #    #
	0x80, // #     
	0xC0, // ##    
	0x3C, //   ####
	0x84, // #    #
	0x84, // #    #
	0xF8, // ##### 
	0x00, //       
	0x00, //       
	0x00, //       

	// @612 'T' (6 pixels wide)
	0x00, //       
	0xFC, // ######
	0xA4, // # #  #
	0xA4, // # #  #
	0x20, //   #   
	0x20, //   #   
	0x20, //   #   
	0x20, //   #   
	0xF8, // ##### 
	0x00, //       
	0x00, //       
	0x00, //       

	// @624 'U' (8 pixels wide)
	0x00, //         
	0xE7, // ###  ###
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x3C, //   ####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @636 'V' (8 pixels wide)
	0x00, //         
	0xE7, // ###  ###
	0x42, //  #    # 
	0x66, //  ##  ## 
	0x24, //   #  #  
	0x24, //   #  #  
	0x3C, //   ####  
	0x18, //    ##   
	0x18, //    ##   
	0x00, //         
	0x00, //         
	0x00, //         

	// @648 'W' (8 pixels wide)
	0x00, //         
	0xE7, // ###  ###
	0x42, //  #    # 
	0x5A, //  # ## # 
	0x5A, //  # ## # 
	0x7A, //  #### # 
	0x66, //  ##  ## 
	0x66, //  ##  ## 
	0x66, //  ##  ## 
	0x00, //         
	0x00, //         
	0x00, //         

	// @660 'X' (8 pixels wide)
	0x00, //         
	0xE7, // ###  ###
	0x66, //  ##  ## 
	0x3C, //   ####  
	0x18, //    ##   
	0x3C, //   ####  
	0x3C, //   ####  
	0x66, //  ##  ## 
	0xE7, // ###  ###
	0x00, //         
	0x00, //         
	0x00, //         

	// @672 'Y' (8 pixels wide)
	0x00, //         
	0xE7, // ###  ###
	0x66, //  ##  ## 
	0x2C, //   # ##  
	0x38, //   ###   
	0x10, //    #    
	0x10, //    #    
	0x10, //    #    
	0x7C, //  #####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @684 'Z' (6 pixels wide)
	0x00, //       
	0xF8, // ##### 
	0x88, // #   # 
	0x90, // #  #  
	0x20, //   #   
	0x20, //   #   
	0x44, //  #   #
	0x84, // #    #
	0x7C, //  #####
	0x00, //       
	0x00, //       
	0x00, //       

	// @696 '[' (3 pixels wide)
	0x00, //    
	0xE0, // ###
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0xE0, // ###
	0x00, //    

	// @708 '\' (6 pixels wide)
	0x80, // #     
	0xC0, // ##    
	0x40, //  #    
	0x60, //  ##   
	0x20, //   #   
	0x30, //   ##  
	0x10, //    #  
	0x18, //    ## 
	0x08, //     # 
	0x0C, //     ##
	0x00, //       
	0x00, //       

	// @720 ']' (3 pixels wide)
	0x00, //    
	0xE0, // ###
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0xE0, // ###
	0x00, //    

	// @732 '^' (6 pixels wide)
	0x00, //       
	0x20, //   #   
	0x30, //   ##  
	0x58, //  # ## 
	0x8C, // #   ##
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @744 '_' (8 pixels wide)
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
	0xFF, // ########

	// @756 '`' (2 pixels wide)
	0x00, //   
	0x80, // # 
	0x40, //  #
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   

	// @768 'a' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0xF0, // ####  
	0x08, //     # 
	0x78, //  #### 
	0x88, // #   # 
	0x88, // #   # 
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       

	// @780 'b' (7 pixels wide)
	0x00, //        
	0xC0, // ##     
	0x40, //  #     
	0x7C, //  ##### 
	0x66, //  ##  ##
	0x42, //  #    #
	0x42, //  #    #
	0x66, //  ##  ##
	0xFC, // ###### 
	0x00, //        
	0x00, //        
	0x00, //        

	// @792 'c' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x7C, //  #####
	0xCC, // ##  ##
	0x84, // #    #
	0x80, // #     
	0x84, // #    #
	0x7C, //  #####
	0x00, //       
	0x00, //       
	0x00, //       

	// @804 'd' (8 pixels wide)
	0x00, //         
	0x06, //      ## 
	0x02, //       # 
	0x7E, //  ###### 
	0xC6, // ##   ## 
	0x82, // #     # 
	0x82, // #     # 
	0xC6, // ##   ## 
	0x7F, //  #######
	0x00, //         
	0x00, //         
	0x00, //         

	// @816 'e' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x3C, //   #### 
	0x46, //  #   ##
	0xFE, // #######
	0x40, //  #     
	0x62, //  ##   #
	0x3E, //   #####
	0x00, //        
	0x00, //        
	0x00, //        

	// @828 'f' (5 pixels wide)
	0x00, //      
	0x78, //  ####
	0x40, //  #   
	0xF8, // #####
	0x40, //  #   
	0x40, //  #   
	0x40, //  #   
	0x40, //  #   
	0xF0, // #### 
	0x00, //      
	0x00, //      
	0x00, //      

	// @840 'g' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x7E, //  ######
	0xCC, // ##  ## 
	0x84, // #    # 
	0x84, // #    # 
	0xCC, // ##  ## 
	0x7C, //  ##### 
	0x04, //      # 
	0x04, //      # 
	0x38, //   ###  

	// @852 'h' (7 pixels wide)
	0x00, //        
	0xC0, // ##     
	0x40, //  #     
	0x78, //  ####  
	0x64, //  ##  # 
	0x44, //  #   # 
	0x44, //  #   # 
	0x44, //  #   # 
	0xEE, // ### ###
	0x00, //        
	0x00, //        
	0x00, //        

	// @864 'i' (6 pixels wide)
	0x00, //       
	0x10, //    #  
	0x00, //       
	0x70, //  ###  
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       

	// @876 'j' (4 pixels wide)
	0x00, //     
	0x20, //   # 
	0x00, //     
	0xF0, // ####
	0x10, //    #
	0x10, //    #
	0x10, //    #
	0x10, //    #
	0x10, //    #
	0x10, //    #
	0x10, //    #
	0xE0, // ### 

	// @888 'k' (6 pixels wide)
	0x00, //       
	0xC0, // ##    
	0x40, //  #    
	0x78, //  #### 
	0x70, //  ###  
	0x60, //  ##   
	0x60, //  ##   
	0x50, //  # #  
	0xDC, // ## ###
	0x00, //       
	0x00, //       
	0x00, //       

	// @900 'l' (6 pixels wide)
	0x00, //       
	0x70, //  ###  
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       

	// @912 'm' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0xFE, // ####### 
	0x52, //  # #  # 
	0x52, //  # #  # 
	0x52, //  # #  # 
	0x52, //  # #  # 
	0xFB, // ##### ##
	0x00, //         
	0x00, //         
	0x00, //         

	// @924 'n' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0xF8, // #####  
	0x44, //  #   # 
	0x44, //  #   # 
	0x44, //  #   # 
	0x44, //  #   # 
	0xEE, // ### ###
	0x00, //        
	0x00, //        
	0x00, //        

	// @936 'o' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x78, //  #### 
	0xCC, // ##  ##
	0x84, // #    #
	0x84, // #    #
	0xCC, // ##  ##
	0x78, //  #### 
	0x00, //       
	0x00, //       
	0x00, //       

	// @948 'p' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0xFC, // ###### 
	0x62, //  ##   #
	0x42, //  #    #
	0x42, //  #    #
	0x66, //  ##  ##
	0x7C, //  ##### 
	0x40, //  #     
	0x40, //  #     
	0xF0, // ####   

	// @960 'q' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x7E, //  ######
	0xCC, // ##  ## 
	0x84, // #    # 
	0x84, // #    # 
	0xCC, // ##  ## 
	0x7C, //  ##### 
	0x04, //      # 
	0x04, //      # 
	0x1E, //    ####

	// @972 'r' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0xFC, // ######
	0x60, //  ##   
	0x40, //  #    
	0x40, //  #    
	0x40, //  #    
	0xF0, // ####  
	0x00, //       
	0x00, //       
	0x00, //       

	// @984 's' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0xF8, // #####
	0x88, // #   #
	0xF0, // #### 
	0x78, //  ####
	0x88, // #   #
	0xF8, // #####
	0x00, //      
	0x00, //      
	0x00, //      

	// @996 't' (6 pixels wide)
	0x00, //       
	0x40, //  #    
	0x40, //  #    
	0xF8, // ##### 
	0x40, //  #    
	0x40, //  #    
	0x40, //  #    
	0x44, //  #   #
	0x3C, //   ####
	0x00, //       
	0x00, //       
	0x00, //       

	// @1008 'u' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0xCC, // ##  ## 
	0x44, //  #   # 
	0x44, //  #   # 
	0x44, //  #   # 
	0x44, //  #   # 
	0x3E, //   #####
	0x00, //        
	0x00, //        
	0x00, //        

	// @1020 'v' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0xEE, // ### ###
	0x44, //  #   # 
	0x6C, //  ## ## 
	0x28, //   # #  
	0x38, //   ###  
	0x10, //    #   
	0x00, //        
	0x00, //        
	0x00, //        

	// @1032 'w' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0xE7, // ###  ###
	0x5A, //  # ## # 
	0x5A, //  # ## # 
	0x7C, //  #####  
	0x7C, //  #####  
	0x24, //   #  #  
	0x00, //         
	0x00, //         
	0x00, //         

	// @1044 'x' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0xFC, // ######
	0x78, //  #### 
	0x30, //   ##  
	0x30, //   ##  
	0x78, //  #### 
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       

	// @1056 'y' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0xE7, // ###  ###
	0x42, //  #    # 
	0x66, //  ##  ## 
	0x24, //   #  #  
	0x3C, //   ####  
	0x18, //    ##   
	0x18, //    ##   
	0x30, //   ##    
	0xF0, // ####    

	// @1068 'z' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0xF8, // #####
	0xB0, // # ## 
	0x30, //   ## 
	0x60, //  ##  
	0xC8, // ##  #
	0xF8, // #####
	0x00, //      
	0x00, //      
	0x00, //      

	// @1080 '{' (3 pixels wide)
	0x00, //    
	0x60, //  ##
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x80, // #  
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x60, //  ##
	0x00, //    

	// @1092 '|' (1 pixels wide)
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

	// @1104 '}' (3 pixels wide)
	0x00, //    
	0xC0, // ## 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x20, //   #
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0xC0, // ## 
	0x00, //    

	// @1116 '~' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x64, //  ##  #
	0x98, // #  ## 
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
};

// Character descriptors for FreeMono 10pt
// { [Char width in bits], [Offset into freeMono_10ptCharBitmaps in bytes] }
static const CharDesc_t s_FreeMonoBold10ptCharDesc[] = {
	{1, s_FreeMonoBold10ptBitmaps + 0}, 		// !
	{4, s_FreeMonoBold10ptBitmaps + 12}, 		// "
	{6, s_FreeMonoBold10ptBitmaps + 24}, 		// #
	{5, s_FreeMonoBold10ptBitmaps + 36}, 		// $
	{6, s_FreeMonoBold10ptBitmaps + 48}, 		// %
	{6, s_FreeMonoBold10ptBitmaps + 60}, 		// &
	{1, s_FreeMonoBold10ptBitmaps + 72}, 		// '
	{2, s_FreeMonoBold10ptBitmaps + 84}, 		// (
	{2, s_FreeMonoBold10ptBitmaps + 96}, 		// )
	{5, s_FreeMonoBold10ptBitmaps + 108}, 		// *
	{7, s_FreeMonoBold10ptBitmaps + 120}, 		// +
	{2, s_FreeMonoBold10ptBitmaps + 132}, 		// ,
	{6, s_FreeMonoBold10ptBitmaps + 144}, 		// -
	{1, s_FreeMonoBold10ptBitmaps + 156}, 		// .
	{6, s_FreeMonoBold10ptBitmaps + 168}, 		// /
	{5, s_FreeMonoBold10ptBitmaps + 180}, 		// 0
	{5, s_FreeMonoBold10ptBitmaps + 192}, 		// 1
	{5, s_FreeMonoBold10ptBitmaps + 204}, 		// 2
	{6, s_FreeMonoBold10ptBitmaps + 216}, 		// 3
	{5, s_FreeMonoBold10ptBitmaps + 228}, 		// 4
	{6, s_FreeMonoBold10ptBitmaps + 240}, 		// 5
	{5, s_FreeMonoBold10ptBitmaps + 252}, 		// 6
	{6, s_FreeMonoBold10ptBitmaps + 264}, 		// 7
	{5, s_FreeMonoBold10ptBitmaps + 276}, 		// 8
	{5, s_FreeMonoBold10ptBitmaps + 288}, 		// 9
	{1, s_FreeMonoBold10ptBitmaps + 300}, 		// :
	{2, s_FreeMonoBold10ptBitmaps + 312}, 		// ;
	{6, s_FreeMonoBold10ptBitmaps + 324}, 		// <
	{6, s_FreeMonoBold10ptBitmaps + 336}, 		// =
	{6, s_FreeMonoBold10ptBitmaps + 348}, 		// >
	{5, s_FreeMonoBold10ptBitmaps + 360}, 		// ?
	{6, s_FreeMonoBold10ptBitmaps + 372}, 		// @
	{8, s_FreeMonoBold10ptBitmaps + 384}, 		// A
	{8, s_FreeMonoBold10ptBitmaps + 396}, 		// B
	{7, s_FreeMonoBold10ptBitmaps + 408}, 		// C
	{7, s_FreeMonoBold10ptBitmaps + 420}, 		// D
	{7, s_FreeMonoBold10ptBitmaps + 432}, 		// E
	{7, s_FreeMonoBold10ptBitmaps + 444}, 		// F
	{8, s_FreeMonoBold10ptBitmaps + 456}, 		// G
	{7, s_FreeMonoBold10ptBitmaps + 468}, 		// H
	{5, s_FreeMonoBold10ptBitmaps + 480}, 		// I
	{6, s_FreeMonoBold10ptBitmaps + 492}, 		// J
	{8, s_FreeMonoBold10ptBitmaps + 504}, 		// K
	{6, s_FreeMonoBold10ptBitmaps + 516}, 		// L
	{8, s_FreeMonoBold10ptBitmaps + 528}, 		// M
	{8, s_FreeMonoBold10ptBitmaps + 540}, 		// N
	{7, s_FreeMonoBold10ptBitmaps + 552}, 		// O
	{7, s_FreeMonoBold10ptBitmaps + 564}, 		// P
	{7, s_FreeMonoBold10ptBitmaps + 576}, 		// Q
	{7, s_FreeMonoBold10ptBitmaps + 588}, 		// R
	{6, s_FreeMonoBold10ptBitmaps + 600}, 		// S
	{6, s_FreeMonoBold10ptBitmaps + 612}, 		// T
	{8, s_FreeMonoBold10ptBitmaps + 624}, 		// U
	{8, s_FreeMonoBold10ptBitmaps + 636}, 		// V
	{8, s_FreeMonoBold10ptBitmaps + 648}, 		// W
	{8, s_FreeMonoBold10ptBitmaps + 660}, 		// X
	{8, s_FreeMonoBold10ptBitmaps + 672}, 		// Y
	{6, s_FreeMonoBold10ptBitmaps + 684}, 		// Z
	{3, s_FreeMonoBold10ptBitmaps + 696}, 		// [
	{6, s_FreeMonoBold10ptBitmaps + 708}, 		// '\'
	{3, s_FreeMonoBold10ptBitmaps + 720}, 		// ]
	{6, s_FreeMonoBold10ptBitmaps + 732}, 		// ^
	{8, s_FreeMonoBold10ptBitmaps + 744}, 		// _
	{2, s_FreeMonoBold10ptBitmaps + 756}, 		// `
	{6, s_FreeMonoBold10ptBitmaps + 768}, 		// a
	{7, s_FreeMonoBold10ptBitmaps + 780}, 		// b
	{6, s_FreeMonoBold10ptBitmaps + 792}, 		// c
	{8, s_FreeMonoBold10ptBitmaps + 804}, 		// d
	{7, s_FreeMonoBold10ptBitmaps + 816}, 		// e
	{5, s_FreeMonoBold10ptBitmaps + 828}, 		// f
	{7, s_FreeMonoBold10ptBitmaps + 840}, 		// g
	{7, s_FreeMonoBold10ptBitmaps + 852}, 		// h
	{6, s_FreeMonoBold10ptBitmaps + 864}, 		// i
	{4, s_FreeMonoBold10ptBitmaps + 876}, 		// j
	{6, s_FreeMonoBold10ptBitmaps + 888}, 		// k
	{6, s_FreeMonoBold10ptBitmaps + 900}, 		// l
	{8, s_FreeMonoBold10ptBitmaps + 912}, 		// m
	{7, s_FreeMonoBold10ptBitmaps + 924}, 		// n
	{6, s_FreeMonoBold10ptBitmaps + 936}, 		// o
	{7, s_FreeMonoBold10ptBitmaps + 948}, 		// p
	{7, s_FreeMonoBold10ptBitmaps + 960}, 		// q
	{6, s_FreeMonoBold10ptBitmaps + 972}, 		// r
	{5, s_FreeMonoBold10ptBitmaps + 984}, 		// s
	{6, s_FreeMonoBold10ptBitmaps + 996}, 		// t
	{7, s_FreeMonoBold10ptBitmaps + 1008}, 		// u
	{7, s_FreeMonoBold10ptBitmaps + 1020}, 		// v
	{8, s_FreeMonoBold10ptBitmaps + 1032}, 		// w
	{6, s_FreeMonoBold10ptBitmaps + 1044}, 		// x
	{8, s_FreeMonoBold10ptBitmaps + 1056}, 		// y
	{5, s_FreeMonoBold10ptBitmaps + 1068}, 		// z
	{3, s_FreeMonoBold10ptBitmaps + 1080}, 		// {
	{1, s_FreeMonoBold10ptBitmaps + 1092}, 		// |
	{3, s_FreeMonoBold10ptBitmaps + 1104}, 		// }
	{6, s_FreeMonoBold10ptBitmaps + 1116}, 		// ~
};

// Font information for FreeMono 10pt
const FontDesc_t iFontFreeMonoBold10pt = {
	FONT_TYPE_VAR_WIDTH,
	8,
	12,
	{.pCharDesc = s_FreeMonoBold10ptCharDesc }
};

