// 
//  Font data for FreeSerif Bold Italic 8pt
// 	https://savannah.gnu.org/projects/freefont/
// 
// 	Font bitmap generated by
// 	http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator

#include "display/ifont.h"

// Character bitmaps for FreeSerif 8pt
static const uint8_t s_FreeSerifBoldIta8ptBitmaps[] = {
	// @0 '!' (4 pixels wide)
	0x00, //     
	0x30, //   ##
	0x30, //   ##
	0x20, //   # 
	0x20, //   # 
	0x20, //   # 
	0x40, //  #  
	0x00, //     
	0xC0, // ##  
	0x00, //     
	0x00, //     

	// @11 '"' (5 pixels wide)
	0x00, //      
	0xD8, // ## ##
	0x90, // #  # 
	0x90, // #  # 
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @22 '#' (6 pixels wide)
	0x00, //       
	0x24, //   #  #
	0x28, //   # # 
	0x7C, //  #####
	0x50, //  # #  
	0x50, //  # #  
	0xF8, // ##### 
	0xA0, // # #   
	0xA0, // # #   
	0x00, //       
	0x00, //       

	// @33 '$' (6 pixels wide)
	0x00, //       
	0x30, //   ##  
	0x5C, //  # ###
	0x54, //  # # #
	0x60, //  ##   
	0x38, //   ### 
	0xAC, // # # ##
	0xAC, // # # ##
	0xB8, // # ### 
	0x40, //  #    
	0x00, //       

	// @44 '%' (9 pixels wide)
	0x00, 0x00, //          
	0x70, 0x00, //  ###     
	0xEE, 0x00, // ### ###  
	0xCC, 0x00, // ##  ##   
	0xDF, 0x80, // ## ######
	0x7A, 0x80, //  #### # #
	0x16, 0x80, //    # ## #
	0x16, 0x80, //    # ## #
	0x23, 0x00, //   #   ## 
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @66 '&' (7 pixels wide)
	0x00, //        
	0x0C, //     ## 
	0x1C, //    ### 
	0x1C, //    ### 
	0x18, //    ##  
	0x7E, //  ######
	0xCC, // ##  ## 
	0xC8, // ##  #  
	0x76, //  ### ##
	0x00, //        
	0x00, //        

	// @77 ''' (2 pixels wide)
	0x00, //   
	0xC0, // ##
	0x80, // # 
	0x80, // # 
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   

	// @88 '(' (3 pixels wide)
	0x00, //    
	0x00, //    
	0x60, //  ##
	0x40, //  # 
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x40, //  # 

	// @99 ')' (3 pixels wide)
	0x00, //    
	0x40, //  # 
	0x40, //  # 
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x60, //  ##
	0x40, //  # 
	0x80, // #  
	0x00, //    

	// @110 '*' (6 pixels wide)
	0x00, //       
	0x30, //   ##  
	0x30, //   ##  
	0xA8, // # # # 
	0xFC, // ######
	0x30, //   ##  
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @121 '+' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x20, //   #  
	0x20, //   #  
	0xF8, // #####
	0x20, //   #  
	0x20, //   #  
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
	0x00, //   
	0x40, //  #
	0x40, //  #
	0x80, // # 

	// @143 '-' (3 pixels wide)
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0xE0, // ###
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    

	// @154 '.' (2 pixels wide)
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

	// @165 '/' (4 pixels wide)
	0x00, //     
	0x10, //    #
	0x10, //    #
	0x20, //   # 
	0x20, //   # 
	0x40, //  #  
	0x40, //  #  
	0x40, //  #  
	0x80, // #   
	0x00, //     
	0x00, //     

	// @176 '0' (6 pixels wide)
	0x00, //       
	0x38, //   ### 
	0x6C, //  ## ##
	0x6C, //  ## ##
	0xEC, // ### ##
	0xCC, // ##  ##
	0xD8, // ## ## 
	0xD8, // ## ## 
	0x70, //  ###  
	0x00, //       
	0x00, //       

	// @187 '1' (4 pixels wide)
	0x00, //     
	0x10, //    #
	0x70, //  ###
	0x30, //   ##
	0x30, //   ##
	0x20, //   # 
	0x20, //   # 
	0x60, //  ## 
	0xF0, // ####
	0x00, //     
	0x00, //     

	// @198 '2' (5 pixels wide)
	0x00, //      
	0x70, //  ### 
	0x98, // #  ##
	0x18, //    ##
	0x10, //    # 
	0x30, //   ## 
	0x60, //  ##  
	0xD0, // ## # 
	0xF0, // #### 
	0x00, //      
	0x00, //      

	// @209 '3' (5 pixels wide)
	0x00, //      
	0x38, //   ###
	0x58, //  # ##
	0x10, //    # 
	0x70, //  ### 
	0x18, //    ##
	0x18, //    ##
	0x10, //    # 
	0xE0, // ###  
	0x00, //      
	0x00, //      

	// @220 '4' (6 pixels wide)
	0x00, //       
	0x04, //      #
	0x08, //     # 
	0x38, //   ### 
	0x58, //  # ## 
	0x98, // #  ## 
	0xF8, // ##### 
	0x10, //    #  
	0x30, //   ##  
	0x00, //       
	0x00, //       

	// @231 '5' (6 pixels wide)
	0x00, //       
	0x3C, //   ####
	0x40, //  #    
	0x60, //  ##   
	0x38, //   ### 
	0x18, //    ## 
	0x18, //    ## 
	0x10, //    #  
	0xE0, // ###   
	0x00, //       
	0x00, //       

	// @242 '6' (6 pixels wide)
	0x00, //       
	0x0C, //     ##
	0x18, //    ## 
	0x70, //  ###  
	0x78, //  #### 
	0xEC, // ### ##
	0xCC, // ##  ##
	0xDC, // ## ###
	0x78, //  #### 
	0x00, //       
	0x00, //       

	// @253 '7' (5 pixels wide)
	0x00, //      
	0x78, //  ####
	0x90, // #  # 
	0x10, //    # 
	0x20, //   #  
	0x20, //   #  
	0x40, //  #   
	0x40, //  #   
	0x80, // #    
	0x00, //      
	0x00, //      

	// @264 '8' (5 pixels wide)
	0x00, //      
	0x70, //  ### 
	0x68, //  ## #
	0x68, //  ## #
	0x30, //   ## 
	0x50, //  # # 
	0xB0, // # ## 
	0xB0, // # ## 
	0x60, //  ##  
	0x00, //      
	0x00, //      

	// @275 '9' (6 pixels wide)
	0x00, //       
	0x38, //   ### 
	0x6C, //  ## ##
	0x6C, //  ## ##
	0x6C, //  ## ##
	0x6C, //  ## ##
	0x38, //   ### 
	0x30, //   ##  
	0xC0, // ##    
	0x00, //       
	0x00, //       

	// @286 ':' (3 pixels wide)
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x60, //  ##
	0x00, //    
	0x00, //    
	0x00, //    
	0xC0, // ## 
	0x00, //    
	0x00, //    

	// @297 ';' (3 pixels wide)
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x60, //  ##
	0x00, //    
	0x00, //    
	0x00, //    
	0x40, //  # 
	0x40, //  # 
	0x80, // #  

	// @308 '<' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x04, //      #
	0x18, //    ## 
	0x60, //  ##   
	0xE0, // ###   
	0x18, //    ## 
	0x04, //      #
	0x00, //       
	0x00, //       

	// @319 '=' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0xF8, // #####
	0x00, //      
	0xF8, // #####
	0x00, //      
	0x00, //      
	0x00, //      

	// @330 '>' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0xE0, // ###  
	0x18, //    ##
	0x18, //    ##
	0x60, //  ##  
	0x80, // #    
	0x00, //      
	0x00, //      

	// @341 '?' (5 pixels wide)
	0x00, //      
	0x70, //  ### 
	0x58, //  # ##
	0x18, //    ##
	0x10, //    # 
	0x20, //   #  
	0x40, //  #   
	0x00, //      
	0xC0, // ##   
	0x00, //      
	0x00, //      

	// @352 '@' (8 pixels wide)
	0x00, //         
	0x3C, //   ####  
	0x62, //  ##   # 
	0xDD, // ## ### #
	0xF5, // #### # #
	0xE9, // ### #  #
	0xFE, // ####### 
	0x60, //  ##     
	0x38, //   ###   
	0x00, //         
	0x00, //         

	// @363 'A' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x0C, //     ## 
	0x1C, //    ### 
	0x2C, //   # ## 
	0x2C, //   # ## 
	0x7C, //  ##### 
	0x44, //  #   # 
	0xCE, // ##  ###
	0x00, //        
	0x00, //        

	// @374 'B' (8 pixels wide)
	0x00, //         
	0x3E, //   ##### 
	0x33, //   ##  ##
	0x33, //   ##  ##
	0x3C, //   ####  
	0x26, //   #  ## 
	0x66, //  ##  ## 
	0x6E, //  ## ### 
	0xFC, // ######  
	0x00, //         
	0x00, //         

	// @385 'C' (8 pixels wide)
	0x00, //         
	0x1F, //    #####
	0x71, //  ###   #
	0x61, //  ##    #
	0xC0, // ##      
	0xC0, // ##      
	0xC0, // ##      
	0xC4, // ##   #  
	0x78, //  ####   
	0x00, //         
	0x00, //         

	// @396 'D' (8 pixels wide)
	0x00, //         
	0x3E, //   ##### 
	0x33, //   ##  ##
	0x33, //   ##  ##
	0x23, //   #   ##
	0x23, //   #   ##
	0x67, //  ##  ###
	0x66, //  ##  ## 
	0xF8, // #####   
	0x00, //         
	0x00, //         

	// @407 'E' (9 pixels wide)
	0x00, 0x00, //          
	0x3F, 0x80, //   #######
	0x31, 0x00, //   ##   # 
	0x32, 0x00, //   ##  #  
	0x3E, 0x00, //   #####  
	0x24, 0x00, //   #  #   
	0x62, 0x00, //  ##   #  
	0x66, 0x00, //  ##  ##  
	0xFE, 0x00, // #######  
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @429 'F' (9 pixels wide)
	0x00, 0x00, //          
	0x3F, 0x80, //   #######
	0x31, 0x00, //   ##   # 
	0x32, 0x00, //   ##  #  
	0x3E, 0x00, //   #####  
	0x24, 0x00, //   #  #   
	0x60, 0x00, //  ##      
	0x60, 0x00, //  ##      
	0xE0, 0x00, // ###      
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @451 'G' (9 pixels wide)
	0x00, 0x00, //          
	0x1E, 0x80, //    #### #
	0x31, 0x00, //   ##   # 
	0x60, 0x00, //  ##      
	0xC0, 0x00, // ##       
	0xC7, 0x00, // ##   ### 
	0xC2, 0x00, // ##    #  
	0xC6, 0x00, // ##   ##  
	0x7C, 0x00, //  #####   
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @473 'H' (9 pixels wide)
	0x00, 0x00, //          
	0x3B, 0x80, //   ### ###
	0x31, 0x00, //   ##   # 
	0x33, 0x00, //   ##  ## 
	0x3F, 0x00, //   ###### 
	0x22, 0x00, //   #   #  
	0x62, 0x00, //  ##   #  
	0x66, 0x00, //  ##  ##  
	0xEF, 0x00, // ### #### 
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @495 'I' (5 pixels wide)
	0x00, //      
	0x38, //   ###
	0x30, //   ## 
	0x30, //   ## 
	0x20, //   #  
	0x20, //   #  
	0x60, //  ##  
	0x60, //  ##  
	0xE0, // ###  
	0x00, //      
	0x00, //      

	// @506 'J' (6 pixels wide)
	0x00, //       
	0x1C, //    ###
	0x08, //     # 
	0x08, //     # 
	0x18, //    ## 
	0x18, //    ## 
	0x10, //    #  
	0x10, //    #  
	0x30, //   ##  
	0xE0, // ###   
	0x00, //       

	// @517 'K' (8 pixels wide)
	0x00, //         
	0x3B, //   ### ##
	0x32, //   ##  # 
	0x34, //   ## #  
	0x28, //   # #   
	0x38, //   ###   
	0x6C, //  ## ##  
	0x6C, //  ## ##  
	0xEE, // ### ### 
	0x00, //         
	0x00, //         

	// @528 'L' (7 pixels wide)
	0x00, //        
	0x38, //   ###  
	0x30, //   ##   
	0x30, //   ##   
	0x20, //   #    
	0x20, //   #    
	0x62, //  ##   #
	0x66, //  ##  ##
	0xFC, // ###### 
	0x00, //        
	0x00, //        

	// @539 'M' (10 pixels wide)
	0x00, 0x00, //           
	0x30, 0xC0, //   ##    ##
	0x31, 0xC0, //   ##   ###
	0x32, 0x80, //   ##  # # 
	0x33, 0x80, //   ##  ### 
	0x3D, 0x80, //   #### ## 
	0x59, 0x00, //  # ##  #  
	0x59, 0x00, //  # ##  #  
	0xF7, 0x80, // #### #### 
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @561 'N' (8 pixels wide)
	0x00, //         
	0x33, //   ##  ##
	0x31, //   ##   #
	0x32, //   ##  # 
	0x2A, //   # # # 
	0x2A, //   # # # 
	0x46, //  #   ## 
	0x44, //  #   #  
	0xE4, // ###  #  
	0x00, //         
	0x00, //         

	// @572 'O' (8 pixels wide)
	0x00, //         
	0x1E, //    #### 
	0x33, //   ##  ##
	0x63, //  ##   ##
	0xE3, // ###   ##
	0xC7, // ##   ###
	0xC6, // ##   ## 
	0xCC, // ##  ##  
	0x78, //  ####   
	0x00, //         
	0x00, //         

	// @583 'P' (7 pixels wide)
	0x00, //        
	0x3C, //   #### 
	0x36, //   ## ##
	0x36, //   ## ##
	0x26, //   #  ##
	0x3C, //   #### 
	0x60, //  ##    
	0x60, //  ##    
	0xE0, // ###    
	0x00, //        
	0x00, //        

	// @594 'Q' (8 pixels wide)
	0x00, //         
	0x1E, //    #### 
	0x33, //   ##  ##
	0x63, //  ##   ##
	0xE3, // ###   ##
	0xC3, // ##    ##
	0xC6, // ##   ## 
	0xC4, // ##   #  
	0x68, //  ## #   
	0x32, //   ##  # 
	0xFC, // ######  

	// @605 'R' (8 pixels wide)
	0x00, //         
	0x3E, //   ##### 
	0x33, //   ##  ##
	0x33, //   ##  ##
	0x3C, //   ####  
	0x2C, //   # ##  
	0x6C, //  ## ##  
	0x66, //  ##  ## 
	0xE7, // ###  ###
	0x00, //         
	0x00, //         

	// @616 'S' (6 pixels wide)
	0x00, //       
	0x30, //   ##  
	0x6C, //  ## ##
	0x64, //  ##  #
	0x30, //   ##  
	0x1C, //    ###
	0x0C, //     ##
	0x8C, // #   ##
	0x78, //  #### 
	0x00, //       
	0x00, //       

	// @627 'T' (8 pixels wide)
	0x00, //         
	0x7F, //  #######
	0xD2, // ## #  # 
	0x32, //   ##  # 
	0x30, //   ##    
	0x20, //   #     
	0x20, //   #     
	0x60, //  ##     
	0xF0, // ####    
	0x00, //         
	0x00, //         

	// @638 'U' (8 pixels wide)
	0x00, //         
	0xF7, // #### ###
	0x62, //  ##   # 
	0x62, //  ##   # 
	0x42, //  #    # 
	0x44, //  #   #  
	0xC4, // ##   #  
	0xC4, // ##   #  
	0x78, //  ####   
	0x00, //         
	0x00, //         

	// @649 'V' (8 pixels wide)
	0x00, //         
	0xF3, // ####  ##
	0x62, //  ##   # 
	0x64, //  ##  #  
	0x24, //   #  #  
	0x28, //   # #   
	0x30, //   ##    
	0x30, //   ##    
	0x20, //   #     
	0x00, //         
	0x00, //         

	// @660 'W' (10 pixels wide)
	0x00, 0x00, //           
	0xFF, 0xC0, // ##########
	0x66, 0x80, //  ##  ## # 
	0x66, 0x80, //  ##  ## # 
	0x2F, 0x00, //   # ####  
	0x37, 0x00, //   ## ###  
	0x37, 0x00, //   ## ###  
	0x26, 0x00, //   #  ##   
	0x22, 0x00, //   #   #   
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @682 'X' (8 pixels wide)
	0x00, //         
	0x7B, //  #### ##
	0x32, //   ##  # 
	0x14, //    # #  
	0x18, //    ##   
	0x18, //    ##   
	0x28, //   # #   
	0x48, //  #  #   
	0xDE, // ## #### 
	0x00, //         
	0x00, //         

	// @693 'Y' (7 pixels wide)
	0x00, //        
	0xE6, // ###  ##
	0x64, //  ##  # 
	0x28, //   # #  
	0x28, //   # #  
	0x30, //   ##   
	0x30, //   ##   
	0x20, //   #    
	0xF0, // ####   
	0x00, //        
	0x00, //        

	// @704 'Z' (7 pixels wide)
	0x00, //        
	0x3E, //   #####
	0x66, //  ##  ##
	0x0C, //     ## 
	0x18, //    ##  
	0x38, //   ###  
	0x32, //   ##  #
	0x64, //  ##  # 
	0xFC, // ###### 
	0x00, //        
	0x00, //        

	// @715 '[' (4 pixels wide)
	0x00, //     
	0x30, //   ##
	0x60, //  ## 
	0x40, //  #  
	0x40, //  #  
	0x40, //  #  
	0x40, //  #  
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0xC0, // ##  

	// @726 '\' (3 pixels wide)
	0x00, //    
	0x80, // #  
	0x80, // #  
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x20, //   #
	0x20, //   #
	0x00, //    
	0x00, //    

	// @737 ']' (5 pixels wide)
	0x00, //      
	0x38, //   ###
	0x10, //    # 
	0x10, //    # 
	0x10, //    # 
	0x10, //    # 
	0x30, //   ## 
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0xE0, // ###  

	// @748 '^' (4 pixels wide)
	0x00, //     
	0x60, //  ## 
	0x60, //  ## 
	0x90, // #  #
	0x90, // #  #
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     

	// @759 '_' (6 pixels wide)
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
	0xFC, // ######

	// @770 '`' (1 pixels wide)
	0x00, //  
	0x00, //  
	0x80, // #
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  

	// @781 'a' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x38, //   ### 
	0x68, //  ## # 
	0xD8, // ## ## 
	0xD8, // ## ## 
	0xFC, // ######
	0x00, //       
	0x00, //       

	// @792 'b' (5 pixels wide)
	0x00, //      
	0xE0, // ###  
	0x40, //  #   
	0x40, //  #   
	0x78, //  ####
	0x58, //  # ##
	0xD8, // ## ##
	0xB0, // # ## 
	0xE0, // ###  
	0x00, //      
	0x00, //      

	// @803 'c' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x38, //   ###
	0x60, //  ##  
	0xC0, // ##   
	0xD0, // ## # 
	0x70, //  ### 
	0x00, //      
	0x00, //      

	// @814 'd' (6 pixels wide)
	0x00, //       
	0x0C, //     ##
	0x0C, //     ##
	0x0C, //     ##
	0x38, //   ### 
	0x68, //  ## # 
	0xD8, // ## ## 
	0xD8, // ## ## 
	0xFC, // ######
	0x00, //       
	0x00, //       

	// @825 'e' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x38, //   ###
	0x68, //  ## #
	0xD0, // ## # 
	0xF0, // #### 
	0x70, //  ### 
	0x00, //      
	0x00, //      

	// @836 'f' (7 pixels wide)
	0x06, //      ##
	0x08, //     #  
	0x08, //     #  
	0x18, //    ##  
	0x3C, //   #### 
	0x10, //    #   
	0x30, //   ##   
	0x20, //   #    
	0x20, //   #    
	0x20, //   #    
	0xC0, // ##     

	// @847 'g' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3E, //   #####
	0x6C, //  ## ## 
	0x18, //    ##  
	0x60, //  ##    
	0x7C, //  ##### 
	0x8C, // #   ## 
	0x78, //  ####  

	// @858 'h' (6 pixels wide)
	0x00, //       
	0x70, //  ###  
	0x20, //   #   
	0x20, //   #   
	0x6C, //  ## ##
	0x7C, //  #####
	0x48, //  #  # 
	0x58, //  # ## 
	0xDC, // ## ###
	0x00, //       
	0x00, //       

	// @869 'i' (4 pixels wide)
	0x00, //     
	0x30, //   ##
	0x00, //     
	0x00, //     
	0xE0, // ### 
	0x40, //  #  
	0x40, //  #  
	0x50, //  # #
	0xE0, // ### 
	0x00, //     
	0x00, //     

	// @880 'j' (6 pixels wide)
	0x00, //       
	0x0C, //     ##
	0x00, //       
	0x00, //       
	0x18, //    ## 
	0x18, //    ## 
	0x10, //    #  
	0x10, //    #  
	0x30, //   ##  
	0x20, //   #   
	0xE0, // ###   

	// @891 'k' (6 pixels wide)
	0x00, //       
	0x70, //  ###  
	0x20, //   #   
	0x20, //   #   
	0x7C, //  #####
	0x50, //  # #  
	0x70, //  ###  
	0x50, //  # #  
	0xD8, // ## ## 
	0x00, //       
	0x00, //       

	// @902 'l' (4 pixels wide)
	0x00, //     
	0x70, //  ###
	0x20, //   # 
	0x20, //   # 
	0x60, //  ## 
	0x40, //  #  
	0x40, //  #  
	0xC0, // ##  
	0xE0, // ### 
	0x00, //     
	0x00, //     

	// @913 'm' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x6D, 0x80, //  ## ## ##
	0x7F, 0x80, //  ########
	0x4B, 0x00, //  #  # ## 
	0x5B, 0x00, //  # ## ## 
	0xD3, 0x80, // ## #  ###
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @935 'n' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x7C, //  #####
	0x6C, //  ## ##
	0x48, //  #  # 
	0xD8, // ## ## 
	0xDC, // ## ###
	0x00, //       
	0x00, //       

	// @946 'o' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x38, //   ### 
	0x6C, //  ## ##
	0xCC, // ##  ##
	0xD8, // ## ## 
	0x70, //  ###  
	0x00, //       
	0x00, //       

	// @957 'p' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3E, //   #####
	0x36, //   ## ##
	0x26, //   #  ##
	0x2C, //   # ## 
	0x78, //  ####  
	0x40, //  #     
	0xE0, // ###    

	// @968 'q' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x38, //   ###
	0x68, //  ## #
	0xD8, // ## ##
	0xD0, // ## # 
	0xF0, // #### 
	0x10, //    # 
	0x30, //   ## 

	// @979 'r' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x78, //  ####
	0x60, //  ##  
	0x40, //  #   
	0x40, //  #   
	0xC0, // ##   
	0x00, //      
	0x00, //      

	// @990 's' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x70, //  ###
	0x50, //  # #
	0x60, //  ## 
	0xA0, // # # 
	0x60, //  ## 
	0x00, //     
	0x00, //     

	// @1001 't' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x10, //    #
	0x20, //   # 
	0x70, //  ###
	0x40, //  #  
	0x40, //  #  
	0xC0, // ##  
	0xE0, // ### 
	0x00, //     
	0x00, //     

	// @1012 'u' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xE8, // ### # 
	0x48, //  #  # 
	0x58, //  # ## 
	0xD8, // ## ## 
	0xFC, // ######
	0x00, //       
	0x00, //       

	// @1023 'v' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0xE8, // ### #
	0x68, //  ## #
	0x70, //  ### 
	0x60, //  ##  
	0x40, //  #   
	0x00, //      
	0x00, //      

	// @1034 'w' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0xEA, // ### # #
	0x6A, //  ## # #
	0x7C, //  ##### 
	0x6C, //  ## ## 
	0x48, //  #  #  
	0x00, //        
	0x00, //        

	// @1045 'x' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x68, //  ## #
	0x30, //   ## 
	0x20, //   #  
	0xA0, // # #  
	0xF0, // #### 
	0x00, //      
	0x00, //      

	// @1056 'y' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x24, //   #  #
	0x34, //   ## #
	0x38, //   ### 
	0x18, //    ## 
	0x10, //    #  
	0x20, //   #   
	0xC0, // ##    

	// @1067 'z' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x70, //  ###
	0xA0, // # # 
	0x40, //  #  
	0x80, // #   
	0xC0, // ##  
	0x60, //  ## 
	0x00, //     

	// @1078 '{' (5 pixels wide)
	0x18, //    ##
	0x30, //   ## 
	0x20, //   #  
	0x20, //   #  
	0x60, //  ##  
	0xC0, // ##   
	0x40, //  #   
	0x40, //  #   
	0xC0, // ##   
	0xC0, // ##   
	0x60, //  ##  

	// @1089 '|' (1 pixels wide)
	0x00, //  
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

	// @1100 '}' (5 pixels wide)
	0x30, //   ## 
	0x18, //    ##
	0x10, //    # 
	0x10, //    # 
	0x10, //    # 
	0x30, //   ## 
	0x20, //   #  
	0x20, //   #  
	0x60, //  ##  
	0x40, //  #   
	0xC0, // ##   

	// @1111 '~' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0xC0, // ##  
	0x30, //   ##
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
};

// Character descriptors for FreeSerif 8pt
// { [Char width in bits], [Offset into freeSerif_8ptCharBitmaps in bytes] }
static const CharDesc_t s_FreeSerifBoldIta8ptCharDesc[] = {
	{4, s_FreeSerifBoldIta8ptBitmaps + 0}, 		// !
	{5, s_FreeSerifBoldIta8ptBitmaps + 11}, 		// "
	{6, s_FreeSerifBoldIta8ptBitmaps + 22}, 		// #
	{6, s_FreeSerifBoldIta8ptBitmaps + 33}, 		// $
	{9, s_FreeSerifBoldIta8ptBitmaps + 44}, 		// %
	{7, s_FreeSerifBoldIta8ptBitmaps + 66}, 		// &
	{2, s_FreeSerifBoldIta8ptBitmaps + 77}, 		// '
	{3, s_FreeSerifBoldIta8ptBitmaps + 88}, 		// (
	{3, s_FreeSerifBoldIta8ptBitmaps + 99}, 		// )
	{6, s_FreeSerifBoldIta8ptBitmaps + 110}, 		// *
	{5, s_FreeSerifBoldIta8ptBitmaps + 121}, 		// +
	{2, s_FreeSerifBoldIta8ptBitmaps + 132}, 		// ,
	{3, s_FreeSerifBoldIta8ptBitmaps + 143}, 		// -
	{2, s_FreeSerifBoldIta8ptBitmaps + 154}, 		// .
	{4, s_FreeSerifBoldIta8ptBitmaps + 165}, 		// /
	{6, s_FreeSerifBoldIta8ptBitmaps + 176}, 		// 0
	{4, s_FreeSerifBoldIta8ptBitmaps + 187}, 		// 1
	{5, s_FreeSerifBoldIta8ptBitmaps + 198}, 		// 2
	{5, s_FreeSerifBoldIta8ptBitmaps + 209}, 		// 3
	{6, s_FreeSerifBoldIta8ptBitmaps + 220}, 		// 4
	{6, s_FreeSerifBoldIta8ptBitmaps + 231}, 		// 5
	{6, s_FreeSerifBoldIta8ptBitmaps + 242}, 		// 6
	{5, s_FreeSerifBoldIta8ptBitmaps + 253}, 		// 7
	{5, s_FreeSerifBoldIta8ptBitmaps + 264}, 		// 8
	{6, s_FreeSerifBoldIta8ptBitmaps + 275}, 		// 9
	{3, s_FreeSerifBoldIta8ptBitmaps + 286}, 		// :
	{3, s_FreeSerifBoldIta8ptBitmaps + 297}, 		// ;
	{6, s_FreeSerifBoldIta8ptBitmaps + 308}, 		// <
	{5, s_FreeSerifBoldIta8ptBitmaps + 319}, 		// =
	{5, s_FreeSerifBoldIta8ptBitmaps + 330}, 		// >
	{5, s_FreeSerifBoldIta8ptBitmaps + 341}, 		// ?
	{8, s_FreeSerifBoldIta8ptBitmaps + 352}, 		// @
	{7, s_FreeSerifBoldIta8ptBitmaps + 363}, 		// A
	{8, s_FreeSerifBoldIta8ptBitmaps + 374}, 		// B
	{8, s_FreeSerifBoldIta8ptBitmaps + 385}, 		// C
	{8, s_FreeSerifBoldIta8ptBitmaps + 396}, 		// D
	{9, s_FreeSerifBoldIta8ptBitmaps + 407}, 		// E
	{9, s_FreeSerifBoldIta8ptBitmaps + 429}, 		// F
	{9, s_FreeSerifBoldIta8ptBitmaps + 451}, 		// G
	{9, s_FreeSerifBoldIta8ptBitmaps + 473}, 		// H
	{5, s_FreeSerifBoldIta8ptBitmaps + 495}, 		// I
	{6, s_FreeSerifBoldIta8ptBitmaps + 506}, 		// J
	{8, s_FreeSerifBoldIta8ptBitmaps + 517}, 		// K
	{7, s_FreeSerifBoldIta8ptBitmaps + 528}, 		// L
	{10,s_FreeSerifBoldIta8ptBitmaps +  539}, 		// M
	{8, s_FreeSerifBoldIta8ptBitmaps + 561}, 		// N
	{8, s_FreeSerifBoldIta8ptBitmaps + 572}, 		// O
	{7, s_FreeSerifBoldIta8ptBitmaps + 583}, 		// P
	{8, s_FreeSerifBoldIta8ptBitmaps + 594}, 		// Q
	{8, s_FreeSerifBoldIta8ptBitmaps + 605}, 		// R
	{6, s_FreeSerifBoldIta8ptBitmaps + 616}, 		// S
	{8, s_FreeSerifBoldIta8ptBitmaps + 627}, 		// T
	{8, s_FreeSerifBoldIta8ptBitmaps + 638}, 		// U
	{8, s_FreeSerifBoldIta8ptBitmaps + 649}, 		// V
	{10,s_FreeSerifBoldIta8ptBitmaps +  660}, 		// W
	{8, s_FreeSerifBoldIta8ptBitmaps + 682}, 		// X
	{7, s_FreeSerifBoldIta8ptBitmaps + 693}, 		// Y
	{7, s_FreeSerifBoldIta8ptBitmaps + 704}, 		// Z
	{4, s_FreeSerifBoldIta8ptBitmaps + 715}, 		// [
	{3, s_FreeSerifBoldIta8ptBitmaps + 726}, 		// '\'
	{5, s_FreeSerifBoldIta8ptBitmaps + 737}, 		// ]
	{4, s_FreeSerifBoldIta8ptBitmaps + 748}, 		// ^
	{6, s_FreeSerifBoldIta8ptBitmaps + 759}, 		// _
	{1, s_FreeSerifBoldIta8ptBitmaps + 770}, 		// `
	{6, s_FreeSerifBoldIta8ptBitmaps + 781}, 		// a
	{5, s_FreeSerifBoldIta8ptBitmaps + 792}, 		// b
	{5, s_FreeSerifBoldIta8ptBitmaps + 803}, 		// c
	{6, s_FreeSerifBoldIta8ptBitmaps + 814}, 		// d
	{5, s_FreeSerifBoldIta8ptBitmaps + 825}, 		// e
	{7, s_FreeSerifBoldIta8ptBitmaps + 836}, 		// f
	{7, s_FreeSerifBoldIta8ptBitmaps + 847}, 		// g
	{6, s_FreeSerifBoldIta8ptBitmaps + 858}, 		// h
	{4, s_FreeSerifBoldIta8ptBitmaps + 869}, 		// i
	{6, s_FreeSerifBoldIta8ptBitmaps + 880}, 		// j
	{6, s_FreeSerifBoldIta8ptBitmaps + 891}, 		// k
	{4, s_FreeSerifBoldIta8ptBitmaps + 902}, 		// l
	{9, s_FreeSerifBoldIta8ptBitmaps + 913}, 		// m
	{6, s_FreeSerifBoldIta8ptBitmaps + 935}, 		// n
	{6, s_FreeSerifBoldIta8ptBitmaps + 946}, 		// o
	{7, s_FreeSerifBoldIta8ptBitmaps + 957}, 		// p
	{5, s_FreeSerifBoldIta8ptBitmaps + 968}, 		// q
	{5, s_FreeSerifBoldIta8ptBitmaps + 979}, 		// r
	{4, s_FreeSerifBoldIta8ptBitmaps + 990}, 		// s
	{4, s_FreeSerifBoldIta8ptBitmaps + 1001}, 		// t
	{6, s_FreeSerifBoldIta8ptBitmaps + 1012}, 		// u
	{5, s_FreeSerifBoldIta8ptBitmaps + 1023}, 		// v
	{7, s_FreeSerifBoldIta8ptBitmaps + 1034}, 		// w
	{5, s_FreeSerifBoldIta8ptBitmaps + 1045}, 		// x
	{6, s_FreeSerifBoldIta8ptBitmaps + 1056}, 		// y
	{4, s_FreeSerifBoldIta8ptBitmaps + 1067}, 		// z
	{5, s_FreeSerifBoldIta8ptBitmaps + 1078}, 		// {
	{1, s_FreeSerifBoldIta8ptBitmaps + 1089}, 		// |
	{5, s_FreeSerifBoldIta8ptBitmaps + 1100}, 		// }
	{4, s_FreeSerifBoldIta8ptBitmaps + 1111}, 		// ~
};

// Font information for FreeSerif 8pt
const FontDesc_t iFontFreeSerifBoldIta8pt = {
	0,
	10,
	11,
	{ .pCharDesc = s_FreeSerifBoldIta8ptCharDesc }
};

