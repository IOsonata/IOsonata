// 
//  Font data for FreeSans 8pt
// 	https://savannah.gnu.org/projects/freefont/
// 
// 	Font bitmap generated by
// 	http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator

#include "display/ifont.h"

// Character bitmaps for FreeSans 8pt
static const uint8_t s_FreeSans8ptBitmaps[] = {
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

	// @11 '"' (3 pixels wide)
	0x00, //    
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

	// @22 '#' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x28, //   # # 
	0x7C, //  #####
	0x28, //   # # 
	0x48, //  #  # 
	0xFC, // ######
	0x50, //  # #  
	0x50, //  # #  
	0x00, //       
	0x00, //       

	// @33 '$' (5 pixels wide)
	0x20, //   #  
	0x70, //  ### 
	0xA8, // # # #
	0xA0, // # #  
	0x70, //  ### 
	0x28, //   # #
	0xA8, // # # #
	0xA8, // # # #
	0x70, //  ### 
	0x20, //   #  
	0x00, //      

	// @44 '%' (10 pixels wide)
	0x00, 0x00, //           
	0x61, 0x00, //  ##    #  
	0x92, 0x00, // #  #  #   
	0x94, 0x00, // #  # #    
	0x64, 0x00, //  ##  #    
	0x09, 0x80, //     #  ## 
	0x12, 0x40, //    #  #  #
	0x12, 0x40, //    #  #  #
	0x21, 0x80, //   #    ## 
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @66 '&' (5 pixels wide)
	0x00, //      
	0x20, //   #  
	0x50, //  # # 
	0x50, //  # # 
	0x20, //   #  
	0xE8, // ### #
	0x98, // #  ##
	0x98, // #  ##
	0x68, //  ## #
	0x00, //      
	0x00, //      

	// @77 ''' (1 pixels wide)
	0x00, //  
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

	// @88 '(' (2 pixels wide)
	0x00, //   
	0x40, //  #
	0x40, //  #
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x40, //  #
	0x40, //  #

	// @99 ')' (3 pixels wide)
	0x00, //    
	0x80, // #  
	0x40, //  # 
	0x40, //  # 
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x40, //  # 
	0x40, //  # 

	// @110 '*' (3 pixels wide)
	0x00, //    
	0x40, //  # 
	0xE0, // ###
	0xA0, // # #
	0x00, //    
	0x00, //    
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

	// @132 ',' (1 pixels wide)
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

	// @143 '-' (2 pixels wide)
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

	// @154 '.' (1 pixels wide)
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

	// @165 '/' (3 pixels wide)
	0x00, //    
	0x20, //   #
	0x20, //   #
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x80, // #  
	0x80, // #  
	0x00, //    
	0x00, //    

	// @176 '0' (6 pixels wide)
	0x00, //       
	0x78, //  #### 
	0xCC, // ##  ##
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0xCC, // ##  ##
	0x78, //  #### 
	0x00, //       
	0x00, //       

	// @187 '1' (2 pixels wide)
	0x00, //   
	0x40, //  #
	0xC0, // ##
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x00, //   
	0x00, //   

	// @198 '2' (5 pixels wide)
	0x00, //      
	0x70, //  ### 
	0x88, // #   #
	0x08, //     #
	0x08, //     #
	0x30, //   ## 
	0x40, //  #   
	0x80, // #    
	0xF8, // #####
	0x00, //      
	0x00, //      

	// @209 '3' (5 pixels wide)
	0x00, //      
	0x70, //  ### 
	0x88, // #   #
	0x08, //     #
	0x30, //   ## 
	0x08, //     #
	0x08, //     #
	0x88, // #   #
	0x70, //  ### 
	0x00, //      
	0x00, //      

	// @220 '4' (5 pixels wide)
	0x00, //      
	0x10, //    # 
	0x30, //   ## 
	0x50, //  # # 
	0x50, //  # # 
	0x90, // #  # 
	0xF8, // #####
	0x10, //    # 
	0x10, //    # 
	0x00, //      
	0x00, //      

	// @231 '5' (6 pixels wide)
	0x00, //       
	0x7C, //  #####
	0x40, //  #    
	0x78, //  #### 
	0x4C, //  #  ##
	0x04, //      #
	0x04, //      #
	0x8C, // #   ##
	0x78, //  #### 
	0x00, //       
	0x00, //       

	// @242 '6' (6 pixels wide)
	0x00, //       
	0x78, //  #### 
	0x44, //  #   #
	0x80, // #     
	0xF8, // ##### 
	0x84, // #    #
	0x84, // #    #
	0xC4, // ##   #
	0x78, //  #### 
	0x00, //       
	0x00, //       

	// @253 '7' (5 pixels wide)
	0x00, //      
	0xF8, // #####
	0x08, //     #
	0x10, //    # 
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x40, //  #   
	0x40, //  #   
	0x00, //      
	0x00, //      

	// @264 '8' (6 pixels wide)
	0x00, //       
	0x78, //  #### 
	0x84, // #    #
	0x84, // #    #
	0x78, //  #### 
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x78, //  #### 
	0x00, //       
	0x00, //       

	// @275 '9' (6 pixels wide)
	0x00, //       
	0x78, //  #### 
	0x8C, // #   ##
	0x84, // #    #
	0x84, // #    #
	0x7C, //  #####
	0x04, //      #
	0x88, // #   # 
	0x70, //  ###  
	0x00, //       
	0x00, //       

	// @286 ':' (1 pixels wide)
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

	// @297 ';' (1 pixels wide)
	0x00, //  
	0x00, //  
	0x00, //  
	0x80, // #
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  
	0x80, // #
	0x80, // #
	0x80, // #

	// @308 '<' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x0C, //     ##
	0x30, //   ##  
	0xC0, // ##    
	0x30, //   ##  
	0x0C, //     ##
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
	0x80, // #    
	0x70, //  ### 
	0x08, //     #
	0x70, //  ### 
	0x80, // #    
	0x00, //      
	0x00, //      

	// @341 '?' (5 pixels wide)
	0x00, //      
	0x70, //  ### 
	0x88, // #   #
	0x08, //     #
	0x18, //    ##
	0x20, //   #  
	0x20, //   #  
	0x00, //      
	0x20, //   #  
	0x00, //      
	0x00, //      

	// @352 '@' (10 pixels wide)
	0x00, 0x00, //           
	0x1F, 0x00, //    #####  
	0x21, 0x80, //   #    ## 
	0x4F, 0x40, //  #  #### #
	0x9A, 0x40, // #  ## #  #
	0x92, 0x40, // #  #  #  #
	0x92, 0xC0, // #  #  # ##
	0x8F, 0x80, // #   ##### 
	0x40, 0x00, //  #        
	0x20, 0x00, //   #       
	0x1E, 0x00, //    ####   

	// @374 'A' (7 pixels wide)
	0x00, //        
	0x18, //    ##  
	0x38, //   ###  
	0x28, //   # #  
	0x2C, //   # ## 
	0x64, //  ##  # 
	0x7C, //  ##### 
	0x46, //  #   ##
	0x82, // #     #
	0x00, //        
	0x00, //        

	// @385 'B' (6 pixels wide)
	0x00, //       
	0xF8, // ##### 
	0x84, // #    #
	0x84, // #    #
	0xF8, // ##### 
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0xF8, // ##### 
	0x00, //       
	0x00, //       

	// @396 'C' (7 pixels wide)
	0x00, //        
	0x3C, //   #### 
	0x42, //  #    #
	0x80, // #      
	0x80, // #      
	0x80, // #      
	0x82, // #     #
	0x46, //  #   ##
	0x3C, //   #### 
	0x00, //        
	0x00, //        

	// @407 'D' (6 pixels wide)
	0x00, //       
	0xF8, // ##### 
	0x88, // #   # 
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x88, // #   # 
	0xF8, // ##### 
	0x00, //       
	0x00, //       

	// @418 'E' (5 pixels wide)
	0x00, //      
	0xF8, // #####
	0x80, // #    
	0x80, // #    
	0xF8, // #####
	0x80, // #    
	0x80, // #    
	0x80, // #    
	0xF8, // #####
	0x00, //      
	0x00, //      

	// @429 'F' (5 pixels wide)
	0x00, //      
	0xF8, // #####
	0x80, // #    
	0x80, // #    
	0xF0, // #### 
	0x80, // #    
	0x80, // #    
	0x80, // #    
	0x80, // #    
	0x00, //      
	0x00, //      

	// @440 'G' (8 pixels wide)
	0x00, //         
	0x3E, //   ##### 
	0x41, //  #     #
	0x80, // #       
	0x80, // #       
	0x87, // #    ###
	0x81, // #      #
	0x43, //  #    ##
	0x3D, //   #### #
	0x00, //         
	0x00, //         

	// @451 'H' (6 pixels wide)
	0x00, //       
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0xFC, // ######
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x00, //       
	0x00, //       

	// @462 'I' (1 pixels wide)
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

	// @473 'J' (5 pixels wide)
	0x00, //      
	0x08, //     #
	0x08, //     #
	0x08, //     #
	0x08, //     #
	0x08, //     #
	0x08, //     #
	0x88, // #   #
	0x70, //  ### 
	0x00, //      
	0x00, //      

	// @484 'K' (6 pixels wide)
	0x00, //       
	0x8C, // #   ##
	0x88, // #   # 
	0x90, // #  #  
	0xA0, // # #   
	0xD0, // ## #  
	0x98, // #  ## 
	0x88, // #   # 
	0x84, // #    #
	0x00, //       
	0x00, //       

	// @495 'L' (4 pixels wide)
	0x00, //     
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0xF0, // ####
	0x00, //     
	0x00, //     

	// @506 'M' (7 pixels wide)
	0x00, //        
	0xC6, // ##   ##
	0xC6, // ##   ##
	0xC6, // ##   ##
	0xCA, // ##  # #
	0xAA, // # # # #
	0xAA, // # # # #
	0xB2, // # ##  #
	0x92, // #  #  #
	0x00, //        
	0x00, //        

	// @517 'N' (6 pixels wide)
	0x00, //       
	0x84, // #    #
	0xC4, // ##   #
	0xA4, // # #  #
	0xA4, // # #  #
	0x94, // #  # #
	0x94, // #  # #
	0x8C, // #   ##
	0x84, // #    #
	0x00, //       
	0x00, //       

	// @528 'O' (8 pixels wide)
	0x00, //         
	0x3C, //   ####  
	0x42, //  #    # 
	0x81, // #      #
	0x81, // #      #
	0x81, // #      #
	0x81, // #      #
	0x42, //  #    # 
	0x3C, //   ####  
	0x00, //         
	0x00, //         

	// @539 'P' (6 pixels wide)
	0x00, //       
	0xF8, // ##### 
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0xF8, // ##### 
	0x80, // #     
	0x80, // #     
	0x80, // #     
	0x00, //       
	0x00, //       

	// @550 'Q' (8 pixels wide)
	0x00, //         
	0x3C, //   ####  
	0x42, //  #    # 
	0x81, // #      #
	0x81, // #      #
	0x81, // #      #
	0x81, // #      #
	0x46, //  #   ## 
	0x3E, //   ##### 
	0x01, //        #
	0x00, //         

	// @561 'R' (6 pixels wide)
	0x00, //       
	0xF8, // ##### 
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0xF8, // ##### 
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x00, //       
	0x00, //       

	// @572 'S' (6 pixels wide)
	0x00, //       
	0x78, //  #### 
	0x84, // #    #
	0x80, // #     
	0xE0, // ###   
	0x1C, //    ###
	0x04, //      #
	0x84, // #    #
	0x78, //  #### 
	0x00, //       
	0x00, //       

	// @583 'T' (5 pixels wide)
	0x00, //      
	0xF8, // #####
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x00, //      
	0x00, //      

	// @594 'U' (6 pixels wide)
	0x00, //       
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x78, //  #### 
	0x00, //       
	0x00, //       

	// @605 'V' (7 pixels wide)
	0x00, //        
	0x82, // #     #
	0x44, //  #   # 
	0x44, //  #   # 
	0x44, //  #   # 
	0x28, //   # #  
	0x28, //   # #  
	0x18, //    ##  
	0x10, //    #   
	0x00, //        
	0x00, //        

	// @616 'W' (10 pixels wide)
	0x00, 0x00, //           
	0x8C, 0x40, // #   ##   #
	0x4C, 0x40, //  #  ##   #
	0x4C, 0x80, //  #  ##  # 
	0x4A, 0x80, //  #  # # # 
	0x52, 0x80, //  # #  # # 
	0x32, 0x80, //   ##  # # 
	0x31, 0x00, //   ##   #  
	0x31, 0x00, //   ##   #  
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @638 'X' (6 pixels wide)
	0x00, //       
	0x8C, // #   ##
	0xC8, // ##  # 
	0x50, //  # #  
	0x20, //   #   
	0x30, //   ##  
	0x50, //  # #  
	0x88, // #   # 
	0x8C, // #   ##
	0x00, //       
	0x00, //       

	// @649 'Y' (7 pixels wide)
	0x00, //        
	0x82, // #     #
	0x44, //  #   # 
	0x28, //   # #  
	0x28, //   # #  
	0x10, //    #   
	0x10, //    #   
	0x10, //    #   
	0x10, //    #   
	0x00, //        
	0x00, //        

	// @660 'Z' (6 pixels wide)
	0x00, //       
	0x7C, //  #####
	0x04, //      #
	0x08, //     # 
	0x10, //    #  
	0x30, //   ##  
	0x20, //   #   
	0x40, //  #    
	0xFC, // ######
	0x00, //       
	0x00, //       

	// @671 '[' (2 pixels wide)
	0x00, //   
	0xC0, // ##
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0xC0, // ##

	// @682 '\' (3 pixels wide)
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

	// @693 ']' (2 pixels wide)
	0x00, //   
	0xC0, // ##
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0xC0, // ##

	// @704 '^' (3 pixels wide)
	0x00, //    
	0x40, //  # 
	0xA0, // # #
	0xA0, // # #
	0xA0, // # #
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    

	// @715 '_' (6 pixels wide)
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

	// @726 '`' (1 pixels wide)
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
	0x00, //  

	// @737 'a' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x70, //  ###  
	0x88, // #   # 
	0x08, //     # 
	0xF8, // ##### 
	0x88, // #   # 
	0xFC, // ######
	0x00, //       
	0x00, //       

	// @748 'b' (5 pixels wide)
	0x00, //      
	0x80, // #    
	0x80, // #    
	0xF0, // #### 
	0x88, // #   #
	0x88, // #   #
	0x88, // #   #
	0x88, // #   #
	0xF0, // #### 
	0x00, //      
	0x00, //      

	// @759 'c' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x70, //  ### 
	0x88, // #   #
	0x80, // #    
	0x80, // #    
	0x88, // #   #
	0x70, //  ### 
	0x00, //      
	0x00, //      

	// @770 'd' (6 pixels wide)
	0x00, //       
	0x04, //      #
	0x04, //      #
	0x7C, //  #####
	0xCC, // ##  ##
	0x84, // #    #
	0x84, // #    #
	0xCC, // ##  ##
	0x74, //  ### #
	0x00, //       
	0x00, //       

	// @781 'e' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x78, //  #### 
	0x84, // #    #
	0xFC, // ######
	0x80, // #     
	0xC4, // ##   #
	0x78, //  #### 
	0x00, //       
	0x00, //       

	// @792 'f' (3 pixels wide)
	0x00, //    
	0x60, //  ##
	0x40, //  # 
	0xE0, // ###
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x00, //    
	0x00, //    

	// @803 'g' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x78, //  ####
	0x88, // #   #
	0x88, // #   #
	0x88, // #   #
	0x88, // #   #
	0x78, //  ####
	0x88, // #   #
	0x70, //  ### 

	// @814 'h' (4 pixels wide)
	0x00, //     
	0x80, // #   
	0x80, // #   
	0xF0, // ####
	0x90, // #  #
	0x90, // #  #
	0x90, // #  #
	0x90, // #  #
	0x90, // #  #
	0x00, //     
	0x00, //     

	// @825 'i' (1 pixels wide)
	0x00, //  
	0x80, // #
	0x00, //  
	0x80, // #
	0x80, // #
	0x80, // #
	0x80, // #
	0x80, // #
	0x80, // #
	0x00, //  
	0x00, //  

	// @836 'j' (2 pixels wide)
	0x00, //   
	0x40, //  #
	0x00, //   
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0xC0, // ##

	// @847 'k' (5 pixels wide)
	0x00, //      
	0x80, // #    
	0x80, // #    
	0x90, // #  # 
	0xA0, // # #  
	0xE0, // ###  
	0xA0, // # #  
	0x90, // #  # 
	0x88, // #   #
	0x00, //      
	0x00, //      

	// @858 'l' (1 pixels wide)
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

	// @869 'm' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0xFE, // #######
	0x92, // #  #  #
	0x92, // #  #  #
	0x92, // #  #  #
	0x92, // #  #  #
	0x92, // #  #  #
	0x00, //        
	0x00, //        

	// @880 'n' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0xF0, // ####
	0x90, // #  #
	0x90, // #  #
	0x90, // #  #
	0x90, // #  #
	0x90, // #  #
	0x00, //     
	0x00, //     

	// @891 'o' (6 pixels wide)
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

	// @902 'p' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0xF0, // #### 
	0x88, // #   #
	0x88, // #   #
	0x88, // #   #
	0x88, // #   #
	0xF0, // #### 
	0x80, // #    
	0x80, // #    

	// @913 'q' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x74, //  ### #
	0xCC, // ##  ##
	0x84, // #    #
	0x84, // #    #
	0xCC, // ##  ##
	0x74, //  ### #
	0x04, //      #
	0x04, //      #

	// @924 'r' (3 pixels wide)
	0x00, //    
	0x00, //    
	0x00, //    
	0xE0, // ###
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x00, //    
	0x00, //    

	// @935 's' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x60, //  ## 
	0x90, // #  #
	0xC0, // ##  
	0x70, //  ###
	0x90, // #  #
	0x60, //  ## 
	0x00, //     
	0x00, //     

	// @946 't' (3 pixels wide)
	0x00, //    
	0x00, //    
	0x40, //  # 
	0xE0, // ###
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x60, //  ##
	0x00, //    
	0x00, //    

	// @957 'u' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x90, // #  #
	0x90, // #  #
	0x90, // #  #
	0x90, // #  #
	0x90, // #  #
	0xF0, // ####
	0x00, //     
	0x00, //     

	// @968 'v' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x88, // #   #
	0x48, //  #  #
	0x50, //  # # 
	0x50, //  # # 
	0x30, //   ## 
	0x20, //   #  
	0x00, //      
	0x00, //      

	// @979 'w' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x99, // #  ##  #
	0x9A, // #  ## # 
	0x5A, //  # ## # 
	0x6A, //  ## # # 
	0x66, //  ##  ## 
	0x24, //   #  #  
	0x00, //         
	0x00, //         

	// @990 'x' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x48, //  #  #
	0x50, //  # # 
	0x20, //   #  
	0x30, //   ## 
	0x50, //  # # 
	0x88, // #   #
	0x00, //      
	0x00, //      

	// @1001 'y' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x88, // #   #
	0x48, //  #  #
	0x50, //  # # 
	0x50, //  # # 
	0x30, //   ## 
	0x20, //   #  
	0x20, //   #  
	0x40, //  #   

	// @1012 'z' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x78, //  ####
	0x10, //    # 
	0x30, //   ## 
	0x20, //   #  
	0x40, //  #   
	0xF8, // #####
	0x00, //      
	0x00, //      

	// @1023 '{' (2 pixels wide)
	0x00, //   
	0xC0, // ##
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0xC0, // ##

	// @1034 '|' (1 pixels wide)
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

	// @1045 '}' (2 pixels wide)
	0x00, //   
	0xC0, // ##
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0xC0, // ##

	// @1056 '~' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x64, //  ##  #
	0x9C, // #  ###
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
};

// Character descriptors for FreeSans 8pt
// { [Char width in bits], [Offset into freeSans_8ptCharBitmaps in bytes] }
static const CharDesc_t s_FreeSans8ptCharDesc[] = {
	{1, s_FreeSans8ptBitmaps + 0}, 			// !
	{3, s_FreeSans8ptBitmaps + 11}, 		// "
	{6, s_FreeSans8ptBitmaps + 22}, 		// #
	{5, s_FreeSans8ptBitmaps + 33}, 		// $
	{10,s_FreeSans8ptBitmaps + 44}, 		// %
	{5, s_FreeSans8ptBitmaps + 66}, 		// &
	{1, s_FreeSans8ptBitmaps + 77}, 		// '
	{2, s_FreeSans8ptBitmaps + 88}, 		// (
	{3, s_FreeSans8ptBitmaps + 99}, 		// )
	{3, s_FreeSans8ptBitmaps + 110}, 		// *
	{5, s_FreeSans8ptBitmaps + 121}, 		// +
	{1, s_FreeSans8ptBitmaps + 132}, 		// ,
	{2, s_FreeSans8ptBitmaps + 143}, 		// -
	{1, s_FreeSans8ptBitmaps + 154}, 		// .
	{3, s_FreeSans8ptBitmaps + 165}, 		// /
	{6, s_FreeSans8ptBitmaps + 176}, 		// 0
	{2, s_FreeSans8ptBitmaps + 187}, 		// 1
	{5, s_FreeSans8ptBitmaps + 198}, 		// 2
	{5, s_FreeSans8ptBitmaps + 209}, 		// 3
	{5, s_FreeSans8ptBitmaps + 220}, 		// 4
	{6, s_FreeSans8ptBitmaps + 231}, 		// 5
	{6, s_FreeSans8ptBitmaps + 242}, 		// 6
	{5, s_FreeSans8ptBitmaps + 253}, 		// 7
	{6, s_FreeSans8ptBitmaps + 264}, 		// 8
	{6, s_FreeSans8ptBitmaps + 275}, 		// 9
	{1, s_FreeSans8ptBitmaps + 286}, 		// :
	{1, s_FreeSans8ptBitmaps + 297}, 		// ;
	{6, s_FreeSans8ptBitmaps + 308}, 		// <
	{5, s_FreeSans8ptBitmaps + 319}, 		// =
	{5, s_FreeSans8ptBitmaps + 330}, 		// >
	{5, s_FreeSans8ptBitmaps + 341}, 		// ?
	{10,s_FreeSans8ptBitmaps + 352}, 		// @
	{7, s_FreeSans8ptBitmaps + 374}, 		// A
	{6, s_FreeSans8ptBitmaps + 385}, 		// B
	{7, s_FreeSans8ptBitmaps + 396}, 		// C
	{6, s_FreeSans8ptBitmaps + 407}, 		// D
	{5, s_FreeSans8ptBitmaps + 418}, 		// E
	{5, s_FreeSans8ptBitmaps + 429}, 		// F
	{8, s_FreeSans8ptBitmaps + 440}, 		// G
	{6, s_FreeSans8ptBitmaps + 451}, 		// H
	{1, s_FreeSans8ptBitmaps + 462}, 		// I
	{5, s_FreeSans8ptBitmaps + 473}, 		// J
	{6, s_FreeSans8ptBitmaps + 484}, 		// K
	{4, s_FreeSans8ptBitmaps + 495}, 		// L
	{7, s_FreeSans8ptBitmaps + 506}, 		// M
	{6, s_FreeSans8ptBitmaps + 517}, 		// N
	{8, s_FreeSans8ptBitmaps + 528}, 		// O
	{6, s_FreeSans8ptBitmaps + 539}, 		// P
	{8, s_FreeSans8ptBitmaps + 550}, 		// Q
	{6, s_FreeSans8ptBitmaps + 561}, 		// R
	{6, s_FreeSans8ptBitmaps + 572}, 		// S
	{5, s_FreeSans8ptBitmaps + 583}, 		// T
	{6, s_FreeSans8ptBitmaps + 594}, 		// U
	{7, s_FreeSans8ptBitmaps + 605}, 		// V
	{10,s_FreeSans8ptBitmaps + 616}, 		// W
	{6, s_FreeSans8ptBitmaps + 638}, 		// X
	{7, s_FreeSans8ptBitmaps + 649}, 		// Y
	{6, s_FreeSans8ptBitmaps + 660}, 		// Z
	{2, s_FreeSans8ptBitmaps + 671}, 		// [
	{3, s_FreeSans8ptBitmaps + 682}, 		// '\'
	{2, s_FreeSans8ptBitmaps + 693}, 		// ]
	{3, s_FreeSans8ptBitmaps + 704}, 		// ^
	{6, s_FreeSans8ptBitmaps + 715}, 		// _
	{1, s_FreeSans8ptBitmaps + 726}, 		// `
	{6, s_FreeSans8ptBitmaps + 737}, 		// a
	{5, s_FreeSans8ptBitmaps + 748}, 		// b
	{5, s_FreeSans8ptBitmaps + 759}, 		// c
	{6, s_FreeSans8ptBitmaps + 770}, 		// d
	{6, s_FreeSans8ptBitmaps + 781}, 		// e
	{3, s_FreeSans8ptBitmaps + 792}, 		// f
	{5, s_FreeSans8ptBitmaps + 803}, 		// g
	{4, s_FreeSans8ptBitmaps + 814}, 		// h
	{1, s_FreeSans8ptBitmaps + 825}, 		// i
	{2, s_FreeSans8ptBitmaps + 836}, 		// j
	{5, s_FreeSans8ptBitmaps + 847}, 		// k
	{1, s_FreeSans8ptBitmaps + 858}, 		// l
	{7, s_FreeSans8ptBitmaps + 869}, 		// m
	{4, s_FreeSans8ptBitmaps + 880}, 		// n
	{6, s_FreeSans8ptBitmaps + 891}, 		// o
	{5, s_FreeSans8ptBitmaps + 902}, 		// p
	{6, s_FreeSans8ptBitmaps + 913}, 		// q
	{3, s_FreeSans8ptBitmaps + 924}, 		// r
	{4, s_FreeSans8ptBitmaps + 935}, 		// s
	{3, s_FreeSans8ptBitmaps + 946}, 		// t
	{4, s_FreeSans8ptBitmaps + 957}, 		// u
	{5, s_FreeSans8ptBitmaps + 968}, 		// v
	{8, s_FreeSans8ptBitmaps + 979}, 		// w
	{5, s_FreeSans8ptBitmaps + 990}, 		// x
	{5, s_FreeSans8ptBitmaps + 1001}, 		// y
	{5, s_FreeSans8ptBitmaps + 1012}, 		// z
	{2, s_FreeSans8ptBitmaps + 1023}, 		// {
	{1, s_FreeSans8ptBitmaps + 1034}, 		// |
	{2, s_FreeSans8ptBitmaps + 1045}, 		// }
	{6, s_FreeSans8ptBitmaps + 1056}, 		// ~
};

// Font information for FreeSans 8pt
const FontDesc_t iFontFreeSans8pt = {
	FONT_TYPE_VAR_WIDTH,
	10,
	11,
	{ .pCharDesc = s_FreeSans8ptCharDesc }
};

