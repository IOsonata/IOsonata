// 
//  Font data for FreeMono 12pt
// 	https://savannah.gnu.org/projects/freefont/
// 
// 	Font bitmap generated by
// 	http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator

#include "display/ifont.h"

// Character bitmaps for FreeMono 12pt
static const uint8_t s_FreeMono12ptBitmaps[] = {
	// @0 '!' (1 pixels wide)
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
	0x80, // #
	0x00, //  
	0x00, //  
	0x00, //  

	// @14 '"' (4 pixels wide)
	0x00, //     
	0xF0, // ####
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
	0x00, //     

	// @28 '#' (7 pixels wide)
	0x00, //        
	0x14, //    # # 
	0x14, //    # # 
	0x14, //    # # 
	0x7E, //  ######
	0x24, //   #  # 
	0x24, //   #  # 
	0xFE, // #######
	0x28, //   # #  
	0x28, //   # #  
	0x28, //   # #  
	0x00, //        
	0x00, //        
	0x00, //        

	// @42 '$' (7 pixels wide)
	0x00, //        
	0x10, //    #   
	0x3C, //   #### 
	0x44, //  #   # 
	0x40, //  #     
	0x38, //   ###  
	0x06, //      ##
	0x02, //       #
	0x86, // #    ##
	0xFC, // ###### 
	0x10, //    #   
	0x00, //        
	0x00, //        
	0x00, //        

	// @56 '%' (6 pixels wide)
	0x00, //       
	0x60, //  ##   
	0x90, // #  #  
	0x90, // #  #  
	0x60, //  ##   
	0x0C, //     ##
	0x70, //  ###  
	0x98, // #  ## 
	0x24, //   #  #
	0x24, //   #  #
	0x18, //    ## 
	0x00, //       
	0x00, //       
	0x00, //       

	// @70 '&' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x38, //   ### 
	0x40, //  #    
	0x40, //  #    
	0x40, //  #    
	0x60, //  ##   
	0x94, // #  # #
	0x94, // #  # #
	0x88, // #   # 
	0x74, //  ### #
	0x00, //       
	0x00, //       
	0x00, //       

	// @84 ''' (1 pixels wide)
	0x00, //  
	0x80, // #
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

	// @98 '(' (2 pixels wide)
	0x00, //   
	0x00, //   
	0x40, //  #
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
	0x00, //   

	// @112 ')' (2 pixels wide)
	0x00, //   
	0x00, //   
	0x80, // # 
	0x80, // # 
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x00, //   

	// @126 '*' (6 pixels wide)
	0x00, //       
	0x20, //   #   
	0x20, //   #   
	0x24, //   #  #
	0xF8, // ##### 
	0x50, //  # #  
	0x50, //  # #  
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @140 '+' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0xF8, // #####
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @154 ',' (3 pixels wide)
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x60, //  ##
	0x40, //  # 
	0xC0, // ## 
	0x80, // #  
	0x80, // #  
	0x00, //    

	// @168 '-' (7 pixels wide)
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
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        

	// @182 '.' (2 pixels wide)
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
	0xC0, // ##
	0x00, //   
	0x00, //   
	0x00, //   

	// @196 '/' (6 pixels wide)
	0x04, //      #
	0x04, //      #
	0x08, //     # 
	0x08, //     # 
	0x10, //    #  
	0x10, //    #  
	0x20, //   #   
	0x20, //   #   
	0x20, //   #   
	0x40, //  #    
	0x40, //  #    
	0x80, // #     
	0x00, //       
	0x00, //       

	// @210 '0' (6 pixels wide)
	0x00, //       
	0x30, //   ##  
	0x48, //  #  # 
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x48, //  #  # 
	0x30, //   ##  
	0x00, //       
	0x00, //       
	0x00, //       

	// @224 '1' (5 pixels wide)
	0x00, //      
	0x20, //   #  
	0x60, //  ##  
	0xA0, // # #  
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

	// @238 '2' (6 pixels wide)
	0x00, //       
	0x78, //  #### 
	0xC8, // ##  # 
	0x84, // #    #
	0x04, //      #
	0x08, //     # 
	0x10, //    #  
	0x20, //   #   
	0x40, //  #    
	0x80, // #     
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       

	// @252 '3' (7 pixels wide)
	0x00, //        
	0xF8, // #####  
	0x0C, //     ## 
	0x04, //      # 
	0x04, //      # 
	0x18, //    ##  
	0x04, //      # 
	0x02, //       #
	0x02, //       #
	0x84, // #    # 
	0x78, //  ####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @266 '4' (6 pixels wide)
	0x00, //       
	0x08, //     # 
	0x38, //   ### 
	0x28, //   # # 
	0x48, //  #  # 
	0x48, //  #  # 
	0x48, //  #  # 
	0x88, // #   # 
	0x7C, //  #####
	0x08, //     # 
	0x1C, //    ###
	0x00, //       
	0x00, //       
	0x00, //       

	// @280 '5' (7 pixels wide)
	0x00, //        
	0x78, //  ####  
	0x40, //  #     
	0x40, //  #     
	0x7C, //  ##### 
	0x46, //  #   ##
	0x02, //       #
	0x02, //       #
	0x02, //       #
	0xC4, // ##   # 
	0x38, //   ###  
	0x00, //        
	0x00, //        
	0x00, //        

	// @294 '6' (6 pixels wide)
	0x00, //       
	0x3C, //   ####
	0x60, //  ##   
	0x40, //  #    
	0x80, // #     
	0xB8, // # ### 
	0xCC, // ##  ##
	0x84, // #    #
	0x84, // #    #
	0x4C, //  #  ##
	0x38, //   ### 
	0x00, //       
	0x00, //       
	0x00, //       

	// @308 '7' (6 pixels wide)
	0x00, //       
	0xFC, // ######
	0x04, //      #
	0x04, //      #
	0x08, //     # 
	0x08, //     # 
	0x08, //     # 
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @322 '8' (6 pixels wide)
	0x00, //       
	0x78, //  #### 
	0xCC, // ##  ##
	0x84, // #    #
	0x84, // #    #
	0x78, //  #### 
	0x4C, //  #  ##
	0x84, // #    #
	0x84, // #    #
	0xCC, // ##  ##
	0x78, //  #### 
	0x00, //       
	0x00, //       
	0x00, //       

	// @336 '9' (7 pixels wide)
	0x00, //        
	0x78, //  ####  
	0xC4, // ##   # 
	0x82, // #     #
	0x82, // #     #
	0xC6, // ##   ##
	0x7A, //  #### #
	0x02, //       #
	0x04, //      # 
	0x08, //     #  
	0xF0, // ####   
	0x00, //        
	0x00, //        
	0x00, //        

	// @350 ':' (2 pixels wide)
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0xC0, // ##
	0xC0, // ##
	0x00, //   
	0x00, //   
	0x00, //   
	0xC0, // ##
	0xC0, // ##
	0x00, //   
	0x00, //   
	0x00, //   

	// @364 ';' (3 pixels wide)
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x60, //  ##
	0x60, //  ##
	0x00, //    
	0x00, //    
	0x00, //    
	0x60, //  ##
	0x40, //  # 
	0xC0, // ## 
	0x80, // #  
	0x00, //    

	// @378 '<' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x02, //       #
	0x0C, //     ## 
	0x30, //   ##   
	0xC0, // ##     
	0x20, //   #    
	0x18, //    ##  
	0x06, //      ##
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        

	// @392 '=' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0xFF, // ########
	0x00, //         
	0xFF, // ########
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @406 '>' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x80, // #     
	0x60, //  ##   
	0x18, //    ## 
	0x04, //      #
	0x18, //    ## 
	0x20, //   #   
	0xC0, // ##    
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @420 '?' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x78, //  #### 
	0x84, // #    #
	0x04, //      #
	0x0C, //     ##
	0x18, //    ## 
	0x20, //   #   
	0x00, //       
	0x00, //       
	0x30, //   ##  
	0x00, //       
	0x00, //       
	0x00, //       

	// @434 '@' (7 pixels wide)
	0x00, //        
	0x38, //   ###  
	0x44, //  #   # 
	0x84, // #    # 
	0x9C, // #  ### 
	0xB4, // # ## # 
	0xA4, // # #  # 
	0xA4, // # #  # 
	0x9E, // #  ####
	0x80, // #      
	0x40, //  #     
	0x3C, //   #### 
	0x00, //        
	0x00, //        

	// @448 'A' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x38, 0x00, //   ###    
	0x14, 0x00, //    # #   
	0x14, 0x00, //    # #   
	0x22, 0x00, //   #   #  
	0x22, 0x00, //   #   #  
	0x3E, 0x00, //   #####  
	0x41, 0x00, //  #     # 
	0x41, 0x00, //  #     # 
	0xE3, 0x80, // ###   ###
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @476 'B' (8 pixels wide)
	0x00, //         
	0x00, //         
	0xFC, // ######  
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x7C, //  #####  
	0x43, //  #    ##
	0x41, //  #     #
	0x41, //  #     #
	0xFE, // ####### 
	0x00, //         
	0x00, //         
	0x00, //         

	// @490 'C' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x3D, //   #### #
	0x43, //  #    ##
	0x81, // #      #
	0x80, // #       
	0x80, // #       
	0x80, // #       
	0xC0, // ##      
	0x61, //  ##    #
	0x3E, //   ##### 
	0x00, //         
	0x00, //         
	0x00, //         

	// @504 'D' (8 pixels wide)
	0x00, //         
	0x00, //         
	0xFC, // ######  
	0x42, //  #    # 
	0x41, //  #     #
	0x41, //  #     #
	0x41, //  #     #
	0x41, //  #     #
	0x41, //  #     #
	0x42, //  #    # 
	0xFC, // ######  
	0x00, //         
	0x00, //         
	0x00, //         

	// @518 'E' (8 pixels wide)
	0x00, //         
	0x00, //         
	0xFE, // ####### 
	0x42, //  #    # 
	0x40, //  #      
	0x48, //  #  #   
	0x78, //  ####   
	0x48, //  #  #   
	0x40, //  #      
	0x41, //  #     #
	0xFF, // ########
	0x00, //         
	0x00, //         
	0x00, //         

	// @532 'F' (8 pixels wide)
	0x00, //         
	0x00, //         
	0xFF, // ########
	0x41, //  #     #
	0x40, //  #      
	0x48, //  #  #   
	0x78, //  ####   
	0x48, //  #  #   
	0x40, //  #      
	0x40, //  #      
	0xF0, // ####    
	0x00, //         
	0x00, //         
	0x00, //         

	// @546 'G' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x3D, 0x00, //   #### # 
	0x43, 0x00, //  #    ## 
	0x80, 0x00, // #        
	0x80, 0x00, // #        
	0x80, 0x00, // #        
	0x87, 0x80, // #    ####
	0x81, 0x00, // #      # 
	0x41, 0x00, //  #     # 
	0x3E, 0x00, //   #####  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @574 'H' (8 pixels wide)
	0x00, //         
	0x00, //         
	0xE7, // ###  ###
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x7E, //  ###### 
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0xE7, // ###  ###
	0x00, //         
	0x00, //         
	0x00, //         

	// @588 'I' (5 pixels wide)
	0x00, //      
	0x00, //      
	0xF8, // #####
	0x20, //   #  
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

	// @602 'J' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x0F, //     ####
	0x02, //       # 
	0x02, //       # 
	0x02, //       # 
	0x02, //       # 
	0x82, // #     # 
	0x82, // #     # 
	0xC6, // ##   ## 
	0x7C, //  #####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @616 'K' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0xE7, 0x00, // ###  ### 
	0x42, 0x00, //  #    #  
	0x44, 0x00, //  #   #   
	0x58, 0x00, //  # ##    
	0x68, 0x00, //  ## #    
	0x44, 0x00, //  #   #   
	0x44, 0x00, //  #   #   
	0x42, 0x00, //  #    #  
	0xE1, 0x80, // ###    ##
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @644 'L' (7 pixels wide)
	0x00, //        
	0x00, //        
	0xE0, // ###    
	0x40, //  #     
	0x40, //  #     
	0x40, //  #     
	0x40, //  #     
	0x40, //  #     
	0x42, //  #    #
	0x42, //  #    #
	0xFE, // #######
	0x00, //        
	0x00, //        
	0x00, //        

	// @658 'M' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0xE1, 0xC0, // ###    ###
	0x61, 0x80, //  ##    ## 
	0x52, 0x80, //  # #  # # 
	0x52, 0x80, //  # #  # # 
	0x4C, 0x80, //  #  ##  # 
	0x4C, 0x80, //  #  ##  # 
	0x40, 0x80, //  #      # 
	0x40, 0x80, //  #      # 
	0xE1, 0xC0, // ###    ###
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @686 'N' (8 pixels wide)
	0x00, //         
	0x00, //         
	0xC7, // ##   ###
	0x62, //  ##   # 
	0x62, //  ##   # 
	0x52, //  # #  # 
	0x52, //  # #  # 
	0x4A, //  #  # # 
	0x4A, //  #  # # 
	0x46, //  #   ## 
	0xE2, // ###   # 
	0x00, //         
	0x00, //         
	0x00, //         

	// @700 'O' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x3C, //   ####  
	0x42, //  #    # 
	0xC3, // ##    ##
	0x81, // #      #
	0x81, // #      #
	0x81, // #      #
	0xC3, // ##    ##
	0x42, //  #    # 
	0x3C, //   ####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @714 'P' (7 pixels wide)
	0x00, //        
	0x00, //        
	0xFC, // ###### 
	0x46, //  #   ##
	0x42, //  #    #
	0x42, //  #    #
	0x46, //  #   ##
	0x7C, //  ##### 
	0x40, //  #     
	0x40, //  #     
	0xF0, // ####   
	0x00, //        
	0x00, //        
	0x00, //        

	// @728 'Q' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x3C, //   ####  
	0x42, //  #    # 
	0xC3, // ##    ##
	0x81, // #      #
	0x81, // #      #
	0x81, // #      #
	0xC3, // ##    ##
	0x42, //  #    # 
	0x3C, //   ####  
	0x19, //    ##  #
	0x26, //   #  ## 
	0x00, //         

	// @742 'R' (8 pixels wide)
	0x00, //         
	0x00, //         
	0xFC, // ######  
	0x46, //  #   ## 
	0x42, //  #    # 
	0x46, //  #   ## 
	0x78, //  ####   
	0x44, //  #   #  
	0x44, //  #   #  
	0x42, //  #    # 
	0xE1, // ###    #
	0x00, //         
	0x00, //         
	0x00, //         

	// @756 'S' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x3A, //   ### #
	0x66, //  ##  ##
	0x42, //  #    #
	0x40, //  #     
	0x3C, //   #### 
	0x02, //       #
	0x82, // #     #
	0xC6, // ##   ##
	0xBC, // # #### 
	0x00, //        
	0x00, //        
	0x00, //        

	// @770 'T' (8 pixels wide)
	0x00, //         
	0x00, //         
	0xFF, // ########
	0x91, // #  #   #
	0x10, //    #    
	0x10, //    #    
	0x10, //    #    
	0x10, //    #    
	0x10, //    #    
	0x10, //    #    
	0x38, //   ###   
	0x00, //         
	0x00, //         
	0x00, //         

	// @784 'U' (8 pixels wide)
	0x00, //         
	0x00, //         
	0xE7, // ###  ###
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x66, //  ##  ## 
	0x3C, //   ####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @798 'V' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0xE3, 0x80, // ###   ###
	0x41, 0x00, //  #     # 
	0x41, 0x00, //  #     # 
	0x22, 0x00, //   #   #  
	0x22, 0x00, //   #   #  
	0x12, 0x00, //    #  #  
	0x14, 0x00, //    # #   
	0x14, 0x00, //    # #   
	0x08, 0x00, //     #    
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @826 'W' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0xF3, 0x80, // ####  ###
	0x40, 0x80, //  #      #
	0x4C, 0x80, //  #  ##  #
	0x4D, 0x00, //  #  ## # 
	0x55, 0x00, //  # # # # 
	0x53, 0x00, //  # #  ## 
	0x53, 0x00, //  # #  ## 
	0x23, 0x00, //   #   ## 
	0x23, 0x00, //   #   ## 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @854 'X' (8 pixels wide)
	0x00, //         
	0x00, //         
	0xC7, // ##   ###
	0x42, //  #    # 
	0x24, //   #  #  
	0x28, //   # #   
	0x10, //    #    
	0x28, //   # #   
	0x44, //  #   #  
	0x42, //  #    # 
	0xE7, // ###  ###
	0x00, //         
	0x00, //         
	0x00, //         

	// @868 'Y' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0xE3, 0x80, // ###   ###
	0x42, 0x00, //  #    #  
	0x22, 0x00, //   #   #  
	0x14, 0x00, //    # #   
	0x08, 0x00, //     #    
	0x08, 0x00, //     #    
	0x08, 0x00, //     #    
	0x08, 0x00, //     #    
	0x1C, 0x00, //    ###   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @896 'Z' (6 pixels wide)
	0x00, //       
	0x00, //       
	0xF8, // ##### 
	0x88, // #   # 
	0x10, //    #  
	0x20, //   #   
	0x20, //   #   
	0x40, //  #    
	0x84, // #    #
	0x84, // #    #
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       

	// @910 '[' (2 pixels wide)
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
	0x80, // # 
	0x80, // # 
	0xC0, // ##
	0x00, //   

	// @924 '\' (6 pixels wide)
	0x80, // #     
	0x80, // #     
	0x40, //  #    
	0x40, //  #    
	0x20, //   #   
	0x20, //   #   
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x08, //     # 
	0x08, //     # 
	0x04, //      #
	0x00, //       
	0x00, //       

	// @938 ']' (2 pixels wide)
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
	0x40, //  #
	0x40, //  #
	0xC0, // ##
	0x00, //   

	// @952 '^' (6 pixels wide)
	0x00, //       
	0x20, //   #   
	0x50, //  # #  
	0x48, //  #  # 
	0x84, // #    #
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @966 '_' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0xFF, 0xC0, // ##########
	0x00, 0x00, //           

	// @994 '`' (2 pixels wide)
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
	0x00, //   
	0x00, //   

	// @1008 'a' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x7C, //  #####  
	0x02, //       # 
	0x7E, //  ###### 
	0xC2, // ##    # 
	0x82, // #     # 
	0x86, // #    ## 
	0x7B, //  #### ##
	0x00, //         
	0x00, //         
	0x00, //         

	// @1022 'b' (7 pixels wide)
	0x00, //        
	0x80, // #      
	0x80, // #      
	0x80, // #      
	0xB8, // # ###  
	0xC4, // ##   # 
	0x82, // #     #
	0x82, // #     #
	0x82, // #     #
	0xC4, // ##   # 
	0xB8, // # ###  
	0x00, //        
	0x00, //        
	0x00, //        

	// @1036 'c' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x3D, //   #### #
	0x43, //  #    ##
	0x81, // #      #
	0x80, // #       
	0x80, // #       
	0x40, //  #      
	0x3F, //   ######
	0x00, //         
	0x00, //         
	0x00, //         

	// @1050 'd' (8 pixels wide)
	0x00, //         
	0x02, //       # 
	0x02, //       # 
	0x02, //       # 
	0x3A, //   ### # 
	0x46, //  #   ## 
	0x82, // #     # 
	0x82, // #     # 
	0x82, // #     # 
	0x46, //  #   ## 
	0x3B, //   ### ##
	0x00, //         
	0x00, //         
	0x00, //         

	// @1064 'e' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3C, //   #### 
	0x42, //  #    #
	0x82, // #     #
	0xFE, // #######
	0x80, // #      
	0x42, //  #    #
	0x3C, //   #### 
	0x00, //        
	0x00, //        
	0x00, //        

	// @1078 'f' (5 pixels wide)
	0x00, //      
	0x38, //   ###
	0x40, //  #   
	0x40, //  #   
	0xF8, // #####
	0x40, //  #   
	0x40, //  #   
	0x40, //  #   
	0x40, //  #   
	0x40, //  #   
	0xF0, // #### 
	0x00, //      
	0x00, //      
	0x00, //      

	// @1092 'g' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3A, //   ### #
	0x46, //  #   ##
	0x82, // #     #
	0x82, // #     #
	0x82, // #     #
	0x46, //  #   ##
	0x3A, //   ### #
	0x02, //       #
	0x02, //       #
	0x1C, //    ### 

	// @1106 'h' (8 pixels wide)
	0x00, //         
	0x40, //  #      
	0x40, //  #      
	0x40, //  #      
	0x5C, //  # ###  
	0x62, //  ##   # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0xE7, // ###  ###
	0x00, //         
	0x00, //         
	0x00, //         

	// @1120 'i' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x20, //   #  
	0x00, //      
	0x60, //  ##  
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0x20, //   #  
	0xF8, // #####
	0x00, //      
	0x00, //      
	0x00, //      

	// @1134 'j' (4 pixels wide)
	0x00, //     
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
	0x10, //    #
	0xE0, // ### 

	// @1148 'k' (7 pixels wide)
	0x00, //        
	0x80, // #      
	0x80, // #      
	0x80, // #      
	0x9C, // #  ### 
	0x90, // #  #   
	0xA0, // # #    
	0xE0, // ###    
	0x90, // #  #   
	0x88, // #   #  
	0x8E, // #   ###
	0x00, //        
	0x00, //        
	0x00, //        

	// @1162 'l' (5 pixels wide)
	0x00, //      
	0x60, //  ##  
	0x20, //   #  
	0x20, //   #  
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

	// @1176 'm' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0xB6, // # ## ## 
	0xC9, // ##  #  #
	0x89, // #   #  #
	0x89, // #   #  #
	0x89, // #   #  #
	0x89, // #   #  #
	0xC9, // ##  #  #
	0x00, //         
	0x00, //         
	0x00, //         

	// @1190 'n' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x5C, //  # ###  
	0x62, //  ##   # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0x42, //  #    # 
	0xE7, // ###  ###
	0x00, //         
	0x00, //         
	0x00, //         

	// @1204 'o' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x3C, //   ####  
	0x42, //  #    # 
	0x81, // #      #
	0x81, // #      #
	0x81, // #      #
	0x42, //  #    # 
	0x3C, //   ####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @1218 'p' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0xDC, // ## ###  
	0x62, //  ##   # 
	0x41, //  #     #
	0x41, //  #     #
	0x41, //  #     #
	0x62, //  ##   # 
	0x5C, //  # ###  
	0x40, //  #      
	0x40, //  #      
	0xE0, // ###     

	// @1232 'q' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x3B, //   ### ##
	0x46, //  #   ## 
	0x82, // #     # 
	0x82, // #     # 
	0x82, // #     # 
	0x46, //  #   ## 
	0x3A, //   ### # 
	0x02, //       # 
	0x02, //       # 
	0x07, //      ###

	// @1246 'r' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xCC, // ##  ##
	0x70, //  ###  
	0x40, //  #    
	0x40, //  #    
	0x40, //  #    
	0x40, //  #    
	0xF0, // ####  
	0x00, //       
	0x00, //       
	0x00, //       

	// @1260 's' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x3C, //   ####
	0x44, //  #   #
	0x40, //  #    
	0x38, //   ### 
	0x04, //      #
	0x84, // #    #
	0xF8, // ##### 
	0x00, //       
	0x00, //       
	0x00, //       

	// @1274 't' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x40, //  #     
	0x40, //  #     
	0xF8, // #####  
	0x40, //  #     
	0x40, //  #     
	0x40, //  #     
	0x40, //  #     
	0x42, //  #    #
	0x3C, //   #### 
	0x00, //        
	0x00, //        
	0x00, //        

	// @1288 'u' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x8C, // #   ##
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x8C, // #   ##
	0x74, //  ### #
	0x00, //       
	0x00, //       
	0x00, //       

	// @1302 'v' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0xF3, 0x80, // ####  ###
	0x21, 0x00, //   #    # 
	0x22, 0x00, //   #   #  
	0x22, 0x00, //   #   #  
	0x14, 0x00, //    # #   
	0x14, 0x00, //    # #   
	0x0C, 0x00, //     ##   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1330 'w' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0xE3, 0x80, // ###   ###
	0x41, 0x00, //  #     # 
	0x49, 0x00, //  #  #  # 
	0x35, 0x00, //   ## # # 
	0x35, 0x00, //   ## # # 
	0x36, 0x00, //   ## ##  
	0x22, 0x00, //   #   #  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1358 'x' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0xE6, // ###  ## 
	0x44, //  #   #  
	0x28, //   # #   
	0x10, //    #    
	0x2C, //   # ##  
	0x42, //  #    # 
	0xE7, // ###  ###
	0x00, //         
	0x00, //         
	0x00, //         

	// @1372 'y' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0xE7, // ###  ###
	0x42, //  #    # 
	0x44, //  #   #  
	0x24, //   #  #  
	0x28, //   # #   
	0x18, //    ##   
	0x10, //    #    
	0x10, //    #    
	0x20, //   #     
	0xF0, // ####    

	// @1386 'z' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xF8, // ##### 
	0x88, // #   # 
	0x10, //    #  
	0x20, //   #   
	0x40, //  #    
	0x80, // #     
	0x7C, //  #####
	0x00, //       
	0x00, //       
	0x00, //       

	// @1400 '{' (3 pixels wide)
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
	0x40, //  # 
	0x40, //  # 
	0x20, //   #
	0x00, //    

	// @1414 '|' (1 pixels wide)
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

	// @1428 '}' (3 pixels wide)
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
	0x40, //  # 
	0x40, //  # 
	0x80, // #  
	0x00, //    

	// @1442 '~' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xC0, // ##    
	0xA4, // # #  #
	0x18, //    ## 
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
};

// Character descriptors for FreeMono 12pt
// { [Char width in bits], [Offset into freeMono_12ptCharBitmaps in bytes] }
static const CharDesc_t s_FreeMono12ptCharDesc[] = {
	{1, s_FreeMono12ptBitmaps + 0}, 		// !
	{4, s_FreeMono12ptBitmaps + 14}, 		// "
	{7, s_FreeMono12ptBitmaps + 28}, 		// #
	{7, s_FreeMono12ptBitmaps + 42}, 		// $
	{6, s_FreeMono12ptBitmaps + 56}, 		// %
	{6, s_FreeMono12ptBitmaps + 70}, 		// &
	{1, s_FreeMono12ptBitmaps + 84}, 		// '
	{2, s_FreeMono12ptBitmaps + 98}, 		// (
	{2, s_FreeMono12ptBitmaps + 112}, 		// )
	{6, s_FreeMono12ptBitmaps + 126}, 		// *
	{5, s_FreeMono12ptBitmaps + 140}, 		// +
	{3, s_FreeMono12ptBitmaps + 154}, 		// ,
	{7, s_FreeMono12ptBitmaps + 168}, 		// -
	{2, s_FreeMono12ptBitmaps + 182}, 		// .
	{6, s_FreeMono12ptBitmaps + 196}, 		// /
	{6, s_FreeMono12ptBitmaps + 210}, 		// 0
	{5, s_FreeMono12ptBitmaps + 224}, 		// 1
	{6, s_FreeMono12ptBitmaps + 238}, 		// 2
	{7, s_FreeMono12ptBitmaps + 252}, 		// 3
	{6, s_FreeMono12ptBitmaps + 266}, 		// 4
	{7, s_FreeMono12ptBitmaps + 280}, 		// 5
	{6, s_FreeMono12ptBitmaps + 294}, 		// 6
	{6, s_FreeMono12ptBitmaps + 308}, 		// 7
	{6, s_FreeMono12ptBitmaps + 322}, 		// 8
	{7, s_FreeMono12ptBitmaps + 336}, 		// 9
	{2, s_FreeMono12ptBitmaps + 350}, 		// :
	{3, s_FreeMono12ptBitmaps + 364}, 		// ;
	{7, s_FreeMono12ptBitmaps + 378}, 		// <
	{8, s_FreeMono12ptBitmaps + 392}, 		// =
	{6, s_FreeMono12ptBitmaps + 406}, 		// >
	{6, s_FreeMono12ptBitmaps + 420}, 		// ?
	{7, s_FreeMono12ptBitmaps + 434}, 		// @
	{9, s_FreeMono12ptBitmaps + 448}, 		// A
	{8, s_FreeMono12ptBitmaps + 476}, 		// B
	{8, s_FreeMono12ptBitmaps + 490}, 		// C
	{8, s_FreeMono12ptBitmaps + 504}, 		// D
	{8, s_FreeMono12ptBitmaps + 518}, 		// E
	{8, s_FreeMono12ptBitmaps + 532}, 		// F
	{9, s_FreeMono12ptBitmaps + 546}, 		// G
	{8, s_FreeMono12ptBitmaps + 574}, 		// H
	{5, s_FreeMono12ptBitmaps + 588}, 		// I
	{8, s_FreeMono12ptBitmaps + 602}, 		// J
	{9, s_FreeMono12ptBitmaps + 616}, 		// K
	{7, s_FreeMono12ptBitmaps + 644}, 		// L
	{10,s_FreeMono12ptBitmaps + 658}, 		// M
	{8, s_FreeMono12ptBitmaps + 686}, 		// N
	{8, s_FreeMono12ptBitmaps + 700}, 		// O
	{7, s_FreeMono12ptBitmaps + 714}, 		// P
	{8, s_FreeMono12ptBitmaps + 728}, 		// Q
	{8, s_FreeMono12ptBitmaps + 742}, 		// R
	{7, s_FreeMono12ptBitmaps + 756}, 		// S
	{8, s_FreeMono12ptBitmaps + 770}, 		// T
	{8, s_FreeMono12ptBitmaps + 784}, 		// U
	{9, s_FreeMono12ptBitmaps + 798}, 		// V
	{9, s_FreeMono12ptBitmaps + 826}, 		// W
	{8, s_FreeMono12ptBitmaps + 854}, 		// X
	{9, s_FreeMono12ptBitmaps + 868}, 		// Y
	{6, s_FreeMono12ptBitmaps + 896}, 		// Z
	{2, s_FreeMono12ptBitmaps + 910}, 		// [
	{6, s_FreeMono12ptBitmaps + 924}, 		// '\'
	{2, s_FreeMono12ptBitmaps + 938}, 		// ]
	{6, s_FreeMono12ptBitmaps + 952}, 		// ^
	{10,s_FreeMono12ptBitmaps + 966}, 		// _
	{2, s_FreeMono12ptBitmaps + 994}, 		// `
	{8, s_FreeMono12ptBitmaps + 1008}, 		// a
	{7, s_FreeMono12ptBitmaps + 1022}, 		// b
	{8, s_FreeMono12ptBitmaps + 1036}, 		// c
	{8, s_FreeMono12ptBitmaps + 1050}, 		// d
	{7, s_FreeMono12ptBitmaps + 1064}, 		// e
	{5, s_FreeMono12ptBitmaps + 1078}, 		// f
	{7, s_FreeMono12ptBitmaps + 1092}, 		// g
	{8, s_FreeMono12ptBitmaps + 1106}, 		// h
	{5, s_FreeMono12ptBitmaps + 1120}, 		// i
	{4, s_FreeMono12ptBitmaps + 1134}, 		// j
	{7, s_FreeMono12ptBitmaps + 1148}, 		// k
	{5, s_FreeMono12ptBitmaps + 1162}, 		// l
	{8, s_FreeMono12ptBitmaps + 1176}, 		// m
	{8, s_FreeMono12ptBitmaps + 1190}, 		// n
	{8, s_FreeMono12ptBitmaps + 1204}, 		// o
	{8, s_FreeMono12ptBitmaps + 1218}, 		// p
	{8, s_FreeMono12ptBitmaps + 1232}, 		// q
	{6, s_FreeMono12ptBitmaps + 1246}, 		// r
	{6, s_FreeMono12ptBitmaps + 1260}, 		// s
	{7, s_FreeMono12ptBitmaps + 1274}, 		// t
	{6, s_FreeMono12ptBitmaps + 1288}, 		// u
	{9, s_FreeMono12ptBitmaps + 1302}, 		// v
	{9, s_FreeMono12ptBitmaps + 1330}, 		// w
	{8, s_FreeMono12ptBitmaps + 1358}, 		// x
	{8, s_FreeMono12ptBitmaps + 1372}, 		// y
	{6, s_FreeMono12ptBitmaps + 1386}, 		// z
	{3, s_FreeMono12ptBitmaps + 1400}, 		// {
	{1, s_FreeMono12ptBitmaps + 1414}, 		// |
	{3, s_FreeMono12ptBitmaps + 1428}, 		// }
	{6, s_FreeMono12ptBitmaps + 1442}, 		// ~
};

// Font information for FreeMono 12pt
const FontDesc_t iFontFreeMono12pt = {
	FONT_TYPE_VAR_WIDTH,
	10,
	14,
	{.pCharDesc = s_FreeMono12ptCharDesc }
};
