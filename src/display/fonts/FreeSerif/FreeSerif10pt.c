// 
//  Font data for FreeSerif 10pt
// 	https://savannah.gnu.org/projects/freefont/
// 
// 	Font bitmap generated by
// 	http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator

#include "display/ifont.h"

// Character bitmaps for FreeSerif 10pt
static const uint8_t s_FreeSerif10ptBitmaps[] = {
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
	0x80, // #
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  

	// @14 '"' (3 pixels wide)
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
	0x00, //    
	0x00, //    
	0x00, //    

	// @28 '#' (6 pixels wide)
	0x00, //       
	0x24, //   #  #
	0x28, //   # # 
	0x28, //   # # 
	0xFC, // ######
	0x48, //  #  # 
	0xFC, // ######
	0x48, //  #  # 
	0x48, //  #  # 
	0x40, //  #    
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @42 '$' (5 pixels wide)
	0x20, //   #  
	0x70, //  ### 
	0xA8, // # # #
	0xA8, // # # #
	0xE0, // ###  
	0x30, //   ## 
	0x28, //   # #
	0xA8, // # # #
	0xA8, // # # #
	0x70, //  ### 
	0x20, //   #  
	0x00, //      
	0x00, //      
	0x00, //      

	// @56 '%' (10 pixels wide)
	0x00, 0x00, //           
	0x30, 0x00, //   ##      
	0x4E, 0x00, //  #  ###   
	0x8C, 0x00, // #   ##    
	0x8C, 0x00, // #   ##    
	0x9B, 0x80, // #  ## ### 
	0x72, 0x40, //  ###  #  #
	0x14, 0x40, //    # #   #
	0x24, 0xC0, //   #  #  ##
	0x27, 0x00, //   #  ###  
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @84 '&' (9 pixels wide)
	0x00, 0x00, //          
	0x18, 0x00, //    ##    
	0x24, 0x00, //   #  #   
	0x24, 0x00, //   #  #   
	0x3B, 0x80, //   ### ###
	0x31, 0x00, //   ##   # 
	0xDA, 0x00, // ## ## #  
	0x8C, 0x00, // #   ##   
	0x86, 0x00, // #    ##  
	0x7B, 0x80, //  #### ###
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @112 ''' (1 pixels wide)
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
	0x00, //  
	0x00, //  
	0x00, //  

	// @126 '(' (3 pixels wide)
	0x00, //    
	0x00, //    
	0x60, //  ##
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x40, //  # 
	0x20, //   #
	0x00, //    
	0x00, //    

	// @140 ')' (3 pixels wide)
	0x00, //    
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
	0x80, // #  
	0x00, //    
	0x00, //    

	// @154 '*' (4 pixels wide)
	0x00, //     
	0x40, //  #  
	0xD0, // ## #
	0x60, //  ## 
	0xD0, // ## #
	0x40, //  #  
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     

	// @168 '+' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
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

	// @182 ',' (2 pixels wide)
	0x00, //   
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
	0x00, //   
	0x00, //   

	// @196 '-' (3 pixels wide)
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
	0x00, //    
	0x00, //    
	0x00, //    

	// @210 '.' (1 pixels wide)
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
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  

	// @224 '/' (4 pixels wide)
	0x00, //     
	0x10, //    #
	0x20, //   # 
	0x20, //   # 
	0x20, //   # 
	0x40, //  #  
	0x40, //  #  
	0x40, //  #  
	0x80, // #   
	0x80, // #   
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     

	// @238 '0' (6 pixels wide)
	0x00, //       
	0x38, //   ### 
	0x48, //  #  # 
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x84, // #    #
	0x48, //  #  # 
	0x78, //  #### 
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @252 '1' (3 pixels wide)
	0x00, //    
	0xC0, // ## 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0xE0, // ###
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    

	// @266 '2' (6 pixels wide)
	0x00, //       
	0x70, //  ###  
	0xC8, // ##  # 
	0x88, // #   # 
	0x08, //     # 
	0x10, //    #  
	0x10, //    #  
	0x20, //   #   
	0x44, //  #   #
	0xF8, // ##### 
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @280 '3' (5 pixels wide)
	0x00, //      
	0x70, //  ### 
	0x88, // #   #
	0x08, //     #
	0x10, //    # 
	0x38, //   ###
	0x08, //     #
	0x08, //     #
	0x08, //     #
	0xF0, // #### 
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @294 '4' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x10, //    # 
	0x70, //  ### 
	0x50, //  # # 
	0x90, // #  # 
	0x90, // #  # 
	0xF8, // #####
	0x10, //    # 
	0x10, //    # 
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @308 '5' (5 pixels wide)
	0x00, //      
	0x38, //   ###
	0x40, //  #   
	0x60, //  ##  
	0x18, //    ##
	0x08, //     #
	0x08, //     #
	0x08, //     #
	0x10, //    # 
	0xE0, // ###  
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @322 '6' (6 pixels wide)
	0x0C, //     ##
	0x30, //   ##  
	0x60, //  ##   
	0x40, //  #    
	0xF8, // ##### 
	0x8C, // #   ##
	0x84, // #    #
	0x84, // #    #
	0x44, //  #   #
	0x78, //  #### 
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @336 '7' (6 pixels wide)
	0x00, //       
	0x7C, //  #####
	0x88, // #   # 
	0x08, //     # 
	0x08, //     # 
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x20, //   #   
	0x20, //   #   
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @350 '8' (5 pixels wide)
	0x00, //      
	0x70, //  ### 
	0x88, // #   #
	0x88, // #   #
	0xD0, // ## # 
	0x30, //   ## 
	0xD8, // ## ##
	0x88, // #   #
	0x88, // #   #
	0x70, //  ### 
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @364 '9' (6 pixels wide)
	0x00, //       
	0x78, //  #### 
	0x88, // #   # 
	0x84, // #    #
	0x84, // #    #
	0xC4, // ##   #
	0x7C, //  #####
	0x08, //     # 
	0x18, //    ## 
	0x30, //   ##  
	0xC0, // ##    
	0x00, //       
	0x00, //       
	0x00, //       

	// @378 ':' (1 pixels wide)
	0x00, //  
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
	0x00, //  

	// @392 ';' (2 pixels wide)
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x80, // # 
	0x00, //   
	0x00, //   
	0x00, //   
	0x00, //   
	0x40, //  #
	0x40, //  #
	0x80, // # 
	0x00, //   
	0x00, //   

	// @406 '<' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x02, //       #
	0x0C, //     ## 
	0x30, //   ##   
	0xC0, // ##     
	0x70, //  ###   
	0x0C, //     ## 
	0x02, //       #
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        

	// @420 '=' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0xFE, // #######
	0x00, //        
	0xFE, // #######
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        

	// @434 '>' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x80, // #      
	0x60, //  ##    
	0x1C, //    ### 
	0x06, //      ##
	0x18, //    ##  
	0x60, //  ##    
	0x80, // #      
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        

	// @448 '?' (4 pixels wide)
	0x00, //     
	0xE0, // ### 
	0xB0, // # ##
	0x10, //    #
	0x10, //    #
	0x20, //   # 
	0x40, //  #  
	0x40, //  #  
	0x00, //     
	0x40, //  #  
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     

	// @462 '@' (10 pixels wide)
	0x00, 0x00, //           
	0x1F, 0x00, //    #####  
	0x61, 0x80, //  ##    ## 
	0x4D, 0x40, //  #  ## # #
	0x9A, 0x40, // #  ## #  #
	0x92, 0x40, // #  #  #  #
	0x92, 0xC0, // #  #  # ##
	0x5F, 0x80, //  # ###### 
	0x60, 0x00, //  ##       
	0x1E, 0x00, //    ####   
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @490 'A' (9 pixels wide)
	0x00, 0x00, //          
	0x08, 0x00, //     #    
	0x08, 0x00, //     #    
	0x1C, 0x00, //    ###   
	0x14, 0x00, //    # #   
	0x26, 0x00, //   #  ##  
	0x22, 0x00, //   #   #  
	0x3F, 0x00, //   ###### 
	0x41, 0x00, //  #     # 
	0xE3, 0x80, // ###   ###
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @518 'B' (7 pixels wide)
	0x00, //        
	0xFC, // ###### 
	0x46, //  #   ##
	0x42, //  #    #
	0x42, //  #    #
	0x7C, //  ##### 
	0x46, //  #   ##
	0x42, //  #    #
	0x42, //  #    #
	0xFC, // ###### 
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        

	// @532 'C' (9 pixels wide)
	0x00, 0x00, //          
	0x3E, 0x80, //   ##### #
	0x61, 0x80, //  ##    ##
	0xC0, 0x80, // ##      #
	0x80, 0x00, // #        
	0x80, 0x00, // #        
	0x80, 0x00, // #        
	0xC0, 0x00, // ##       
	0x60, 0x80, //  ##     #
	0x3F, 0x00, //   ###### 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @560 'D' (8 pixels wide)
	0x00, //         
	0xFC, // ######  
	0x46, //  #   ## 
	0x43, //  #    ##
	0x41, //  #     #
	0x41, //  #     #
	0x41, //  #     #
	0x41, //  #     #
	0x46, //  #   ## 
	0xFC, // ######  
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @574 'E' (8 pixels wide)
	0x00, //         
	0xFF, // ########
	0x41, //  #     #
	0x40, //  #      
	0x42, //  #    # 
	0x7E, //  ###### 
	0x42, //  #    # 
	0x40, //  #      
	0x42, //  #    # 
	0xFE, // ####### 
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @588 'F' (8 pixels wide)
	0x00, //         
	0xFF, // ########
	0x41, //  #     #
	0x40, //  #      
	0x42, //  #    # 
	0x7E, //  ###### 
	0x42, //  #    # 
	0x40, //  #      
	0x40, //  #      
	0xE0, // ###     
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @602 'G' (9 pixels wide)
	0x00, 0x00, //          
	0x3D, 0x00, //   #### # 
	0x63, 0x00, //  ##   ## 
	0x40, 0x00, //  #       
	0x80, 0x00, // #        
	0x83, 0x80, // #     ###
	0x81, 0x00, // #      # 
	0x81, 0x00, // #      # 
	0x61, 0x00, //  ##    # 
	0x3E, 0x00, //   #####  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @630 'H' (9 pixels wide)
	0x00, 0x00, //          
	0xE3, 0x80, // ###   ###
	0x41, 0x00, //  #     # 
	0x41, 0x00, //  #     # 
	0x41, 0x00, //  #     # 
	0x7F, 0x00, //  ####### 
	0x41, 0x00, //  #     # 
	0x41, 0x00, //  #     # 
	0x41, 0x00, //  #     # 
	0xE3, 0x80, // ###   ###
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @658 'I' (3 pixels wide)
	0x00, //    
	0xE0, // ###
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0xE0, // ###
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    

	// @672 'J' (4 pixels wide)
	0x00, //     
	0x70, //  ###
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

	// @686 'K' (8 pixels wide)
	0x00, //         
	0xEF, // ### ####
	0x44, //  #   #  
	0x48, //  #  #   
	0x50, //  # #    
	0x70, //  ###    
	0x58, //  # ##   
	0x4C, //  #  ##  
	0x46, //  #   ## 
	0xEF, // ### ####
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @700 'L' (8 pixels wide)
	0x00, //         
	0xE0, // ###     
	0x40, //  #      
	0x40, //  #      
	0x40, //  #      
	0x40, //  #      
	0x40, //  #      
	0x41, //  #     #
	0x42, //  #    # 
	0xFE, // ####### 
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @714 'M' (11 pixels wide)
	0x00, 0x00, //            
	0xE0, 0x60, // ###      ##
	0x70, 0xC0, //  ###    ## 
	0x70, 0xC0, //  ###    ## 
	0x51, 0x40, //  # #   # # 
	0x59, 0x40, //  # ##  # # 
	0x4A, 0x40, //  #  # #  # 
	0x4E, 0x40, //  #  ###  # 
	0x44, 0x40, //  #   #   # 
	0xE4, 0xE0, // ###  #  ###
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @742 'N' (9 pixels wide)
	0x00, 0x00, //          
	0xE3, 0x80, // ###   ###
	0x61, 0x00, //  ##    # 
	0x71, 0x00, //  ###   # 
	0x59, 0x00, //  # ##  # 
	0x4D, 0x00, //  #  ## # 
	0x45, 0x00, //  #   # # 
	0x43, 0x00, //  #    ## 
	0x43, 0x00, //  #    ## 
	0xE1, 0x00, // ###    # 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @770 'O' (8 pixels wide)
	0x00, //         
	0x3C, //   ####  
	0x42, //  #    # 
	0x81, // #      #
	0x81, // #      #
	0x81, // #      #
	0x81, // #      #
	0x81, // #      #
	0x42, //  #    # 
	0x3C, //   ####  
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @784 'P' (7 pixels wide)
	0x00, //        
	0xFC, // ###### 
	0x46, //  #   ##
	0x42, //  #    #
	0x42, //  #    #
	0x46, //  #   ##
	0x7C, //  ##### 
	0x40, //  #     
	0x40, //  #     
	0xE0, // ###    
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        

	// @798 'Q' (8 pixels wide)
	0x00, //         
	0x3C, //   ####  
	0x42, //  #    # 
	0x81, // #      #
	0x81, // #      #
	0x81, // #      #
	0x81, // #      #
	0x81, // #      #
	0x42, //  #    # 
	0x3C, //   ####  
	0x0C, //     ##  
	0x03, //       ##
	0x00, //         
	0x00, //         

	// @812 'R' (9 pixels wide)
	0x00, 0x00, //          
	0xFC, 0x00, // ######   
	0x46, 0x00, //  #   ##  
	0x42, 0x00, //  #    #  
	0x46, 0x00, //  #   ##  
	0x78, 0x00, //  ####    
	0x5C, 0x00, //  # ###   
	0x4E, 0x00, //  #  ###  
	0x47, 0x00, //  #   ### 
	0xE3, 0x80, // ###   ###
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @840 'S' (5 pixels wide)
	0x00, //      
	0x68, //  ## #
	0x98, // #  ##
	0x80, // #    
	0xC0, // ##   
	0x70, //  ### 
	0x18, //    ##
	0x08, //     #
	0x88, // #   #
	0xF0, // #### 
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @854 'T' (8 pixels wide)
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
	0x00, //         

	// @868 'U' (9 pixels wide)
	0x00, 0x00, //          
	0xE3, 0x80, // ###   ###
	0x41, 0x00, //  #     # 
	0x41, 0x00, //  #     # 
	0x41, 0x00, //  #     # 
	0x41, 0x00, //  #     # 
	0x41, 0x00, //  #     # 
	0x41, 0x00, //  #     # 
	0x63, 0x00, //  ##   ## 
	0x3E, 0x00, //   #####  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @896 'V' (9 pixels wide)
	0x00, 0x00, //          
	0xF3, 0x80, // ####  ###
	0x61, 0x00, //  ##    # 
	0x21, 0x00, //   #    # 
	0x22, 0x00, //   #   #  
	0x12, 0x00, //    #  #  
	0x14, 0x00, //    # #   
	0x0C, 0x00, //     ##   
	0x0C, 0x00, //     ##   
	0x08, 0x00, //     #    
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @924 'W' (12 pixels wide)
	0x00, 0x00, //             
	0xEF, 0x30, // ### ####  ##
	0x46, 0x20, //  #   ##   # 
	0x62, 0x20, //  ##   #   # 
	0x22, 0x40, //   #   #  #  
	0x37, 0x40, //   ## ### #  
	0x15, 0x40, //    # # # #  
	0x19, 0x80, //    ##  ##   
	0x18, 0x80, //    ##   #   
	0x00, 0x80, //         #   
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @952 'X' (9 pixels wide)
	0x00, 0x00, //          
	0xF3, 0x80, // ####  ###
	0x21, 0x00, //   #    # 
	0x32, 0x00, //   ##  #  
	0x1C, 0x00, //    ###   
	0x0C, 0x00, //     ##   
	0x14, 0x00, //    # #   
	0x22, 0x00, //   #   #  
	0x43, 0x00, //  #    ## 
	0xE7, 0x80, // ###  ####
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @980 'Y' (9 pixels wide)
	0x00, 0x00, //          
	0xF3, 0x80, // ####  ###
	0x61, 0x00, //  ##    # 
	0x32, 0x00, //   ##  #  
	0x14, 0x00, //    # #   
	0x08, 0x00, //     #    
	0x08, 0x00, //     #    
	0x08, 0x00, //     #    
	0x08, 0x00, //     #    
	0x1C, 0x00, //    ###   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1008 'Z' (8 pixels wide)
	0x00, //         
	0x7E, //  ###### 
	0x84, // #    #  
	0x0C, //     ##  
	0x08, //     #   
	0x10, //    #    
	0x30, //   ##    
	0x61, //  ##    #
	0x43, //  #    ##
	0xFE, // ####### 
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @1022 '[' (2 pixels wide)
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
	0xC0, // ##
	0x00, //   
	0x00, //   

	// @1036 '\' (4 pixels wide)
	0x00, //     
	0x80, // #   
	0x80, // #   
	0x40, //  #  
	0x40, //  #  
	0x40, //  #  
	0x20, //   # 
	0x20, //   # 
	0x20, //   # 
	0x10, //    #
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     

	// @1050 ']' (2 pixels wide)
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
	0xC0, // ##
	0x00, //   
	0x00, //   

	// @1064 '^' (6 pixels wide)
	0x00, //       
	0x30, //   ##  
	0x30, //   ##  
	0x48, //  #  # 
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

	// @1078 '_' (7 pixels wide)
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

	// @1092 '`' (2 pixels wide)
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

	// @1106 'a' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x70, //  ### 
	0x90, // #  # 
	0x10, //    # 
	0xF0, // #### 
	0x90, // #  # 
	0xF8, // #####
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @1120 'b' (6 pixels wide)
	0x00, //       
	0xC0, // ##    
	0x40, //  #    
	0x40, //  #    
	0x58, //  # ## 
	0x6C, //  ## ##
	0x44, //  #   #
	0x44, //  #   #
	0x44, //  #   #
	0x38, //   ### 
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @1134 'c' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x78, //  ####
	0xC0, // ##   
	0x80, // #    
	0x80, // #    
	0xC0, // ##   
	0x78, //  ####
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @1148 'd' (5 pixels wide)
	0x00, //      
	0x18, //    ##
	0x08, //     #
	0x08, //     #
	0x78, //  ####
	0x88, // #   #
	0x88, // #   #
	0x88, // #   #
	0xC8, // ##  #
	0x78, //  ####
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @1162 'e' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x70, //  ### 
	0xF8, // #####
	0x80, // #    
	0x80, // #    
	0xC8, // ##  #
	0x70, //  ### 
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @1176 'f' (5 pixels wide)
	0x38, //   ###
	0x40, //  #   
	0x40, //  #   
	0x40, //  #   
	0xE0, // ###  
	0x40, //  #   
	0x40, //  #   
	0x40, //  #   
	0x40, //  #   
	0xE0, // ###  
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @1190 'g' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3E, //   #####
	0x48, //  #  #  
	0x48, //  #  #  
	0x30, //   ##   
	0x40, //  #     
	0x78, //  ####  
	0x84, // #    # 
	0x84, // #    # 
	0x78, //  ####  
	0x00, //        

	// @1204 'h' (6 pixels wide)
	0x00, //       
	0xC0, // ##    
	0x40, //  #    
	0x40, //  #    
	0x58, //  # ## 
	0x68, //  ## # 
	0x48, //  #  # 
	0x48, //  #  # 
	0x48, //  #  # 
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @1218 'i' (3 pixels wide)
	0x00, //    
	0x40, //  # 
	0x00, //    
	0x00, //    
	0x40, //  # 
	0xC0, // ## 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0xE0, // ###
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    

	// @1232 'j' (3 pixels wide)
	0x00, //    
	0x20, //   #
	0x00, //    
	0x00, //    
	0x20, //   #
	0x60, //  ##
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0xC0, // ## 
	0x00, //    

	// @1246 'k' (6 pixels wide)
	0x00, //       
	0xC0, // ##    
	0x40, //  #    
	0x40, //  #    
	0x5C, //  # ###
	0x50, //  # #  
	0x60, //  ##   
	0x50, //  # #  
	0x48, //  #  # 
	0xEC, // ### ##
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @1260 'l' (3 pixels wide)
	0x00, //    
	0xC0, // ## 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0xE0, // ###
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    

	// @1274 'm' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0xDD, 0x80, // ## ### ## 
	0x66, 0x80, //  ##  ## # 
	0x44, 0x80, //  #   #  # 
	0x44, 0x80, //  #   #  # 
	0x44, 0x80, //  #   #  # 
	0xEF, 0xC0, // ### ######
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1302 'n' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xD8, // ## ## 
	0x68, //  ## # 
	0x48, //  #  # 
	0x48, //  #  # 
	0x48, //  #  # 
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @1316 'o' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x78, //  #### 
	0x8C, // #   ##
	0x84, // #    #
	0x84, // #    #
	0xC4, // ##   #
	0x78, //  #### 
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @1330 'p' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xD8, // ## ## 
	0x64, //  ##  #
	0x44, //  #   #
	0x44, //  #   #
	0x44, //  #   #
	0x78, //  #### 
	0x40, //  #    
	0x40, //  #    
	0xE0, // ###   
	0x00, //       

	// @1344 'q' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x70, //  ###  
	0xC8, // ##  # 
	0x88, // #   # 
	0x88, // #   # 
	0xC8, // ##  # 
	0x78, //  #### 
	0x08, //     # 
	0x08, //     # 
	0x1C, //    ###
	0x00, //       

	// @1358 'r' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x50, //  # #
	0x60, //  ## 
	0x40, //  #  
	0x40, //  #  
	0x40, //  #  
	0xE0, // ### 
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     

	// @1372 's' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x70, //  ###
	0x90, // #  #
	0xC0, // ##  
	0x30, //   ##
	0x90, // #  #
	0xE0, // ### 
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     

	// @1386 't' (3 pixels wide)
	0x00, //    
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
	0x00, //    
	0x00, //    

	// @1400 'u' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xD8, // ## ## 
	0x48, //  #  # 
	0x48, //  #  # 
	0x48, //  #  # 
	0x48, //  #  # 
	0x7C, //  #####
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @1414 'v' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xEC, // ### ##
	0x48, //  #  # 
	0x48, //  #  # 
	0x30, //   ##  
	0x30, //   ##  
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @1428 'w' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0xDD, 0x80, // ## ### ##
	0x49, 0x00, //  #  #  # 
	0x49, 0x00, //  #  #  # 
	0x36, 0x00, //   ## ##  
	0x36, 0x00, //   ## ##  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1456 'x' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xFC, // ######
	0x50, //  # #  
	0x20, //   #   
	0x30, //   ##  
	0x48, //  #  # 
	0xCC, // ##  ##
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @1470 'y' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xEC, // ### ##
	0x48, //  #  # 
	0x48, //  #  # 
	0x28, //   # # 
	0x30, //   ##  
	0x10, //    #  
	0x20, //   #   
	0x20, //   #   
	0xC0, // ##    
	0x00, //       

	// @1484 'z' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xF8, // ##### 
	0x90, // #  #  
	0x10, //    #  
	0x20, //   #   
	0x44, //  #   #
	0xF8, // ##### 
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @1498 '{' (2 pixels wide)
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
	0x00, //   

	// @1512 '|' (1 pixels wide)
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
	0x00, //  
	0x00, //  
	0x00, //  
	0x00, //  

	// @1526 '}' (2 pixels wide)
	0x00, //   
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

	// @1540 '~' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0xC8, // ##  #
	0x38, //   ###
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
};

// Character descriptors for FreeSerif 10pt
// { [Char width in bits], [Offset into freeSerif_10ptCharBitmaps in bytes] }
static const CharDesc_t s_FreeSerif10ptCharDesc[] = {
	{1, s_FreeSerif10ptBitmaps + 0}, 		// !
	{3, s_FreeSerif10ptBitmaps + 14}, 		// "
	{6, s_FreeSerif10ptBitmaps + 28}, 		// #
	{5, s_FreeSerif10ptBitmaps + 42}, 		// $
	{10,s_FreeSerif10ptBitmaps + 56}, 		// %
	{9, s_FreeSerif10ptBitmaps + 84}, 		// &
	{1, s_FreeSerif10ptBitmaps + 112}, 		// '
	{3, s_FreeSerif10ptBitmaps + 126}, 		// (
	{3, s_FreeSerif10ptBitmaps + 140}, 		// )
	{4, s_FreeSerif10ptBitmaps + 154}, 		// *
	{5, s_FreeSerif10ptBitmaps + 168}, 		// +
	{2, s_FreeSerif10ptBitmaps + 182}, 		// ,
	{3, s_FreeSerif10ptBitmaps + 196}, 		// -
	{1, s_FreeSerif10ptBitmaps + 210}, 		// .
	{4, s_FreeSerif10ptBitmaps + 224}, 		// /
	{6, s_FreeSerif10ptBitmaps + 238}, 		// 0
	{3, s_FreeSerif10ptBitmaps + 252}, 		// 1
	{6, s_FreeSerif10ptBitmaps + 266}, 		// 2
	{5, s_FreeSerif10ptBitmaps + 280}, 		// 3
	{5, s_FreeSerif10ptBitmaps + 294}, 		// 4
	{5, s_FreeSerif10ptBitmaps + 308}, 		// 5
	{6, s_FreeSerif10ptBitmaps + 322}, 		// 6
	{6, s_FreeSerif10ptBitmaps + 336}, 		// 7
	{5, s_FreeSerif10ptBitmaps + 350}, 		// 8
	{6, s_FreeSerif10ptBitmaps + 364}, 		// 9
	{1, s_FreeSerif10ptBitmaps + 378}, 		// :
	{2, s_FreeSerif10ptBitmaps + 392}, 		// ;
	{7, s_FreeSerif10ptBitmaps + 406}, 		// <
	{7, s_FreeSerif10ptBitmaps + 420}, 		// =
	{7, s_FreeSerif10ptBitmaps + 434}, 		// >
	{4, s_FreeSerif10ptBitmaps + 448}, 		// ?
	{10,s_FreeSerif10ptBitmaps + 462}, 		// @
	{9, s_FreeSerif10ptBitmaps + 490}, 		// A
	{7, s_FreeSerif10ptBitmaps + 518}, 		// B
	{9, s_FreeSerif10ptBitmaps + 532}, 		// C
	{8, s_FreeSerif10ptBitmaps + 560}, 		// D
	{8, s_FreeSerif10ptBitmaps + 574}, 		// E
	{8, s_FreeSerif10ptBitmaps + 588}, 		// F
	{9, s_FreeSerif10ptBitmaps + 602}, 		// G
	{9, s_FreeSerif10ptBitmaps + 630}, 		// H
	{3, s_FreeSerif10ptBitmaps + 658}, 		// I
	{4, s_FreeSerif10ptBitmaps + 672}, 		// J
	{8, s_FreeSerif10ptBitmaps + 686}, 		// K
	{8, s_FreeSerif10ptBitmaps + 700}, 		// L
	{11,s_FreeSerif10ptBitmaps + 714}, 		// M
	{9, s_FreeSerif10ptBitmaps + 742}, 		// N
	{8, s_FreeSerif10ptBitmaps + 770}, 		// O
	{7, s_FreeSerif10ptBitmaps + 784}, 		// P
	{8, s_FreeSerif10ptBitmaps + 798}, 		// Q
	{9, s_FreeSerif10ptBitmaps + 812}, 		// R
	{5, s_FreeSerif10ptBitmaps + 840}, 		// S
	{8, s_FreeSerif10ptBitmaps + 854}, 		// T
	{9, s_FreeSerif10ptBitmaps + 868}, 		// U
	{9, s_FreeSerif10ptBitmaps + 896}, 		// V
	{12,s_FreeSerif10ptBitmaps + 924}, 		// W
	{9, s_FreeSerif10ptBitmaps + 952}, 		// X
	{9, s_FreeSerif10ptBitmaps + 980}, 		// Y
	{8, s_FreeSerif10ptBitmaps + 1008}, 	// Z
	{2, s_FreeSerif10ptBitmaps + 1022}, 	// [
	{4, s_FreeSerif10ptBitmaps + 1036}, 	// '\'
	{2, s_FreeSerif10ptBitmaps + 1050}, 	// ]
	{6, s_FreeSerif10ptBitmaps + 1064}, 	// ^
	{7, s_FreeSerif10ptBitmaps + 1078}, 	// _
	{2, s_FreeSerif10ptBitmaps + 1092}, 	// `
	{5, s_FreeSerif10ptBitmaps + 1106}, 	// a
	{6, s_FreeSerif10ptBitmaps + 1120}, 	// b
	{5, s_FreeSerif10ptBitmaps + 1134}, 	// c
	{5, s_FreeSerif10ptBitmaps + 1148}, 	// d
	{5, s_FreeSerif10ptBitmaps + 1162}, 	// e
	{5, s_FreeSerif10ptBitmaps + 1176}, 	// f
	{7, s_FreeSerif10ptBitmaps + 1190}, 	// g
	{6, s_FreeSerif10ptBitmaps + 1204}, 	// h
	{3, s_FreeSerif10ptBitmaps + 1218}, 	// i
	{3, s_FreeSerif10ptBitmaps + 1232}, 	// j
	{6, s_FreeSerif10ptBitmaps + 1246}, 	// k
	{3, s_FreeSerif10ptBitmaps + 1260}, 	// l
	{10,s_FreeSerif10ptBitmaps + 1274}, 	// m
	{6, s_FreeSerif10ptBitmaps + 1302}, 	// n
	{6, s_FreeSerif10ptBitmaps + 1316}, 	// o
	{6, s_FreeSerif10ptBitmaps + 1330}, 	// p
	{6, s_FreeSerif10ptBitmaps + 1344}, 	// q
	{4, s_FreeSerif10ptBitmaps + 1358}, 	// r
	{4, s_FreeSerif10ptBitmaps + 1372}, 	// s
	{3, s_FreeSerif10ptBitmaps + 1386}, 	// t
	{6, s_FreeSerif10ptBitmaps + 1400}, 	// u
	{6, s_FreeSerif10ptBitmaps + 1414}, 	// v
	{9, s_FreeSerif10ptBitmaps + 1428}, 	// w
	{6, s_FreeSerif10ptBitmaps + 1456}, 	// x
	{6, s_FreeSerif10ptBitmaps + 1470}, 	// y
	{6, s_FreeSerif10ptBitmaps + 1484}, 	// z
	{2, s_FreeSerif10ptBitmaps + 1498}, 	// {
	{1, s_FreeSerif10ptBitmaps + 1512}, 	// |
	{2, s_FreeSerif10ptBitmaps + 1526}, 	// }
	{5, s_FreeSerif10ptBitmaps + 1540}, 	// ~
};

// Font information for FreeSerif 10pt
const FontDesc_t iFontFreeSerif10pt = {
	FONT_TYPE_VAR_WIDTH,
	12,
	14,
	{ .pCharDesc = s_FreeSerif10ptCharDesc }
};
