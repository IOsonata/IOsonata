// 
//  Font data for FreeSans 10pt
// 	https://savannah.gnu.org/projects/freefont/
// 
// 	Font bitmap generated by
// 	http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator

#include "display/ifont.h"

// Character bitmaps for FreeSans 10pt
static const uint8_t s_FreeSansBoldIta10ptBitmaps[] = {
	// @0 '!' (4 pixels wide)
	0x00, //     
	0x30, //   ##
	0x30, //   ##
	0x30, //   ##
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0x40, //  #  
	0x00, //     
	0xC0, // ##  
	0xC0, // ##  
	0x00, //     
	0x00, //     
	0x00, //     

	// @14 '"' (5 pixels wide)
	0x00, //      
	0xD8, // ## ##
	0xD8, // ## ##
	0x90, // #  # 
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

	// @28 '#' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x12, //    #  # 
	0x7F, //  #######
	0x7E, //  ###### 
	0x2C, //   # ##  
	0x68, //  ## #   
	0xFC, // ######  
	0xFC, // ######  
	0x90, // #  #    
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @42 '$' (7 pixels wide)
	0x00, //        
	0x08, //     #  
	0x3E, //   #####
	0x6E, //  ## ###
	0x76, //  ### ##
	0x70, //  ###   
	0x38, //   ###  
	0x1C, //    ### 
	0xDC, // ## ### 
	0xEC, // ### ## 
	0xF8, // #####  
	0x20, //   #    
	0x20, //   #    
	0x00, //        

	// @56 '%' (9 pixels wide)
	0x00, 0x00, //          
	0x60, 0x00, //  ##      
	0xF1, 0x00, // ####   # 
	0x92, 0x00, // #  #  #  
	0x94, 0x00, // #  # #   
	0xF8, 0x00, // #####    
	0x68, 0x00, //  ## #    
	0x13, 0x00, //    #  ## 
	0x27, 0x80, //   #  ####
	0x27, 0x80, //   #  ####
	0x43, 0x00, //  #    ## 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @84 '&' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x1C, 0x00, //    ###   
	0x3E, 0x00, //   #####  
	0x36, 0x00, //   ## ##  
	0x3C, 0x00, //   ####   
	0x39, 0x80, //   ###  ##
	0xCF, 0x00, // ##  #### 
	0xCE, 0x00, // ##  ###  
	0xFF, 0x00, // ######## 
	0x77, 0x00, //  ### ### 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @112 ''' (2 pixels wide)
	0x00, //   
	0xC0, // ##
	0xC0, // ##
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

	// @126 '(' (5 pixels wide)
	0x00, //      
	0x08, //     #
	0x10, //    # 
	0x30, //   ## 
	0x60, //  ##  
	0x60, //  ##  
	0xC0, // ##   
	0xC0, // ##   
	0xC0, // ##   
	0xC0, // ##   
	0xC0, // ##   
	0x40, //  #   
	0x60, //  ##  
	0x00, //      

	// @140 ')' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x30, //   ## 
	0x10, //    # 
	0x18, //    ##
	0x18, //    ##
	0x18, //    ##
	0x18, //    ##
	0x18, //    ##
	0x30, //   ## 
	0x30, //   ## 
	0x60, //  ##  
	0x40, //  #   
	0x80, // #    

	// @154 '*' (4 pixels wide)
	0x00, //     
	0x20, //   # 
	0xB0, // # ##
	0x60, //  ## 
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

	// @168 '+' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x18, //    ##  
	0xFE, // #######
	0xFE, // #######
	0x30, //   ##   
	0x30, //   ##   
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
	0xC0, // ##
	0x40, //  #
	0x80, // # 
	0x80, // # 
	0x00, //   

	// @196 '-' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0xF0, // ####
	0xF0, // ####
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     

	// @210 '.' (2 pixels wide)
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

	// @224 '/' (5 pixels wide)
	0x00, //      
	0x00, //      
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
	0x00, //      
	0x00, //      

	// @238 '0' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x1C, //    ### 
	0x7E, //  ######
	0x66, //  ##  ##
	0xE6, // ###  ##
	0xC6, // ##   ##
	0xCE, // ##  ###
	0xCC, // ##  ## 
	0xFC, // ###### 
	0x70, //  ###   
	0x00, //        
	0x00, //        
	0x00, //        

	// @252 '1' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x18, //    ##
	0xF8, // #####
	0xF0, // #### 
	0x30, //   ## 
	0x30, //   ## 
	0x30, //   ## 
	0x20, //   #  
	0x60, //  ##  
	0x60, //  ##  
	0x00, //      
	0x00, //      
	0x00, //      

	// @266 '2' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x1C, //    ### 
	0x3E, //   #####
	0x66, //  ##  ##
	0x06, //      ##
	0x0C, //     ## 
	0x18, //    ##  
	0x60, //  ##    
	0xFC, // ###### 
	0xF8, // #####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @280 '3' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x3C, //   #### 
	0x7E, //  ######
	0x66, //  ##  ##
	0x1C, //    ### 
	0x1C, //    ### 
	0x0C, //     ## 
	0xCC, // ##  ## 
	0xFC, // ###### 
	0x78, //  ####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @294 '4' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x0E, //     ###
	0x1C, //    ### 
	0x1C, //    ### 
	0x2C, //   # ## 
	0x4C, //  #  ## 
	0xFE, // #######
	0xFC, // ###### 
	0x18, //    ##  
	0x18, //    ##  
	0x00, //        
	0x00, //        
	0x00, //        

	// @308 '5' (8 pixels wide)
	0x00, //         
	0x1F, //    #####
	0x3E, //   ##### 
	0x20, //   #     
	0x3C, //   ####  
	0x7E, //  ###### 
	0x66, //  ##  ## 
	0x06, //      ## 
	0xCE, // ##  ### 
	0xFC, // ######  
	0x78, //  ####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @322 '6' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x1E, //    #### 
	0x3E, //   ##### 
	0x7F, //  #######
	0x7E, //  ###### 
	0xE6, // ###  ## 
	0xC6, // ##   ## 
	0xCE, // ##  ### 
	0xFC, // ######  
	0x78, //  ####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @336 '7' (7 pixels wide)
	0x00, //        
	0xFE, // #######
	0xFE, // #######
	0x0C, //     ## 
	0x18, //    ##  
	0x30, //   ##   
	0x20, //   #    
	0x60, //  ##    
	0x40, //  #     
	0xC0, // ##     
	0xC0, // ##     
	0x00, //        
	0x00, //        
	0x00, //        

	// @350 '8' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x1E, //    #### 
	0x7F, //  #######
	0x63, //  ##   ##
	0x3E, //   ##### 
	0x7E, //  ###### 
	0xC6, // ##   ## 
	0xC6, // ##   ## 
	0xFC, // ######  
	0x78, //  ####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @364 '9' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x1C, //    ### 
	0x3E, //   #####
	0x66, //  ##  ##
	0x66, //  ##  ##
	0x66, //  ##  ##
	0x7E, //  ######
	0xFC, // ###### 
	0xFC, // ###### 
	0x78, //  ####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @378 ':' (3 pixels wide)
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x60, //  ##
	0x60, //  ##
	0x00, //    
	0x00, //    
	0x00, //    
	0xC0, // ## 
	0xC0, // ## 
	0x00, //    
	0x00, //    
	0x00, //    

	// @392 ';' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x30, //   ##
	0x30, //   ##
	0x00, //     
	0x00, //     
	0x00, //     
	0xC0, // ##  
	0x40, //  #  
	0x80, // #   
	0x80, // #   
	0x00, //     

	// @406 '<' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x06, //      ##
	0x3E, //   #####
	0xF0, // ####   
	0xF0, // ####   
	0x3C, //   #### 
	0x04, //      # 
	0x00, //        
	0x00, //        
	0x00, //        

	// @420 '=' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x7E, //  ######
	0x7E, //  ######
	0x00, //        
	0xFC, // ###### 
	0xFC, // ###### 
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        

	// @434 '>' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x40, //  #     
	0x78, //  ####  
	0x1E, //    ####
	0x1E, //    ####
	0xF8, // #####  
	0xC0, // ##     
	0x00, //        
	0x00, //        
	0x00, //        

	// @448 '?' (6 pixels wide)
	0x00, //       
	0x78, //  #### 
	0xFC, // ######
	0xCC, // ##  ##
	0x0C, //     ##
	0x18, //    ## 
	0x30, //   ##  
	0x60, //  ##   
	0x00, //       
	0x60, //  ##   
	0x60, //  ##   
	0x00, //       
	0x00, //       
	0x00, //       

	// @462 '@' (12 pixels wide)
	0x03, 0xC0, //       ####  
	0x0F, 0xE0, //     ####### 
	0x3C, 0x70, //   ####   ###
	0x70, 0x30, //  ###      ##
	0x67, 0x50, //  ##  ### # #
	0xCC, 0x90, // ##  ##  #  #
	0xD8, 0x90, // ## ##   #  #
	0xD9, 0x20, // ## ##  #  # 
	0xDF, 0xE0, // ## ######## 
	0xCC, 0xC0, // ##  ##  ##  
	0x60, 0x00, //  ##         
	0x71, 0x00, //  ###   #    
	0x1F, 0x00, //    #####    
	0x00, 0x00, //             

	// @490 'A' (8 pixels wide)
	0x00, //         
	0x0E, //     ### 
	0x0E, //     ### 
	0x1E, //    #### 
	0x1E, //    #### 
	0x36, //   ## ## 
	0x33, //   ##  ##
	0x7F, //  #######
	0x7F, //  #######
	0xC3, // ##    ##
	0xC3, // ##    ##
	0x00, //         
	0x00, //         
	0x00, //         

	// @504 'B' (9 pixels wide)
	0x00, 0x00, //          
	0x3F, 0x00, //   ###### 
	0x3F, 0x80, //   #######
	0x31, 0x80, //   ##   ##
	0x61, 0x80, //  ##    ##
	0x7F, 0x00, //  ####### 
	0x7F, 0x00, //  ####### 
	0x63, 0x00, //  ##   ## 
	0x63, 0x00, //  ##   ## 
	0xFE, 0x00, // #######  
	0xFC, 0x00, // ######   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @532 'C' (9 pixels wide)
	0x00, 0x00, //          
	0x1F, 0x00, //    ##### 
	0x3F, 0x80, //   #######
	0x71, 0x80, //  ###   ##
	0x60, 0x00, //  ##      
	0xC0, 0x00, // ##       
	0xC0, 0x00, // ##       
	0xC0, 0x00, // ##       
	0xC3, 0x00, // ##    ## 
	0x7E, 0x00, //  ######  
	0x3C, 0x00, //   ####   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @560 'D' (9 pixels wide)
	0x00, 0x00, //          
	0x3F, 0x00, //   ###### 
	0x3F, 0x80, //   #######
	0x31, 0x80, //   ##   ##
	0x61, 0x80, //  ##    ##
	0x61, 0x80, //  ##    ##
	0x61, 0x80, //  ##    ##
	0x63, 0x00, //  ##   ## 
	0x43, 0x00, //  #    ## 
	0xFE, 0x00, // #######  
	0xFC, 0x00, // ######   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @588 'E' (9 pixels wide)
	0x00, 0x00, //          
	0x3F, 0x80, //   #######
	0x3F, 0x80, //   #######
	0x30, 0x00, //   ##     
	0x60, 0x00, //  ##      
	0x7F, 0x00, //  ####### 
	0x7F, 0x00, //  ####### 
	0x60, 0x00, //  ##      
	0x40, 0x00, //  #       
	0xFE, 0x00, // #######  
	0xFE, 0x00, // #######  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @616 'F' (9 pixels wide)
	0x00, 0x00, //          
	0x3F, 0x80, //   #######
	0x3F, 0x00, //   ###### 
	0x20, 0x00, //   #      
	0x60, 0x00, //  ##      
	0x7E, 0x00, //  ######  
	0x7E, 0x00, //  ######  
	0x60, 0x00, //  ##      
	0xC0, 0x00, // ##       
	0xC0, 0x00, // ##       
	0xC0, 0x00, // ##       
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @644 'G' (9 pixels wide)
	0x00, 0x00, //          
	0x1F, 0x00, //    ##### 
	0x3F, 0x80, //   #######
	0x71, 0x80, //  ###   ##
	0x60, 0x00, //  ##      
	0xC0, 0x00, // ##       
	0xC7, 0x80, // ##   ####
	0xC7, 0x80, // ##   ####
	0xC1, 0x80, // ##     ##
	0xFF, 0x00, // ######## 
	0x3D, 0x00, //   #### # 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @672 'H' (9 pixels wide)
	0x00, 0x00, //          
	0x30, 0x80, //   ##    #
	0x31, 0x80, //   ##   ##
	0x61, 0x80, //  ##    ##
	0x7F, 0x80, //  ########
	0x7F, 0x80, //  ########
	0x61, 0x00, //  ##    # 
	0x63, 0x00, //  ##   ## 
	0xC3, 0x00, // ##    ## 
	0xC3, 0x00, // ##    ## 
	0xC3, 0x00, // ##    ## 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @700 'I' (4 pixels wide)
	0x00, //     
	0x30, //   ##
	0x30, //   ##
	0x20, //   # 
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0x40, //  #  
	0xC0, // ##  
	0xC0, // ##  
	0x00, //     
	0x00, //     
	0x00, //     

	// @714 'J' (7 pixels wide)
	0x00, //        
	0x06, //      ##
	0x06, //      ##
	0x06, //      ##
	0x06, //      ##
	0x0C, //     ## 
	0x0C, //     ## 
	0xCC, // ##  ## 
	0xCC, // ##  ## 
	0xF8, // #####  
	0x70, //  ###   
	0x00, //        
	0x00, //        
	0x00, //        

	// @728 'K' (9 pixels wide)
	0x00, 0x00, //          
	0x31, 0x80, //   ##   ##
	0x33, 0x00, //   ##  ## 
	0x66, 0x00, //  ##  ##  
	0x6C, 0x00, //  ## ##   
	0x78, 0x00, //  ####    
	0x7C, 0x00, //  #####   
	0x6C, 0x00, //  ## ##   
	0xC6, 0x00, // ##   ##  
	0xC6, 0x00, // ##   ##  
	0xC3, 0x00, // ##    ## 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @756 'L' (7 pixels wide)
	0x00, //        
	0x30, //   ##   
	0x30, //   ##   
	0x30, //   ##   
	0x60, //  ##    
	0x60, //  ##    
	0x60, //  ##    
	0x60, //  ##    
	0x60, //  ##    
	0xFE, // #######
	0xFE, // #######
	0x00, //        
	0x00, //        
	0x00, //        

	// @770 'M' (11 pixels wide)
	0x00, 0x00, //            
	0x38, 0x60, //   ###    ##
	0x38, 0xE0, //   ###   ###
	0x38, 0xE0, //   ###   ###
	0x39, 0x40, //   ###  # # 
	0x6D, 0xC0, //  ## ## ### 
	0x6E, 0xC0, //  ## ### ## 
	0x6E, 0xC0, //  ## ### ## 
	0x4E, 0xC0, //  #  ### ## 
	0xCC, 0x80, // ##  ##  #  
	0xCD, 0x80, // ##  ## ##  
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @798 'N' (9 pixels wide)
	0x00, 0x00, //          
	0x30, 0x80, //   ##    #
	0x31, 0x80, //   ##   ##
	0x79, 0x80, //  ####  ##
	0x79, 0x80, //  ####  ##
	0x69, 0x80, //  ## #  ##
	0x6D, 0x00, //  ## ## # 
	0x4F, 0x00, //  #  #### 
	0xC7, 0x00, // ##   ### 
	0xC7, 0x00, // ##   ### 
	0xC7, 0x00, // ##   ### 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @826 'O' (10 pixels wide)
	0x00, 0x00, //           
	0x0F, 0x00, //     ####  
	0x3F, 0x80, //   ####### 
	0x71, 0xC0, //  ###   ###
	0x60, 0xC0, //  ##     ##
	0xC0, 0xC0, // ##      ##
	0xC0, 0xC0, // ##      ##
	0xC1, 0x80, // ##     ## 
	0xE3, 0x80, // ###   ### 
	0x7F, 0x00, //  #######  
	0x3C, 0x00, //   ####    
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @854 'P' (9 pixels wide)
	0x00, 0x00, //          
	0x3F, 0x00, //   ###### 
	0x3F, 0x80, //   #######
	0x31, 0x80, //   ##   ##
	0x61, 0x80, //  ##    ##
	0x63, 0x80, //  ##   ###
	0x7F, 0x00, //  ####### 
	0x7E, 0x00, //  ######  
	0x40, 0x00, //  #       
	0xC0, 0x00, // ##       
	0xC0, 0x00, // ##       
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @882 'Q' (10 pixels wide)
	0x00, 0x00, //           
	0x0F, 0x00, //     ####  
	0x3F, 0x80, //   ####### 
	0x71, 0xC0, //  ###   ###
	0x60, 0xC0, //  ##     ##
	0xC0, 0xC0, // ##      ##
	0xC0, 0xC0, // ##      ##
	0xC7, 0x80, // ##   #### 
	0xE7, 0x80, // ###  #### 
	0x7F, 0x00, //  #######  
	0x3D, 0x80, //   #### ## 
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @910 'R' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0x80, //   ####### 
	0x3F, 0xC0, //   ########
	0x30, 0xC0, //   ##    ##
	0x60, 0xC0, //  ##     ##
	0x7F, 0x80, //  ######## 
	0x7F, 0x00, //  #######  
	0x63, 0x00, //  ##   ##  
	0x63, 0x00, //  ##   ##  
	0xC3, 0x00, // ##    ##  
	0xC3, 0x00, // ##    ##  
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @938 'S' (9 pixels wide)
	0x00, 0x00, //          
	0x1F, 0x00, //    ##### 
	0x3F, 0x80, //   #######
	0x61, 0x80, //  ##    ##
	0x60, 0x00, //  ##      
	0x78, 0x00, //  ####    
	0x3F, 0x00, //   ###### 
	0x03, 0x00, //       ## 
	0xC3, 0x00, // ##    ## 
	0xFE, 0x00, // #######  
	0x7C, 0x00, //  #####   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @966 'T' (8 pixels wide)
	0x00, //         
	0xFF, // ########
	0xFF, // ########
	0x18, //    ##   
	0x30, //   ##    
	0x30, //   ##    
	0x30, //   ##    
	0x30, //   ##    
	0x30, //   ##    
	0x60, //  ##     
	0x60, //  ##     
	0x00, //         
	0x00, //         
	0x00, //         

	// @980 'U' (8 pixels wide)
	0x00, //         
	0x63, //  ##   ##
	0x63, //  ##   ##
	0x63, //  ##   ##
	0x63, //  ##   ##
	0xC3, // ##    ##
	0xC6, // ##   ## 
	0xC6, // ##   ## 
	0xC6, // ##   ## 
	0xFC, // ######  
	0x78, //  ####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @994 'V' (8 pixels wide)
	0x00, //         
	0xC3, // ##    ##
	0x43, //  #    ##
	0x46, //  #   ## 
	0x66, //  ##  ## 
	0x6C, //  ## ##  
	0x6C, //  ## ##  
	0x78, //  ####   
	0x78, //  ####   
	0x70, //  ###    
	0x60, //  ##     
	0x00, //         
	0x00, //         
	0x00, //         

	// @1008 'W' (13 pixels wide)
	0x00, 0x00, //              
	0xC7, 0x18, // ##   ###   ##
	0xC7, 0x30, // ##   ###  ## 
	0xCF, 0x30, // ##  ####  ## 
	0xCB, 0x20, // ##  # ##  #  
	0xDB, 0x60, // ## ## ## ##  
	0xD3, 0x40, // ## #  ## #   
	0xF3, 0xC0, // ####  ####   
	0xF3, 0xC0, // ####  ####   
	0x63, 0x80, //  ##   ###    
	0x63, 0x80, //  ##   ###    
	0x00, 0x00, //              
	0x00, 0x00, //              
	0x00, 0x00, //              

	// @1036 'X' (9 pixels wide)
	0x00, 0x00, //          
	0x31, 0x80, //   ##   ##
	0x33, 0x00, //   ##  ## 
	0x3E, 0x00, //   #####  
	0x1E, 0x00, //    ####  
	0x1C, 0x00, //    ###   
	0x1C, 0x00, //    ###   
	0x3C, 0x00, //   ####   
	0x6E, 0x00, //  ## ###  
	0xE6, 0x00, // ###  ##  
	0xC6, 0x00, // ##   ##  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1064 'Y' (7 pixels wide)
	0x00, //        
	0xC6, // ##   ##
	0xCC, // ##  ## 
	0xCC, // ##  ## 
	0xD8, // ## ##  
	0x70, //  ###   
	0x70, //  ###   
	0x60, //  ##    
	0x60, //  ##    
	0x60, //  ##    
	0x60, //  ##    
	0x00, //        
	0x00, //        
	0x00, //        

	// @1078 'Z' (9 pixels wide)
	0x00, 0x00, //          
	0x3F, 0x80, //   #######
	0x3F, 0x80, //   #######
	0x03, 0x80, //       ###
	0x07, 0x00, //      ### 
	0x0E, 0x00, //     ###  
	0x1C, 0x00, //    ###   
	0x38, 0x00, //   ###    
	0x70, 0x00, //  ###     
	0x7F, 0x00, //  ####### 
	0xFF, 0x00, // ######## 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1106 '[' (6 pixels wide)
	0x00, //       
	0x1C, //    ###
	0x3C, //   ####
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0x70, //  ###  
	0x60, //  ##   
	0x60, //  ##   
	0x60, //  ##   
	0x60, //  ##   
	0x70, //  ###  
	0xF0, // ####  

	// @1120 '\' (2 pixels wide)
	0x00, //   
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x80, // # 
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x00, //   
	0x00, //   
	0x00, //   

	// @1134 ']' (6 pixels wide)
	0x00, //       
	0x3C, //   ####
	0x38, //   ### 
	0x18, //    ## 
	0x18, //    ## 
	0x18, //    ## 
	0x18, //    ## 
	0x38, //   ### 
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0xF0, // ####  
	0xE0, // ###   

	// @1148 '^' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x18, //    ##
	0x38, //   ###
	0x78, //  ####
	0x48, //  #  #
	0xC8, // ##  #
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @1162 '_' (8 pixels wide)
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
	0x00, //         
	0xFF, // ########

	// @1176 '`' (2 pixels wide)
	0x00, //   
	0xC0, // ##
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

	// @1190 'a' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3C, //   #### 
	0x7E, //  ######
	0x66, //  ##  ##
	0x7E, //  ######
	0xCC, // ##  ## 
	0xFC, // ###### 
	0x6C, //  ## ## 
	0x00, //        
	0x00, //        
	0x00, //        

	// @1204 'b' (7 pixels wide)
	0x00, //        
	0x30, //   ##   
	0x60, //  ##    
	0x60, //  ##    
	0x7C, //  ##### 
	0x7E, //  ######
	0x66, //  ##  ##
	0xC6, // ##   ##
	0xCE, // ##  ###
	0xFC, // ###### 
	0xF8, // #####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @1218 'c' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3C, //   #### 
	0x7E, //  ######
	0xE6, // ###  ##
	0xC0, // ##     
	0xCC, // ##  ## 
	0xFC, // ###### 
	0x78, //  ####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @1232 'd' (8 pixels wide)
	0x00, //         
	0x03, //       ##
	0x03, //       ##
	0x03, //       ##
	0x3E, //   ##### 
	0x7E, //  ###### 
	0xE6, // ###  ## 
	0xC6, // ##   ## 
	0xCE, // ##  ### 
	0xFC, // ######  
	0x7C, //  #####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @1246 'e' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3C, //   #### 
	0x7E, //  ######
	0xFE, // #######
	0xFE, // #######
	0xCC, // ##  ## 
	0xFC, // ###### 
	0x78, //  ####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @1260 'f' (5 pixels wide)
	0x00, //      
	0x18, //    ##
	0x38, //   ###
	0x78, //  ####
	0x78, //  ####
	0x60, //  ##  
	0x60, //  ##  
	0x60, //  ##  
	0x60, //  ##  
	0xC0, // ##   
	0xC0, // ##   
	0x00, //      
	0x00, //      
	0x00, //      

	// @1274 'g' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x1D, //    ### #
	0x3F, //   ######
	0x73, //  ###  ##
	0x63, //  ##   ##
	0x63, //  ##   ##
	0x7E, //  ###### 
	0x3E, //   ##### 
	0xC6, // ##   ## 
	0x7C, //  #####  
	0x78, //  ####   

	// @1288 'h' (7 pixels wide)
	0x00, //        
	0x30, //   ##   
	0x20, //   #    
	0x60, //  ##    
	0x7C, //  ##### 
	0x7E, //  ######
	0x66, //  ##  ##
	0x46, //  #   ##
	0xC4, // ##   # 
	0xCC, // ##  ## 
	0xCC, // ##  ## 
	0x00, //        
	0x00, //        
	0x00, //        

	// @1302 'i' (4 pixels wide)
	0x00, //     
	0x30, //   ##
	0x30, //   ##
	0x00, //     
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0x40, //  #  
	0xC0, // ##  
	0xC0, // ##  
	0x00, //     
	0x00, //     
	0x00, //     

	// @1316 'j' (6 pixels wide)
	0x00, //       
	0x0C, //     ##
	0x08, //     # 
	0x00, //       
	0x18, //    ## 
	0x18, //    ## 
	0x18, //    ## 
	0x10, //    #  
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0x60, //  ##   
	0xE0, // ###   

	// @1330 'k' (7 pixels wide)
	0x00, //        
	0x30, //   ##   
	0x60, //  ##    
	0x60, //  ##    
	0x66, //  ##  ##
	0x6C, //  ## ## 
	0x78, //  ####  
	0xF8, // #####  
	0xD8, // ## ##  
	0xDC, // ## ### 
	0xCC, // ##  ## 
	0x00, //        
	0x00, //        
	0x00, //        

	// @1344 'l' (4 pixels wide)
	0x00, //     
	0x30, //   ##
	0x30, //   ##
	0x20, //   # 
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0x40, //  #  
	0xC0, // ##  
	0xC0, // ##  
	0x00, //     
	0x00, //     
	0x00, //     

	// @1358 'm' (11 pixels wide)
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x6C, 0xC0, //  ## ##  ## 
	0x7F, 0xE0, //  ##########
	0x66, 0x60, //  ##  ##  ##
	0x44, 0x40, //  #   #   # 
	0xCC, 0xC0, // ##  ##  ## 
	0xCC, 0xC0, // ##  ##  ## 
	0xCC, 0xC0, // ##  ##  ## 
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1386 'n' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x6C, //  ## ## 
	0x7E, //  ######
	0x66, //  ##  ##
	0x46, //  #   ##
	0xC4, // ##   # 
	0xCC, // ##  ## 
	0xCC, // ##  ## 
	0x00, //        
	0x00, //        
	0x00, //        

	// @1400 'o' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3C, //   #### 
	0x7E, //  ######
	0xE6, // ###  ##
	0xC6, // ##   ##
	0xCE, // ##  ###
	0xFC, // ###### 
	0x78, //  ####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @1414 'p' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x3E, //   ##### 
	0x3F, //   ######
	0x33, //   ##  ##
	0x23, //   #   ##
	0x67, //  ##  ###
	0x7E, //  ###### 
	0x7C, //  #####  
	0x40, //  #      
	0xC0, // ##      
	0xC0, // ##      

	// @1428 'q' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3A, //   ### #
	0x7E, //  ######
	0xE6, // ###  ##
	0xC6, // ##   ##
	0xC6, // ##   ##
	0xFC, // ###### 
	0x7C, //  ##### 
	0x0C, //     ## 
	0x0C, //     ## 
	0x0C, //     ## 

	// @1442 'r' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x68, //  ## #
	0x78, //  ####
	0x60, //  ##  
	0x40, //  #   
	0xC0, // ##   
	0xC0, // ##   
	0xC0, // ##   
	0x00, //      
	0x00, //      
	0x00, //      

	// @1456 's' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x38, //   ### 
	0x7C, //  #####
	0x6C, //  ## ##
	0x7C, //  #####
	0xCC, // ##  ##
	0xFC, // ######
	0x78, //  #### 
	0x00, //       
	0x00, //       
	0x00, //       

	// @1470 't' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x30, //   ##
	0xF0, // ####
	0xF0, // ####
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0x40, //  #  
	0xE0, // ### 
	0xE0, // ### 
	0x00, //     
	0x00, //     
	0x00, //     

	// @1484 'u' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x62, //  ##   #
	0x66, //  ##  ##
	0x46, //  #   ##
	0xC6, // ##   ##
	0xC6, // ##   ##
	0xFC, // ###### 
	0x7C, //  ##### 
	0x00, //        
	0x00, //        
	0x00, //        

	// @1498 'v' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xCC, // ##  ##
	0xCC, // ##  ##
	0xD8, // ## ## 
	0xD8, // ## ## 
	0xF0, // ####  
	0xE0, // ###   
	0x60, //  ##   
	0x00, //       
	0x00, //       
	0x00, //       

	// @1512 'w' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0xDC, 0xC0, // ## ###  ##
	0xDD, 0x80, // ## ### ## 
	0xDD, 0x80, // ## ### ## 
	0xED, 0x00, // ### ## #  
	0xEF, 0x00, // ### ####  
	0xEE, 0x00, // ### ###   
	0xEE, 0x00, // ### ###   
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1540 'x' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x66, //  ##  ##
	0x6C, //  ## ## 
	0x38, //   ###  
	0x30, //   ##   
	0x78, //  ####  
	0xD8, // ## ##  
	0xCC, // ##  ## 
	0x00, //        
	0x00, //        
	0x00, //        

	// @1554 'y' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x66, //  ##  ##
	0x66, //  ##  ##
	0x6C, //  ## ## 
	0x6C, //  ## ## 
	0x78, //  ####  
	0x70, //  ###   
	0x30, //   ##   
	0x60, //  ##    
	0xE0, // ###    
	0xC0, // ##     

	// @1568 'z' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3E, //   #####
	0x3E, //   #####
	0x0C, //     ## 
	0x18, //    ##  
	0x70, //  ###   
	0xFC, // ###### 
	0xFC, // ###### 
	0x00, //        
	0x00, //        
	0x00, //        

	// @1582 '{' (5 pixels wide)
	0x00, //      
	0x18, //    ##
	0x38, //   ###
	0x30, //   ## 
	0x30, //   ## 
	0x60, //  ##  
	0xE0, // ###  
	0xE0, // ###  
	0x60, //  ##  
	0x40, //  #   
	0xC0, // ##   
	0xC0, // ##   
	0xE0, // ###  
	0x60, //  ##  

	// @1596 '|' (3 pixels wide)
	0x00, //    
	0x20, //   #
	0x20, //   #
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x80, // #  
	0x00, //    

	// @1610 '}' (5 pixels wide)
	0x00, //      
	0x38, //   ###
	0x38, //   ###
	0x18, //    ##
	0x18, //    ##
	0x10, //    # 
	0x30, //   ## 
	0x38, //   ###
	0x38, //   ###
	0x30, //   ## 
	0x60, //  ##  
	0x60, //  ##  
	0xE0, // ###  
	0xC0, // ##   

	// @1624 '~' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x74, //  ### #
	0x9C, // #  ###
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
};

// Character descriptors for FreeSans 10pt
// { [Char width in bits], [Offset into freeSans_10ptCharBitmaps in bytes] }
static const CharDesc_t s_FreeSansBoldIta10ptCharDesc[] = {
	{4, s_FreeSansBoldIta10ptBitmaps + 0}, 			// !
	{5, s_FreeSansBoldIta10ptBitmaps + 14}, 		// "
	{8, s_FreeSansBoldIta10ptBitmaps + 28}, 		// #
	{7, s_FreeSansBoldIta10ptBitmaps + 42}, 		// $
	{9, s_FreeSansBoldIta10ptBitmaps + 56}, 		// %
	{9, s_FreeSansBoldIta10ptBitmaps + 84}, 		// &
	{2, s_FreeSansBoldIta10ptBitmaps + 112}, 		// '
	{5, s_FreeSansBoldIta10ptBitmaps + 126}, 		// (
	{5, s_FreeSansBoldIta10ptBitmaps + 140}, 		// )
	{4, s_FreeSansBoldIta10ptBitmaps + 154}, 		// *
	{7, s_FreeSansBoldIta10ptBitmaps + 168}, 		// +
	{2, s_FreeSansBoldIta10ptBitmaps + 182}, 		// ,
	{4, s_FreeSansBoldIta10ptBitmaps + 196}, 		// -
	{2, s_FreeSansBoldIta10ptBitmaps + 210}, 		// .
	{5, s_FreeSansBoldIta10ptBitmaps + 224}, 		// /
	{7, s_FreeSansBoldIta10ptBitmaps + 238}, 		// 0
	{5, s_FreeSansBoldIta10ptBitmaps + 252}, 		// 1
	{7, s_FreeSansBoldIta10ptBitmaps + 266}, 		// 2
	{7, s_FreeSansBoldIta10ptBitmaps + 280}, 		// 3
	{7, s_FreeSansBoldIta10ptBitmaps + 294}, 		// 4
	{8, s_FreeSansBoldIta10ptBitmaps + 308}, 		// 5
	{8, s_FreeSansBoldIta10ptBitmaps + 322}, 		// 6
	{7, s_FreeSansBoldIta10ptBitmaps + 336}, 		// 7
	{8, s_FreeSansBoldIta10ptBitmaps + 350}, 		// 8
	{7, s_FreeSansBoldIta10ptBitmaps + 364}, 		// 9
	{3, s_FreeSansBoldIta10ptBitmaps + 378}, 		// :
	{4, s_FreeSansBoldIta10ptBitmaps + 392}, 		// ;
	{7, s_FreeSansBoldIta10ptBitmaps + 406}, 		// <
	{7, s_FreeSansBoldIta10ptBitmaps + 420}, 		// =
	{7, s_FreeSansBoldIta10ptBitmaps + 434}, 		// >
	{6, s_FreeSansBoldIta10ptBitmaps + 448}, 		// ?
	{12,s_FreeSansBoldIta10ptBitmaps + 462}, 		// @
	{8, s_FreeSansBoldIta10ptBitmaps + 490}, 		// A
	{9, s_FreeSansBoldIta10ptBitmaps + 504}, 		// B
	{9, s_FreeSansBoldIta10ptBitmaps + 532}, 		// C
	{9, s_FreeSansBoldIta10ptBitmaps + 560}, 		// D
	{9, s_FreeSansBoldIta10ptBitmaps + 588}, 		// E
	{9, s_FreeSansBoldIta10ptBitmaps + 616}, 		// F
	{9, s_FreeSansBoldIta10ptBitmaps + 644}, 		// G
	{9, s_FreeSansBoldIta10ptBitmaps + 672}, 		// H
	{4, s_FreeSansBoldIta10ptBitmaps + 700}, 		// I
	{7, s_FreeSansBoldIta10ptBitmaps + 714}, 		// J
	{9, s_FreeSansBoldIta10ptBitmaps + 728}, 		// K
	{7, s_FreeSansBoldIta10ptBitmaps + 756}, 		// L
	{11,s_FreeSansBoldIta10ptBitmaps + 770}, 		// M
	{9, s_FreeSansBoldIta10ptBitmaps + 798}, 		// N
	{10,s_FreeSansBoldIta10ptBitmaps + 826}, 		// O
	{9, s_FreeSansBoldIta10ptBitmaps + 854}, 		// P
	{10,s_FreeSansBoldIta10ptBitmaps + 882}, 		// Q
	{10,s_FreeSansBoldIta10ptBitmaps + 910}, 		// R
	{9, s_FreeSansBoldIta10ptBitmaps + 938}, 		// S
	{8, s_FreeSansBoldIta10ptBitmaps + 966}, 		// T
	{8, s_FreeSansBoldIta10ptBitmaps + 980}, 		// U
	{8, s_FreeSansBoldIta10ptBitmaps + 994}, 		// V
	{13,s_FreeSansBoldIta10ptBitmaps + 1008}, 		// W
	{9, s_FreeSansBoldIta10ptBitmaps + 1036}, 		// X
	{7, s_FreeSansBoldIta10ptBitmaps + 1064}, 		// Y
	{9, s_FreeSansBoldIta10ptBitmaps + 1078}, 		// Z
	{6, s_FreeSansBoldIta10ptBitmaps + 1106}, 		// [
	{2, s_FreeSansBoldIta10ptBitmaps + 1120}, 		// '\'
	{6, s_FreeSansBoldIta10ptBitmaps + 1134}, 		// ]
	{5, s_FreeSansBoldIta10ptBitmaps + 1148}, 		// ^
	{8, s_FreeSansBoldIta10ptBitmaps + 1162}, 		// _
	{2, s_FreeSansBoldIta10ptBitmaps + 1176}, 		// `
	{7, s_FreeSansBoldIta10ptBitmaps + 1190}, 		// a
	{7, s_FreeSansBoldIta10ptBitmaps + 1204}, 		// b
	{7, s_FreeSansBoldIta10ptBitmaps + 1218}, 		// c
	{8, s_FreeSansBoldIta10ptBitmaps + 1232}, 		// d
	{7, s_FreeSansBoldIta10ptBitmaps + 1246}, 		// e
	{5, s_FreeSansBoldIta10ptBitmaps + 1260}, 		// f
	{8, s_FreeSansBoldIta10ptBitmaps + 1274}, 		// g
	{7, s_FreeSansBoldIta10ptBitmaps + 1288}, 		// h
	{4, s_FreeSansBoldIta10ptBitmaps + 1302}, 		// i
	{6, s_FreeSansBoldIta10ptBitmaps + 1316}, 		// j
	{7, s_FreeSansBoldIta10ptBitmaps + 1330}, 		// k
	{4, s_FreeSansBoldIta10ptBitmaps + 1344}, 		// l
	{11,s_FreeSansBoldIta10ptBitmaps + 1358}, 		// m
	{7, s_FreeSansBoldIta10ptBitmaps + 1386}, 		// n
	{7, s_FreeSansBoldIta10ptBitmaps + 1400}, 		// o
	{8, s_FreeSansBoldIta10ptBitmaps + 1414}, 		// p
	{7, s_FreeSansBoldIta10ptBitmaps + 1428}, 		// q
	{5, s_FreeSansBoldIta10ptBitmaps + 1442}, 		// r
	{6, s_FreeSansBoldIta10ptBitmaps + 1456}, 		// s
	{4, s_FreeSansBoldIta10ptBitmaps + 1470}, 		// t
	{7, s_FreeSansBoldIta10ptBitmaps + 1484}, 		// u
	{6, s_FreeSansBoldIta10ptBitmaps + 1498}, 		// v
	{10,s_FreeSansBoldIta10ptBitmaps + 1512}, 		// w
	{7, s_FreeSansBoldIta10ptBitmaps + 1540}, 		// x
	{7, s_FreeSansBoldIta10ptBitmaps + 1554}, 		// y
	{7, s_FreeSansBoldIta10ptBitmaps + 1568}, 		// z
	{5, s_FreeSansBoldIta10ptBitmaps + 1582}, 		// {
	{3, s_FreeSansBoldIta10ptBitmaps + 1596}, 		// |
	{5, s_FreeSansBoldIta10ptBitmaps + 1610}, 		// }
	{6, s_FreeSansBoldIta10ptBitmaps + 1624}, 		// ~
};

// Font information for FreeSans 10pt
const FontDesc_t iFontFreeSansBoldIta10pt = {
	0,
	13,
	14,
	{.pCharDesc = s_FreeSansBoldIta10ptCharDesc }
};
