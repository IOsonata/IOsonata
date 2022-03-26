// 
//  Font data for FreeSerif Bold Italic 10pt
// 	https://savannah.gnu.org/projects/freefont/
// 
// 	Font bitmap generated by
// 	http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator

#include "display/ifont.h"

// Character bitmaps for FreeSerif 10pt
static const uint8_t s_FreeSerifBoldIta10ptBitmaps[] = {
	// @0 '!' (4 pixels wide)
	0x00, //     
	0x10, //    #
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
	0x00, //     

	// @13 '"' (5 pixels wide)
	0x00, //      
	0xD8, // ## ##
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
	0x00, //      

	// @26 '#' (7 pixels wide)
	0x00, //        
	0x12, //    #  #
	0x14, //    # # 
	0x34, //   ## # 
	0x7E, //  ######
	0x28, //   # #  
	0xFC, // ###### 
	0x50, //  # #   
	0x90, // #  #   
	0xA0, // # #    
	0x00, //        
	0x00, //        
	0x00, //        

	// @39 '$' (8 pixels wide)
	0x08, //     #   
	0x38, //   ###   
	0x4F, //  #  ####
	0x52, //  # #  # 
	0x70, //  ###    
	0x38, //   ###   
	0x1C, //    ###  
	0xAC, // # # ##  
	0xAC, // # # ##  
	0x78, //  ####   
	0x40, //  #      
	0x00, //         
	0x00, //         

	// @52 '%' (10 pixels wide)
	0x00, 0x00, //           
	0x31, 0x00, //   ##   #  
	0x6E, 0x00, //  ## ###   
	0xCC, 0x00, // ##  ##    
	0xDC, 0x00, // ## ###    
	0x79, 0xC0, //  ####  ###
	0x0F, 0x40, //     #### #
	0x16, 0x40, //    # ##  #
	0x16, 0x80, //    # ## # 
	0x23, 0x80, //   #   ### 
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @78 '&' (9 pixels wide)
	0x00, 0x00, //          
	0x0E, 0x00, //     ###  
	0x1A, 0x00, //    ## #  
	0x1A, 0x00, //    ## #  
	0x1C, 0x00, //    ###   
	0x3B, 0x80, //   ### ###
	0x49, 0x00, //  #  #  # 
	0xCE, 0x00, // ##  ###  
	0xC4, 0x00, // ##   #   
	0x7B, 0x00, //  #### ## 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @104 ''' (2 pixels wide)
	0x00, //   
	0xC0, // ##
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
	0x00, //   

	// @117 '(' (4 pixels wide)
	0x00, //     
	0x10, //    #
	0x20, //   # 
	0x40, //  #  
	0xC0, // ##  
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0x40, //  #  
	0x00, //     

	// @130 ')' (4 pixels wide)
	0x00, //     
	0x20, //   # 
	0x20, //   # 
	0x10, //    #
	0x10, //    #
	0x10, //    #
	0x10, //    #
	0x10, //    #
	0x30, //   ##
	0x20, //   # 
	0x40, //  #  
	0x80, // #   
	0x00, //     

	// @143 '*' (6 pixels wide)
	0x00, //       
	0x30, //   ##  
	0x30, //   ##  
	0xAC, // # # ##
	0x70, //  ###  
	0xAC, // # # ##
	0x30, //   ##  
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @156 '+' (5 pixels wide)
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

	// @169 ',' (2 pixels wide)
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
	0x40, //  #
	0x80, // # 
	0x00, //   

	// @182 '-' (3 pixels wide)
	0x00, //    
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

	// @195 '.' (2 pixels wide)
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

	// @208 '/' (5 pixels wide)
	0x00, //      
	0x08, //     #
	0x18, //    ##
	0x10, //    # 
	0x30, //   ## 
	0x20, //   #  
	0x20, //   #  
	0x40, //  #   
	0x40, //  #   
	0x80, // #    
	0x00, //      
	0x00, //      
	0x00, //      

	// @221 '0' (7 pixels wide)
	0x00, //        
	0x1C, //    ### 
	0x36, //   ## ##
	0x66, //  ##  ##
	0xE6, // ###  ##
	0xEE, // ### ###
	0xCE, // ##  ###
	0xCC, // ##  ## 
	0xD8, // ## ##  
	0x70, //  ###   
	0x00, //        
	0x00, //        
	0x00, //        

	// @234 '1' (5 pixels wide)
	0x00, //      
	0x08, //     #
	0x38, //   ###
	0x18, //    ##
	0x10, //    # 
	0x30, //   ## 
	0x30, //   ## 
	0x30, //   ## 
	0x20, //   #  
	0xF8, // #####
	0x00, //      
	0x00, //      
	0x00, //      

	// @247 '2' (6 pixels wide)
	0x00, //       
	0x38, //   ### 
	0x4C, //  #  ##
	0x0C, //     ##
	0x0C, //     ##
	0x18, //    ## 
	0x10, //    #  
	0x20, //   #   
	0x48, //  #  # 
	0xF8, // ##### 
	0x00, //       
	0x00, //       
	0x00, //       

	// @260 '3' (6 pixels wide)
	0x00, //       
	0x38, //   ### 
	0x4C, //  #  ##
	0x0C, //     ##
	0x18, //    ## 
	0x38, //   ### 
	0x0C, //     ##
	0x0C, //     ##
	0x08, //     # 
	0xF0, // ####  
	0x00, //       
	0x00, //       
	0x00, //       

	// @273 '4' (7 pixels wide)
	0x00, //        
	0x02, //       #
	0x04, //      # 
	0x0C, //     ## 
	0x3C, //   #### 
	0x4C, //  #  ## 
	0x88, // #   #  
	0xFC, // ###### 
	0x18, //    ##  
	0x18, //    ##  
	0x00, //        
	0x00, //        
	0x00, //        

	// @286 '5' (7 pixels wide)
	0x00, //        
	0x3E, //   #####
	0x40, //  #     
	0x70, //  ###   
	0x78, //  ####  
	0x0C, //     ## 
	0x0C, //     ## 
	0x0C, //     ## 
	0x18, //    ##  
	0xF0, // ####   
	0x00, //        
	0x00, //        
	0x00, //        

	// @299 '6' (7 pixels wide)
	0x00, //        
	0x06, //      ##
	0x18, //    ##  
	0x30, //   ##   
	0x78, //  ####  
	0xEC, // ### ## 
	0xCC, // ##  ## 
	0xCC, // ##  ## 
	0xD8, // ## ##  
	0x70, //  ###   
	0x00, //        
	0x00, //        
	0x00, //        

	// @312 '7' (6 pixels wide)
	0x00, //       
	0x7C, //  #####
	0x88, // #   # 
	0x18, //    ## 
	0x10, //    #  
	0x30, //   ##  
	0x20, //   #   
	0x60, //  ##   
	0x40, //  #    
	0x80, // #     
	0x00, //       
	0x00, //       
	0x00, //       

	// @325 '8' (6 pixels wide)
	0x00, //       
	0x38, //   ### 
	0x34, //   ## #
	0x34, //   ## #
	0x28, //   # # 
	0x10, //    #  
	0x78, //  #### 
	0x98, // #  ## 
	0xD8, // ## ## 
	0x70, //  ###  
	0x00, //       
	0x00, //       
	0x00, //       

	// @338 '9' (7 pixels wide)
	0x00, //        
	0x1C, //    ### 
	0x36, //   ## ##
	0x66, //  ##  ##
	0x66, //  ##  ##
	0x6E, //  ## ###
	0x3C, //   #### 
	0x18, //    ##  
	0x30, //   ##   
	0xC0, // ##     
	0x00, //        
	0x00, //        
	0x00, //        

	// @351 ':' (3 pixels wide)
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
	0x60, //  ##
	0x00, //    
	0x00, //    
	0x00, //    
	0x00, //    
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
	0x00, //    
	0x00, //    
	0x00, //    
	0xC0, // ## 
	0xC0, // ## 
	0x40, //  # 
	0x80, // #  
	0x00, //    

	// @377 '<' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x02, //       #
	0x0E, //     ###
	0x30, //   ##   
	0xC0, // ##     
	0x70, //  ###   
	0x0C, //     ## 
	0x02, //       #
	0x00, //        
	0x00, //        
	0x00, //        

	// @390 '=' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xFC, // ######
	0x00, //       
	0x00, //       
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @403 '>' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x80, // #     
	0xE0, // ###   
	0x38, //   ### 
	0x04, //      #
	0x18, //    ## 
	0xE0, // ###   
	0x80, // #     
	0x00, //       
	0x00, //       
	0x00, //       

	// @416 '?' (5 pixels wide)
	0x00, //      
	0x70, //  ### 
	0x58, //  # ##
	0x18, //    ##
	0x18, //    ##
	0x30, //   ## 
	0x40, //  #   
	0x40, //  #   
	0x00, //      
	0xC0, // ##   
	0x00, //      
	0x00, //      
	0x00, //      

	// @429 '@' (9 pixels wide)
	0x00, 0x00, //          
	0x1E, 0x00, //    ####  
	0x63, 0x00, //  ##   ## 
	0x4E, 0x80, //  #  ### #
	0xCA, 0x80, // ##  # # #
	0xD2, 0x80, // ## #  # #
	0xD4, 0x80, // ## # #  #
	0xDF, 0x00, // ## ##### 
	0x60, 0x00, //  ##      
	0x3C, 0x00, //   ####   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @455 'A' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x06, 0x00, //      ##  
	0x0E, 0x00, //     ###  
	0x16, 0x00, //    # ##  
	0x16, 0x00, //    # ##  
	0x26, 0x00, //   #  ##  
	0x3E, 0x00, //   #####  
	0x43, 0x00, //  #    ## 
	0xE7, 0x80, // ###  ####
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @481 'B' (9 pixels wide)
	0x00, 0x00, //          
	0x3F, 0x00, //   ###### 
	0x19, 0x80, //    ##  ##
	0x31, 0x80, //   ##   ##
	0x33, 0x00, //   ##  ## 
	0x3C, 0x00, //   ####   
	0x33, 0x00, //   ##  ## 
	0x63, 0x00, //  ##   ## 
	0x67, 0x00, //  ##  ### 
	0xFE, 0x00, // #######  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @507 'C' (9 pixels wide)
	0x00, 0x00, //          
	0x1E, 0x80, //    #### #
	0x31, 0x00, //   ##   # 
	0x61, 0x00, //  ##    # 
	0xE0, 0x00, // ###      
	0xC0, 0x00, // ##       
	0xC0, 0x00, // ##       
	0xC0, 0x00, // ##       
	0x64, 0x00, //  ##  #   
	0x38, 0x00, //   ###    
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @533 'D' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0x00, //   ######  
	0x19, 0x80, //    ##  ## 
	0x30, 0xC0, //   ##    ##
	0x30, 0xC0, //   ##    ##
	0x30, 0xC0, //   ##    ##
	0x31, 0xC0, //   ##   ###
	0x61, 0x80, //  ##    ## 
	0x63, 0x00, //  ##   ##  
	0xFE, 0x00, // #######   
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @559 'E' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0xC0, //   ########
	0x18, 0x80, //    ##   # 
	0x30, 0x80, //   ##    # 
	0x33, 0x00, //   ##  ##  
	0x3F, 0x00, //   ######  
	0x32, 0x00, //   ##  #   
	0x61, 0x00, //  ##    #  
	0x63, 0x00, //  ##   ##  
	0xFF, 0x00, // ########  
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @585 'F' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0xC0, //   ########
	0x18, 0x80, //    ##   # 
	0x30, 0x80, //   ##    # 
	0x33, 0x00, //   ##  ##  
	0x3F, 0x00, //   ######  
	0x32, 0x00, //   ##  #   
	0x60, 0x00, //  ##       
	0x60, 0x00, //  ##       
	0xF0, 0x00, // ####      
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @611 'G' (9 pixels wide)
	0x00, 0x00, //          
	0x1E, 0x80, //    #### #
	0x31, 0x00, //   ##   # 
	0x60, 0x00, //  ##      
	0x60, 0x00, //  ##      
	0xC0, 0x00, // ##       
	0xC7, 0x00, // ##   ### 
	0xC2, 0x00, // ##    #  
	0xC6, 0x00, // ##   ##  
	0x7C, 0x00, //  #####   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @637 'H' (11 pixels wide)
	0x00, 0x00, //            
	0x3D, 0xE0, //   #### ####
	0x18, 0xC0, //    ##   ## 
	0x31, 0x80, //   ##   ##  
	0x31, 0x80, //   ##   ##  
	0x3F, 0x80, //   #######  
	0x31, 0x80, //   ##   ##  
	0x63, 0x00, //  ##   ##   
	0x63, 0x00, //  ##   ##   
	0xF7, 0x80, // #### ####  
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @663 'I' (6 pixels wide)
	0x00, //       
	0x3C, //   ####
	0x18, //    ## 
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0x60, //  ##   
	0x60, //  ##   
	0xF0, // ####  
	0x00, //       
	0x00, //       
	0x00, //       

	// @676 'J' (7 pixels wide)
	0x00, //        
	0x0E, //     ###
	0x0C, //     ## 
	0x0C, //     ## 
	0x0C, //     ## 
	0x0C, //     ## 
	0x18, //    ##  
	0x18, //    ##  
	0x18, //    ##  
	0xD0, // ## #   
	0xF0, // ####   
	0x00, //        
	0x00, //        

	// @689 'K' (9 pixels wide)
	0x00, 0x00, //          
	0x3D, 0x80, //   #### ##
	0x19, 0x00, //    ##  # 
	0x32, 0x00, //   ##  #  
	0x34, 0x00, //   ## #   
	0x3C, 0x00, //   ####   
	0x3C, 0x00, //   ####   
	0x66, 0x00, //  ##  ##  
	0x66, 0x00, //  ##  ##  
	0xF7, 0x00, // #### ### 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @715 'L' (9 pixels wide)
	0x00, 0x00, //          
	0x3C, 0x00, //   ####   
	0x18, 0x00, //    ##    
	0x30, 0x00, //   ##     
	0x30, 0x00, //   ##     
	0x30, 0x00, //   ##     
	0x30, 0x00, //   ##     
	0x60, 0x80, //  ##     #
	0x63, 0x80, //  ##   ###
	0xFF, 0x00, // ######## 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @741 'M' (12 pixels wide)
	0x00, 0x00, //             
	0x38, 0x70, //   ###    ###
	0x18, 0x60, //    ##    ## 
	0x18, 0xE0, //    ##   ### 
	0x39, 0x60, //   ###  # ## 
	0x2A, 0x40, //   # # #  #  
	0x2A, 0xC0, //   # # # ##  
	0x4C, 0xC0, //  #  ##  ##  
	0x48, 0xC0, //  #  #   ##  
	0xEB, 0xC0, // ### # ####  
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @767 'N' (10 pixels wide)
	0x00, 0x00, //           
	0x31, 0xC0, //   ##   ###
	0x18, 0x80, //    ##   # 
	0x18, 0x80, //    ##   # 
	0x2D, 0x00, //   # ## #  
	0x25, 0x00, //   #  # #  
	0x25, 0x00, //   #  # #  
	0x47, 0x00, //  #   ###  
	0x42, 0x00, //  #    #   
	0xE2, 0x00, // ###   #   
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @793 'O' (9 pixels wide)
	0x00, 0x00, //          
	0x0F, 0x00, //     #### 
	0x31, 0x80, //   ##   ##
	0x61, 0x80, //  ##    ##
	0x61, 0x80, //  ##    ##
	0xC1, 0x80, // ##     ##
	0xC3, 0x00, // ##    ## 
	0xC3, 0x00, // ##    ## 
	0xC6, 0x00, // ##   ##  
	0x78, 0x00, //  ####    
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @819 'P' (8 pixels wide)
	0x00, //         
	0x3E, //   ##### 
	0x1B, //    ## ##
	0x33, //   ##  ##
	0x33, //   ##  ##
	0x3E, //   ##### 
	0x30, //   ##    
	0x60, //  ##     
	0x60, //  ##     
	0xF0, // ####    
	0x00, //         
	0x00, //         
	0x00, //         

	// @832 'Q' (9 pixels wide)
	0x00, 0x00, //          
	0x0E, 0x00, //     ###  
	0x31, 0x00, //   ##   # 
	0x71, 0x80, //  ###   ##
	0x61, 0x80, //  ##    ##
	0xE1, 0x80, // ###    ##
	0xC3, 0x80, // ##    ###
	0xC3, 0x00, // ##    ## 
	0xC3, 0x00, // ##    ## 
	0x46, 0x00, //  #   ##  
	0x38, 0x00, //   ###    
	0x71, 0x00, //  ###   # 
	0xFE, 0x00, // #######  

	// @858 'R' (9 pixels wide)
	0x00, 0x00, //          
	0x3F, 0x00, //   ###### 
	0x19, 0x80, //    ##  ##
	0x31, 0x80, //   ##   ##
	0x33, 0x80, //   ##  ###
	0x3E, 0x00, //   #####  
	0x3E, 0x00, //   #####  
	0x66, 0x00, //  ##  ##  
	0x67, 0x00, //  ##  ### 
	0xF3, 0x80, // ####  ###
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @884 'S' (6 pixels wide)
	0x00, //       
	0x34, //   ## #
	0x6C, //  ## ##
	0x64, //  ##  #
	0x70, //  ###  
	0x38, //   ### 
	0x0C, //     ##
	0x0C, //     ##
	0x8C, // #   ##
	0x78, //  #### 
	0x00, //       
	0x00, //       
	0x00, //       

	// @897 'T' (9 pixels wide)
	0x00, 0x00, //          
	0x7F, 0x80, //  ########
	0xD9, 0x00, // ## ##  # 
	0x99, 0x00, // #  ##  # 
	0x18, 0x00, //    ##    
	0x30, 0x00, //   ##     
	0x30, 0x00, //   ##     
	0x30, 0x00, //   ##     
	0x20, 0x00, //   #      
	0xF8, 0x00, // #####    
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @923 'U' (10 pixels wide)
	0x00, 0x00, //           
	0xFB, 0xC0, // ##### ####
	0x30, 0x80, //   ##    # 
	0x61, 0x00, //  ##    #  
	0x61, 0x00, //  ##    #  
	0x61, 0x00, //  ##    #  
	0xE3, 0x00, // ###   ##  
	0xC2, 0x00, // ##    #   
	0xC4, 0x00, // ##   #    
	0x78, 0x00, //  ####     
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @949 'V' (8 pixels wide)
	0x00, //         
	0xF3, // ####  ##
	0x61, //  ##    #
	0x62, //  ##   # 
	0x64, //  ##  #  
	0x68, //  ## #   
	0x68, //  ## #   
	0x70, //  ###    
	0x20, //   #     
	0x20, //   #     
	0x00, //         
	0x00, //         
	0x00, //         

	// @962 'W' (11 pixels wide)
	0x00, 0x00, //            
	0xEF, 0x60, // ### #### ##
	0x66, 0x40, //  ##  ##  # 
	0x66, 0x40, //  ##  ##  # 
	0x6E, 0x80, //  ## ### #  
	0x76, 0x80, //  ### ## #  
	0x77, 0x00, //  ### ###   
	0x63, 0x00, //  ##   ##   
	0x62, 0x00, //  ##   #    
	0x02, 0x00, //       #    
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @988 'X' (9 pixels wide)
	0x00, 0x00, //          
	0x3D, 0x80, //   #### ##
	0x19, 0x00, //    ##  # 
	0x1A, 0x00, //    ## #  
	0x1C, 0x00, //    ###   
	0x0C, 0x00, //     ##   
	0x1C, 0x00, //    ###   
	0x2C, 0x00, //   # ##   
	0x46, 0x00, //  #   ##  
	0xEF, 0x00, // ### #### 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1014 'Y' (7 pixels wide)
	0x00, //        
	0xE6, // ###  ##
	0x44, //  #   # 
	0x64, //  ##  # 
	0x68, //  ## #  
	0x30, //   ##   
	0x20, //   #    
	0x60, //  ##    
	0x60, //  ##    
	0xF0, // ####   
	0x00, //        
	0x00, //        
	0x00, //        

	// @1027 'Z' (8 pixels wide)
	0x00, //         
	0x3F, //   ######
	0x27, //   #  ###
	0x46, //  #   ## 
	0x0C, //     ##  
	0x18, //    ##   
	0x38, //   ###   
	0x71, //  ###   #
	0x62, //  ##   # 
	0xFE, // ####### 
	0x00, //         
	0x00, //         
	0x00, //         

	// @1040 '[' (5 pixels wide)
	0x00, //      
	0x38, //   ###
	0x20, //   #  
	0x20, //   #  
	0x60, //  ##  
	0x40, //  #   
	0x40, //  #   
	0x40, //  #   
	0xC0, // ##   
	0x80, // #    
	0x80, // #    
	0xC0, // ##   
	0x00, //      

	// @1053 '\' (3 pixels wide)
	0x00, //    
	0x80, // #  
	0x80, // #  
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x40, //  # 
	0x20, //   #
	0x20, //   #
	0x00, //    
	0x00, //    
	0x00, //    

	// @1066 ']' (5 pixels wide)
	0x00, //      
	0x18, //    ##
	0x08, //     #
	0x08, //     #
	0x18, //    ##
	0x10, //    # 
	0x10, //    # 
	0x10, //    # 
	0x30, //   ## 
	0x20, //   #  
	0x20, //   #  
	0xE0, // ###  
	0x00, //      

	// @1079 '^' (5 pixels wide)
	0x00, //      
	0x20, //   #  
	0x70, //  ### 
	0x50, //  # # 
	0xC8, // ##  #
	0x88, // #   #
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      

	// @1092 '_' (7 pixels wide)
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

	// @1105 '`' (2 pixels wide)
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

	// @1118 'a' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x1C, //    ### 
	0x74, //  ### # 
	0x6C, //  ## ## 
	0xCC, // ##  ## 
	0xDC, // ## ### 
	0xEE, // ### ###
	0x00, //        
	0x00, //        
	0x00, //        

	// @1131 'b' (7 pixels wide)
	0x00, //        
	0x70, //  ###   
	0x30, //   ##   
	0x30, //   ##   
	0x2E, //   # ###
	0x36, //   ## ##
	0x66, //  ##  ##
	0x4C, //  #  ## 
	0xCC, // ##  ## 
	0x70, //  ###   
	0x00, //        
	0x00, //        
	0x00, //        

	// @1144 'c' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x38, //   ###
	0x68, //  ## #
	0xC0, // ##   
	0xC0, // ##   
	0xD0, // ## # 
	0x70, //  ### 
	0x00, //      
	0x00, //      
	0x00, //      

	// @1157 'd' (7 pixels wide)
	0x00, //        
	0x06, //      ##
	0x06, //      ##
	0x06, //      ##
	0x1C, //    ### 
	0x64, //  ##  # 
	0x6C, //  ## ## 
	0xCC, // ##  ## 
	0xDC, // ## ### 
	0xEE, // ### ###
	0x00, //        
	0x00, //        
	0x00, //        

	// @1170 'e' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x38, //   ###
	0x68, //  ## #
	0xC8, // ##  #
	0xF0, // #### 
	0xC0, // ##   
	0x70, //  ### 
	0x00, //      
	0x00, //      
	0x00, //      

	// @1183 'f' (8 pixels wide)
	0x07, //      ###
	0x04, //      #  
	0x08, //     #   
	0x08, //     #   
	0x1C, //    ###  
	0x18, //    ##   
	0x10, //    #    
	0x10, //    #    
	0x30, //   ##    
	0x30, //   ##    
	0x20, //   #     
	0x20, //   #     
	0xC0, // ##      

	// @1196 'g' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3E, //   #####
	0x6C, //  ## ## 
	0x6C, //  ## ## 
	0x18, //    ##  
	0x60, //  ##    
	0x78, //  ####  
	0x8C, // #   ## 
	0x8C, // #   ## 
	0x78, //  ####  

	// @1209 'h' (7 pixels wide)
	0x00, //        
	0x30, //   ##   
	0x30, //   ##   
	0x30, //   ##   
	0x26, //   #  ##
	0x7E, //  ######
	0x64, //  ##  # 
	0x6C, //  ## ## 
	0x4C, //  #  ## 
	0xCE, // ##  ###
	0x00, //        
	0x00, //        
	0x00, //        

	// @1222 'i' (4 pixels wide)
	0x00, //     
	0x30, //   ##
	0x00, //     
	0x00, //     
	0xC0, // ##  
	0x40, //  #  
	0xC0, // ##  
	0x80, // #   
	0xE0, // ### 
	0xC0, // ##  
	0x00, //     
	0x00, //     
	0x00, //     

	// @1235 'j' (6 pixels wide)
	0x00, //       
	0x0C, //     ##
	0x00, //       
	0x00, //       
	0x18, //    ## 
	0x08, //     # 
	0x18, //    ## 
	0x18, //    ## 
	0x10, //    #  
	0x10, //    #  
	0x30, //   ##  
	0x20, //   #   
	0xE0, // ###   

	// @1248 'k' (7 pixels wide)
	0x00, //        
	0x30, //   ##   
	0x30, //   ##   
	0x30, //   ##   
	0x2E, //   # ###
	0x64, //  ##  # 
	0x78, //  ####  
	0x78, //  ####  
	0x58, //  # ##  
	0xCC, // ##  ## 
	0x00, //        
	0x00, //        
	0x00, //        

	// @1261 'l' (3 pixels wide)
	0x00, //    
	0x60, //  ##
	0x20, //   #
	0x60, //  ##
	0x40, //  # 
	0xC0, // ## 
	0xC0, // ## 
	0x80, // #  
	0xC0, // ## 
	0xE0, // ###
	0x00, //    
	0x00, //    
	0x00, //    

	// @1274 'm' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x6E, 0xC0, //  ## ### ##
	0x77, 0xC0, //  ### #####
	0x65, 0x80, //  ##  # ## 
	0x6D, 0x80, //  ## ## ## 
	0x4D, 0x80, //  #  ## ## 
	0xC9, 0xC0, // ##  #  ###
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1300 'n' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x7E, //  ######
	0x36, //   ## ##
	0x6C, //  ## ## 
	0x6C, //  ## ## 
	0x4C, //  #  ## 
	0xCE, // ##  ###
	0x00, //        
	0x00, //        
	0x00, //        

	// @1313 'o' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3C, //   #### 
	0x66, //  ##  ##
	0xE6, // ###  ##
	0xCE, // ##  ###
	0xCC, // ##  ## 
	0x78, //  ####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @1326 'p' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x3F, //   ######
	0x1B, //    ## ##
	0x13, //    #  ##
	0x36, //   ## ## 
	0x26, //   #  ## 
	0x3C, //   ####  
	0x60, //  ##     
	0x60, //  ##     
	0xF0, // ####    

	// @1339 'q' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x3C, //   ####
	0x6C, //  ## ##
	0x6C, //  ## ##
	0xC8, // ##  # 
	0xD8, // ## ## 
	0xF8, // ##### 
	0x10, //    #  
	0x10, //    #  
	0x38, //   ### 

	// @1352 'r' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x68, //  ## #
	0x70, //  ### 
	0x60, //  ##  
	0x40, //  #   
	0x40, //  #   
	0xC0, // ##   
	0x00, //      
	0x00, //      
	0x00, //      

	// @1365 's' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x38, //   ###
	0x48, //  #  #
	0x60, //  ##  
	0x30, //   ## 
	0x90, // #  # 
	0x60, //  ##  
	0x00, //      
	0x00, //      
	0x00, //      

	// @1378 't' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x10, //    #
	0x20, //   # 
	0x70, //  ###
	0x60, //  ## 
	0x60, //  ## 
	0x40, //  #  
	0xC0, // ##  
	0xE0, // ### 
	0x00, //     
	0x00, //     
	0x00, //     

	// @1391 'u' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0xE4, // ###  # 
	0x64, //  ##  # 
	0x6C, //  ## ## 
	0xCC, // ##  ## 
	0xFC, // ###### 
	0xEE, // ### ###
	0x00, //        
	0x00, //        
	0x00, //        

	// @1404 'v' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0xE8, // ### #
	0x68, //  ## #
	0x68, //  ## #
	0x70, //  ### 
	0x60, //  ##  
	0x40, //  #   
	0x00, //      
	0x00, //      
	0x00, //      

	// @1417 'w' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0xEA, // ### # #
	0x6A, //  ## # #
	0x7A, //  #### #
	0x6C, //  ## ## 
	0x68, //  ## #  
	0x48, //  #  #  
	0x00, //        
	0x00, //        
	0x00, //        

	// @1430 'x' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x6C, //  ## ##
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0xD4, // ## # #
	0xD8, // ## ## 
	0x00, //       
	0x00, //       
	0x00, //       

	// @1443 'y' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x64, //  ##  #
	0x24, //   #  #
	0x24, //   #  #
	0x38, //   ### 
	0x38, //   ### 
	0x30, //   ##  
	0x20, //   #   
	0x20, //   #   
	0xC0, // ##    

	// @1456 'z' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x78, //  ####
	0x90, // #  # 
	0x20, //   #  
	0x40, //  #   
	0xC0, // ##   
	0x40, //  #   
	0x30, //   ## 
	0x00, //      
	0x00, //      

	// @1469 '{' (5 pixels wide)
	0x18, //    ##
	0x30, //   ## 
	0x30, //   ## 
	0x20, //   #  
	0x40, //  #   
	0x60, //  ##  
	0x60, //  ##  
	0x40, //  #   
	0x40, //  #   
	0xC0, // ##   
	0xC0, // ##   
	0x60, //  ##  
	0x00, //      

	// @1482 '|' (1 pixels wide)
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

	// @1495 '}' (6 pixels wide)
	0x18, //    ## 
	0x0C, //     ##
	0x08, //     # 
	0x08, //     # 
	0x08, //     # 
	0x1C, //    ###
	0x18, //    ## 
	0x10, //    #  
	0x30, //   ##  
	0x30, //   ##  
	0x20, //   #   
	0xE0, // ###   
	0x00, //       

	// @1508 '~' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0xE0, // ###  
	0x18, //    ##
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
};

// Character descriptors for FreeSerif 10pt
// { [Char width in bits], [Offset into freeSerif_10ptCharBitmaps in bytes] }
static const CharDesc_t s_FreeSerifBoldIta10ptCharDesc[] = {
	{4, s_FreeSerifBoldIta10ptBitmaps + 0}, 		// !
	{5, s_FreeSerifBoldIta10ptBitmaps + 13}, 		// "
	{7, s_FreeSerifBoldIta10ptBitmaps + 26}, 		// #
	{8, s_FreeSerifBoldIta10ptBitmaps + 39}, 		// $
	{10,s_FreeSerifBoldIta10ptBitmaps +  52}, 		// %
	{9, s_FreeSerifBoldIta10ptBitmaps + 78}, 		// &
	{2, s_FreeSerifBoldIta10ptBitmaps + 104}, 		// '
	{4, s_FreeSerifBoldIta10ptBitmaps + 117}, 		// (
	{4, s_FreeSerifBoldIta10ptBitmaps + 130}, 		// )
	{6, s_FreeSerifBoldIta10ptBitmaps + 143}, 		// *
	{5, s_FreeSerifBoldIta10ptBitmaps + 156}, 		// +
	{2, s_FreeSerifBoldIta10ptBitmaps + 169}, 		// ,
	{3, s_FreeSerifBoldIta10ptBitmaps + 182}, 		// -
	{2, s_FreeSerifBoldIta10ptBitmaps + 195}, 		// .
	{5, s_FreeSerifBoldIta10ptBitmaps + 208}, 		// /
	{7, s_FreeSerifBoldIta10ptBitmaps + 221}, 		// 0
	{5, s_FreeSerifBoldIta10ptBitmaps + 234}, 		// 1
	{6, s_FreeSerifBoldIta10ptBitmaps + 247}, 		// 2
	{6, s_FreeSerifBoldIta10ptBitmaps + 260}, 		// 3
	{7, s_FreeSerifBoldIta10ptBitmaps + 273}, 		// 4
	{7, s_FreeSerifBoldIta10ptBitmaps + 286}, 		// 5
	{7, s_FreeSerifBoldIta10ptBitmaps + 299}, 		// 6
	{6, s_FreeSerifBoldIta10ptBitmaps + 312}, 		// 7
	{6, s_FreeSerifBoldIta10ptBitmaps + 325}, 		// 8
	{7, s_FreeSerifBoldIta10ptBitmaps + 338}, 		// 9
	{3, s_FreeSerifBoldIta10ptBitmaps + 351}, 		// :
	{3, s_FreeSerifBoldIta10ptBitmaps + 364}, 		// ;
	{7, s_FreeSerifBoldIta10ptBitmaps + 377}, 		// <
	{6, s_FreeSerifBoldIta10ptBitmaps + 390}, 		// =
	{6, s_FreeSerifBoldIta10ptBitmaps + 403}, 		// >
	{5, s_FreeSerifBoldIta10ptBitmaps + 416}, 		// ?
	{9, s_FreeSerifBoldIta10ptBitmaps + 429}, 		// @
	{9, s_FreeSerifBoldIta10ptBitmaps + 455}, 		// A
	{9, s_FreeSerifBoldIta10ptBitmaps + 481}, 		// B
	{9, s_FreeSerifBoldIta10ptBitmaps + 507}, 		// C
	{10,s_FreeSerifBoldIta10ptBitmaps +  533}, 		// D
	{10,s_FreeSerifBoldIta10ptBitmaps +  559}, 		// E
	{10,s_FreeSerifBoldIta10ptBitmaps +  585}, 		// F
	{9, s_FreeSerifBoldIta10ptBitmaps + 611}, 		// G
	{11,s_FreeSerifBoldIta10ptBitmaps +  637}, 		// H
	{6, s_FreeSerifBoldIta10ptBitmaps + 663}, 		// I
	{7, s_FreeSerifBoldIta10ptBitmaps + 676}, 		// J
	{9, s_FreeSerifBoldIta10ptBitmaps + 689}, 		// K
	{9, s_FreeSerifBoldIta10ptBitmaps + 715}, 		// L
	{12,s_FreeSerifBoldIta10ptBitmaps +  741}, 		// M
	{10,s_FreeSerifBoldIta10ptBitmaps +  767}, 		// N
	{9, s_FreeSerifBoldIta10ptBitmaps + 793}, 		// O
	{8, s_FreeSerifBoldIta10ptBitmaps + 819}, 		// P
	{9, s_FreeSerifBoldIta10ptBitmaps + 832}, 		// Q
	{9, s_FreeSerifBoldIta10ptBitmaps + 858}, 		// R
	{6, s_FreeSerifBoldIta10ptBitmaps + 884}, 		// S
	{9, s_FreeSerifBoldIta10ptBitmaps + 897}, 		// T
	{10,s_FreeSerifBoldIta10ptBitmaps +  923}, 		// U
	{8, s_FreeSerifBoldIta10ptBitmaps + 949}, 		// V
	{11,s_FreeSerifBoldIta10ptBitmaps +  962}, 		// W
	{9, s_FreeSerifBoldIta10ptBitmaps + 988}, 		// X
	{7, s_FreeSerifBoldIta10ptBitmaps + 1014}, 		// Y
	{8, s_FreeSerifBoldIta10ptBitmaps + 1027}, 		// Z
	{5, s_FreeSerifBoldIta10ptBitmaps + 1040}, 		// [
	{3, s_FreeSerifBoldIta10ptBitmaps + 1053}, 		// '\'
	{5, s_FreeSerifBoldIta10ptBitmaps + 1066}, 		// ]
	{5, s_FreeSerifBoldIta10ptBitmaps + 1079}, 		// ^
	{7, s_FreeSerifBoldIta10ptBitmaps + 1092}, 		// _
	{2, s_FreeSerifBoldIta10ptBitmaps + 1105}, 		// `
	{7, s_FreeSerifBoldIta10ptBitmaps + 1118}, 		// a
	{7, s_FreeSerifBoldIta10ptBitmaps + 1131}, 		// b
	{5, s_FreeSerifBoldIta10ptBitmaps + 1144}, 		// c
	{7, s_FreeSerifBoldIta10ptBitmaps + 1157}, 		// d
	{5, s_FreeSerifBoldIta10ptBitmaps + 1170}, 		// e
	{8, s_FreeSerifBoldIta10ptBitmaps + 1183}, 		// f
	{7, s_FreeSerifBoldIta10ptBitmaps + 1196}, 		// g
	{7, s_FreeSerifBoldIta10ptBitmaps + 1209}, 		// h
	{4, s_FreeSerifBoldIta10ptBitmaps + 1222}, 		// i
	{6, s_FreeSerifBoldIta10ptBitmaps + 1235}, 		// j
	{7, s_FreeSerifBoldIta10ptBitmaps + 1248}, 		// k
	{3, s_FreeSerifBoldIta10ptBitmaps + 1261}, 		// l
	{10,s_FreeSerifBoldIta10ptBitmaps +  1274}, 		// m
	{7, s_FreeSerifBoldIta10ptBitmaps + 1300}, 		// n
	{7, s_FreeSerifBoldIta10ptBitmaps + 1313}, 		// o
	{8, s_FreeSerifBoldIta10ptBitmaps + 1326}, 		// p
	{6, s_FreeSerifBoldIta10ptBitmaps + 1339}, 		// q
	{5, s_FreeSerifBoldIta10ptBitmaps + 1352}, 		// r
	{5, s_FreeSerifBoldIta10ptBitmaps + 1365}, 		// s
	{4, s_FreeSerifBoldIta10ptBitmaps + 1378}, 		// t
	{7, s_FreeSerifBoldIta10ptBitmaps + 1391}, 		// u
	{5, s_FreeSerifBoldIta10ptBitmaps + 1404}, 		// v
	{7, s_FreeSerifBoldIta10ptBitmaps + 1417}, 		// w
	{6, s_FreeSerifBoldIta10ptBitmaps + 1430}, 		// x
	{6, s_FreeSerifBoldIta10ptBitmaps + 1443}, 		// y
	{5, s_FreeSerifBoldIta10ptBitmaps + 1456}, 		// z
	{5, s_FreeSerifBoldIta10ptBitmaps + 1469}, 		// {
	{1, s_FreeSerifBoldIta10ptBitmaps + 1482}, 		// |
	{6, s_FreeSerifBoldIta10ptBitmaps + 1495}, 		// }
	{5, s_FreeSerifBoldIta10ptBitmaps + 1508}, 		// ~
};

// Font information for FreeSerif 10pt
const FontDesc_t iFontFreeSerifBoldIta10pt = {
	0,
	12,
	13,
	{ .pCharDesc = s_FreeSerifBoldIta10ptCharDesc }
};
