// 
//  Font data for FreeSerif Italic 12pt
// 	https://savannah.gnu.org/projects/freefont/
// 
// 	Font bitmap generated by
// 	http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator

#include "display/ifont.h"

// Character bitmaps for FreeSerif 12pt
static const uint8_t s_FreeSerifIta12ptBitmaps[] = {
	// @0 '!' (4 pixels wide)
	0x00, //     
	0x10, //    #
	0x10, //    #
	0x10, //    #
	0x20, //   # 
	0x20, //   # 
	0x20, //   # 
	0x40, //  #  
	0x40, //  #  
	0x00, //     
	0x00, //     
	0xC0, // ##  
	0x00, //     
	0x00, //     
	0x00, //     

	// @15 '"' (5 pixels wide)
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
	0x00, //      

	// @30 '#' (9 pixels wide)
	0x00, 0x00, //          
	0x09, 0x00, //     #  # 
	0x09, 0x00, //     #  # 
	0x12, 0x00, //    #  #  
	0x7F, 0x80, //  ########
	0x12, 0x00, //    #  #  
	0x24, 0x00, //   #  #   
	0x24, 0x00, //   #  #   
	0xFE, 0x00, // #######  
	0x48, 0x00, //  #  #    
	0x48, 0x00, //  #  #    
	0x88, 0x00, // #   #    
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @60 '$' (7 pixels wide)
	0x04, //      # 
	0x38, //   ###  
	0x6E, //  ## ###
	0x6A, //  ## # #
	0x68, //  ## #  
	0x38, //   ###  
	0x1C, //    ### 
	0x1E, //    ####
	0x16, //    # ##
	0xB6, // # ## ##
	0xAE, // # # ###
	0x78, //  ####  
	0x20, //   #    
	0x20, //   #    
	0x00, //        

	// @75 '%' (12 pixels wide)
	0x00, 0x00, //             
	0x38, 0x80, //   ###   #   
	0x67, 0x80, //  ##  ####   
	0xE5, 0x00, // ###  # #    
	0xC5, 0x00, // ##   # #    
	0xC6, 0x00, // ##   ##     
	0xCC, 0xE0, // ##  ##  ### 
	0x75, 0x90, //  ### # ##  #
	0x0B, 0x10, //     # ##   #
	0x0B, 0x10, //     # ##   #
	0x13, 0x20, //    #  ##  # 
	0x11, 0xC0, //    #   ###  
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @105 '&' (10 pixels wide)
	0x00, 0x00, //           
	0x07, 0x00, //      ###  
	0x0D, 0x80, //     ## ## 
	0x0D, 0x80, //     ## ## 
	0x0F, 0x00, //     ####  
	0x0C, 0x00, //     ##    
	0x3D, 0xC0, //   #### ###
	0x64, 0x80, //  ##  #  # 
	0xC7, 0x00, // ##   ###  
	0xC2, 0x00, // ##    #   
	0xE3, 0x00, // ###   ##  
	0x7D, 0xC0, //  ##### ###
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @135 ''' (2 pixels wide)
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
	0x00, //   

	// @150 '(' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x18, //    ##
	0x10, //    # 
	0x20, //   #  
	0x60, //  ##  
	0x60, //  ##  
	0xC0, // ##   
	0xC0, // ##   
	0xC0, // ##   
	0xC0, // ##   
	0xC0, // ##   
	0x40, //  #   
	0x40, //  #   
	0x20, //   #  

	// @165 ')' (5 pixels wide)
	0x00, //      
	0x20, //   #  
	0x20, //   #  
	0x10, //    # 
	0x10, //    # 
	0x18, //    ##
	0x18, //    ##
	0x18, //    ##
	0x18, //    ##
	0x38, //   ###
	0x30, //   ## 
	0x20, //   #  
	0x40, //  #   
	0x80, // #    
	0x00, //      

	// @180 '*' (6 pixels wide)
	0x00, //       
	0x30, //   ##  
	0x30, //   ##  
	0xA4, // # #  #
	0x78, //  #### 
	0x68, //  ## # 
	0xA4, // # #  #
	0x30, //   ##  
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @195 '+' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x10, //    #   
	0x10, //    #   
	0x10, //    #   
	0xFE, // #######
	0x10, //    #   
	0x10, //    #   
	0x10, //    #   
	0x10, //    #   
	0x00, //        
	0x00, //        
	0x00, //        

	// @210 ',' (2 pixels wide)
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
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x80, // # 
	0x00, //   

	// @225 '-' (3 pixels wide)
	0x00, //    
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
	0x00, //    

	// @240 '.' (2 pixels wide)
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

	// @255 '/' (7 pixels wide)
	0x00, //        
	0x02, //       #
	0x04, //      # 
	0x04, //      # 
	0x08, //     #  
	0x18, //    ##  
	0x10, //    #   
	0x30, //   ##   
	0x20, //   #    
	0x40, //  #     
	0x40, //  #     
	0x80, // #      
	0x00, //        
	0x00, //        
	0x00, //        

	// @270 '0' (8 pixels wide)
	0x00, //         
	0x1E, //    #### 
	0x32, //   ##  # 
	0x63, //  ##   ##
	0x63, //  ##   ##
	0xE3, // ###   ##
	0xC3, // ##    ##
	0xC7, // ##   ###
	0xC6, // ##   ## 
	0xC6, // ##   ## 
	0x4C, //  #  ##  
	0x78, //  ####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @285 '1' (6 pixels wide)
	0x04, //      #
	0x38, //   ### 
	0x08, //     # 
	0x08, //     # 
	0x18, //    ## 
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x30, //   ##  
	0x20, //   #   
	0x20, //   #   
	0xF0, // ####  
	0x00, //       
	0x00, //       
	0x00, //       

	// @300 '2' (7 pixels wide)
	0x00, //        
	0x3C, //   #### 
	0x4E, //  #  ###
	0x86, // #    ##
	0x06, //      ##
	0x04, //      # 
	0x08, //     #  
	0x10, //    #   
	0x20, //   #    
	0x40, //  #     
	0xC4, // ##   # 
	0xF8, // #####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @315 '3' (8 pixels wide)
	0x00, //         
	0x1E, //    #### 
	0x23, //   #   ##
	0x03, //       ##
	0x0C, //     ##  
	0x3C, //   ####  
	0x0E, //     ### 
	0x06, //      ## 
	0x06, //      ## 
	0x06, //      ## 
	0x0C, //     ##  
	0xF0, // ####    
	0x00, //         
	0x00, //         
	0x00, //         

	// @330 '4' (8 pixels wide)
	0x00, //         
	0x01, //        #
	0x02, //       # 
	0x0E, //     ### 
	0x12, //    #  # 
	0x24, //   #  #  
	0x24, //   #  #  
	0x44, //  #   #  
	0xFE, // ####### 
	0x08, //     #   
	0x08, //     #   
	0x08, //     #   
	0x00, //         
	0x00, //         
	0x00, //         

	// @345 '5' (8 pixels wide)
	0x00, //         
	0x0F, //     ####
	0x10, //    #    
	0x10, //    #    
	0x1C, //    ###  
	0x0E, //     ### 
	0x06, //      ## 
	0x06, //      ## 
	0x06, //      ## 
	0x04, //      #  
	0x08, //     #   
	0xF0, // ####    
	0x00, //         
	0x00, //         
	0x00, //         

	// @360 '6' (9 pixels wide)
	0x03, 0x80, //       ###
	0x0E, 0x00, //     ###  
	0x18, 0x00, //    ##    
	0x30, 0x00, //   ##     
	0x6E, 0x00, //  ## ###  
	0x73, 0x00, //  ###  ## 
	0xC3, 0x00, // ##    ## 
	0xC3, 0x00, // ##    ## 
	0xC3, 0x00, // ##    ## 
	0xC6, 0x00, // ##   ##  
	0x46, 0x00, //  #   ##  
	0x3C, 0x00, //   ####   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @390 '7' (7 pixels wide)
	0x00, //        
	0x7E, //  ######
	0x82, // #     #
	0x04, //      # 
	0x04, //      # 
	0x08, //     #  
	0x08, //     #  
	0x10, //    #   
	0x20, //   #    
	0x20, //   #    
	0x40, //  #     
	0x40, //  #     
	0x00, //        
	0x00, //        
	0x00, //        

	// @405 '8' (9 pixels wide)
	0x1F, 0x00, //    ##### 
	0x31, 0x80, //   ##   ##
	0x31, 0x80, //   ##   ##
	0x31, 0x80, //   ##   ##
	0x1B, 0x00, //    ## ## 
	0x0C, 0x00, //     ##   
	0x36, 0x00, //   ## ##  
	0xC3, 0x00, // ##    ## 
	0xC3, 0x00, // ##    ## 
	0xC3, 0x00, // ##    ## 
	0x66, 0x00, //  ##  ##  
	0x3C, 0x00, //   ####   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @435 '9' (8 pixels wide)
	0x00, //         
	0x3C, //   ####  
	0x62, //  ##   # 
	0xC3, // ##    ##
	0xC3, // ##    ##
	0xC3, // ##    ##
	0xC7, // ##   ###
	0x7E, //  ###### 
	0x0C, //     ##  
	0x18, //    ##   
	0x70, //  ###    
	0xC0, // ##      
	0x00, //         
	0x00, //         
	0x00, //         

	// @450 ':' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x30, //   ##
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0xC0, // ##  
	0x00, //     
	0x00, //     
	0x00, //     

	// @465 ';' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x30, //   ##
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x40, //  #  
	0x40, //  #  
	0x40, //  #  
	0x80, // #   
	0x00, //     

	// @480 '<' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x03, //       ##
	0x0C, //     ##  
	0x30, //   ##    
	0xC0, // ##      
	0xE0, // ###     
	0x38, //   ###   
	0x06, //      ## 
	0x01, //        #
	0x00, //         
	0x00, //         
	0x00, //         

	// @495 '=' (8 pixels wide)
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

	// @510 '>' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0xC0, // ##      
	0x30, //   ##    
	0x0C, //     ##  
	0x03, //       ##
	0x07, //      ###
	0x1C, //    ###  
	0x60, //  ##     
	0x80, // #       
	0x00, //         
	0x00, //         
	0x00, //         

	// @525 '?' (6 pixels wide)
	0x00, //       
	0x78, //  #### 
	0x4C, //  #  ##
	0x0C, //     ##
	0x0C, //     ##
	0x18, //    ## 
	0x10, //    #  
	0x20, //   #   
	0x40, //  #    
	0x40, //  #    
	0x00, //       
	0xC0, // ##    
	0x00, //       
	0x00, //       
	0x00, //       

	// @540 '@' (11 pixels wide)
	0x00, 0x00, //            
	0x0F, 0x00, //     ####   
	0x30, 0xC0, //   ##    ## 
	0x66, 0xE0, //  ##  ## ###
	0xCD, 0x20, // ##  ## #  #
	0xD9, 0x20, // ## ##  #  #
	0xD9, 0x20, // ## ##  #  #
	0xD9, 0x20, // ## ##  #  #
	0xD9, 0x40, // ## ##  # # 
	0x6F, 0x80, //  ## #####  
	0x30, 0x00, //   ##       
	0x0F, 0x00, //     ####   
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @570 'A' (10 pixels wide)
	0x00, 0x00, //           
	0x02, 0x00, //       #   
	0x02, 0x00, //       #   
	0x07, 0x00, //      ###  
	0x0B, 0x00, //     # ##  
	0x0B, 0x00, //     # ##  
	0x13, 0x00, //    #  ##  
	0x1F, 0x00, //    #####  
	0x21, 0x00, //   #    #  
	0x41, 0x80, //  #     ## 
	0x41, 0x80, //  #     ## 
	0xE3, 0xC0, // ###   ####
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @600 'B' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0x80, //   ####### 
	0x18, 0xC0, //    ##   ##
	0x18, 0xC0, //    ##   ##
	0x18, 0xC0, //    ##   ##
	0x11, 0x80, //    #   ## 
	0x3E, 0x00, //   #####   
	0x31, 0x80, //   ##   ## 
	0x21, 0x80, //   #    ## 
	0x21, 0x80, //   #    ## 
	0x63, 0x00, //  ##   ##  
	0xFE, 0x00, // #######   
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @630 'C' (11 pixels wide)
	0x00, 0x00, //            
	0x0F, 0x20, //     ####  #
	0x18, 0xC0, //    ##   ## 
	0x30, 0x40, //   ##     # 
	0x60, 0x00, //  ##        
	0xC0, 0x00, // ##         
	0xC0, 0x00, // ##         
	0xC0, 0x00, // ##         
	0xC0, 0x00, // ##         
	0xC0, 0x00, // ##         
	0x61, 0x00, //  ##    #   
	0x3E, 0x00, //   #####    
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @660 'D' (12 pixels wide)
	0x00, 0x00, //             
	0x3F, 0xC0, //   ########  
	0x18, 0x60, //    ##    ## 
	0x18, 0x30, //    ##     ##
	0x18, 0x30, //    ##     ##
	0x10, 0x30, //    #      ##
	0x30, 0x30, //   ##      ##
	0x30, 0x30, //   ##      ##
	0x20, 0x60, //   #      ## 
	0x20, 0xE0, //   #     ### 
	0x61, 0x80, //  ##    ##   
	0xFE, 0x00, // #######     
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @690 'E' (12 pixels wide)
	0x00, 0x00, //             
	0x3F, 0xF0, //   ##########
	0x18, 0x20, //    ##     # 
	0x18, 0x20, //    ##     # 
	0x18, 0x00, //    ##       
	0x11, 0x80, //    #   ##   
	0x3F, 0x00, //   ######    
	0x32, 0x00, //   ##  #     
	0x20, 0x00, //   #         
	0x20, 0x40, //   #      #  
	0x60, 0xC0, //  ##     ##  
	0xFF, 0x80, // #########   
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @720 'F' (12 pixels wide)
	0x00, 0x00, //             
	0x3F, 0xF0, //   ##########
	0x18, 0x20, //    ##     # 
	0x18, 0x20, //    ##     # 
	0x18, 0x00, //    ##       
	0x11, 0x80, //    #   ##   
	0x3F, 0x00, //   ######    
	0x32, 0x00, //   ##  #     
	0x20, 0x00, //   #         
	0x20, 0x00, //   #         
	0x60, 0x00, //  ##         
	0xF0, 0x00, // ####        
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @750 'G' (11 pixels wide)
	0x00, 0x00, //            
	0x0F, 0x20, //     ####  #
	0x18, 0xC0, //    ##   ## 
	0x30, 0x40, //   ##     # 
	0x60, 0x40, //  ##      # 
	0x60, 0x00, //  ##        
	0xC0, 0x00, // ##         
	0xC3, 0xC0, // ##    #### 
	0xC1, 0x80, // ##     ##  
	0xC1, 0x00, // ##     #   
	0x61, 0x00, //  ##    #   
	0x3E, 0x00, //   #####    
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @780 'H' (12 pixels wide)
	0x00, 0x00, //             
	0x3E, 0xF0, //   ##### ####
	0x18, 0x20, //    ##     # 
	0x18, 0x60, //    ##    ## 
	0x10, 0x60, //    #     ## 
	0x10, 0x40, //    #     #  
	0x3F, 0xC0, //   ########  
	0x30, 0xC0, //   ##    ##  
	0x20, 0xC0, //   #     ##  
	0x20, 0x80, //   #     #   
	0x61, 0x80, //  ##    ##   
	0xF3, 0xC0, // ####  ####  
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @810 'I' (6 pixels wide)
	0x00, //       
	0x3C, //   ####
	0x18, //    ## 
	0x18, //    ## 
	0x18, //    ## 
	0x10, //    #  
	0x30, //   ##  
	0x30, //   ##  
	0x20, //   #   
	0x20, //   #   
	0x60, //  ##   
	0xF0, // ####  
	0x00, //       
	0x00, //       
	0x00, //       

	// @825 'J' (8 pixels wide)
	0x00, //         
	0x0F, //     ####
	0x06, //      ## 
	0x04, //      #  
	0x04, //      #  
	0x0C, //     ##  
	0x0C, //     ##  
	0x08, //     #   
	0x08, //     #   
	0x18, //    ##   
	0xD0, // ## #    
	0xE0, // ###     
	0x00, //         
	0x00, //         
	0x00, //         

	// @840 'K' (11 pixels wide)
	0x00, 0x00, //            
	0x3E, 0xE0, //   ##### ###
	0x18, 0x40, //    ##    # 
	0x18, 0x80, //    ##   #  
	0x1B, 0x00, //    ## ##   
	0x14, 0x00, //    # #     
	0x3C, 0x00, //   ####     
	0x34, 0x00, //   ## #     
	0x26, 0x00, //   #  ##    
	0x23, 0x00, //   #   ##   
	0x61, 0x00, //  ##    #   
	0xF7, 0xC0, // #### ##### 
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @870 'L' (10 pixels wide)
	0x00, 0x00, //           
	0x3E, 0x00, //   #####   
	0x18, 0x00, //    ##     
	0x18, 0x00, //    ##     
	0x18, 0x00, //    ##     
	0x10, 0x00, //    #      
	0x30, 0x00, //   ##      
	0x30, 0x00, //   ##      
	0x20, 0x00, //   #       
	0x20, 0x40, //   #      #
	0x60, 0x80, //  ##     # 
	0xFF, 0x80, // ######### 
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @900 'M' (14 pixels wide)
	0x00, 0x00, //               
	0x38, 0x1C, //   ###      ###
	0x08, 0x18, //     #      ## 
	0x1C, 0x38, //    ###    ### 
	0x1C, 0x58, //    ###   # ## 
	0x1C, 0x50, //    ###   # #  
	0x2C, 0xB0, //   # ##  # ##  
	0x25, 0x30, //   #  # #  ##  
	0x25, 0x20, //   #  # #  #   
	0x46, 0x20, //  #   ##   #   
	0x44, 0x60, //  #   #   ##   
	0xE4, 0xF0, // ###  #  ####  
	0x00, 0x00, //               
	0x00, 0x00, //               
	0x00, 0x00, //               

	// @930 'N' (12 pixels wide)
	0x00, 0x00, //             
	0x38, 0xF0, //   ###   ####
	0x18, 0x60, //    ##    ## 
	0x1C, 0x40, //    ###   #  
	0x14, 0x40, //    # #   #  
	0x14, 0x40, //    # #   #  
	0x26, 0xC0, //   #  ## ##  
	0x22, 0x80, //   #   # #   
	0x22, 0x80, //   #   # #   
	0x21, 0x80, //   #    ##   
	0x41, 0x00, //  #     #    
	0xE0, 0x00, // ###         
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @960 'O' (10 pixels wide)
	0x00, 0x00, //           
	0x0F, 0x00, //     ####  
	0x19, 0x80, //    ##  ## 
	0x30, 0xC0, //   ##    ##
	0x60, 0xC0, //  ##     ##
	0x60, 0xC0, //  ##     ##
	0xC0, 0xC0, // ##      ##
	0xC1, 0x80, // ##     ## 
	0xC1, 0x80, // ##     ## 
	0xC3, 0x00, // ##    ##  
	0x66, 0x00, //  ##  ##   
	0x3C, 0x00, //   ####    
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @990 'P' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0x80, //   ####### 
	0x18, 0xC0, //    ##   ##
	0x18, 0xC0, //    ##   ##
	0x18, 0xC0, //    ##   ##
	0x11, 0x80, //    #   ## 
	0x3F, 0x00, //   ######  
	0x30, 0x00, //   ##      
	0x20, 0x00, //   #       
	0x20, 0x00, //   #       
	0x60, 0x00, //  ##       
	0xF0, 0x00, // ####      
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1020 'Q' (10 pixels wide)
	0x00, 0x00, //           
	0x0F, 0x00, //     ####  
	0x19, 0x80, //    ##  ## 
	0x30, 0xC0, //   ##    ##
	0x60, 0xC0, //  ##     ##
	0x60, 0xC0, //  ##     ##
	0xC0, 0xC0, // ##      ##
	0xC0, 0xC0, // ##      ##
	0xC1, 0x80, // ##     ## 
	0xC1, 0x80, // ##     ## 
	0xC3, 0x00, // ##    ##  
	0x64, 0x00, //  ##  #    
	0x18, 0x00, //    ##     
	0x30, 0x80, //   ##    # 
	0xDF, 0x00, // ## #####  

	// @1050 'R' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0x80, //   ####### 
	0x18, 0xC0, //    ##   ##
	0x18, 0xC0, //    ##   ##
	0x18, 0xC0, //    ##   ##
	0x11, 0x80, //    #   ## 
	0x3F, 0x00, //   ######  
	0x36, 0x00, //   ## ##   
	0x23, 0x00, //   #   ##  
	0x23, 0x00, //   #   ##  
	0x63, 0x80, //  ##   ### 
	0xF1, 0xC0, // ####   ###
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1080 'S' (9 pixels wide)
	0x00, 0x00, //          
	0x1C, 0x80, //    ###  #
	0x33, 0x00, //   ##  ## 
	0x31, 0x00, //   ##   # 
	0x30, 0x00, //   ##     
	0x18, 0x00, //    ##    
	0x0C, 0x00, //     ##   
	0x06, 0x00, //      ##  
	0x46, 0x00, //  #   ##  
	0x46, 0x00, //  #   ##  
	0x66, 0x00, //  ##  ##  
	0x9C, 0x00, // #  ###   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1110 'T' (10 pixels wide)
	0x00, 0x00, //           
	0x7F, 0xC0, //  #########
	0x58, 0x80, //  # ##   # 
	0x98, 0x80, // #  ##   # 
	0x18, 0x00, //    ##     
	0x10, 0x00, //    #      
	0x30, 0x00, //   ##      
	0x30, 0x00, //   ##      
	0x20, 0x00, //   #       
	0x20, 0x00, //   #       
	0x60, 0x00, //  ##       
	0xF8, 0x00, // #####     
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1140 'U' (11 pixels wide)
	0x00, 0x00, //            
	0x79, 0xE0, //  ####  ####
	0x60, 0xC0, //  ##     ## 
	0x60, 0x80, //  ##     #  
	0x60, 0x80, //  ##     #  
	0x40, 0x80, //  #      #  
	0xC1, 0x00, // ##     #   
	0xC1, 0x00, // ##     #   
	0xC1, 0x00, // ##     #   
	0xC2, 0x00, // ##    #    
	0xC2, 0x00, // ##    #    
	0x7C, 0x00, //  #####     
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1170 'V' (10 pixels wide)
	0x00, 0x00, //           
	0xF1, 0xC0, // ####   ###
	0x60, 0x80, //  ##     # 
	0x61, 0x00, //  ##    #  
	0x61, 0x00, //  ##    #  
	0x22, 0x00, //   #   #   
	0x22, 0x00, //   #   #   
	0x24, 0x00, //   #  #    
	0x38, 0x00, //   ###     
	0x38, 0x00, //   ###     
	0x30, 0x00, //   ##      
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1200 'W' (14 pixels wide)
	0x00, 0x00, //               
	0xF7, 0x9C, // #### ####  ###
	0x63, 0x08, //  ##   ##    # 
	0x63, 0x10, //  ##   ##   #  
	0x63, 0x10, //  ##   ##   #  
	0x27, 0x20, //   #  ###  #   
	0x2B, 0x40, //   # # ## #    
	0x29, 0x40, //   # #  # #    
	0x31, 0x80, //   ##   ##     
	0x31, 0x80, //   ##   ##     
	0x21, 0x00, //   #    #      
	0x21, 0x00, //   #    #      
	0x00, 0x00, //               
	0x00, 0x00, //               
	0x00, 0x00, //               

	// @1230 'X' (11 pixels wide)
	0x00, 0x00, //            
	0x3C, 0xE0, //   ####  ###
	0x18, 0x40, //    ##    # 
	0x18, 0x80, //    ##   #  
	0x09, 0x00, //     #  #   
	0x0E, 0x00, //     ###    
	0x04, 0x00, //      #     
	0x0E, 0x00, //     ###    
	0x12, 0x00, //    #  #    
	0x23, 0x00, //   #   ##   
	0x43, 0x00, //  #    ##   
	0xE7, 0xC0, // ###  ##### 
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1260 'Y' (9 pixels wide)
	0x00, 0x00, //          
	0xF3, 0x80, // ####  ###
	0x61, 0x00, //  ##    # 
	0x62, 0x00, //  ##   #  
	0x24, 0x00, //   #  #   
	0x28, 0x00, //   # #    
	0x38, 0x00, //   ###    
	0x30, 0x00, //   ##     
	0x30, 0x00, //   ##     
	0x20, 0x00, //   #      
	0x20, 0x00, //   #      
	0xF0, 0x00, // ####     
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1290 'Z' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0xC0, //   ########
	0x20, 0x80, //   #     # 
	0x41, 0x80, //  #     ## 
	0x03, 0x00, //       ##  
	0x06, 0x00, //      ##   
	0x0C, 0x00, //     ##    
	0x08, 0x00, //     #     
	0x10, 0x00, //    #      
	0x30, 0x40, //   ##     #
	0x60, 0x80, //  ##     # 
	0xFF, 0x80, // ######### 
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1320 '[' (6 pixels wide)
	0x00, //       
	0x0C, //     ##
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x20, //   #   
	0x20, //   #   
	0x20, //   #   
	0x20, //   #   
	0x40, //  #    
	0x40, //  #    
	0x40, //  #    
	0xE0, // ###   
	0x00, //       

	// @1335 '\' (5 pixels wide)
	0x00, //      
	0x80, // #    
	0xC0, // ##   
	0x40, //  #   
	0x40, //  #   
	0x20, //   #  
	0x20, //   #  
	0x30, //   ## 
	0x10, //    # 
	0x10, //    # 
	0x08, //     #
	0x08, //     #
	0x00, //      
	0x00, //      
	0x00, //      

	// @1350 ']' (6 pixels wide)
	0x00, //       
	0x1C, //    ###
	0x04, //      #
	0x08, //     # 
	0x08, //     # 
	0x08, //     # 
	0x08, //     # 
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x20, //   #   
	0xE0, // ###   
	0x00, //       

	// @1365 '^' (7 pixels wide)
	0x00, //        
	0x10, //    #   
	0x28, //   # #  
	0x28, //   # #  
	0x44, //  #   # 
	0x44, //  #   # 
	0x86, // #    ##
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        

	// @1380 '_' (8 pixels wide)
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
	0x00, //         

	// @1395 '`' (2 pixels wide)
	0x00, //   
	0x80, // # 
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

	// @1410 'a' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x1F, //    #####
	0x31, //   ##   #
	0x61, //  ##    #
	0xC3, // ##    ##
	0xC2, // ##    # 
	0xC7, // ##   ###
	0x7F, //  #######
	0x00, //         
	0x00, //         
	0x00, //         

	// @1425 'b' (7 pixels wide)
	0x00, //        
	0x60, //  ##    
	0x20, //   #    
	0x20, //   #    
	0x40, //  #     
	0x5C, //  # ### 
	0x66, //  ##  ##
	0xC6, // ##   ##
	0x86, // #    ##
	0x8C, // #   ## 
	0x98, // #  ##  
	0xF0, // ####   
	0x00, //        
	0x00, //        
	0x00, //        

	// @1440 'c' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x1F, //    #####
	0x33, //   ##  ##
	0x60, //  ##     
	0xC0, // ##      
	0xC0, // ##      
	0xC4, // ##   #  
	0x78, //  ####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @1455 'd' (9 pixels wide)
	0x00, 0x00, //          
	0x03, 0x00, //       ## 
	0x01, 0x00, //        # 
	0x01, 0x00, //        # 
	0x02, 0x00, //       #  
	0x0E, 0x00, //     ###  
	0x32, 0x00, //   ##  #  
	0x62, 0x00, //  ##   #  
	0xC6, 0x00, // ##   ##  
	0xC6, 0x00, // ##   ##  
	0xCE, 0x80, // ##  ### #
	0x77, 0x00, //  ### ### 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1485 'e' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x1C, //    ###
	0x34, //   ## #
	0x64, //  ##  #
	0xD8, // ## ## 
	0xE0, // ###   
	0xC0, // ##    
	0x78, //  #### 
	0x00, //       
	0x00, //       
	0x00, //       

	// @1500 'f' (10 pixels wide)
	0x03, 0xC0, //       ####
	0x02, 0x00, //       #   
	0x04, 0x00, //      #    
	0x04, 0x00, //      #    
	0x0C, 0x00, //     ##    
	0x1E, 0x00, //    ####   
	0x08, 0x00, //     #     
	0x08, 0x00, //     #     
	0x08, 0x00, //     #     
	0x10, 0x00, //    #      
	0x10, 0x00, //    #      
	0x10, 0x00, //    #      
	0x10, 0x00, //    #      
	0x20, 0x00, //   #       
	0xC0, 0x00, // ##        

	// @1530 'g' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x1C, //    ###  
	0x77, //  ### ###
	0x66, //  ##  ## 
	0x66, //  ##  ## 
	0x1C, //    ###  
	0x60, //  ##     
	0x78, //  ####   
	0x84, // #    #  
	0x84, // #    #  
	0x78, //  ####   

	// @1545 'h' (9 pixels wide)
	0x00, 0x00, //          
	0x30, 0x00, //   ##     
	0x10, 0x00, //    #     
	0x10, 0x00, //    #     
	0x20, 0x00, //   #      
	0x23, 0x80, //   #   ###
	0x2D, 0x80, //   # ## ##
	0x71, 0x80, //  ###   ##
	0x63, 0x00, //  ##   ## 
	0x43, 0x00, //  #    ## 
	0x46, 0x80, //  #   ## #
	0xC7, 0x00, // ##   ### 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1575 'i' (4 pixels wide)
	0x00, //     
	0x30, //   ##
	0x00, //     
	0x00, //     
	0x00, //     
	0xC0, // ##  
	0x40, //  #  
	0x40, //  #  
	0xC0, // ##  
	0x80, // #   
	0x80, // #   
	0xE0, // ### 
	0x00, //     
	0x00, //     
	0x00, //     

	// @1590 'j' (7 pixels wide)
	0x00, //        
	0x06, //      ##
	0x00, //        
	0x00, //        
	0x00, //        
	0x0C, //     ## 
	0x08, //     #  
	0x08, //     #  
	0x08, //     #  
	0x18, //    ##  
	0x10, //    #   
	0x10, //    #   
	0x10, //    #   
	0x30, //   ##   
	0xE0, // ###    

	// @1605 'k' (7 pixels wide)
	0x00, //        
	0x30, //   ##   
	0x10, //    #   
	0x10, //    #   
	0x20, //   #    
	0x2E, //   # ###
	0x24, //   #  # 
	0x68, //  ## #  
	0x70, //  ###   
	0x58, //  # ##  
	0x4A, //  #  # #
	0xCC, // ##  ## 
	0x00, //        
	0x00, //        
	0x00, //        

	// @1620 'l' (4 pixels wide)
	0x00, //     
	0x70, //  ###
	0x20, //   # 
	0x20, //   # 
	0x20, //   # 
	0x60, //  ## 
	0x40, //  #  
	0x40, //  #  
	0x40, //  #  
	0xC0, // ##  
	0xD0, // ## #
	0xE0, // ### 
	0x00, //     
	0x00, //     
	0x00, //     

	// @1635 'm' (11 pixels wide)
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x67, 0x60, //  ##  ### ##
	0x2B, 0xE0, //   # # #####
	0x73, 0x40, //  ###  ## # 
	0x66, 0x40, //  ##  ##  # 
	0x44, 0x40, //  #   #   # 
	0x44, 0xE0, //  #   #  ###
	0xC4, 0xC0, // ##   #  ## 
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1665 'n' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x67, //  ##  ###
	0x3B, //   ### ##
	0x66, //  ##  ## 
	0x64, //  ##  #  
	0x44, //  #   #  
	0x4D, //  #  ## #
	0xCE, // ##  ### 
	0x00, //         
	0x00, //         
	0x00, //         

	// @1680 'o' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x1E, //    #### 
	0x33, //   ##  ##
	0x63, //  ##   ##
	0xC3, // ##    ##
	0xC6, // ##   ## 
	0xCC, // ##  ##  
	0x78, //  ####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @1695 'p' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x3F, 0x00, //   ###### 
	0x19, 0x80, //    ##  ##
	0x11, 0x80, //    #   ##
	0x31, 0x80, //   ##   ##
	0x23, 0x00, //   #   ## 
	0x26, 0x00, //   #  ##  
	0x3C, 0x00, //   ####   
	0x40, 0x00, //  #       
	0x40, 0x00, //  #       
	0xE0, 0x00, // ###      

	// @1725 'q' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x1F, //    #####
	0x32, //   ##  # 
	0x62, //  ##   # 
	0xC2, // ##    # 
	0xC4, // ##   #  
	0xDC, // ## ###  
	0xE4, // ###  #  
	0x0C, //     ##  
	0x08, //     #   
	0x3C, //   ####  

	// @1740 'r' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x6C, //  ## ##
	0x30, //   ##  
	0x20, //   #   
	0x60, //  ##   
	0x40, //  #    
	0x40, //  #    
	0xC0, // ##    
	0x00, //       
	0x00, //       
	0x00, //       

	// @1755 's' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x30, //   ##  
	0x6C, //  ## ##
	0x60, //  ##   
	0x30, //   ##  
	0x98, // #  ## 
	0x98, // #  ## 
	0x70, //  ###  
	0x00, //       
	0x00, //       
	0x00, //       

	// @1770 't' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x20, //   # 
	0x70, //  ###
	0x60, //  ## 
	0x40, //  #  
	0x40, //  #  
	0x40, //  #  
	0xA0, // # # 
	0xE0, // ### 
	0x00, //     
	0x00, //     
	0x00, //     

	// @1785 'u' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0xC2, // ##    #
	0x42, //  #    #
	0xC6, // ##   ##
	0x86, // #    ##
	0x9C, // #  ### 
	0x9E, // #  ####
	0xEE, // ### ###
	0x00, //        
	0x00, //        
	0x00, //        

	// @1800 'v' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xC4, // ##   #
	0xC4, // ##   #
	0x64, //  ##  #
	0x68, //  ## # 
	0x70, //  ###  
	0x60, //  ##   
	0x40, //  #    
	0x00, //       
	0x00, //       
	0x00, //       

	// @1815 'w' (11 pixels wide)
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0xE2, 0x20, // ###   #   #
	0x22, 0x20, //   #   #   #
	0x36, 0x20, //   ## ##   #
	0x3A, 0x40, //   ### #  # 
	0x3A, 0x80, //   ### # #  
	0x33, 0x00, //   ##  ##   
	0x22, 0x00, //   #   #    
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1845 'x' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x36, //   ## ##
	0x5A, //  # ## #
	0x10, //    #   
	0x10, //    #   
	0x30, //   ##   
	0x4C, //  #  ## 
	0x8C, // #   ## 
	0x00, //        
	0x00, //        
	0x00, //        

	// @1860 'y' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x61, //  ##    #
	0x31, //   ##   #
	0x11, //    #   #
	0x12, //    #  # 
	0x1C, //    ###  
	0x0C, //     ##  
	0x08, //     #   
	0x10, //    #    
	0x20, //   #     
	0xC0, // ##      

	// @1875 'z' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3E, //   #####
	0x44, //  #   # 
	0x08, //     #  
	0x10, //    #   
	0x20, //   #    
	0x40, //  #     
	0xF0, // ####   
	0x1C, //    ### 
	0x00, //        
	0x00, //        

	// @1890 '{' (5 pixels wide)
	0x00, //      
	0x08, //     #
	0x10, //    # 
	0x10, //    # 
	0x30, //   ## 
	0x20, //   #  
	0x60, //  ##  
	0x20, //   #  
	0x40, //  #   
	0x40, //  #   
	0x40, //  #   
	0xC0, // ##   
	0xC0, // ##   
	0xE0, // ###  
	0xF0, // #### 

	// @1905 '|' (1 pixels wide)
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
	0x00, //  
	0x00, //  
	0x00, //  

	// @1920 '}' (6 pixels wide)
	0x3C, //   ####
	0x0C, //     ##
	0x08, //     # 
	0x08, //     # 
	0x18, //    ## 
	0x10, //    #  
	0x10, //    #  
	0x10, //    #  
	0x38, //   ### 
	0x20, //   #   
	0x20, //   #   
	0x20, //   #   
	0x60, //  ##   
	0x40, //  #    
	0x80, // #     

	// @1935 '~' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0xE2, // ###   #
	0x9C, // #  ### 
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
};

// Character descriptors for FreeSerif 12pt
// { [Char width in bits], [Offset into freeSerif_12ptCharBitmaps in bytes] }
static const CharDesc_t s_FreeSerifIta12ptCharDesc[] = {
	{4, s_FreeSerifIta12ptBitmaps + 0}, 		// !
	{5, s_FreeSerifIta12ptBitmaps + 15}, 		// "
	{9, s_FreeSerifIta12ptBitmaps + 30}, 		// #
	{7, s_FreeSerifIta12ptBitmaps + 60}, 		// $
	{12,s_FreeSerifIta12ptBitmaps + 75}, 		// %
	{10,s_FreeSerifIta12ptBitmaps + 105}, 		// &
	{2, s_FreeSerifIta12ptBitmaps + 135}, 		// '
	{5, s_FreeSerifIta12ptBitmaps + 150}, 		// (
	{5, s_FreeSerifIta12ptBitmaps + 165}, 		// )
	{6, s_FreeSerifIta12ptBitmaps + 180}, 		// *
	{7, s_FreeSerifIta12ptBitmaps + 195}, 		// +
	{2, s_FreeSerifIta12ptBitmaps + 210}, 		// ,
	{3, s_FreeSerifIta12ptBitmaps + 225}, 		// -
	{2, s_FreeSerifIta12ptBitmaps + 240}, 		// .
	{7, s_FreeSerifIta12ptBitmaps + 255}, 		// /
	{8, s_FreeSerifIta12ptBitmaps + 270}, 		// 0
	{6, s_FreeSerifIta12ptBitmaps + 285}, 		// 1
	{7, s_FreeSerifIta12ptBitmaps + 300}, 		// 2
	{8, s_FreeSerifIta12ptBitmaps + 315}, 		// 3
	{8, s_FreeSerifIta12ptBitmaps + 330}, 		// 4
	{8, s_FreeSerifIta12ptBitmaps + 345}, 		// 5
	{9, s_FreeSerifIta12ptBitmaps + 360}, 		// 6
	{7, s_FreeSerifIta12ptBitmaps + 390}, 		// 7
	{9, s_FreeSerifIta12ptBitmaps + 405}, 		// 8
	{8, s_FreeSerifIta12ptBitmaps + 435}, 		// 9
	{4, s_FreeSerifIta12ptBitmaps + 450}, 		// :
	{4, s_FreeSerifIta12ptBitmaps + 465}, 		// ;
	{8, s_FreeSerifIta12ptBitmaps + 480}, 		// <
	{8, s_FreeSerifIta12ptBitmaps + 495}, 		// =
	{8, s_FreeSerifIta12ptBitmaps + 510}, 		// >
	{6, s_FreeSerifIta12ptBitmaps + 525}, 		// ?
	{11,s_FreeSerifIta12ptBitmaps + 540}, 		// @
	{10,s_FreeSerifIta12ptBitmaps + 570}, 		// A
	{10,s_FreeSerifIta12ptBitmaps + 600}, 		// B
	{11,s_FreeSerifIta12ptBitmaps + 630}, 		// C
	{12,s_FreeSerifIta12ptBitmaps + 660}, 		// D
	{12,s_FreeSerifIta12ptBitmaps + 690}, 		// E
	{12,s_FreeSerifIta12ptBitmaps + 720}, 		// F
	{11,s_FreeSerifIta12ptBitmaps + 750}, 		// G
	{12,s_FreeSerifIta12ptBitmaps + 780}, 		// H
	{6, s_FreeSerifIta12ptBitmaps + 810}, 		// I
	{8, s_FreeSerifIta12ptBitmaps + 825}, 		// J
	{11,s_FreeSerifIta12ptBitmaps + 840}, 		// K
	{10,s_FreeSerifIta12ptBitmaps + 870}, 		// L
	{14,s_FreeSerifIta12ptBitmaps + 900}, 		// M
	{12,s_FreeSerifIta12ptBitmaps + 930}, 		// N
	{10,s_FreeSerifIta12ptBitmaps + 960}, 		// O
	{10,s_FreeSerifIta12ptBitmaps + 990}, 		// P
	{10,s_FreeSerifIta12ptBitmaps + 1020}, 		// Q
	{10,s_FreeSerifIta12ptBitmaps + 1050}, 		// R
	{9, s_FreeSerifIta12ptBitmaps + 1080}, 		// S
	{10,s_FreeSerifIta12ptBitmaps + 1110}, 		// T
	{11,s_FreeSerifIta12ptBitmaps + 1140}, 		// U
	{10,s_FreeSerifIta12ptBitmaps + 1170}, 		// V
	{14,s_FreeSerifIta12ptBitmaps + 1200}, 		// W
	{11,s_FreeSerifIta12ptBitmaps + 1230}, 		// X
	{9, s_FreeSerifIta12ptBitmaps + 1260}, 		// Y
	{10,s_FreeSerifIta12ptBitmaps + 1290}, 		// Z
	{6, s_FreeSerifIta12ptBitmaps + 1320}, 		// [
	{5, s_FreeSerifIta12ptBitmaps + 1335}, 		// '\'
	{6, s_FreeSerifIta12ptBitmaps + 1350}, 		// ]
	{7, s_FreeSerifIta12ptBitmaps + 1365}, 		// ^
	{8, s_FreeSerifIta12ptBitmaps + 1380}, 		// _
	{2, s_FreeSerifIta12ptBitmaps + 1395}, 		// `
	{8, s_FreeSerifIta12ptBitmaps + 1410}, 		// a
	{7, s_FreeSerifIta12ptBitmaps + 1425}, 		// b
	{8, s_FreeSerifIta12ptBitmaps + 1440}, 		// c
	{9, s_FreeSerifIta12ptBitmaps + 1455}, 		// d
	{6, s_FreeSerifIta12ptBitmaps + 1485}, 		// e
	{10,s_FreeSerifIta12ptBitmaps + 1500}, 		// f
	{8, s_FreeSerifIta12ptBitmaps + 1530}, 		// g
	{9, s_FreeSerifIta12ptBitmaps + 1545}, 		// h
	{4, s_FreeSerifIta12ptBitmaps + 1575}, 		// i
	{7, s_FreeSerifIta12ptBitmaps + 1590}, 		// j
	{7, s_FreeSerifIta12ptBitmaps + 1605}, 		// k
	{4, s_FreeSerifIta12ptBitmaps + 1620}, 		// l
	{11,s_FreeSerifIta12ptBitmaps + 1635}, 		// m
	{8, s_FreeSerifIta12ptBitmaps + 1665}, 		// n
	{8, s_FreeSerifIta12ptBitmaps + 1680}, 		// o
	{9, s_FreeSerifIta12ptBitmaps + 1695}, 		// p
	{8, s_FreeSerifIta12ptBitmaps + 1725}, 		// q
	{6, s_FreeSerifIta12ptBitmaps + 1740}, 		// r
	{6, s_FreeSerifIta12ptBitmaps + 1755}, 		// s
	{4, s_FreeSerifIta12ptBitmaps + 1770}, 		// t
	{7, s_FreeSerifIta12ptBitmaps + 1785}, 		// u
	{6, s_FreeSerifIta12ptBitmaps + 1800}, 		// v
	{11,s_FreeSerifIta12ptBitmaps + 1815}, 		// w
	{7, s_FreeSerifIta12ptBitmaps + 1845}, 		// x
	{8, s_FreeSerifIta12ptBitmaps + 1860}, 		// y
	{7, s_FreeSerifIta12ptBitmaps + 1875}, 		// z
	{5, s_FreeSerifIta12ptBitmaps + 1890}, 		// {
	{1, s_FreeSerifIta12ptBitmaps + 1905}, 		// |
	{6, s_FreeSerifIta12ptBitmaps + 1920}, 		// }
	{7, s_FreeSerifIta12ptBitmaps + 1935}, 		// ~
};

// Font information for FreeSerif 12pt
const FontDesc_t iFontFreeSerifIta12pt = {
	0,
	14,
	15,
	{ .pCharDesc = s_FreeSerifIta12ptCharDesc }
};
