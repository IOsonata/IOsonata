// 
//  Font data for FreeSerif Bold Italic 12pt
// 	https://savannah.gnu.org/projects/freefont/
// 
// 	Font bitmap generated by
// 	http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator

#include "display/ifont.h"

// Character bitmaps for FreeSerif 12pt
static const uint8_t s_FreeSerifBoldIta12ptBitmaps[] = {
	// @0 '!' (6 pixels wide)
	0x00, //       
	0x0C, //     ##
	0x1C, //    ###
	0x1C, //    ###
	0x18, //    ## 
	0x18, //    ## 
	0x10, //    #  
	0x10, //    #  
	0x20, //   #   
	0x20, //   #   
	0x00, //       
	0xE0, // ###   
	0xE0, // ###   
	0x00, //       
	0x00, //       

	// @15 '"' (7 pixels wide)
	0x00, //        
	0xC6, // ##   ##
	0xC6, // ##   ##
	0x84, // #    # 
	0x84, // #    # 
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

	// @30 '#' (8 pixels wide)
	0x00, //         
	0x09, //     #  #
	0x1B, //    ## ##
	0x12, //    #  # 
	0x7F, //  #######
	0x26, //   #  ## 
	0x24, //   #  #  
	0x24, //   #  #  
	0xFE, // ####### 
	0x48, //  #  #   
	0xD8, // ## ##   
	0x90, // #  #    
	0x00, //         
	0x00, //         
	0x00, //         

	// @45 '$' (9 pixels wide)
	0x04, 0x00, //      #   
	0x3C, 0x00, //   ####   
	0x27, 0x80, //   #  ####
	0x49, 0x00, //  #  #  # 
	0x69, 0x00, //  ## #  # 
	0x78, 0x00, //  ####    
	0x3C, 0x00, //   ####   
	0x1E, 0x00, //    ####  
	0x16, 0x00, //    # ##  
	0x96, 0x00, // #  # ##  
	0xA6, 0x00, // # #  ##  
	0xEC, 0x00, // ### ##   
	0x38, 0x00, //   ###    
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @75 '%' (12 pixels wide)
	0x00, 0x00, //             
	0x38, 0x80, //   ###   #   
	0x67, 0x00, //  ##  ###    
	0xE5, 0x00, // ###  # #    
	0xC6, 0x00, // ##   ##     
	0xCE, 0x00, // ##  ###     
	0xCA, 0xE0, // ##  # # ### 
	0x75, 0x90, //  ### # ##  #
	0x05, 0x90, //      # ##  #
	0x0B, 0x10, //     # ##   #
	0x0B, 0x30, //     # ##  ##
	0x13, 0x20, //    #  ##  # 
	0x11, 0xC0, //    #   ###  
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @105 '&' (11 pixels wide)
	0x00, 0x00, //            
	0x07, 0x00, //      ###   
	0x0C, 0x80, //     ##  #  
	0x0C, 0x80, //     ##  #  
	0x0D, 0x00, //     ## #   
	0x0E, 0x00, //     ###    
	0x1C, 0xE0, //    ###  ###
	0x7E, 0x40, //  ######  # 
	0x66, 0x40, //  ##  ##  # 
	0xE6, 0x80, // ###  ## #  
	0xE3, 0x00, // ###   ##   
	0xF3, 0x80, // ####  ###  
	0x7D, 0xE0, //  ##### ####
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @135 ''' (2 pixels wide)
	0x00, //   
	0xC0, // ##
	0xC0, // ##
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

	// @150 '(' (4 pixels wide)
	0x00, //     
	0x10, //    #
	0x20, //   # 
	0x60, //  ## 
	0x40, //  #  
	0xC0, // ##  
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0x80, // #   
	0x40, //  #  
	0x00, //     

	// @165 ')' (5 pixels wide)
	0x00, //      
	0x10, //    # 
	0x10, //    # 
	0x10, //    # 
	0x08, //     #
	0x08, //     #
	0x08, //     #
	0x08, //     #
	0x08, //     #
	0x18, //    ##
	0x18, //    ##
	0x10, //    # 
	0x20, //   #  
	0x40, //  #   
	0x80, // #    

	// @180 '*' (7 pixels wide)
	0x00, //        
	0x10, //    #   
	0x10, //    #   
	0xD6, // ## # ##
	0x38, //   ###  
	0x10, //    #   
	0xFC, // ###### 
	0x10, //    #   
	0x10, //    #   
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

	// @210 ',' (3 pixels wide)
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
	0x60, //  ##
	0x60, //  ##
	0x20, //   #
	0x40, //  # 
	0x80, // #  

	// @225 '-' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0xF0, // ####
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     

	// @240 '.' (3 pixels wide)
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
	0xE0, // ###
	0xE0, // ###
	0x00, //    
	0x00, //    

	// @255 '/' (6 pixels wide)
	0x00, //       
	0x04, //      #
	0x0C, //     ##
	0x08, //     # 
	0x18, //    ## 
	0x10, //    #  
	0x30, //   ##  
	0x20, //   #   
	0x20, //   #   
	0x40, //  #    
	0x40, //  #    
	0xC0, // ##    
	0x00, //       
	0x00, //       
	0x00, //       

	// @270 '0' (7 pixels wide)
	0x00, //        
	0x1C, //    ### 
	0x32, //   ##  #
	0x22, //   #   #
	0x62, //  ##   #
	0x62, //  ##   #
	0xC6, // ##   ##
	0xC6, // ##   ##
	0xC4, // ##   # 
	0x8C, // #   ## 
	0x8C, // #   ## 
	0x98, // #  ##  
	0x70, //  ###   
	0x00, //        
	0x00, //        

	// @285 '1' (7 pixels wide)
	0x00, //        
	0x06, //      ##
	0x3C, //   #### 
	0x0C, //     ## 
	0x0C, //     ## 
	0x18, //    ##  
	0x18, //    ##  
	0x18, //    ##  
	0x38, //   ###  
	0x30, //   ##   
	0x30, //   ##   
	0x70, //  ###   
	0xFC, // ###### 
	0x00, //        
	0x00, //        

	// @300 '2' (8 pixels wide)
	0x00, //         
	0x1E, //    #### 
	0x67, //  ##  ###
	0x47, //  #   ###
	0x07, //      ###
	0x07, //      ###
	0x0E, //     ### 
	0x0C, //     ##  
	0x18, //    ##   
	0x30, //   ##    
	0x30, //   ##    
	0x62, //  ##   # 
	0xFC, // ######  
	0x00, //         
	0x00, //         

	// @315 '3' (9 pixels wide)
	0x00, 0x00, //          
	0x0F, 0x00, //     #### 
	0x33, 0x80, //   ##  ###
	0x03, 0x80, //       ###
	0x03, 0x00, //       ## 
	0x0C, 0x00, //     ##   
	0x1E, 0x00, //    ####  
	0x0E, 0x00, //     ###  
	0x06, 0x00, //      ##  
	0x06, 0x00, //      ##  
	0x06, 0x00, //      ##  
	0xCC, 0x00, // ##  ##   
	0x78, 0x00, //  ####    
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @345 '4' (8 pixels wide)
	0x00, //         
	0x01, //        #
	0x03, //       ##
	0x07, //      ###
	0x0E, //     ### 
	0x16, //    # ## 
	0x26, //   #  ## 
	0x46, //  #   ## 
	0x8C, // #   ##  
	0xFE, // ####### 
	0x0C, //     ##  
	0x1C, //    ###  
	0x00, //         
	0x00, //         
	0x00, //         

	// @360 '5' (7 pixels wide)
	0x00, //        
	0x1E, //    ####
	0x20, //   #    
	0x20, //   #    
	0x38, //   ###  
	0x7C, //  ##### 
	0x0C, //     ## 
	0x04, //      # 
	0x04, //      # 
	0x04, //      # 
	0x04, //      # 
	0xC8, // ##  #  
	0xF0, // ####   
	0x00, //        
	0x00, //        

	// @375 '6' (8 pixels wide)
	0x00, //         
	0x03, //       ##
	0x0E, //     ### 
	0x18, //    ##   
	0x30, //   ##    
	0x7C, //  #####  
	0x66, //  ##  ## 
	0xE6, // ###  ## 
	0xC6, // ##   ## 
	0xC6, // ##   ## 
	0xCE, // ##  ### 
	0x4C, //  #  ##  
	0x78, //  ####   
	0x00, //         
	0x00, //         

	// @390 '7' (7 pixels wide)
	0x00, //        
	0x7E, //  ######
	0x86, // #    ##
	0x04, //      # 
	0x0C, //     ## 
	0x08, //     #  
	0x18, //    ##  
	0x10, //    #   
	0x30, //   ##   
	0x60, //  ##    
	0x40, //  #     
	0xC0, // ##     
	0x00, //        
	0x00, //        
	0x00, //        

	// @405 '8' (7 pixels wide)
	0x00, //        
	0x1C, //    ### 
	0x32, //   ##  #
	0x32, //   ##  #
	0x32, //   ##  #
	0x34, //   ## # 
	0x18, //    ##  
	0x2C, //   # ## 
	0xCC, // ##  ## 
	0x8C, // #   ## 
	0x8C, // #   ## 
	0xC8, // ##  #  
	0x38, //   ###  
	0x00, //        
	0x00, //        

	// @420 '9' (8 pixels wide)
	0x00, //         
	0x1E, //    #### 
	0x32, //   ##  # 
	0x73, //  ###  ##
	0x63, //  ##   ##
	0x63, //  ##   ##
	0x67, //  ##  ###
	0x66, //  ##  ## 
	0x3E, //   ##### 
	0x0C, //     ##  
	0x18, //    ##   
	0x70, //  ###    
	0xC0, // ##      
	0x00, //         
	0x00, //         

	// @435 ':' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x38, //   ###
	0x38, //   ###
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0xE0, // ###  
	0xE0, // ###  
	0x00, //      
	0x00, //      

	// @450 ';' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x38, //   ###
	0x38, //   ###
	0x00, //      
	0x00, //      
	0x00, //      
	0x60, //  ##  
	0x60, //  ##  
	0x20, //   #  
	0x40, //  #   
	0x80, // #    

	// @465 '<' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x03, 0x80, //       ###
	0x0F, 0x00, //     #### 
	0x38, 0x00, //   ###    
	0xE0, 0x00, // ###      
	0x70, 0x00, //  ###     
	0x1C, 0x00, //    ###   
	0x07, 0x80, //      ####
	0x01, 0x80, //        ##
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

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
	0x70, //  ###    
	0x1E, //    #### 
	0x03, //       ##
	0x0F, //     ####
	0x3C, //   ####  
	0xE0, // ###     
	0x80, // #       
	0x00, //         
	0x00, //         
	0x00, //         

	// @525 '?' (7 pixels wide)
	0x00, //        
	0x1C, //    ### 
	0x2E, //   # ###
	0x2E, //   # ###
	0x0E, //     ###
	0x0C, //     ## 
	0x18, //    ##  
	0x30, //   ##   
	0x60, //  ##    
	0x40, //  #     
	0x00, //        
	0xE0, // ###    
	0xE0, // ###    
	0x00, //        
	0x00, //        

	// @540 '@' (12 pixels wide)
	0x00, 0x00, //             
	0x1F, 0x80, //    ######   
	0x30, 0xC0, //   ##    ##  
	0x43, 0x20, //  #    ##  # 
	0xC4, 0x90, // ##   #  #  #
	0x89, 0x90, // #   #  ##  #
	0x99, 0x10, // #  ##  #   #
	0x91, 0x10, // #  #   #   #
	0x97, 0x20, // #  # ###  # 
	0x59, 0xC0, //  # ##  ###  
	0x30, 0x00, //   ##        
	0x1F, 0x80, //    ######   
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @570 'A' (11 pixels wide)
	0x00, 0x00, //            
	0x01, 0x00, //        #   
	0x03, 0x00, //       ##   
	0x03, 0x00, //       ##   
	0x07, 0x00, //      ###   
	0x0B, 0x00, //     # ##   
	0x0B, 0x80, //     # ###  
	0x11, 0x80, //    #   ##  
	0x1F, 0x80, //    ######  
	0x21, 0x80, //   #    ##  
	0x41, 0x80, //  #     ##  
	0x41, 0x80, //  #     ##  
	0xE3, 0xE0, // ###   #####
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @600 'B' (11 pixels wide)
	0x00, 0x00, //            
	0x3F, 0xC0, //   ######## 
	0x1C, 0xE0, //    ###  ###
	0x18, 0xE0, //    ##   ###
	0x18, 0xE0, //    ##   ###
	0x19, 0xC0, //    ##  ### 
	0x3E, 0x00, //   #####    
	0x31, 0x80, //   ##   ##  
	0x31, 0xC0, //   ##   ### 
	0x31, 0xC0, //   ##   ### 
	0x71, 0xC0, //  ###   ### 
	0x73, 0x80, //  ###  ###  
	0xFF, 0x00, // ########   
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @630 'C' (11 pixels wide)
	0x00, 0x00, //            
	0x0F, 0xA0, //     ##### #
	0x1C, 0xE0, //    ###  ###
	0x38, 0x40, //   ###    # 
	0x70, 0x40, //  ###     # 
	0x70, 0x00, //  ###       
	0xE0, 0x00, // ###        
	0xE0, 0x00, // ###        
	0xE0, 0x00, // ###        
	0xE0, 0x00, // ###        
	0xE0, 0x80, // ###     #  
	0x71, 0x00, //  ###   #   
	0x3E, 0x00, //   #####    
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @660 'D' (12 pixels wide)
	0x00, 0x00, //             
	0x3F, 0xC0, //   ########  
	0x1C, 0xE0, //    ###  ### 
	0x1C, 0x70, //    ###   ###
	0x18, 0x70, //    ##    ###
	0x18, 0x70, //    ##    ###
	0x38, 0x70, //   ###    ###
	0x30, 0x70, //   ##     ###
	0x30, 0xF0, //   ##    ####
	0x30, 0xE0, //   ##    ### 
	0x71, 0xC0, //  ###   ###  
	0x73, 0x80, //  ###  ###   
	0xFF, 0x00, // ########    
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @690 'E' (12 pixels wide)
	0x00, 0x00, //             
	0x3F, 0xF0, //   ##########
	0x1C, 0x70, //    ###   ###
	0x1C, 0x20, //    ###    # 
	0x18, 0x80, //    ##   #   
	0x19, 0x80, //    ##  ##   
	0x3F, 0x80, //   #######   
	0x31, 0x00, //   ##   #    
	0x31, 0x00, //   ##   #    
	0x30, 0x40, //   ##     #  
	0x70, 0xC0, //  ###    ##  
	0x71, 0x80, //  ###   ##   
	0xFF, 0x80, // #########   
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @720 'F' (12 pixels wide)
	0x00, 0x00, //             
	0x3F, 0xF0, //   ##########
	0x1C, 0x70, //    ###   ###
	0x1C, 0x20, //    ###    # 
	0x18, 0x80, //    ##   #   
	0x19, 0x80, //    ##  ##   
	0x3F, 0x80, //   #######   
	0x31, 0x00, //   ##   #    
	0x31, 0x00, //   ##   #    
	0x30, 0x00, //   ##        
	0x70, 0x00, //  ###        
	0x70, 0x00, //  ###        
	0xF8, 0x00, // #####       
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @750 'G' (11 pixels wide)
	0x00, 0x00, //            
	0x0F, 0xA0, //     ##### #
	0x1C, 0x60, //    ###   ##
	0x38, 0x40, //   ###    # 
	0x70, 0x00, //  ###       
	0x70, 0x00, //  ###       
	0xE0, 0x00, // ###        
	0xE3, 0xE0, // ###   #####
	0xE1, 0xC0, // ###    ### 
	0xE1, 0x80, // ###    ##  
	0xE1, 0x80, // ###    ##  
	0x71, 0x80, //  ###   ##  
	0x3E, 0x00, //   #####    
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @780 'H' (13 pixels wide)
	0x00, 0x00, //              
	0x3E, 0xF8, //   ##### #####
	0x1C, 0x30, //    ###    ## 
	0x1C, 0x70, //    ###   ### 
	0x18, 0x60, //    ##    ##  
	0x18, 0x60, //    ##    ##  
	0x3F, 0xE0, //   #########  
	0x30, 0xE0, //   ##    ###  
	0x30, 0xC0, //   ##    ##   
	0x30, 0xC0, //   ##    ##   
	0x71, 0xC0, //  ###   ###   
	0x71, 0xC0, //  ###   ###   
	0xFB, 0xE0, // ##### #####  
	0x00, 0x00, //              
	0x00, 0x00, //              

	// @810 'I' (7 pixels wide)
	0x00, //        
	0x3E, //   #####
	0x1C, //    ### 
	0x18, //    ##  
	0x18, //    ##  
	0x18, //    ##  
	0x38, //   ###  
	0x30, //   ##   
	0x30, //   ##   
	0x30, //   ##   
	0x70, //  ###   
	0x70, //  ###   
	0xF8, // #####  
	0x00, //        
	0x00, //        

	// @825 'J' (9 pixels wide)
	0x00, 0x00, //          
	0x0F, 0x80, //     #####
	0x03, 0x00, //       ## 
	0x07, 0x00, //      ### 
	0x06, 0x00, //      ##  
	0x06, 0x00, //      ##  
	0x0E, 0x00, //     ###  
	0x0E, 0x00, //     ###  
	0x0C, 0x00, //     ##   
	0x0C, 0x00, //     ##   
	0x1C, 0x00, //    ###   
	0x18, 0x00, //    ##    
	0xD8, 0x00, // ## ##    
	0xF0, 0x00, // ####     
	0x00, 0x00, //          

	// @855 'K' (12 pixels wide)
	0x00, 0x00, //             
	0x3E, 0xF0, //   ##### ####
	0x1C, 0x40, //    ###   #  
	0x1C, 0x80, //    ###  #   
	0x19, 0x00, //    ##  #    
	0x1A, 0x00, //    ## #     
	0x3E, 0x00, //   #####     
	0x36, 0x00, //   ## ##     
	0x37, 0x00, //   ## ###    
	0x33, 0x00, //   ##  ##    
	0x73, 0x80, //  ###  ###   
	0x71, 0x80, //  ###   ##   
	0xFB, 0xC0, // ##### ####  
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @885 'L' (10 pixels wide)
	0x00, 0x00, //           
	0x3E, 0x00, //   #####   
	0x1C, 0x00, //    ###    
	0x18, 0x00, //    ##     
	0x18, 0x00, //    ##     
	0x18, 0x00, //    ##     
	0x38, 0x00, //   ###     
	0x30, 0x00, //   ##      
	0x30, 0x00, //   ##      
	0x30, 0x40, //   ##     #
	0x70, 0xC0, //  ###    ##
	0x71, 0x80, //  ###   ## 
	0xFF, 0x80, // ######### 
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @915 'M' (15 pixels wide)
	0x00, 0x00, //                
	0x3C, 0x1E, //   ####     ####
	0x0C, 0x1C, //     ##     ### 
	0x1C, 0x3C, //    ###    #### 
	0x1C, 0x38, //    ###    ###  
	0x1C, 0x58, //    ###   # ##  
	0x2C, 0x98, //   # ##  #  ##  
	0x2E, 0xB8, //   # ### # ###  
	0x2F, 0x30, //   # ####  ##   
	0x27, 0x30, //   #  ###  ##   
	0x46, 0x70, //  #   ##  ###   
	0x44, 0x70, //  #   #   ###   
	0xF4, 0xF8, // #### #  #####  
	0x00, 0x00, //                
	0x00, 0x00, //                

	// @945 'N' (12 pixels wide)
	0x00, 0x00, //             
	0x78, 0xF0, //  ####   ####
	0x38, 0x60, //   ###    ## 
	0x38, 0x40, //   ###    #  
	0x2C, 0x40, //   # ##   #  
	0x2C, 0x40, //   # ##   #  
	0x46, 0x80, //  #   ## #   
	0x46, 0x80, //  #   ## #   
	0x47, 0x80, //  #   ####   
	0x43, 0x00, //  #    ##    
	0x83, 0x00, // #     ##    
	0x81, 0x00, // #      #    
	0x41, 0x00, //  #     #    
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @975 'O' (11 pixels wide)
	0x00, 0x00, //            
	0x07, 0x80, //      ####  
	0x18, 0xC0, //    ##   ## 
	0x38, 0xE0, //   ###   ###
	0x70, 0xE0, //  ###    ###
	0x70, 0xE0, //  ###    ###
	0xF0, 0xE0, // ####    ###
	0xE1, 0xE0, // ###    ####
	0xE1, 0xC0, // ###    ### 
	0xE1, 0xC0, // ###    ### 
	0xE3, 0x80, // ###   ###  
	0x67, 0x00, //  ##  ###   
	0x3C, 0x00, //   ####     
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1005 'P' (11 pixels wide)
	0x00, 0x00, //            
	0x3F, 0xC0, //   ######## 
	0x1C, 0xE0, //    ###  ###
	0x18, 0xE0, //    ##   ###
	0x18, 0xE0, //    ##   ###
	0x19, 0xC0, //    ##  ### 
	0x3F, 0x80, //   #######  
	0x30, 0x00, //   ##       
	0x30, 0x00, //   ##       
	0x30, 0x00, //   ##       
	0x70, 0x00, //  ###       
	0x70, 0x00, //  ###       
	0xF8, 0x00, // #####      
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1035 'Q' (11 pixels wide)
	0x00, 0x00, //            
	0x07, 0x80, //      ####  
	0x18, 0xC0, //    ##   ## 
	0x38, 0xE0, //   ###   ###
	0x70, 0xE0, //  ###    ###
	0x70, 0xE0, //  ###    ###
	0xF0, 0xE0, // ####    ###
	0xE1, 0xE0, // ###    ####
	0xE1, 0xC0, // ###    ### 
	0xE1, 0x80, // ###    ##  
	0xE3, 0x80, // ###   ###  
	0x66, 0x00, //  ##  ##    
	0x18, 0x00, //    ##      
	0x38, 0x40, //   ###    # 
	0xDF, 0x80, // ## ######  

	// @1065 'R' (11 pixels wide)
	0x00, 0x00, //            
	0x3F, 0xC0, //   ######## 
	0x1C, 0xE0, //    ###  ###
	0x1C, 0xE0, //    ###  ###
	0x18, 0xE0, //    ##   ###
	0x19, 0xC0, //    ##  ### 
	0x3F, 0x00, //   ######   
	0x3F, 0x00, //   ######   
	0x33, 0x00, //   ##  ##   
	0x33, 0x80, //   ##  ###  
	0x73, 0x80, //  ###  ###  
	0x71, 0x80, //  ###   ##  
	0xF9, 0xE0, // #####  ####
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1095 'S' (9 pixels wide)
	0x00, 0x00, //          
	0x0C, 0x80, //     ##  #
	0x13, 0x00, //    #  ## 
	0x31, 0x00, //   ##   # 
	0x31, 0x00, //   ##   # 
	0x38, 0x00, //   ###    
	0x1C, 0x00, //    ###   
	0x0E, 0x00, //     ###  
	0x07, 0x00, //      ### 
	0x03, 0x00, //       ## 
	0x43, 0x00, //  #    ## 
	0x66, 0x00, //  ##  ##  
	0x9C, 0x00, // #  ###   
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1125 'T' (11 pixels wide)
	0x00, 0x00, //            
	0x3F, 0xE0, //   #########
	0x2E, 0xE0, //   # ### ###
	0x4C, 0x40, //  #  ##   # 
	0x0C, 0x40, //     ##   # 
	0x0C, 0x00, //     ##     
	0x1C, 0x00, //    ###     
	0x18, 0x00, //    ##      
	0x18, 0x00, //    ##      
	0x38, 0x00, //   ###      
	0x30, 0x00, //   ##       
	0x30, 0x00, //   ##       
	0xFC, 0x00, // ######     
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1155 'U' (11 pixels wide)
	0x00, 0x00, //            
	0x79, 0xE0, //  ####  ####
	0x70, 0x40, //  ###     # 
	0x70, 0x80, //  ###    #  
	0x60, 0x80, //  ##     #  
	0x60, 0x80, //  ##     #  
	0xE0, 0x80, // ###     #  
	0xC1, 0x00, // ##     #   
	0xE1, 0x00, // ###    #   
	0xE1, 0x00, // ###    #   
	0xE2, 0x00, // ###   #    
	0xE2, 0x00, // ###   #    
	0x7C, 0x00, //  #####     
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1185 'V' (11 pixels wide)
	0x00, 0x00, //            
	0xF8, 0xE0, // #####   ###
	0x70, 0x40, //  ###     # 
	0x70, 0x80, //  ###    #  
	0x31, 0x00, //   ##   #   
	0x31, 0x00, //   ##   #   
	0x32, 0x00, //   ##  #    
	0x34, 0x00, //   ## #     
	0x3C, 0x00, //   ####     
	0x38, 0x00, //   ###      
	0x30, 0x00, //   ##       
	0x10, 0x00, //    #       
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1215 'W' (14 pixels wide)
	0x00, 0x00, //               
	0xFB, 0xFC, // ##### ########
	0x71, 0xC8, //  ###   ###  # 
	0x71, 0x88, //  ###   ##   # 
	0x31, 0x90, //   ##   ##  #  
	0x37, 0x90, //   ## ####  #  
	0x35, 0xA0, //   ## # ## #   
	0x39, 0xA0, //   ###  ## #   
	0x39, 0xE0, //   ###  ####   
	0x31, 0xC0, //   ##   ###    
	0x31, 0xC0, //   ##   ###    
	0x20, 0x80, //   #     #     
	0x00, 0x00, //               
	0x00, 0x00, //               
	0x00, 0x00, //               

	// @1245 'X' (11 pixels wide)
	0x00, 0x00, //            
	0x3E, 0xE0, //   ##### ###
	0x1C, 0x40, //    ###   # 
	0x0C, 0x80, //     ##  #  
	0x0D, 0x00, //     ## #   
	0x0E, 0x00, //     ###    
	0x06, 0x00, //      ##    
	0x0E, 0x00, //     ###    
	0x16, 0x00, //    # ##    
	0x13, 0x00, //    #  ##   
	0x23, 0x00, //   #   ##   
	0x43, 0x80, //  #    ###  
	0xE7, 0xC0, // ###  ##### 
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1275 'Y' (9 pixels wide)
	0x00, 0x00, //          
	0xFB, 0x80, // ##### ###
	0x61, 0x00, //  ##    # 
	0x71, 0x00, //  ###   # 
	0x32, 0x00, //   ##  #  
	0x34, 0x00, //   ## #   
	0x38, 0x00, //   ###    
	0x38, 0x00, //   ###    
	0x30, 0x00, //   ##     
	0x30, 0x00, //   ##     
	0x30, 0x00, //   ##     
	0x70, 0x00, //  ###     
	0xF8, 0x00, // #####    
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1305 'Z' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0xC0, //   ########
	0x31, 0x80, //   ##   ## 
	0x63, 0x80, //  ##   ### 
	0x03, 0x00, //       ##  
	0x06, 0x00, //      ##   
	0x0E, 0x00, //     ###   
	0x0C, 0x00, //     ##    
	0x18, 0x00, //    ##     
	0x38, 0x40, //   ###    #
	0x30, 0xC0, //   ##    ##
	0x61, 0x80, //  ##    ## 
	0xFF, 0x80, // ######### 
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1335 '[' (6 pixels wide)
	0x00, //       
	0x3C, //   ####
	0x30, //   ##  
	0x30, //   ##  
	0x20, //   #   
	0x20, //   #   
	0x60, //  ##   
	0x60, //  ##   
	0x40, //  #    
	0x40, //  #    
	0xC0, // ##    
	0xC0, // ##    
	0x80, // #     
	0x80, // #     
	0xE0, // ###   

	// @1350 '\' (4 pixels wide)
	0x00, //     
	0x80, // #   
	0xC0, // ##  
	0x40, //  #  
	0x40, //  #  
	0x40, //  #  
	0x60, //  ## 
	0x20, //   # 
	0x20, //   # 
	0x20, //   # 
	0x30, //   ##
	0x10, //    #
	0x00, //     
	0x00, //     
	0x00, //     

	// @1365 ']' (6 pixels wide)
	0x00, //       
	0x1C, //    ###
	0x04, //      #
	0x0C, //     ##
	0x0C, //     ##
	0x08, //     # 
	0x08, //     # 
	0x18, //    ## 
	0x18, //    ## 
	0x18, //    ## 
	0x10, //    #  
	0x10, //    #  
	0x30, //   ##  
	0x30, //   ##  
	0xE0, // ###   

	// @1380 '^' (7 pixels wide)
	0x00, //        
	0x10, //    #   
	0x38, //   ###  
	0x28, //   # #  
	0x64, //  ##  # 
	0x44, //  #   # 
	0xC6, // ##   ##
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        

	// @1395 '_' (8 pixels wide)
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

	// @1410 '`' (3 pixels wide)
	0x00, //    
	0x80, // #  
	0xC0, // ## 
	0x20, //   #
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

	// @1425 'a' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x1F, //    #####
	0x36, //   ## ## 
	0x66, //  ##  ## 
	0x66, //  ##  ## 
	0xC4, // ##   #  
	0xCC, // ##  ##  
	0xCE, // ##  ### 
	0x7C, //  #####  
	0x00, //         
	0x00, //         

	// @1440 'b' (8 pixels wide)
	0x00, //         
	0x78, //  ####   
	0x18, //    ##   
	0x30, //   ##    
	0x30, //   ##    
	0x3E, //   ##### 
	0x73, //  ###  ##
	0x63, //  ##   ##
	0x63, //  ##   ##
	0x46, //  #   ## 
	0xC6, // ##   ## 
	0xCC, // ##  ##  
	0x70, //  ###    
	0x00, //         
	0x00, //         

	// @1455 'c' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x1C, //    ###
	0x34, //   ## #
	0x70, //  ###  
	0xE0, // ###   
	0xE0, // ###   
	0xE0, // ###   
	0xE8, // ### # 
	0x70, //  ###  
	0x00, //       
	0x00, //       

	// @1470 'd' (9 pixels wide)
	0x00, 0x00, //          
	0x03, 0x80, //       ###
	0x01, 0x80, //        ##
	0x03, 0x00, //       ## 
	0x03, 0x00, //       ## 
	0x0F, 0x00, //     #### 
	0x32, 0x00, //   ##  #  
	0x66, 0x00, //  ##  ##  
	0x66, 0x00, //  ##  ##  
	0xC6, 0x00, // ##   ##  
	0xCE, 0x00, // ##  ###  
	0xCF, 0x00, // ##  #### 
	0xF6, 0x00, // #### ##  
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1500 'e' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x1C, //    ###
	0x34, //   ## #
	0x74, //  ### #
	0xE8, // ### # 
	0xF0, // ####  
	0xE0, // ###   
	0xE0, // ###   
	0x78, //  #### 
	0x00, //       
	0x00, //       

	// @1515 'f' (10 pixels wide)
	0x01, 0xC0, //        ###
	0x03, 0x40, //       ## #
	0x03, 0x00, //       ##  
	0x06, 0x00, //      ##   
	0x06, 0x00, //      ##   
	0x1F, 0x00, //    #####  
	0x06, 0x00, //      ##   
	0x0C, 0x00, //     ##    
	0x0C, 0x00, //     ##    
	0x0C, 0x00, //     ##    
	0x08, 0x00, //     #     
	0x18, 0x00, //    ##     
	0x18, 0x00, //    ##     
	0x90, 0x00, // #  #      
	0xE0, 0x00, // ###       

	// @1545 'g' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x38, //   ###   
	0x27, //   #  ###
	0x66, //  ##  ## 
	0x6E, //  ## ### 
	0x3C, //   ####  
	0x40, //  #      
	0x78, //  ####   
	0x8C, // #   ##  
	0x8C, // #   ##  
	0x78, //  ####   

	// @1560 'h' (8 pixels wide)
	0x00, //         
	0x38, //   ###   
	0x18, //    ##   
	0x18, //    ##   
	0x10, //    #    
	0x33, //   ##  ##
	0x3F, //   ######
	0x33, //   ##  ##
	0x33, //   ##  ##
	0x66, //  ##  ## 
	0x66, //  ##  ## 
	0x67, //  ##  ###
	0xC6, // ##   ## 
	0x00, //         
	0x00, //         

	// @1575 'i' (5 pixels wide)
	0x00, //      
	0x38, //   ###
	0x38, //   ###
	0x00, //      
	0x00, //      
	0xE0, // ###  
	0x60, //  ##  
	0x40, //  #   
	0xC0, // ##   
	0xC0, // ##   
	0xC0, // ##   
	0xD0, // ## # 
	0xE0, // ###  
	0x00, //      
	0x00, //      

	// @1590 'j' (8 pixels wide)
	0x00, //         
	0x07, //      ###
	0x07, //      ###
	0x00, //         
	0x18, //    ##   
	0x0E, //     ### 
	0x0C, //     ##  
	0x0C, //     ##  
	0x0C, //     ##  
	0x08, //     #   
	0x18, //    ##   
	0x18, //    ##   
	0x18, //    ##   
	0xB0, // # ##    
	0xE0, // ###     

	// @1605 'k' (8 pixels wide)
	0x00, //         
	0x38, //   ###   
	0x18, //    ##   
	0x18, //    ##   
	0x18, //    ##   
	0x37, //   ## ###
	0x32, //   ##  # 
	0x34, //   ## #  
	0x28, //   # #   
	0x7C, //  #####  
	0x6C, //  ## ##  
	0x6D, //  ## ## #
	0xC6, // ##   ## 
	0x00, //         
	0x00, //         

	// @1620 'l' (4 pixels wide)
	0x00, //     
	0x70, //  ###
	0x30, //   ##
	0x30, //   ##
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0x40, //  #  
	0xC0, // ##  
	0xC0, // ##  
	0xC0, // ##  
	0xE0, // ### 
	0xC0, // ##  
	0x00, //     
	0x00, //     

	// @1635 'm' (12 pixels wide)
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x77, 0x30, //  ### ###  ##
	0x3B, 0xF0, //   ### ######
	0x33, 0x60, //   ##  ## ## 
	0x73, 0x60, //  ###  ## ## 
	0x66, 0x60, //  ##  ##  ## 
	0x66, 0x60, //  ##  ##  ## 
	0x66, 0x60, //  ##  ##  ## 
	0xCC, 0x70, // ##  ##   ###
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @1665 'n' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x77, //  ### ###
	0x3B, //   ### ##
	0x36, //   ## ## 
	0x76, //  ### ## 
	0x66, //  ##  ## 
	0x66, //  ##  ## 
	0x66, //  ##  ## 
	0xC7, // ##   ###
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
	0xE3, // ###   ##
	0xC7, // ##   ###
	0xC6, // ##   ## 
	0xCC, // ##  ##  
	0x78, //  ####   
	0x00, //         
	0x00, //         

	// @1695 'p' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x3B, 0x80, //   ### ###
	0x1D, 0x80, //    ### ##
	0x19, 0x80, //    ##  ##
	0x31, 0x80, //   ##   ##
	0x33, 0x00, //   ##  ## 
	0x33, 0x00, //   ##  ## 
	0x36, 0x00, //   ## ##  
	0x6C, 0x00, //  ## ##   
	0x60, 0x00, //  ##      
	0xF0, 0x00, // ####     

	// @1725 'q' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x1F, //    #####
	0x32, //   ##  # 
	0x66, //  ##  ## 
	0x66, //  ##  ## 
	0xC6, // ##   ## 
	0xCC, // ##  ##  
	0xCC, // ##  ##  
	0xFC, // ######  
	0x18, //    ##   
	0x3C, //   ####  

	// @1740 'r' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x76, //  ### ##
	0x3E, //   #####
	0x70, //  ###   
	0x60, //  ##    
	0x60, //  ##    
	0x60, //  ##    
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
	0x34, //   ## #
	0x4C, //  #  ##
	0x44, //  #   #
	0x60, //  ##   
	0x30, //   ##  
	0x10, //    #  
	0x90, // #  #  
	0x60, //  ##   
	0x00, //       
	0x00, //       

	// @1770 't' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x10, //    #
	0x20, //   # 
	0x70, //  ###
	0x60, //  ## 
	0x40, //  #  
	0xC0, // ##  
	0xC0, // ##  
	0xC0, // ##  
	0xC0, // ##  
	0xE0, // ### 
	0x00, //     
	0x00, //     

	// @1785 'u' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0xE6, // ###  ##
	0x66, //  ##  ##
	0x64, //  ##  # 
	0x6C, //  ## ## 
	0x4C, //  #  ## 
	0xCC, // ##  ## 
	0xFE, // #######
	0xCC, // ##  ## 
	0x00, //        
	0x00, //        

	// @1800 'v' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x80, // #     
	0x64, //  ##  #
	0x64, //  ##  #
	0x68, //  ## # 
	0x68, //  ## # 
	0x70, //  ###  
	0x60, //  ##   
	0x40, //  #    
	0x00, //       
	0x00, //       
	0x00, //       

	// @1815 'w' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x80, 0x00, // #        
	0x64, 0x80, //  ##  #  #
	0x6C, 0x80, //  ## ##  #
	0x6D, 0x00, //  ## ## # 
	0x7D, 0x00, //  ##### # 
	0x6E, 0x00, //  ## ###  
	0x44, 0x00, //  #   #   
	0x40, 0x00, //  #       
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1845 'x' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x73, //  ###  ##
	0x3C, //   ####  
	0x38, //   ###   
	0x18, //    ##   
	0x18, //    ##   
	0x38, //   ###   
	0xBA, // # ### # 
	0xCC, // ##  ##  
	0x00, //         
	0x00, //         

	// @1860 'y' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x39, //   ###  #
	0x19, //    ##  #
	0x19, //    ##  #
	0x1A, //    ## # 
	0x0A, //     # # 
	0x0C, //     ##  
	0x0C, //     ##  
	0x08, //     #   
	0x10, //    #    
	0xE0, // ###     

	// @1875 'z' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x7C, //  #####
	0x98, // #  ## 
	0x10, //    #  
	0x20, //   #   
	0x40, //  #    
	0xE0, // ###   
	0x68, //  ## # 
	0x38, //   ### 
	0x00, //       
	0x00, //       

	// @1890 '{' (7 pixels wide)
	0x0E, //     ###
	0x18, //    ##  
	0x10, //    #   
	0x10, //    #   
	0x30, //   ##   
	0x30, //   ##   
	0x20, //   #    
	0x60, //  ##    
	0x60, //  ##    
	0x80, // #      
	0x40, //  #     
	0xC0, // ##     
	0xC0, // ##     
	0xC0, // ##     
	0xC0, // ##     

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
	0x0C, //     ##
	0x04, //      #
	0x04, //      #
	0x04, //      #
	0x04, //      #
	0x0C, //     ##
	0x0C, //     ##
	0x08, //     # 
	0x08, //     # 
	0x18, //    ## 
	0x18, //    ## 
	0x10, //    #  
	0x30, //   ##  
	0x30, //   ##  
	0xE0, // ###   

	// @1935 '~' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0xF2, // ####  #
	0x0E, //     ###
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
};

// Character descriptors for FreeSerif 12pt
// { [Char width in bits], [Offset into freeSerif_12ptCharBitmaps in bytes] }
static const CharDesc_t s_FreeSerifBoldIta12ptCharDesc[] = {
	{6, s_FreeSerifBoldIta12ptBitmaps + 0}, 		// !
	{7, s_FreeSerifBoldIta12ptBitmaps + 15}, 		// "
	{8, s_FreeSerifBoldIta12ptBitmaps + 30}, 		// #
	{9, s_FreeSerifBoldIta12ptBitmaps + 45}, 		// $
	{12,s_FreeSerifBoldIta12ptBitmaps + 75}, 		// %
	{11,s_FreeSerifBoldIta12ptBitmaps + 105}, 		// &
	{2, s_FreeSerifBoldIta12ptBitmaps + 135}, 		// '
	{4, s_FreeSerifBoldIta12ptBitmaps + 150}, 		// (
	{5, s_FreeSerifBoldIta12ptBitmaps + 165}, 		// )
	{7, s_FreeSerifBoldIta12ptBitmaps + 180}, 		// *
	{7, s_FreeSerifBoldIta12ptBitmaps + 195}, 		// +
	{3, s_FreeSerifBoldIta12ptBitmaps + 210}, 		// ,
	{4, s_FreeSerifBoldIta12ptBitmaps + 225}, 		// -
	{3, s_FreeSerifBoldIta12ptBitmaps + 240}, 		// .
	{6, s_FreeSerifBoldIta12ptBitmaps + 255}, 		// /
	{7, s_FreeSerifBoldIta12ptBitmaps + 270}, 		// 0
	{7, s_FreeSerifBoldIta12ptBitmaps + 285}, 		// 1
	{8, s_FreeSerifBoldIta12ptBitmaps + 300}, 		// 2
	{9, s_FreeSerifBoldIta12ptBitmaps + 315}, 		// 3
	{8, s_FreeSerifBoldIta12ptBitmaps + 345}, 		// 4
	{7, s_FreeSerifBoldIta12ptBitmaps + 360}, 		// 5
	{8, s_FreeSerifBoldIta12ptBitmaps + 375}, 		// 6
	{7, s_FreeSerifBoldIta12ptBitmaps + 390}, 		// 7
	{7, s_FreeSerifBoldIta12ptBitmaps + 405}, 		// 8
	{8, s_FreeSerifBoldIta12ptBitmaps + 420}, 		// 9
	{5, s_FreeSerifBoldIta12ptBitmaps + 435}, 		// :
	{5, s_FreeSerifBoldIta12ptBitmaps + 450}, 		// ;
	{9, s_FreeSerifBoldIta12ptBitmaps + 465}, 		// <
	{8, s_FreeSerifBoldIta12ptBitmaps + 495}, 		// =
	{8, s_FreeSerifBoldIta12ptBitmaps + 510}, 		// >
	{7, s_FreeSerifBoldIta12ptBitmaps + 525}, 		// ?
	{12,s_FreeSerifBoldIta12ptBitmaps + 540}, 		// @
	{11,s_FreeSerifBoldIta12ptBitmaps + 570}, 		// A
	{11,s_FreeSerifBoldIta12ptBitmaps + 600}, 		// B
	{11,s_FreeSerifBoldIta12ptBitmaps + 630}, 		// C
	{12,s_FreeSerifBoldIta12ptBitmaps + 660}, 		// D
	{12,s_FreeSerifBoldIta12ptBitmaps + 690}, 		// E
	{12,s_FreeSerifBoldIta12ptBitmaps + 720}, 		// F
	{11,s_FreeSerifBoldIta12ptBitmaps + 750}, 		// G
	{13,s_FreeSerifBoldIta12ptBitmaps + 780}, 		// H
	{7, s_FreeSerifBoldIta12ptBitmaps + 810}, 		// I
	{9, s_FreeSerifBoldIta12ptBitmaps + 825}, 		// J
	{12,s_FreeSerifBoldIta12ptBitmaps + 855}, 		// K
	{10,s_FreeSerifBoldIta12ptBitmaps + 885}, 		// L
	{15,s_FreeSerifBoldIta12ptBitmaps + 915}, 		// M
	{12,s_FreeSerifBoldIta12ptBitmaps + 945}, 		// N
	{11,s_FreeSerifBoldIta12ptBitmaps + 975}, 		// O
	{11,s_FreeSerifBoldIta12ptBitmaps + 1005}, 		// P
	{11,s_FreeSerifBoldIta12ptBitmaps + 1035}, 		// Q
	{11,s_FreeSerifBoldIta12ptBitmaps + 1065}, 		// R
	{9, s_FreeSerifBoldIta12ptBitmaps + 1095}, 		// S
	{11,s_FreeSerifBoldIta12ptBitmaps + 1125}, 		// T
	{11,s_FreeSerifBoldIta12ptBitmaps + 1155}, 		// U
	{11,s_FreeSerifBoldIta12ptBitmaps + 1185}, 		// V
	{14,s_FreeSerifBoldIta12ptBitmaps + 1215}, 		// W
	{11,s_FreeSerifBoldIta12ptBitmaps + 1245}, 		// X
	{9, s_FreeSerifBoldIta12ptBitmaps + 1275}, 		// Y
	{10,s_FreeSerifBoldIta12ptBitmaps + 1305}, 		// Z
	{6, s_FreeSerifBoldIta12ptBitmaps + 1335}, 		// [
	{4, s_FreeSerifBoldIta12ptBitmaps + 1350}, 		// '\'
	{6, s_FreeSerifBoldIta12ptBitmaps + 1365}, 		// ]
	{7, s_FreeSerifBoldIta12ptBitmaps + 1380}, 		// ^
	{8, s_FreeSerifBoldIta12ptBitmaps + 1395}, 		// _
	{3, s_FreeSerifBoldIta12ptBitmaps + 1410}, 		// `
	{8, s_FreeSerifBoldIta12ptBitmaps + 1425}, 		// a
	{8, s_FreeSerifBoldIta12ptBitmaps + 1440}, 		// b
	{6, s_FreeSerifBoldIta12ptBitmaps + 1455}, 		// c
	{9, s_FreeSerifBoldIta12ptBitmaps + 1470}, 		// d
	{6, s_FreeSerifBoldIta12ptBitmaps + 1500}, 		// e
	{10,s_FreeSerifBoldIta12ptBitmaps + 1515}, 		// f
	{8, s_FreeSerifBoldIta12ptBitmaps + 1545}, 		// g
	{8, s_FreeSerifBoldIta12ptBitmaps + 1560}, 		// h
	{5, s_FreeSerifBoldIta12ptBitmaps + 1575}, 		// i
	{8, s_FreeSerifBoldIta12ptBitmaps + 1590}, 		// j
	{8, s_FreeSerifBoldIta12ptBitmaps + 1605}, 		// k
	{4, s_FreeSerifBoldIta12ptBitmaps + 1620}, 		// l
	{12,s_FreeSerifBoldIta12ptBitmaps + 1635}, 		// m
	{8, s_FreeSerifBoldIta12ptBitmaps + 1665}, 		// n
	{8, s_FreeSerifBoldIta12ptBitmaps + 1680}, 		// o
	{9, s_FreeSerifBoldIta12ptBitmaps + 1695}, 		// p
	{8, s_FreeSerifBoldIta12ptBitmaps + 1725}, 		// q
	{7, s_FreeSerifBoldIta12ptBitmaps + 1740}, 		// r
	{6, s_FreeSerifBoldIta12ptBitmaps + 1755}, 		// s
	{4, s_FreeSerifBoldIta12ptBitmaps + 1770}, 		// t
	{7, s_FreeSerifBoldIta12ptBitmaps + 1785}, 		// u
	{6, s_FreeSerifBoldIta12ptBitmaps + 1800}, 		// v
	{9, s_FreeSerifBoldIta12ptBitmaps + 1815}, 		// w
	{8, s_FreeSerifBoldIta12ptBitmaps + 1845}, 		// x
	{8, s_FreeSerifBoldIta12ptBitmaps + 1860}, 		// y
	{6, s_FreeSerifBoldIta12ptBitmaps + 1875}, 		// z
	{7, s_FreeSerifBoldIta12ptBitmaps + 1890}, 		// {
	{1, s_FreeSerifBoldIta12ptBitmaps + 1905}, 		// |
	{6, s_FreeSerifBoldIta12ptBitmaps + 1920}, 		// }
	{7, s_FreeSerifBoldIta12ptBitmaps + 1935}, 		// ~
};

// Font information for FreeSerif 12pt
const FontDesc_t iFontFreeSerifBoldIta12pt = {
	FONT_TYPE_VAR_WIDTH,
	15,
	15,
	{ .pCharDesc = s_FreeSerifBoldIta12ptCharDesc }
};