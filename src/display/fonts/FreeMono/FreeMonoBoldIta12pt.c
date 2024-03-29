// 
//  Font data for FreeMono 12pt
// 	https://savannah.gnu.org/projects/freefont/
// 
// 	Font bitmap generated by
// 	http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator

#include "display/ifont.h"

// Character bitmaps for FreeMono 12pt
static const uint8_t s_FreeMonoBoldIta12ptBitmaps[] = {
	// @0 '!' (3 pixels wide)
	0x00, //    
	0x60, //  ##
	0x60, //  ##
	0x60, //  ##
	0x60, //  ##
	0xC0, // ## 
	0xC0, // ## 
	0xC0, // ## 
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
	0x98, // #  ##
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

	// @28 '#' (8 pixels wide)
	0x00, //         
	0x1A, //    ## # 
	0x16, //    # ## 
	0x36, //   ## ## 
	0x7F, //  #######
	0x7F, //  #######
	0x2C, //   # ##  
	0xFE, // ####### 
	0xFE, // ####### 
	0x58, //  # ##   
	0xD8, // ## ##   
	0xD8, // ## ##   
	0x00, //         
	0x00, //         

	// @42 '$' (8 pixels wide)
	0x0C, //     ##  
	0x0C, //     ##  
	0x3F, //   ######
	0x62, //  ##   # 
	0x60, //  ##     
	0x7C, //  #####  
	0x1E, //    #### 
	0x86, // #    ## 
	0xC6, // ##   ## 
	0xFC, // ######  
	0x30, //   ##    
	0x30, //   ##    
	0x20, //   #     
	0x00, //         

	// @56 '%' (7 pixels wide)
	0x00, //        
	0x38, //   ###  
	0x64, //  ##  # 
	0x44, //  #   # 
	0x48, //  #  #  
	0x3E, //   #####
	0x70, //  ###   
	0x9E, // #  ####
	0x22, //   #   #
	0x22, //   #   #
	0x1C, //    ### 
	0x00, //        
	0x00, //        
	0x00, //        

	// @70 '&' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x0E, //     ### 
	0x1E, //    #### 
	0x30, //   ##    
	0x30, //   ##    
	0x78, //  ####   
	0xFF, // ########
	0xCE, // ##  ### 
	0xFE, // ####### 
	0x7E, //  ###### 
	0x00, //         
	0x00, //         
	0x00, //         

	// @84 ''' (2 pixels wide)
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

	// @98 '(' (4 pixels wide)
	0x00, //     
	0x10, //    #
	0x30, //   ##
	0x60, //  ## 
	0x60, //  ## 
	0x40, //  #  
	0xC0, // ##  
	0xC0, // ##  
	0xC0, // ##  
	0xC0, // ##  
	0xC0, // ##  
	0xE0, // ### 
	0x60, //  ## 
	0x00, //     

	// @112 ')' (4 pixels wide)
	0x00, //     
	0x60, //  ## 
	0x70, //  ###
	0x30, //   ##
	0x30, //   ##
	0x30, //   ##
	0x30, //   ##
	0x30, //   ##
	0x30, //   ##
	0x60, //  ## 
	0x60, //  ## 
	0x40, //  #  
	0xC0, // ##  
	0x00, //     

	// @126 '*' (7 pixels wide)
	0x00, //        
	0x18, //    ##  
	0x10, //    #   
	0xDE, // ## ####
	0xFC, // ###### 
	0x78, //  ####  
	0xD8, // ## ##  
	0x88, // #   #  
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        

	// @140 '+' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x08, //     #   
	0x08, //     #   
	0x18, //    ##   
	0xFF, // ########
	0xFF, // ########
	0x10, //    #    
	0x10, //    #    
	0x30, //   ##    
	0x30, //   ##    
	0x00, //         
	0x00, //         
	0x00, //         

	// @154 ',' (4 pixels wide)
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x00, //     
	0x30, //   ##
	0x60, //  ## 
	0x40, //  #  
	0xC0, // ##  
	0x80, // #   

	// @168 '-' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0xFF, // ########
	0xFF, // ########
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

	// @196 '/' (10 pixels wide)
	0x00, 0xC0, //         ##
	0x01, 0x80, //        ## 
	0x01, 0x00, //        #  
	0x03, 0x00, //       ##  
	0x06, 0x00, //      ##   
	0x04, 0x00, //      #    
	0x0C, 0x00, //     ##    
	0x18, 0x00, //    ##     
	0x30, 0x00, //   ##      
	0x30, 0x00, //   ##      
	0x60, 0x00, //  ##       
	0xC0, 0x00, // ##        
	0x80, 0x00, // #         
	0x00, 0x00, //           

	// @224 '0' (8 pixels wide)
	0x1E, //    #### 
	0x3F, //   ######
	0x23, //   #   ##
	0x63, //  ##   ##
	0x43, //  #    ##
	0xC2, // ##    # 
	0xC2, // ##    # 
	0xC6, // ##   ## 
	0xC6, // ##   ## 
	0xFC, // ######  
	0x78, //  ####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @238 '1' (6 pixels wide)
	0x00, //       
	0x1C, //    ###
	0x78, //  #### 
	0x48, //  #  # 
	0x18, //    ## 
	0x18, //    ## 
	0x10, //    #  
	0x10, //    #  
	0x30, //   ##  
	0xFC, // ######
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       

	// @252 '2' (9 pixels wide)
	0x07, 0x00, //      ### 
	0x0F, 0x80, //     #####
	0x19, 0x80, //    ##  ##
	0x11, 0x80, //    #   ##
	0x03, 0x80, //       ###
	0x07, 0x00, //      ### 
	0x0E, 0x00, //     ###  
	0x3C, 0x00, //   ####   
	0x70, 0x00, //  ###     
	0xFF, 0x00, // ######## 
	0xFE, 0x00, // #######  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @280 '3' (8 pixels wide)
	0x1E, //    #### 
	0x3F, //   ######
	0x03, //       ##
	0x03, //       ##
	0x1E, //    #### 
	0x1C, //    ###  
	0x06, //      ## 
	0x06, //      ## 
	0x0E, //     ### 
	0xFC, // ######  
	0xF8, // #####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @294 '4' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x06, //      ##
	0x0E, //     ###
	0x16, //    # ##
	0x24, //   #  # 
	0x44, //  #   # 
	0xFE, // #######
	0xFE, // #######
	0x3C, //   #### 
	0x3C, //   #### 
	0x00, //        
	0x00, //        
	0x00, //        

	// @308 '5' (7 pixels wide)
	0x00, //        
	0x7E, //  ######
	0x7E, //  ######
	0x60, //  ##    
	0x7C, //  ##### 
	0x7E, //  ######
	0xC6, // ##   ##
	0x06, //      ##
	0x0E, //     ###
	0xFC, // ###### 
	0xF8, // #####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @322 '6' (9 pixels wide)
	0x07, 0x80, //      ####
	0x1F, 0x80, //    ######
	0x38, 0x00, //   ###    
	0x70, 0x00, //  ###     
	0x7E, 0x00, //  ######  
	0xFF, 0x00, // ######## 
	0xE3, 0x00, // ###   ## 
	0xC3, 0x00, // ##    ## 
	0xC7, 0x00, // ##   ### 
	0xFE, 0x00, // #######  
	0x7C, 0x00, //  #####   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @350 '7' (7 pixels wide)
	0x00, //        
	0xFE, // #######
	0xFE, // #######
	0x06, //      ##
	0x04, //      # 
	0x0C, //     ## 
	0x08, //     #  
	0x18, //    ##  
	0x30, //   ##   
	0x30, //   ##   
	0x60, //  ##    
	0x00, //        
	0x00, //        
	0x00, //        

	// @364 '8' (8 pixels wide)
	0x1E, //    #### 
	0x3F, //   ######
	0x63, //  ##   ##
	0x63, //  ##   ##
	0x66, //  ##  ## 
	0x3C, //   ####  
	0x7C, //  #####  
	0xC6, // ##   ## 
	0xC6, // ##   ## 
	0xFC, // ######  
	0x78, //  ####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @378 '9' (8 pixels wide)
	0x1E, //    #### 
	0x3F, //   ######
	0x73, //  ###  ##
	0x63, //  ##   ##
	0x67, //  ##  ###
	0x7F, //  #######
	0x3E, //   ##### 
	0x06, //      ## 
	0x1C, //    ###  
	0xF8, // #####   
	0xF0, // ####    
	0x00, //         
	0x00, //         
	0x00, //         

	// @392 ':' (3 pixels wide)
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

	// @406 ';' (5 pixels wide)
	0x00, //      
	0x00, //      
	0x00, //      
	0x00, //      
	0x18, //    ##
	0x18, //    ##
	0x00, //      
	0x00, //      
	0x00, //      
	0x30, //   ## 
	0x60, //  ##  
	0x40, //  #   
	0xC0, // ##   
	0x80, // #    

	// @420 '<' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x03, //       ##
	0x0E, //     ### 
	0x38, //   ###   
	0xE0, // ###     
	0x78, //  ####   
	0x1C, //    ###  
	0x06, //      ## 
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @434 '=' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x7F, 0x80, //  ########
	0x7F, 0x80, //  ########
	0x00, 0x00, //          
	0xFF, 0x80, // #########
	0xFF, 0x00, // ######## 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @462 '>' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x20, 0x00, //   #      
	0x38, 0x00, //   ###    
	0x0E, 0x00, //     ###  
	0x07, 0x80, //      ####
	0x0E, 0x00, //     ###  
	0x78, 0x00, //  ####    
	0xC0, 0x00, // ##       
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @490 '?' (6 pixels wide)
	0x00, //       
	0x78, //  #### 
	0xFC, // ######
	0x8C, // #   ##
	0x0C, //     ##
	0x38, //   ### 
	0xF0, // ####  
	0xC0, // ##    
	0x00, //       
	0xC0, // ##    
	0xC0, // ##    
	0x00, //       
	0x00, //       
	0x00, //       

	// @504 '@' (8 pixels wide)
	0x1E, //    #### 
	0x3F, //   ######
	0x33, //   ##  ##
	0x63, //  ##   ##
	0x4E, //  #  ### 
	0xDE, // ## #### 
	0xB6, // # ## ## 
	0xB4, // # ## #  
	0xBC, // # ####  
	0xDE, // ## #### 
	0xC0, // ##      
	0xF8, // #####   
	0x70, //  ###    
	0x00, //         

	// @518 'A' (10 pixels wide)
	0x00, 0x00, //           
	0x1F, 0x00, //    #####  
	0x1F, 0x00, //    #####  
	0x07, 0x00, //      ###  
	0x0F, 0x00, //     ####  
	0x19, 0x80, //    ##  ## 
	0x19, 0x80, //    ##  ## 
	0x3F, 0x80, //   ####### 
	0x3F, 0x80, //   ####### 
	0xF3, 0xC0, // ####  ####
	0xF3, 0xC0, // ####  ####
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @546 'B' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0x80, //   ####### 
	0x3F, 0xC0, //   ########
	0x18, 0xC0, //    ##   ##
	0x10, 0xC0, //    #    ##
	0x1F, 0x80, //    ###### 
	0x3F, 0x80, //   ####### 
	0x30, 0xC0, //   ##    ##
	0x30, 0xC0, //   ##    ##
	0x7F, 0x80, //  ######## 
	0xFF, 0x00, // ########  
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @574 'C' (10 pixels wide)
	0x00, 0x00, //           
	0x1F, 0xC0, //    #######
	0x3F, 0xC0, //   ########
	0x71, 0x80, //  ###   ## 
	0x61, 0x80, //  ##    ## 
	0xC0, 0x00, // ##        
	0xC0, 0x00, // ##        
	0xC0, 0x00, // ##        
	0xE3, 0x00, // ###   ##  
	0x7F, 0x00, //  #######  
	0x3C, 0x00, //   ####    
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @602 'D' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0x00, //   ######  
	0x3F, 0x80, //   ####### 
	0x11, 0xC0, //    #   ###
	0x10, 0xC0, //    #    ##
	0x30, 0xC0, //   ##    ##
	0x30, 0xC0, //   ##    ##
	0x21, 0xC0, //   #    ###
	0x21, 0x80, //   #    ## 
	0x7F, 0x00, //  #######  
	0xFE, 0x00, // #######   
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @630 'E' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0xC0, //   ########
	0x3F, 0xC0, //   ########
	0x10, 0xC0, //    #    ##
	0x16, 0x00, //    # ##   
	0x3E, 0x00, //   #####   
	0x3C, 0x00, //   ####    
	0x34, 0x00, //   ## #    
	0x20, 0x80, //   #     # 
	0xFF, 0x80, // ######### 
	0xFF, 0x80, // ######### 
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @658 'F' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0xC0, //   ########
	0x3F, 0xC0, //   ########
	0x10, 0xC0, //    #    ##
	0x12, 0x00, //    #  #   
	0x3E, 0x00, //   #####   
	0x3E, 0x00, //   #####   
	0x24, 0x00, //   #  #    
	0x20, 0x00, //   #       
	0xFC, 0x00, // ######    
	0xF8, 0x00, // #####     
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @686 'G' (10 pixels wide)
	0x00, 0x00, //           
	0x0F, 0x40, //     #### #
	0x3F, 0xC0, //   ########
	0x70, 0x80, //  ###    # 
	0x60, 0x00, //  ##       
	0xC0, 0x00, // ##        
	0xC7, 0x80, // ##   #### 
	0xCF, 0x80, // ##  ##### 
	0xC1, 0x00, // ##     #  
	0xFF, 0x00, // ########  
	0x3E, 0x00, //   #####   
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @714 'H' (10 pixels wide)
	0x00, 0x00, //           
	0x3D, 0xC0, //   #### ###
	0x3D, 0xC0, //   #### ###
	0x10, 0x80, //    #    # 
	0x10, 0x80, //    #    # 
	0x3F, 0x80, //   ####### 
	0x3F, 0x80, //   ####### 
	0x21, 0x00, //   #    #  
	0x21, 0x00, //   #    #  
	0xF7, 0x80, // #### #### 
	0xF7, 0x80, // #### #### 
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @742 'I' (9 pixels wide)
	0x00, 0x00, //          
	0x3F, 0x80, //   #######
	0x3F, 0x00, //   ###### 
	0x0C, 0x00, //     ##   
	0x08, 0x00, //     #    
	0x08, 0x00, //     #    
	0x18, 0x00, //    ##    
	0x18, 0x00, //    ##    
	0x18, 0x00, //    ##    
	0xFE, 0x00, // #######  
	0xFE, 0x00, // #######  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @770 'J' (11 pixels wide)
	0x00, 0x00, //            
	0x0F, 0xE0, //     #######
	0x0F, 0xC0, //     ###### 
	0x01, 0x00, //        #   
	0x01, 0x00, //        #   
	0x03, 0x00, //       ##   
	0x03, 0x00, //       ##   
	0x43, 0x00, //  #    ##   
	0xC6, 0x00, // ##   ##    
	0xFE, 0x00, // #######    
	0x78, 0x00, //  ####      
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @798 'K' (11 pixels wide)
	0x00, 0x00, //            
	0x3C, 0xE0, //   ####  ###
	0x3D, 0xE0, //   #### ####
	0x11, 0x80, //    #   ##  
	0x17, 0x00, //    # ###   
	0x3E, 0x00, //   #####    
	0x3F, 0x00, //   ######   
	0x23, 0x00, //   #   ##   
	0x23, 0x00, //   #   ##   
	0xF9, 0xC0, // #####  ### 
	0xF1, 0xC0, // ####   ### 
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @826 'L' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0x00, //   ######  
	0x3E, 0x00, //   #####   
	0x08, 0x00, //     #     
	0x18, 0x00, //    ##     
	0x18, 0x00, //    ##     
	0x18, 0x00, //    ##     
	0x10, 0xC0, //    #    ##
	0x10, 0x80, //    #    # 
	0x7F, 0x80, //  ######## 
	0xFF, 0x80, // ######### 
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @854 'M' (12 pixels wide)
	0x00, 0x00, //             
	0x38, 0x70, //   ###    ###
	0x38, 0xE0, //   ###   ### 
	0x39, 0xC0, //   ###  ###  
	0x39, 0xC0, //   ###  ###  
	0x2F, 0xC0, //   # ######  
	0x2E, 0xC0, //   # ### ##  
	0x6E, 0xC0, //  ## ### ##  
	0x60, 0x80, //  ##     #   
	0xF3, 0xC0, // ####  ####  
	0xF3, 0xC0, // ####  ####  
	0x00, 0x00, //             
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @882 'N' (11 pixels wide)
	0x00, 0x00, //            
	0x39, 0xE0, //   ###  ####
	0x39, 0xE0, //   ###  ####
	0x1C, 0xC0, //    ###  ## 
	0x1C, 0xC0, //    ###  ## 
	0x3E, 0xC0, //   ##### ## 
	0x36, 0x80, //   ## ## #  
	0x26, 0x80, //   #  ## #  
	0x23, 0x80, //   #   ###  
	0x7B, 0x80, //  #### ###  
	0xF1, 0x80, // ####   ##  
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @910 'O' (9 pixels wide)
	0x00, 0x00, //          
	0x1E, 0x00, //    ####  
	0x3F, 0x00, //   ###### 
	0x73, 0x80, //  ###  ###
	0x61, 0x80, //  ##    ##
	0xC1, 0x80, // ##     ##
	0xC1, 0x80, // ##     ##
	0xC3, 0x00, // ##    ## 
	0xE7, 0x00, // ###  ### 
	0x7E, 0x00, //  ######  
	0x3C, 0x00, //   ####   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @938 'P' (10 pixels wide)
	0x00, 0x00, //           
	0x3F, 0x80, //   ####### 
	0x3F, 0xC0, //   ########
	0x18, 0xC0, //    ##   ##
	0x10, 0xC0, //    #    ##
	0x11, 0xC0, //    #   ###
	0x3F, 0x80, //   ####### 
	0x3F, 0x00, //   ######  
	0x30, 0x00, //   ##      
	0x7C, 0x00, //  #####    
	0xFC, 0x00, // ######    
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @966 'Q' (9 pixels wide)
	0x00, 0x00, //          
	0x1E, 0x00, //    ####  
	0x3F, 0x00, //   ###### 
	0x73, 0x80, //  ###  ###
	0x61, 0x80, //  ##    ##
	0xC1, 0x80, // ##     ##
	0xC1, 0x80, // ##     ##
	0xC3, 0x00, // ##    ## 
	0xE7, 0x00, // ###  ### 
	0x7E, 0x00, //  ######  
	0x3C, 0x00, //   ####   
	0x7E, 0x00, //  ######  
	0xFE, 0x00, // #######  
	0x00, 0x00, //          

	// @994 'R' (11 pixels wide)
	0x00, 0x00, //            
	0x3F, 0x80, //   #######  
	0x3F, 0xC0, //   ######## 
	0x10, 0xC0, //    #    ## 
	0x11, 0xC0, //    #   ### 
	0x3F, 0x80, //   #######  
	0x3F, 0x00, //   ######   
	0x23, 0x00, //   #   ##   
	0x21, 0x80, //   #    ##  
	0xF9, 0xE0, // #####  ####
	0xF0, 0xE0, // ####    ###
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1022 'S' (9 pixels wide)
	0x00, 0x00, //          
	0x1D, 0x80, //    ### ##
	0x3F, 0x80, //   #######
	0x63, 0x00, //  ##   ## 
	0x60, 0x00, //  ##      
	0x70, 0x00, //  ###     
	0x3E, 0x00, //   #####  
	0x46, 0x00, //  #   ##  
	0xC6, 0x00, // ##   ##  
	0xFC, 0x00, // ######   
	0xB8, 0x00, // # ###    
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1050 'T' (9 pixels wide)
	0x00, 0x00, //          
	0x7F, 0x80, //  ########
	0x7F, 0x80, //  ########
	0xC9, 0x80, // ##  #  ##
	0xC9, 0x00, // ##  #  # 
	0x18, 0x00, //    ##    
	0x18, 0x00, //    ##    
	0x18, 0x00, //    ##    
	0x10, 0x00, //    #     
	0x7C, 0x00, //  #####   
	0xFC, 0x00, // ######   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1078 'U' (10 pixels wide)
	0x00, 0x00, //           
	0xF3, 0xC0, // ####  ####
	0xF3, 0xC0, // ####  ####
	0x61, 0x80, //  ##    ## 
	0x41, 0x00, //  #     #  
	0x43, 0x00, //  #    ##  
	0xC3, 0x00, // ##    ##  
	0xC3, 0x00, // ##    ##  
	0xE6, 0x00, // ###  ##   
	0xFE, 0x00, // #######   
	0x78, 0x00, //  ####     
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1106 'V' (10 pixels wide)
	0x00, 0x00, //           
	0xF3, 0xC0, // ####  ####
	0xF3, 0xC0, // ####  ####
	0x41, 0x00, //  #     #  
	0x63, 0x00, //  ##   ##  
	0x66, 0x00, //  ##  ##   
	0x66, 0x00, //  ##  ##   
	0x2C, 0x00, //   # ##    
	0x28, 0x00, //   # #     
	0x38, 0x00, //   ###     
	0x30, 0x00, //   ##      
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1134 'W' (10 pixels wide)
	0x00, 0x00, //           
	0xF3, 0xC0, // ####  ####
	0xF3, 0xC0, // ####  ####
	0x41, 0x80, //  #     ## 
	0xCD, 0x00, // ##  ## #  
	0xDD, 0x00, // ## ### #  
	0xFF, 0x00, // ########  
	0xFF, 0x00, // ########  
	0xEE, 0x00, // ### ###   
	0xCE, 0x00, // ##  ###   
	0xC6, 0x00, // ##   ##   
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1162 'X' (11 pixels wide)
	0x00, 0x00, //            
	0x39, 0xE0, //   ###  ####
	0x39, 0xE0, //   ###  ####
	0x19, 0x80, //    ##  ##  
	0x0F, 0x00, //     ####   
	0x0E, 0x00, //     ###    
	0x0E, 0x00, //     ###    
	0x1F, 0x00, //    #####   
	0x33, 0x00, //   ##  ##   
	0xF3, 0x80, // ####  ###  
	0xF7, 0x80, // #### ####  
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1190 'Y' (9 pixels wide)
	0x00, 0x00, //          
	0x73, 0x80, //  ###  ###
	0xF3, 0x80, // ####  ###
	0x63, 0x00, //  ##   ## 
	0x36, 0x00, //   ## ##  
	0x3C, 0x00, //   ####   
	0x18, 0x00, //    ##    
	0x10, 0x00, //    #     
	0x10, 0x00, //    #     
	0xFC, 0x00, // ######   
	0xFC, 0x00, // ######   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1218 'Z' (9 pixels wide)
	0x00, 0x00, //          
	0x3F, 0x80, //   #######
	0x3F, 0x00, //   ###### 
	0x23, 0x00, //   #   ## 
	0x06, 0x00, //      ##  
	0x0C, 0x00, //     ##   
	0x18, 0x00, //    ##    
	0x30, 0x00, //   ##     
	0x63, 0x00, //  ##   ## 
	0xFF, 0x00, // ######## 
	0xFE, 0x00, // #######  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1246 '[' (6 pixels wide)
	0x00, //       
	0x3C, //   ####
	0x38, //   ### 
	0x20, //   #   
	0x60, //  ##   
	0x60, //  ##   
	0x60, //  ##   
	0x40, //  #    
	0x40, //  #    
	0xC0, // ##    
	0xC0, // ##    
	0xE0, // ###   
	0xE0, // ###   
	0x00, //       

	// @1260 '\' (5 pixels wide)
	0xC0, // ##   
	0x40, //  #   
	0x60, //  ##  
	0x60, //  ##  
	0x60, //  ##  
	0x20, //   #  
	0x30, //   ## 
	0x30, //   ## 
	0x30, //   ## 
	0x10, //    # 
	0x18, //    ##
	0x18, //    ##
	0x10, //    # 
	0x00, //      

	// @1274 ']' (6 pixels wide)
	0x00, //       
	0x3C, //   ####
	0x38, //   ### 
	0x08, //     # 
	0x18, //    ## 
	0x18, //    ## 
	0x10, //    #  
	0x10, //    #  
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0xE0, // ###   
	0xE0, // ###   
	0x00, //       

	// @1288 '^' (6 pixels wide)
	0x00, //       
	0x10, //    #  
	0x38, //   ### 
	0x6C, //  ## ##
	0xCC, // ##  ##
	0x84, // #    #
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       

	// @1302 '_' (10 pixels wide)
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
	0x7F, 0xC0, //  #########
	0xFF, 0xC0, // ##########

	// @1330 '`' (2 pixels wide)
	0x80, // # 
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

	// @1344 'a' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x1E, 0x00, //    ####  
	0x3F, 0x00, //   ###### 
	0x3F, 0x00, //   ###### 
	0x7F, 0x00, //  ####### 
	0xC2, 0x00, // ##    #  
	0xFF, 0x80, // #########
	0x7F, 0x80, //  ########
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1372 'b' (10 pixels wide)
	0x00, 0x00, //           
	0x38, 0x00, //   ###     
	0x38, 0x00, //   ###     
	0x10, 0x00, //    #      
	0x17, 0x80, //    # #### 
	0x3F, 0xC0, //   ########
	0x30, 0xC0, //   ##    ##
	0x20, 0xC0, //   #     ##
	0x21, 0xC0, //   #    ###
	0xFF, 0x80, // ######### 
	0xFF, 0x00, // ########  
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1400 'c' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x1E, 0x80, //    #### #
	0x7F, 0x80, //  ########
	0x61, 0x00, //  ##    # 
	0xC0, 0x00, // ##       
	0xC0, 0x00, // ##       
	0xFE, 0x00, // #######  
	0x7C, 0x00, //  #####   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1428 'd' (9 pixels wide)
	0x00, 0x00, //          
	0x01, 0x80, //        ##
	0x03, 0x80, //       ###
	0x01, 0x80, //        ##
	0x1F, 0x80, //    ######
	0x7F, 0x00, //  ####### 
	0xE1, 0x00, // ###    # 
	0xC1, 0x00, // ##     # 
	0xC3, 0x00, // ##    ## 
	0xFF, 0x80, // #########
	0x7B, 0x80, //  #### ###
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1456 'e' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x1E, //    #### 
	0x7F, //  #######
	0x61, //  ##    #
	0xFF, // ########
	0xFF, // ########
	0xFE, // ####### 
	0x7C, //  #####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @1470 'f' (10 pixels wide)
	0x00, 0x00, //           
	0x07, 0xC0, //      #####
	0x0F, 0xC0, //     ######
	0x08, 0x00, //     #     
	0x3F, 0x00, //   ######  
	0x3F, 0x00, //   ######  
	0x18, 0x00, //    ##     
	0x10, 0x00, //    #      
	0x10, 0x00, //    #      
	0x7E, 0x00, //  ######   
	0xFE, 0x00, // #######   
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1498 'g' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x1D, 0xC0, //    ### ###
	0x7F, 0xC0, //  #########
	0xE3, 0x00, // ###   ##  
	0xC3, 0x00, // ##    ##  
	0xC3, 0x00, // ##    ##  
	0xFE, 0x00, // #######   
	0x7A, 0x00, //  #### #   
	0x06, 0x00, //      ##   
	0x7C, 0x00, //  #####    
	0x78, 0x00, //  ####     

	// @1526 'h' (10 pixels wide)
	0x00, 0x00, //           
	0x70, 0x00, //  ###      
	0x70, 0x00, //  ###      
	0x30, 0x00, //   ##      
	0x2F, 0x00, //   # ####  
	0x3F, 0x80, //   ####### 
	0x61, 0x80, //  ##    ## 
	0x61, 0x00, //  ##    #  
	0x61, 0x00, //  ##    #  
	0xE3, 0xC0, // ###   ####
	0xE3, 0xC0, // ###   ####
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1554 'i' (8 pixels wide)
	0x00, //         
	0x04, //      #  
	0x0C, //     ##  
	0x00, //         
	0x3C, //   ####  
	0x3C, //   ####  
	0x0C, //     ##  
	0x08, //     #   
	0x18, //    ##   
	0xFF, // ########
	0xFF, // ########
	0x00, //         
	0x00, //         
	0x00, //         

	// @1568 'j' (8 pixels wide)
	0x00, //         
	0x03, //       ##
	0x02, //       # 
	0x00, //         
	0x3F, //   ######
	0x3F, //   ######
	0x03, //       ##
	0x02, //       # 
	0x02, //       # 
	0x06, //      ## 
	0x06, //      ## 
	0x04, //      #  
	0xFC, // ######  
	0xF8, // #####   

	// @1582 'k' (9 pixels wide)
	0x00, 0x00, //          
	0x30, 0x00, //   ##     
	0x70, 0x00, //  ###     
	0x30, 0x00, //   ##     
	0x37, 0x80, //   ## ####
	0x37, 0x00, //   ## ### 
	0x3C, 0x00, //   ####   
	0x38, 0x00, //   ###    
	0x6C, 0x00, //  ## ##   
	0xEF, 0x00, // ### #### 
	0xCF, 0x00, // ##  #### 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1610 'l' (8 pixels wide)
	0x00, //         
	0x1E, //    #### 
	0x1C, //    ###  
	0x04, //      #  
	0x0C, //     ##  
	0x0C, //     ##  
	0x08, //     #   
	0x08, //     #   
	0x18, //    ##   
	0xFF, // ########
	0xFF, // ########
	0x00, //         
	0x00, //         
	0x00, //         

	// @1624 'm' (11 pixels wide)
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x7E, 0xC0, //  ###### ## 
	0x7F, 0xE0, //  ##########
	0x26, 0x60, //   #  ##  ##
	0x64, 0x40, //  ##  #   # 
	0x64, 0x40, //  ##  #   # 
	0xEE, 0xE0, // ### ### ###
	0xEE, 0xE0, // ### ### ###
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	// @1652 'n' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x7F, 0x00, //  #######  
	0x7F, 0x80, //  ######## 
	0x61, 0x80, //  ##    ## 
	0x61, 0x00, //  ##    #  
	0x61, 0x00, //  ##    #  
	0xE3, 0xC0, // ###   ####
	0xE3, 0xC0, // ###   ####
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1680 'o' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x1E, 0x00, //    ####  
	0x7F, 0x80, //  ########
	0xE1, 0x80, // ###    ##
	0xC1, 0x80, // ##     ##
	0xC3, 0x80, // ##    ###
	0xFF, 0x00, // ######## 
	0x3C, 0x00, //   ####   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1708 'p' (11 pixels wide)
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x3B, 0xC0, //   ### #### 
	0x3F, 0xE0, //   #########
	0x18, 0x60, //    ##    ##
	0x10, 0x60, //    #     ##
	0x18, 0xE0, //    ##   ###
	0x3F, 0xC0, //   ######## 
	0x37, 0x80, //   ## ####  
	0x30, 0x00, //   ##       
	0x78, 0x00, //  ####      
	0xF8, 0x00, // #####      

	// @1736 'q' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x3D, 0xC0, //   #### ###
	0x7F, 0xC0, //  #########
	0xE1, 0x00, // ###    #  
	0xC1, 0x00, // ##     #  
	0xC3, 0x00, // ##    ##  
	0xFF, 0x00, // ########  
	0x7A, 0x00, //  #### #   
	0x06, 0x00, //      ##   
	0x1F, 0x00, //    #####  
	0x1F, 0x00, //    #####  

	// @1764 'r' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x7B, 0x80, //  #### ###
	0x7F, 0x80, //  ########
	0x38, 0x00, //   ###    
	0x30, 0x00, //   ##     
	0x30, 0x00, //   ##     
	0xFE, 0x00, // #######  
	0xFC, 0x00, // ######   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1792 's' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x3E, //   #####
	0x7E, //  ######
	0x62, //  ##   #
	0x3E, //   #####
	0xC6, // ##   ##
	0xFE, // #######
	0xF8, // #####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @1806 't' (8 pixels wide)
	0x00, //         
	0x10, //    #    
	0x30, //   ##    
	0x30, //   ##    
	0xFE, // ####### 
	0xFE, // ####### 
	0x20, //   #     
	0x60, //  ##     
	0x60, //  ##     
	0x7F, //  #######
	0x3E, //   ##### 
	0x00, //         
	0x00, //         
	0x00, //         

	// @1820 'u' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0xF7, 0x80, // #### ####
	0xF7, 0x00, // #### ### 
	0x61, 0x00, //  ##    # 
	0x63, 0x00, //  ##   ## 
	0x63, 0x00, //  ##   ## 
	0x7F, 0x00, //  ####### 
	0x3B, 0x00, //   ### ## 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1848 'v' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0xF7, 0xC0, // #### #####
	0xF7, 0x80, // #### #### 
	0x63, 0x00, //  ##   ##  
	0x26, 0x00, //   #  ##   
	0x3C, 0x00, //   ####    
	0x38, 0x00, //   ###     
	0x38, 0x00, //   ###     
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1876 'w' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0xF1, 0xC0, // ####   ###
	0xF1, 0xC0, // ####   ###
	0x6D, 0x80, //  ## ## ## 
	0x7D, 0x00, //  ##### #  
	0x7F, 0x00, //  #######  
	0x76, 0x00, //  ### ##   
	0x66, 0x00, //  ##  ##   
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1904 'x' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x3D, 0xC0, //   #### ###
	0x39, 0xC0, //   ###  ###
	0x1F, 0x00, //    #####  
	0x0E, 0x00, //     ###   
	0x1F, 0x00, //    #####  
	0x33, 0x80, //   ##  ### 
	0xF7, 0x80, // #### #### 
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1932 'y' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x39, 0xC0, //   ###  ###
	0x39, 0xC0, //   ###  ###
	0x31, 0x80, //   ##   ## 
	0x13, 0x00, //    #  ##  
	0x1E, 0x00, //    ####   
	0x1C, 0x00, //    ###    
	0x18, 0x00, //    ##     
	0x18, 0x00, //    ##     
	0xF8, 0x00, // #####     
	0xF8, 0x00, // #####     

	// @1960 'z' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x3F, 0x80, //   #######
	0x3F, 0x80, //   #######
	0x26, 0x00, //   #  ##  
	0x1C, 0x00, //    ###   
	0x30, 0x00, //   ##     
	0x7F, 0x00, //  ####### 
	0xFF, 0x00, // ######## 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1988 '{' (5 pixels wide)
	0x00, //      
	0x18, //    ##
	0x38, //   ###
	0x30, //   ## 
	0x30, //   ## 
	0x20, //   #  
	0xE0, // ###  
	0xE0, // ###  
	0x60, //  ##  
	0x60, //  ##  
	0x60, //  ##  
	0x60, //  ##  
	0x60, //  ##  
	0x00, //      

	// @2002 '|' (4 pixels wide)
	0x00, //     
	0x10, //    #
	0x30, //   ##
	0x30, //   ##
	0x20, //   # 
	0x20, //   # 
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0x40, //  #  
	0xC0, // ##  
	0xC0, // ##  
	0xC0, // ##  
	0x00, //     

	// @2016 '}' (6 pixels wide)
	0x00, //       
	0x18, //    ## 
	0x18, //    ## 
	0x18, //    ## 
	0x18, //    ## 
	0x18, //    ## 
	0x1C, //    ###
	0x1C, //    ###
	0x10, //    #  
	0x30, //   ##  
	0x30, //   ##  
	0xE0, // ###   
	0xE0, // ###   
	0x00, //       

	// @2030 '~' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x70, //  ###    
	0xF8, // #####   
	0x1F, //    #####
	0x0E, //     ### 
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
};

// Character descriptors for FreeMono 12pt
// { [Char width in bits], [Offset into freeMono_12ptCharBitmaps in bytes] }
static const CharDesc_t s_FreeMonoBoldIta12ptCharDesc[] = {
	{3, s_FreeMonoBoldIta12ptBitmaps + 0}, 			// !
	{5, s_FreeMonoBoldIta12ptBitmaps + 14}, 		// "
	{8, s_FreeMonoBoldIta12ptBitmaps + 28}, 		// #
	{8, s_FreeMonoBoldIta12ptBitmaps + 42}, 		// $
	{7, s_FreeMonoBoldIta12ptBitmaps + 56}, 		// %
	{8, s_FreeMonoBoldIta12ptBitmaps + 70}, 		// &
	{2, s_FreeMonoBoldIta12ptBitmaps + 84}, 		// '
	{4, s_FreeMonoBoldIta12ptBitmaps + 98}, 		// (
	{4, s_FreeMonoBoldIta12ptBitmaps + 112}, 		// )
	{7, s_FreeMonoBoldIta12ptBitmaps + 126}, 		// *
	{8, s_FreeMonoBoldIta12ptBitmaps + 140}, 		// +
	{4, s_FreeMonoBoldIta12ptBitmaps + 154}, 		// ,
	{8, s_FreeMonoBoldIta12ptBitmaps + 168}, 		// -
	{2, s_FreeMonoBoldIta12ptBitmaps + 182}, 		// .
	{10,s_FreeMonoBoldIta12ptBitmaps + 196}, 		// /
	{8, s_FreeMonoBoldIta12ptBitmaps + 224}, 		// 0
	{6, s_FreeMonoBoldIta12ptBitmaps + 238}, 		// 1
	{9, s_FreeMonoBoldIta12ptBitmaps + 252}, 		// 2
	{8, s_FreeMonoBoldIta12ptBitmaps + 280}, 		// 3
	{7, s_FreeMonoBoldIta12ptBitmaps + 294}, 		// 4
	{7, s_FreeMonoBoldIta12ptBitmaps + 308}, 		// 5
	{9, s_FreeMonoBoldIta12ptBitmaps + 322}, 		// 6
	{7, s_FreeMonoBoldIta12ptBitmaps + 350}, 		// 7
	{8, s_FreeMonoBoldIta12ptBitmaps + 364}, 		// 8
	{8, s_FreeMonoBoldIta12ptBitmaps + 378}, 		// 9
	{3, s_FreeMonoBoldIta12ptBitmaps + 392}, 		// :
	{5, s_FreeMonoBoldIta12ptBitmaps + 406}, 		// ;
	{8, s_FreeMonoBoldIta12ptBitmaps + 420}, 		// <
	{9, s_FreeMonoBoldIta12ptBitmaps + 434}, 		// =
	{9, s_FreeMonoBoldIta12ptBitmaps + 462}, 		// >
	{6, s_FreeMonoBoldIta12ptBitmaps + 490}, 		// ?
	{8, s_FreeMonoBoldIta12ptBitmaps + 504}, 		// @
	{10,s_FreeMonoBoldIta12ptBitmaps + 518}, 		// A
	{10,s_FreeMonoBoldIta12ptBitmaps + 546}, 		// B
	{10,s_FreeMonoBoldIta12ptBitmaps + 574}, 		// C
	{10,s_FreeMonoBoldIta12ptBitmaps + 602}, 		// D
	{10,s_FreeMonoBoldIta12ptBitmaps + 630}, 		// E
	{10,s_FreeMonoBoldIta12ptBitmaps + 658}, 		// F
	{10,s_FreeMonoBoldIta12ptBitmaps + 686}, 		// G
	{10,s_FreeMonoBoldIta12ptBitmaps + 714}, 		// H
	{9, s_FreeMonoBoldIta12ptBitmaps + 742}, 		// I
	{11,s_FreeMonoBoldIta12ptBitmaps + 770}, 		// J
	{11,s_FreeMonoBoldIta12ptBitmaps + 798}, 		// K
	{10,s_FreeMonoBoldIta12ptBitmaps + 826}, 		// L
	{12,s_FreeMonoBoldIta12ptBitmaps + 854}, 		// M
	{11,s_FreeMonoBoldIta12ptBitmaps + 882}, 		// N
	{9, s_FreeMonoBoldIta12ptBitmaps + 910}, 		// O
	{10,s_FreeMonoBoldIta12ptBitmaps + 938}, 		// P
	{9, s_FreeMonoBoldIta12ptBitmaps + 966}, 		// Q
	{11,s_FreeMonoBoldIta12ptBitmaps + 994}, 		// R
	{9, s_FreeMonoBoldIta12ptBitmaps + 1022}, 		// S
	{9, s_FreeMonoBoldIta12ptBitmaps + 1050}, 		// T
	{10,s_FreeMonoBoldIta12ptBitmaps + 1078}, 		// U
	{10,s_FreeMonoBoldIta12ptBitmaps + 1106}, 		// V
	{10,s_FreeMonoBoldIta12ptBitmaps + 1134}, 		// W
	{11,s_FreeMonoBoldIta12ptBitmaps + 1162}, 		// X
	{9, s_FreeMonoBoldIta12ptBitmaps + 1190}, 		// Y
	{9, s_FreeMonoBoldIta12ptBitmaps + 1218}, 		// Z
	{6, s_FreeMonoBoldIta12ptBitmaps + 1246}, 		// [
	{5, s_FreeMonoBoldIta12ptBitmaps + 1260}, 		// '\'
	{6, s_FreeMonoBoldIta12ptBitmaps + 1274}, 		// ]
	{6, s_FreeMonoBoldIta12ptBitmaps + 1288}, 		// ^
	{10,s_FreeMonoBoldIta12ptBitmaps + 1302}, 		// _
	{2, s_FreeMonoBoldIta12ptBitmaps + 1330}, 		// `
	{9, s_FreeMonoBoldIta12ptBitmaps + 1344}, 		// a
	{10,s_FreeMonoBoldIta12ptBitmaps + 1372}, 		// b
	{9, s_FreeMonoBoldIta12ptBitmaps + 1400}, 		// c
	{9, s_FreeMonoBoldIta12ptBitmaps + 1428}, 		// d
	{8, s_FreeMonoBoldIta12ptBitmaps + 1456}, 		// e
	{10,s_FreeMonoBoldIta12ptBitmaps + 1470}, 		// f
	{10,s_FreeMonoBoldIta12ptBitmaps + 1498}, 		// g
	{10,s_FreeMonoBoldIta12ptBitmaps + 1526}, 		// h
	{8, s_FreeMonoBoldIta12ptBitmaps + 1554}, 		// i
	{8, s_FreeMonoBoldIta12ptBitmaps + 1568}, 		// j
	{9, s_FreeMonoBoldIta12ptBitmaps + 1582}, 		// k
	{8, s_FreeMonoBoldIta12ptBitmaps + 1610}, 		// l
	{11,s_FreeMonoBoldIta12ptBitmaps + 1624}, 		// m
	{10,s_FreeMonoBoldIta12ptBitmaps + 1652}, 		// n
	{9, s_FreeMonoBoldIta12ptBitmaps + 1680}, 		// o
	{11,s_FreeMonoBoldIta12ptBitmaps + 1708}, 		// p
	{10,s_FreeMonoBoldIta12ptBitmaps + 1736}, 		// q
	{9, s_FreeMonoBoldIta12ptBitmaps + 1764}, 		// r
	{7, s_FreeMonoBoldIta12ptBitmaps + 1792}, 		// s
	{8, s_FreeMonoBoldIta12ptBitmaps + 1806}, 		// t
	{9, s_FreeMonoBoldIta12ptBitmaps + 1820}, 		// u
	{10,s_FreeMonoBoldIta12ptBitmaps + 1848}, 		// v
	{10,s_FreeMonoBoldIta12ptBitmaps + 1876}, 		// w
	{10,s_FreeMonoBoldIta12ptBitmaps + 1904}, 		// x
	{10,s_FreeMonoBoldIta12ptBitmaps + 1932}, 		// y
	{9, s_FreeMonoBoldIta12ptBitmaps + 1960}, 		// z
	{5, s_FreeMonoBoldIta12ptBitmaps + 1988}, 		// {
	{4, s_FreeMonoBoldIta12ptBitmaps + 2002}, 		// |
	{6, s_FreeMonoBoldIta12ptBitmaps + 2016}, 		// }
	{8, s_FreeMonoBoldIta12ptBitmaps + 2030}, 		// ~
};

// Font information for FreeMono 12pt
const FontDesc_t iFontFreeMonoBoldIta12pt = {
	FONT_TYPE_VAR_WIDTH,
	12,
	14,
	{.pCharDesc = s_FreeMonoBoldIta12ptCharDesc }
};
