// 
//  Font data for FreeMono 12pt
// https://savannah.gnu.org/projects/freefont/
// 
// Font bitmap generated by
// http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator

#include "display/ifont.h"

// Character bitmaps for FreeMono 12pt
static const uint8_t s_FreeMonoBold12ptBitmaps[] = {
	// @0 '!' (2 pixels wide)
	0x00, //   
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0x00, //   
	0xC0, // ##
	0xC0, // ##
	0x00, //   
	0x00, //   
	0x00, //   

	// @14 '"' (4 pixels wide)
	0x00, //     
	0x90, // #  #
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

	// @28 '#' (8 pixels wide)
	0x00, //         
	0x36, //   ## ## 
	0x36, //   ## ## 
	0x36, //   ## ## 
	0xFF, // ########
	0xFF, // ########
	0x3C, //   ####  
	0x7F, //  #######
	0x7F, //  #######
	0x6C, //  ## ##  
	0x6C, //  ## ##  
	0x6C, //  ## ##  
	0x00, //         
	0x00, //         

	// @42 '$' (7 pixels wide)
	0x30, //   ##   
	0x78, //  ####  
	0xF8, // #####  
	0xC8, // ##  #  
	0xC0, // ##     
	0xFC, // ###### 
	0x3E, //   #####
	0x86, // #    ##
	0xC6, // ##   ##
	0xFC, // ###### 
	0x30, //   ##   
	0x30, //   ##   
	0x30, //   ##   
	0x00, //        

	// @56 '%' (7 pixels wide)
	0x00, //        
	0x70, //  ###   
	0xC8, // ##  #  
	0x88, // #   #  
	0xC8, // ##  #  
	0x7E, //  ######
	0x70, //  ###   
	0x8C, // #   ## 
	0x12, //    #  #
	0x12, //    #  #
	0x0C, //     ## 
	0x00, //        
	0x00, //        
	0x00, //        

	// @70 '&' (7 pixels wide)
	0x00, //        
	0x00, //        
	0x3C, //   #### 
	0x7C, //  ##### 
	0x60, //  ##    
	0x60, //  ##    
	0x70, //  ###   
	0xDE, // ## ####
	0xCC, // ##  ## 
	0xFE, // #######
	0x7E, //  ######
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

	// @98 '(' (4 pixels wide)
	0x00, //     
	0x30, //   ##
	0x60, //  ## 
	0x60, //  ## 
	0x40, //  #  
	0xC0, // ##  
	0xC0, // ##  
	0xC0, // ##  
	0xC0, // ##  
	0xC0, // ##  
	0x60, //  ## 
	0x60, //  ## 
	0x30, //   ##
	0x00, //     

	// @112 ')' (4 pixels wide)
	0x00, //     
	0xC0, // ##  
	0x60, //  ## 
	0x60, //  ## 
	0x20, //   # 
	0x30, //   ##
	0x30, //   ##
	0x30, //   ##
	0x30, //   ##
	0x30, //   ##
	0x60, //  ## 
	0x60, //  ## 
	0xC0, // ##  
	0x00, //     

	// @126 '*' (8 pixels wide)
	0x00, //         
	0x18, //    ##   
	0x18, //    ##   
	0xFF, // ########
	0x7E, //  ###### 
	0x3C, //   ####  
	0x7E, //  ###### 
	0x24, //   #  #  
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @140 '+' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0xFC, // ######
	0xFC, // ######
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
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
	0x00, //    
	0x60, //  ##
	0x40, //  # 
	0xC0, // ## 
	0x80, // #  
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

	// @196 '/' (7 pixels wide)
	0x02, //       #
	0x06, //      ##
	0x04, //      # 
	0x0C, //     ## 
	0x0C, //     ## 
	0x18, //    ##  
	0x18, //    ##  
	0x30, //   ##   
	0x30, //   ##   
	0x60, //  ##    
	0x60, //  ##    
	0xC0, // ##     
	0x40, //  #     
	0x00, //        

	// @210 '0' (8 pixels wide)
	0x3C, //   ####  
	0x7E, //  ###### 
	0xE7, // ###  ###
	0xC3, // ##    ##
	0xC3, // ##    ##
	0xC3, // ##    ##
	0xC3, // ##    ##
	0xC3, // ##    ##
	0xE7, // ###  ###
	0x7E, //  ###### 
	0x3C, //   ####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @224 '1' (6 pixels wide)
	0x00, //       
	0x70, //  ###  
	0xF0, // ####  
	0xB0, // # ##  
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0xFC, // ######
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       

	// @238 '2' (7 pixels wide)
	0x78, //  ####  
	0xFC, // ###### 
	0xC6, // ##   ##
	0x06, //      ##
	0x06, //      ##
	0x0C, //     ## 
	0x18, //    ##  
	0x30, //   ##   
	0x60, //  ##    
	0xFE, // #######
	0xFE, // #######
	0x00, //        
	0x00, //        
	0x00, //        

	// @252 '3' (7 pixels wide)
	0x78, //  ####  
	0xFE, // #######
	0x06, //      ##
	0x06, //      ##
	0x1C, //    ### 
	0x1C, //    ### 
	0x06, //      ##
	0x06, //      ##
	0x06, //      ##
	0xFC, // ###### 
	0x78, //  ####  
	0x00, //        
	0x00, //        
	0x00, //        

	// @266 '4' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x38, //   ### 
	0x38, //   ### 
	0x58, //  # ## 
	0x58, //  # ## 
	0x98, // #  ## 
	0xFC, // ######
	0xFC, // ######
	0x3C, //   ####
	0x3C, //   ####
	0x00, //       
	0x00, //       
	0x00, //       

	// @280 '5' (8 pixels wide)
	0x00, //         
	0x7C, //  #####  
	0x7C, //  #####  
	0x60, //  ##     
	0x7C, //  #####  
	0x7F, //  #######
	0x03, //       ##
	0x03, //       ##
	0x43, //  #    ##
	0xFE, // ####### 
	0x7C, //  #####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @294 '6' (7 pixels wide)
	0x1E, //    ####
	0x3E, //   #####
	0x60, //  ##    
	0xC0, // ##     
	0xF8, // #####  
	0xFC, // ###### 
	0xC6, // ##   ##
	0xC6, // ##   ##
	0xC6, // ##   ##
	0x7C, //  ##### 
	0x38, //   ###  
	0x00, //        
	0x00, //        
	0x00, //        

	// @308 '7' (7 pixels wide)
	0x00, //        
	0xFE, // #######
	0xFE, // #######
	0x06, //      ##
	0x06, //      ##
	0x04, //      # 
	0x0C, //     ## 
	0x0C, //     ## 
	0x18, //    ##  
	0x18, //    ##  
	0x10, //    #   
	0x00, //        
	0x00, //        
	0x00, //        

	// @322 '8' (8 pixels wide)
	0x3C, //   ####  
	0x7E, //  ###### 
	0xC3, // ##    ##
	0xC3, // ##    ##
	0xC3, // ##    ##
	0x7E, //  ###### 
	0x7E, //  ###### 
	0xC3, // ##    ##
	0xC3, // ##    ##
	0xFF, // ########
	0x3C, //   ####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @336 '9' (7 pixels wide)
	0x38, //   ###  
	0x7C, //  ##### 
	0xC6, // ##   ##
	0xC6, // ##   ##
	0xC6, // ##   ##
	0x7E, //  ######
	0x3E, //   #####
	0x06, //      ##
	0x0C, //     ## 
	0xF8, // #####  
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
	0x80, // #  

	// @378 '<' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x03, //       ##
	0x0E, //     ### 
	0x38, //   ###   
	0xE0, // ###     
	0x38, //   ###   
	0x0E, //     ### 
	0x03, //       ##
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @392 '=' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0xFF, // ########
	0xFF, // ########
	0x00, //         
	0xFF, // ########
	0xFF, // ########
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @406 '>' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0xC0, // ##      
	0x70, //  ###    
	0x3C, //   ####  
	0x0F, //     ####
	0x1C, //    ###  
	0x70, //  ###    
	0xC0, // ##      
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         

	// @420 '?' (7 pixels wide)
	0x00, //        
	0x7C, //  ##### 
	0xFE, // #######
	0xC6, // ##   ##
	0x06, //      ##
	0x3C, //   #### 
	0x10, //    #   
	0x00, //        
	0x00, //        
	0x30, //   ##   
	0x30, //   ##   
	0x00, //        
	0x00, //        
	0x00, //        

	// @434 '@' (9 pixels wide)
	0x3C, 0x00, //   ####   
	0x7E, 0x00, //  ######  
	0xE3, 0x00, // ###   ## 
	0xC3, 0x00, // ##    ## 
	0xCF, 0x00, // ##  #### 
	0xDF, 0x00, // ## ##### 
	0xF3, 0x00, // ####  ## 
	0xF3, 0x00, // ####  ## 
	0xFF, 0x80, // #########
	0xCF, 0x80, // ##  #####
	0x60, 0x00, //  ##      
	0x7F, 0x00, //  ####### 
	0x3E, 0x00, //   #####  
	0x00, 0x00, //          

	// @462 'A' (10 pixels wide)
	0x00, 0x00, //           
	0x7C, 0x00, //  #####    
	0x7E, 0x00, //  ######   
	0x1E, 0x00, //    ####   
	0x1E, 0x00, //    ####   
	0x33, 0x00, //   ##  ##  
	0x33, 0x00, //   ##  ##  
	0x3F, 0x00, //   ######  
	0x7F, 0x80, //  ######## 
	0xF3, 0xC0, // ####  ####
	0xF3, 0xC0, // ####  ####
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @490 'B' (9 pixels wide)
	0x00, 0x00, //          
	0xFE, 0x00, // #######  
	0xFF, 0x00, // ######## 
	0x63, 0x00, //  ##   ## 
	0x63, 0x00, //  ##   ## 
	0x7E, 0x00, //  ######  
	0x7F, 0x00, //  ####### 
	0x61, 0x80, //  ##    ##
	0x61, 0x80, //  ##    ##
	0xFF, 0x80, // #########
	0xFF, 0x00, // ######## 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @518 'C' (8 pixels wide)
	0x00, //         
	0x3F, //   ######
	0x7F, //  #######
	0xE3, // ###   ##
	0xC0, // ##      
	0xC0, // ##      
	0xC0, // ##      
	0xC0, // ##      
	0xE0, // ###     
	0x7F, //  #######
	0x3E, //   ##### 
	0x00, //         
	0x00, //         
	0x00, //         

	// @532 'D' (9 pixels wide)
	0x00, 0x00, //          
	0xFE, 0x00, // #######  
	0xFF, 0x00, // ######## 
	0x63, 0x80, //  ##   ###
	0x61, 0x80, //  ##    ##
	0x61, 0x80, //  ##    ##
	0x61, 0x80, //  ##    ##
	0x61, 0x80, //  ##    ##
	0x63, 0x80, //  ##   ###
	0xFF, 0x00, // ######## 
	0xFE, 0x00, // #######  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @560 'E' (8 pixels wide)
	0x00, //         
	0xFF, // ########
	0xFF, // ########
	0x63, //  ##   ##
	0x78, //  ####   
	0x78, //  ####   
	0x78, //  ####   
	0x7B, //  #### ##
	0x63, //  ##   ##
	0xFF, // ########
	0xFF, // ########
	0x00, //         
	0x00, //         
	0x00, //         

	// @574 'F' (8 pixels wide)
	0x00, //         
	0xFF, // ########
	0xFF, // ########
	0x63, //  ##   ##
	0x78, //  ####   
	0x78, //  ####   
	0x78, //  ####   
	0x78, //  ####   
	0x60, //  ##     
	0xF8, // #####   
	0xF8, // #####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @588 'G' (9 pixels wide)
	0x00, 0x00, //          
	0x3F, 0x00, //   ###### 
	0x7F, 0x00, //  ####### 
	0xE3, 0x00, // ###   ## 
	0xC0, 0x00, // ##       
	0xC0, 0x00, // ##       
	0xCF, 0x80, // ##  #####
	0xCF, 0x80, // ##  #####
	0xE3, 0x00, // ###   ## 
	0x7F, 0x00, //  ####### 
	0x3E, 0x00, //   #####  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @616 'H' (10 pixels wide)
	0x00, 0x00, //           
	0x73, 0x80, //  ###  ### 
	0x73, 0x80, //  ###  ### 
	0x31, 0x80, //   ##   ## 
	0x31, 0x80, //   ##   ## 
	0x3F, 0x80, //   ####### 
	0x3F, 0x80, //   ####### 
	0x31, 0x80, //   ##   ## 
	0x31, 0x80, //   ##   ## 
	0xF3, 0xC0, // ####  ####
	0xF3, 0xC0, // ####  ####
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @644 'I' (6 pixels wide)
	0x00, //       
	0xFC, // ######
	0xFC, // ######
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0x30, //   ##  
	0xFC, // ######
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       

	// @658 'J' (8 pixels wide)
	0x00, //         
	0x1F, //    #####
	0x1F, //    #####
	0x06, //      ## 
	0x06, //      ## 
	0x06, //      ## 
	0x06, //      ## 
	0xC6, // ##   ## 
	0xC6, // ##   ## 
	0xFE, // ####### 
	0x78, //  ####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @672 'K' (8 pixels wide)
	0x00, //         
	0xFE, // ####### 
	0xFE, // ####### 
	0x6C, //  ## ##  
	0x78, //  ####   
	0x70, //  ###    
	0x78, //  ####   
	0x6C, //  ## ##  
	0x6C, //  ## ##  
	0xF7, // #### ###
	0xF7, // #### ###
	0x00, //         
	0x00, //         
	0x00, //         

	// @686 'L' (8 pixels wide)
	0x00, //         
	0xF0, // ####    
	0xF0, // ####    
	0x60, //  ##     
	0x60, //  ##     
	0x60, //  ##     
	0x60, //  ##     
	0x63, //  ##   ##
	0x63, //  ##   ##
	0xFF, // ########
	0xFF, // ########
	0x00, //         
	0x00, //         
	0x00, //         

	// @700 'M' (10 pixels wide)
	0x00, 0x00, //           
	0xE1, 0xC0, // ###    ###
	0xF3, 0xC0, // ####  ####
	0x73, 0x80, //  ###  ### 
	0x73, 0x80, //  ###  ### 
	0x7F, 0x80, //  ######## 
	0x6D, 0x80, //  ## ## ## 
	0x6D, 0x80, //  ## ## ## 
	0x61, 0x80, //  ##    ## 
	0xF3, 0xC0, // ####  ####
	0xF3, 0xC0, // ####  ####
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @728 'N' (10 pixels wide)
	0x00, 0x00, //           
	0xE3, 0xC0, // ###   ####
	0xF3, 0xC0, // ####  ####
	0x79, 0x80, //  ####  ## 
	0x79, 0x80, //  ####  ## 
	0x6D, 0x80, //  ## ## ## 
	0x6D, 0x80, //  ## ## ## 
	0x67, 0x80, //  ##  #### 
	0x67, 0x80, //  ##  #### 
	0xF3, 0x80, // ####  ### 
	0xF1, 0x80, // ####   ## 
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @756 'O' (10 pixels wide)
	0x00, 0x00, //           
	0x1E, 0x00, //    ####   
	0x7F, 0x80, //  ######## 
	0x61, 0x80, //  ##    ## 
	0xC0, 0xC0, // ##      ##
	0xC0, 0xC0, // ##      ##
	0xC0, 0xC0, // ##      ##
	0xC0, 0xC0, // ##      ##
	0x61, 0x80, //  ##    ## 
	0x7F, 0x80, //  ######## 
	0x1E, 0x00, //    ####   
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @784 'P' (8 pixels wide)
	0x00, //         
	0xFC, // ######  
	0xFE, // ####### 
	0x63, //  ##   ##
	0x63, //  ##   ##
	0x63, //  ##   ##
	0x7E, //  ###### 
	0x7E, //  ###### 
	0x60, //  ##     
	0xF8, // #####   
	0xF8, // #####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @798 'Q' (10 pixels wide)
	0x00, 0x00, //           
	0x1E, 0x00, //    ####   
	0x7F, 0x80, //  ######## 
	0x61, 0x80, //  ##    ## 
	0xC0, 0xC0, // ##      ##
	0xC0, 0xC0, // ##      ##
	0xC0, 0xC0, // ##      ##
	0xC0, 0xC0, // ##      ##
	0x61, 0x80, //  ##    ## 
	0x7F, 0x80, //  ######## 
	0x1E, 0x00, //    ####   
	0x3F, 0xC0, //   ########
	0x3F, 0x80, //   ####### 
	0x00, 0x00, //           

	// @826 'R' (9 pixels wide)
	0x00, 0x00, //          
	0xFE, 0x00, // #######  
	0xFF, 0x00, // ######## 
	0x63, 0x00, //  ##   ## 
	0x63, 0x00, //  ##   ## 
	0x7E, 0x00, //  ######  
	0x7C, 0x00, //  #####   
	0x66, 0x00, //  ##  ##  
	0x63, 0x00, //  ##   ## 
	0xF3, 0x80, // ####  ###
	0xF1, 0x80, // ####   ##
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @854 'S' (8 pixels wide)
	0x00, //         
	0x3F, //   ######
	0x7F, //  #######
	0xC3, // ##    ##
	0xC0, // ##      
	0xFC, // ######  
	0x3E, //   ##### 
	0x07, //      ###
	0xC3, // ##    ##
	0xFE, // ####### 
	0xFC, // ######  
	0x00, //         
	0x00, //         
	0x00, //         

	// @868 'T' (8 pixels wide)
	0x00, //         
	0xFF, // ########
	0xFF, // ########
	0xDB, // ## ## ##
	0xDB, // ## ## ##
	0x18, //    ##   
	0x18, //    ##   
	0x18, //    ##   
	0x18, //    ##   
	0x3C, //   ####  
	0x3C, //   ####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @882 'U' (10 pixels wide)
	0x00, 0x00, //           
	0xF3, 0xC0, // ####  ####
	0xF3, 0xC0, // ####  ####
	0x61, 0x80, //  ##    ## 
	0x61, 0x80, //  ##    ## 
	0x61, 0x80, //  ##    ## 
	0x61, 0x80, //  ##    ## 
	0x61, 0x80, //  ##    ## 
	0x61, 0x80, //  ##    ## 
	0x3F, 0x00, //   ######  
	0x1E, 0x00, //    ####   
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @910 'V' (10 pixels wide)
	0x00, 0x00, //           
	0xF3, 0xC0, // ####  ####
	0xF3, 0xC0, // ####  ####
	0x61, 0x80, //  ##    ## 
	0x21, 0x00, //   #    #  
	0x33, 0x00, //   ##  ##  
	0x33, 0x00, //   ##  ##  
	0x1A, 0x00, //    ## #   
	0x1E, 0x00, //    ####   
	0x0C, 0x00, //     ##    
	0x0C, 0x00, //     ##    
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @938 'W' (10 pixels wide)
	0x00, 0x00, //           
	0xF3, 0xC0, // ####  ####
	0xF3, 0xC0, // ####  ####
	0x41, 0x80, //  #     ## 
	0x4D, 0x80, //  #  ## ## 
	0x7D, 0x80, //  ##### ## 
	0x7F, 0x80, //  ######## 
	0x7F, 0x00, //  #######  
	0x73, 0x00, //  ###  ##  
	0x73, 0x00, //  ###  ##  
	0x31, 0x00, //   ##   #  
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @966 'X' (10 pixels wide)
	0x00, 0x00, //           
	0xE3, 0x80, // ###   ### 
	0xE3, 0x80, // ###   ### 
	0x63, 0x00, //  ##   ##  
	0x3E, 0x00, //   #####   
	0x1C, 0x00, //    ###    
	0x3E, 0x00, //   #####   
	0x36, 0x00, //   ## ##   
	0x63, 0x00, //  ##   ##  
	0xF3, 0xC0, // ####  ####
	0xF3, 0xC0, // ####  ####
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @994 'Y' (9 pixels wide)
	0x00, 0x00, //          
	0xE3, 0x80, // ###   ###
	0xE3, 0x80, // ###   ###
	0x63, 0x00, //  ##   ## 
	0x36, 0x00, //   ## ##  
	0x1E, 0x00, //    ####  
	0x0C, 0x00, //     ##   
	0x0C, 0x00, //     ##   
	0x0C, 0x00, //     ##   
	0x1E, 0x00, //    ####  
	0x1E, 0x00, //    ####  
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1022 'Z' (7 pixels wide)
	0x00, //        
	0xFC, // ###### 
	0xFC, // ###### 
	0xCC, // ##  ## 
	0xD8, // ## ##  
	0x10, //    #   
	0x30, //   ##   
	0x66, //  ##  ##
	0xC6, // ##   ##
	0xFE, // #######
	0xFE, // #######
	0x00, //        
	0x00, //        
	0x00, //        

	// @1036 '[' (3 pixels wide)
	0x00, //    
	0xE0, // ###
	0xE0, // ###
	0xC0, // ## 
	0xC0, // ## 
	0xC0, // ## 
	0xC0, // ## 
	0xC0, // ## 
	0xC0, // ## 
	0xC0, // ## 
	0xC0, // ## 
	0xE0, // ###
	0xE0, // ###
	0x00, //    

	// @1050 '\' (7 pixels wide)
	0xC0, // ##     
	0x40, //  #     
	0x60, //  ##    
	0x20, //   #    
	0x30, //   ##   
	0x10, //    #   
	0x18, //    ##  
	0x18, //    ##  
	0x0C, //     ## 
	0x0C, //     ## 
	0x06, //      ##
	0x06, //      ##
	0x02, //       #
	0x00, //        

	// @1064 ']' (3 pixels wide)
	0x00, //    
	0xE0, // ###
	0xE0, // ###
	0x60, //  ##
	0x60, //  ##
	0x60, //  ##
	0x60, //  ##
	0x60, //  ##
	0x60, //  ##
	0x60, //  ##
	0x60, //  ##
	0xE0, // ###
	0xE0, // ###
	0x00, //    

	// @1078 '^' (7 pixels wide)
	0x00, //        
	0x10, //    #   
	0x38, //   ###  
	0x3C, //   #### 
	0x66, //  ##  ##
	0xC2, // ##    #
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        
	0x00, //        

	// @1092 '_' (10 pixels wide)
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
	0xFF, 0xC0, // ##########

	// @1120 '`' (3 pixels wide)
	0x80, // #  
	0x40, //  # 
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

	// @1134 'a' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x7C, //  #####  
	0x7E, //  ###### 
	0x3E, //   ##### 
	0xFE, // ####### 
	0xC6, // ##   ## 
	0xFF, // ########
	0x7F, //  #######
	0x00, //         
	0x00, //         
	0x00, //         

	// @1148 'b' (10 pixels wide)
	0x00, 0x00, //           
	0xE0, 0x00, // ###       
	0xE0, 0x00, // ###       
	0x60, 0x00, //  ##       
	0x6F, 0x00, //  ## ####  
	0x7F, 0x80, //  ######## 
	0x71, 0xC0, //  ###   ###
	0x60, 0xC0, //  ##     ##
	0x71, 0xC0, //  ###   ###
	0xFF, 0x80, // ######### 
	0xEF, 0x00, // ### ####  
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           

	// @1176 'c' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x3F, //   ######
	0x7F, //  #######
	0xE3, // ###   ##
	0xC0, // ##      
	0xC1, // ##     #
	0x7F, //  #######
	0x3E, //   ##### 
	0x00, //         
	0x00, //         
	0x00, //         

	// @1190 'd' (9 pixels wide)
	0x00, 0x00, //          
	0x07, 0x00, //      ### 
	0x07, 0x00, //      ### 
	0x03, 0x00, //       ## 
	0x3B, 0x00, //   ### ## 
	0x7F, 0x00, //  ####### 
	0xE3, 0x00, // ###   ## 
	0xC3, 0x00, // ##    ## 
	0xC3, 0x00, // ##    ## 
	0x7F, 0x80, //  ########
	0x3F, 0x80, //   #######
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1218 'e' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x3C, //   ####  
	0xFE, // ####### 
	0xC2, // ##    # 
	0xFF, // ########
	0xFF, // ########
	0xFF, // ########
	0x7E, //  ###### 
	0x00, //         
	0x00, //         
	0x00, //         

	// @1232 'f' (6 pixels wide)
	0x00, //       
	0x3C, //   ####
	0x7C, //  #####
	0x60, //  ##   
	0xFC, // ######
	0xFC, // ######
	0x60, //  ##   
	0x60, //  ##   
	0x60, //  ##   
	0xF8, // ##### 
	0xF8, // ##### 
	0x00, //       
	0x00, //       
	0x00, //       

	// @1246 'g' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x3F, 0x80, //   #######
	0x7F, 0x80, //  ########
	0xC3, 0x00, // ##    ## 
	0xC3, 0x00, // ##    ## 
	0xE7, 0x00, // ###  ### 
	0x7F, 0x00, //  ####### 
	0x3B, 0x00, //   ### ## 
	0x03, 0x00, //       ## 
	0x1F, 0x00, //    ##### 
	0x1E, 0x00, //    ####  

	// @1274 'h' (9 pixels wide)
	0x00, 0x00, //          
	0xE0, 0x00, // ###      
	0xE0, 0x00, // ###      
	0x60, 0x00, //  ##      
	0x6E, 0x00, //  ## ###  
	0x7F, 0x00, //  ####### 
	0x63, 0x00, //  ##   ## 
	0x63, 0x00, //  ##   ## 
	0x63, 0x00, //  ##   ## 
	0xF7, 0x80, // #### ####
	0xF7, 0x80, // #### ####
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1302 'i' (7 pixels wide)
	0x00, //        
	0x18, //    ##  
	0x18, //    ##  
	0x00, //        
	0x78, //  ####  
	0x78, //  ####  
	0x18, //    ##  
	0x18, //    ##  
	0x18, //    ##  
	0xFE, // #######
	0xFE, // #######
	0x00, //        
	0x00, //        
	0x00, //        

	// @1316 'j' (5 pixels wide)
	0x00, //      
	0x30, //   ## 
	0x30, //   ## 
	0x00, //      
	0xF8, // #####
	0xF8, // #####
	0x18, //    ##
	0x18, //    ##
	0x18, //    ##
	0x18, //    ##
	0x18, //    ##
	0x18, //    ##
	0xF8, // #####
	0xF0, // #### 

	// @1330 'k' (8 pixels wide)
	0x00, //         
	0xE0, // ###     
	0xE0, // ###     
	0x60, //  ##     
	0x7E, //  ###### 
	0x7E, //  ###### 
	0x78, //  ####   
	0x70, //  ###    
	0x7C, //  #####  
	0xEF, // ### ####
	0xEF, // ### ####
	0x00, //         
	0x00, //         
	0x00, //         

	// @1344 'l' (7 pixels wide)
	0x00, //        
	0x78, //  ####  
	0x78, //  ####  
	0x18, //    ##  
	0x18, //    ##  
	0x18, //    ##  
	0x18, //    ##  
	0x18, //    ##  
	0x18, //    ##  
	0xFE, // #######
	0xFE, // #######
	0x00, //        
	0x00, //        
	0x00, //        

	// @1358 'm' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0xFB, 0x00, // ##### ## 
	0xFF, 0x80, // #########
	0x6D, 0x80, //  ## ## ##
	0x6D, 0x80, //  ## ## ##
	0x6D, 0x80, //  ## ## ##
	0xED, 0x80, // ### ## ##
	0xED, 0x80, // ### ## ##
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1386 'n' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0xEE, // ### ### 
	0xFF, // ########
	0x63, //  ##   ##
	0x63, //  ##   ##
	0x63, //  ##   ##
	0xF7, // #### ###
	0xF7, // #### ###
	0x00, //         
	0x00, //         
	0x00, //         

	// @1400 'o' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x3C, //   ####  
	0x7E, //  ###### 
	0xC3, // ##    ##
	0xC3, // ##    ##
	0xC3, // ##    ##
	0x7E, //  ###### 
	0x3C, //   ####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @1414 'p' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0xEF, 0x00, // ### ####  
	0xFF, 0x80, // ######### 
	0x70, 0xC0, //  ###    ##
	0x60, 0xC0, //  ##     ##
	0x71, 0xC0, //  ###   ###
	0x7F, 0x80, //  ######## 
	0x6F, 0x00, //  ## ####  
	0x60, 0x00, //  ##       
	0xF8, 0x00, // #####     
	0xF8, 0x00, // #####     

	// @1442 'q' (10 pixels wide)
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x00, 0x00, //           
	0x3D, 0xC0, //   #### ###
	0x7F, 0xC0, //  #########
	0xE3, 0x80, // ###   ### 
	0xC1, 0x80, // ##     ## 
	0xE3, 0x80, // ###   ### 
	0x7F, 0x80, //  ######## 
	0x3D, 0x80, //   #### ## 
	0x01, 0x80, //        ## 
	0x07, 0xC0, //      #####
	0x07, 0xC0, //      #####

	// @1470 'r' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0xEE, // ### ### 
	0xFF, // ########
	0x70, //  ###    
	0x60, //  ##     
	0x60, //  ##     
	0xF8, // #####   
	0xF8, // #####   
	0x00, //         
	0x00, //         
	0x00, //         

	// @1484 's' (8 pixels wide)
	0x00, //         
	0x00, //         
	0x00, //         
	0x00, //         
	0x3F, //   ######
	0x7F, //  #######
	0x63, //  ##   ##
	0x3E, //   ##### 
	0x43, //  #    ##
	0xFF, // ########
	0x7C, //  #####  
	0x00, //         
	0x00, //         
	0x00, //         

	// @1498 't' (7 pixels wide)
	0x00, //        
	0x60, //  ##    
	0x60, //  ##    
	0x60, //  ##    
	0xFC, // ###### 
	0xFC, // ###### 
	0x60, //  ##    
	0x60, //  ##    
	0x62, //  ##   #
	0x7E, //  ######
	0x3C, //   #### 
	0x00, //        
	0x00, //        
	0x00, //        

	// @1512 'u' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0xE7, 0x00, // ###  ### 
	0xE7, 0x00, // ###  ### 
	0x63, 0x00, //  ##   ## 
	0x63, 0x00, //  ##   ## 
	0x63, 0x00, //  ##   ## 
	0x7F, 0x80, //  ########
	0x3F, 0x80, //   #######
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1540 'v' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0xF7, 0x80, // #### ####
	0xF7, 0x80, // #### ####
	0x63, 0x00, //  ##   ## 
	0x36, 0x00, //   ## ##  
	0x36, 0x00, //   ## ##  
	0x1C, 0x00, //    ###   
	0x1C, 0x00, //    ###   
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1568 'w' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0xE3, 0x80, // ###   ###
	0xE3, 0x80, // ###   ###
	0x6D, 0x80, //  ## ## ##
	0x7F, 0x80, //  ########
	0x37, 0x00, //   ## ### 
	0x37, 0x00, //   ## ### 
	0x23, 0x00, //   #   ## 
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1596 'x' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x77, 0x00, //  ### ### 
	0x77, 0x00, //  ### ### 
	0x1C, 0x00, //    ###   
	0x1C, 0x00, //    ###   
	0x36, 0x00, //   ## ##  
	0x77, 0x00, //  ### ### 
	0xF7, 0x80, // #### ####
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          

	// @1624 'y' (9 pixels wide)
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0x00, 0x00, //          
	0xE3, 0x80, // ###   ###
	0xE3, 0x80, // ###   ###
	0x63, 0x00, //  ##   ## 
	0x62, 0x00, //  ##   #  
	0x36, 0x00, //   ## ##  
	0x3C, 0x00, //   ####   
	0x18, 0x00, //    ##    
	0x38, 0x00, //   ###    
	0xF8, 0x00, // #####    
	0xF8, 0x00, // #####    

	// @1652 'z' (6 pixels wide)
	0x00, //       
	0x00, //       
	0x00, //       
	0x00, //       
	0xFC, // ######
	0xFC, // ######
	0xD8, // ## ## 
	0x30, //   ##  
	0x40, //  #    
	0xFC, // ######
	0xFC, // ######
	0x00, //       
	0x00, //       
	0x00, //       

	// @1666 '{' (4 pixels wide)
	0x00, //     
	0x30, //   ##
	0x70, //  ###
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0xC0, // ##  
	0xC0, // ##  
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0x70, //  ###
	0x30, //   ##
	0x00, //     

	// @1680 '|' (2 pixels wide)
	0x00, //   
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0xC0, // ##
	0x00, //   

	// @1694 '}' (4 pixels wide)
	0x00, //     
	0xC0, // ##  
	0xE0, // ### 
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0x30, //   ##
	0x30, //   ##
	0x60, //  ## 
	0x60, //  ## 
	0x60, //  ## 
	0xE0, // ### 
	0xC0, // ##  
	0x00, //     

	// @1708 '~' (8 pixels wide)
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
static const CharDesc_t s_FreeMonoBold12ptCharDesc[] = {
	{2, s_FreeMonoBold12ptBitmaps + 0}, 		// !
	{4, s_FreeMonoBold12ptBitmaps + 14}, 		// "
	{8, s_FreeMonoBold12ptBitmaps + 28}, 		// #
	{7, s_FreeMonoBold12ptBitmaps + 42}, 		// $
	{7, s_FreeMonoBold12ptBitmaps + 56}, 		// %
	{7, s_FreeMonoBold12ptBitmaps + 70}, 		// &
	{1, s_FreeMonoBold12ptBitmaps + 84}, 		// '
	{4, s_FreeMonoBold12ptBitmaps + 98}, 		// (
	{4, s_FreeMonoBold12ptBitmaps + 112}, 		// )
	{8, s_FreeMonoBold12ptBitmaps + 126}, 		// *
	{6, s_FreeMonoBold12ptBitmaps + 140}, 		// +
	{3, s_FreeMonoBold12ptBitmaps + 154}, 		// ,
	{8, s_FreeMonoBold12ptBitmaps + 168}, 		// -
	{2, s_FreeMonoBold12ptBitmaps + 182}, 		// .
	{7, s_FreeMonoBold12ptBitmaps + 196}, 		// /
	{8, s_FreeMonoBold12ptBitmaps + 210}, 		// 0
	{6, s_FreeMonoBold12ptBitmaps + 224}, 		// 1
	{7, s_FreeMonoBold12ptBitmaps + 238}, 		// 2
	{7, s_FreeMonoBold12ptBitmaps + 252}, 		// 3
	{6, s_FreeMonoBold12ptBitmaps + 266}, 		// 4
	{8, s_FreeMonoBold12ptBitmaps + 280}, 		// 5
	{7, s_FreeMonoBold12ptBitmaps + 294}, 		// 6
	{7, s_FreeMonoBold12ptBitmaps + 308}, 		// 7
	{8, s_FreeMonoBold12ptBitmaps + 322}, 		// 8
	{7, s_FreeMonoBold12ptBitmaps + 336}, 		// 9
	{2, s_FreeMonoBold12ptBitmaps + 350}, 		// :
	{3, s_FreeMonoBold12ptBitmaps + 364}, 		// ;
	{8, s_FreeMonoBold12ptBitmaps + 378}, 		// <
	{8, s_FreeMonoBold12ptBitmaps + 392}, 		// =
	{8, s_FreeMonoBold12ptBitmaps + 406}, 		// >
	{7, s_FreeMonoBold12ptBitmaps + 420}, 		// ?
	{9, s_FreeMonoBold12ptBitmaps + 434}, 		// @
	{10,s_FreeMonoBold12ptBitmaps + 462}, 		// A
	{9, s_FreeMonoBold12ptBitmaps + 490}, 		// B
	{8, s_FreeMonoBold12ptBitmaps + 518}, 		// C
	{9, s_FreeMonoBold12ptBitmaps + 532}, 		// D
	{8, s_FreeMonoBold12ptBitmaps + 560}, 		// E
	{8, s_FreeMonoBold12ptBitmaps + 574}, 		// F
	{9, s_FreeMonoBold12ptBitmaps + 588}, 		// G
	{10,s_FreeMonoBold12ptBitmaps + 616}, 		// H
	{6, s_FreeMonoBold12ptBitmaps + 644}, 		// I
	{8, s_FreeMonoBold12ptBitmaps + 658}, 		// J
	{8, s_FreeMonoBold12ptBitmaps + 672}, 		// K
	{8, s_FreeMonoBold12ptBitmaps + 686}, 		// L
	{10,s_FreeMonoBold12ptBitmaps + 700}, 		// M
	{10,s_FreeMonoBold12ptBitmaps + 728}, 		// N
	{10,s_FreeMonoBold12ptBitmaps + 756}, 		// O
	{8, s_FreeMonoBold12ptBitmaps + 784}, 		// P
	{10,s_FreeMonoBold12ptBitmaps + 798}, 		// Q
	{9, s_FreeMonoBold12ptBitmaps + 826}, 		// R
	{8, s_FreeMonoBold12ptBitmaps + 854}, 		// S
	{8, s_FreeMonoBold12ptBitmaps + 868}, 		// T
	{10,s_FreeMonoBold12ptBitmaps + 882}, 		// U
	{10,s_FreeMonoBold12ptBitmaps + 910}, 		// V
	{10,s_FreeMonoBold12ptBitmaps + 938}, 		// W
	{10,s_FreeMonoBold12ptBitmaps + 966}, 		// X
	{9, s_FreeMonoBold12ptBitmaps + 994}, 		// Y
	{7, s_FreeMonoBold12ptBitmaps + 1022}, 		// Z
	{3, s_FreeMonoBold12ptBitmaps + 1036}, 		// [
	{7, s_FreeMonoBold12ptBitmaps + 1050}, 		// '\'
	{3, s_FreeMonoBold12ptBitmaps + 1064}, 		// ]
	{7, s_FreeMonoBold12ptBitmaps + 1078}, 		// ^
	{10,s_FreeMonoBold12ptBitmaps + 1092}, 		// _
	{3, s_FreeMonoBold12ptBitmaps + 1120}, 		// `
	{8, s_FreeMonoBold12ptBitmaps + 1134}, 		// a
	{10,s_FreeMonoBold12ptBitmaps + 1148}, 		// b
	{8, s_FreeMonoBold12ptBitmaps + 1176}, 		// c
	{9, s_FreeMonoBold12ptBitmaps + 1190}, 		// d
	{8, s_FreeMonoBold12ptBitmaps + 1218}, 		// e
	{6, s_FreeMonoBold12ptBitmaps + 1232}, 		// f
	{9, s_FreeMonoBold12ptBitmaps + 1246}, 		// g
	{9, s_FreeMonoBold12ptBitmaps + 1274}, 		// h
	{7, s_FreeMonoBold12ptBitmaps + 1302}, 		// i
	{5, s_FreeMonoBold12ptBitmaps + 1316}, 		// j
	{8, s_FreeMonoBold12ptBitmaps + 1330}, 		// k
	{7, s_FreeMonoBold12ptBitmaps + 1344}, 		// l
	{9, s_FreeMonoBold12ptBitmaps + 1358}, 		// m
	{8, s_FreeMonoBold12ptBitmaps + 1386}, 		// n
	{8, s_FreeMonoBold12ptBitmaps + 1400}, 		// o
	{10,s_FreeMonoBold12ptBitmaps + 1414}, 		// p
	{10,s_FreeMonoBold12ptBitmaps + 1442}, 		// q
	{8, s_FreeMonoBold12ptBitmaps + 1470}, 		// r
	{8, s_FreeMonoBold12ptBitmaps + 1484}, 		// s
	{7, s_FreeMonoBold12ptBitmaps + 1498}, 		// t
	{9, s_FreeMonoBold12ptBitmaps + 1512}, 		// u
	{9, s_FreeMonoBold12ptBitmaps + 1540}, 		// v
	{9, s_FreeMonoBold12ptBitmaps + 1568}, 		// w
	{9, s_FreeMonoBold12ptBitmaps + 1596}, 		// x
	{9, s_FreeMonoBold12ptBitmaps + 1624}, 		// y
	{6, s_FreeMonoBold12ptBitmaps + 1652}, 		// z
	{4, s_FreeMonoBold12ptBitmaps + 1666}, 		// {
	{2, s_FreeMonoBold12ptBitmaps + 1680}, 		// |
	{4, s_FreeMonoBold12ptBitmaps + 1694}, 		// }
	{8, s_FreeMonoBold12ptBitmaps + 1708}, 		// ~
};

// Font information for FreeMono 12pt
const FontDesc_t iFontFreeMonoBold12pt = {
	FONT_TYPE_VAR_WIDTH,
	10,
	14,
	{.pCharDesc = s_FreeMonoBold12ptCharDesc }
};
