# STM32 Data Persistence over Power Cycle 
 Using an STM32F103RB to showcase the data persistence feature by writing a string,say "Hello World", into a specific location in the  Flash memory.

 ## Getting started 
 I've used STM32 CUBE IDE(you can use Keil as well) as well as the STM32 Cube Programmer. I'm using the STM Cube Programmer since I'm still figuring out the new IDE, and the easiest way to check what's in the memory is through this.

 ## Important Notes
 These might seem obvious, but putting it just to cover everyone in the spectrum (including me)
 * FLASH_STORAGE is the target location in flash memory that we're trying to save the string.
 * Page_Size is something to be extremely careful of . it differs from each STM based on its capabilities. A page, memory page, or virtual page is a fixed-length contiguous block of virtual memory, described by a single entry in the page table. It is the smallest unit of data for memory management in a virtual memory operating system. (as defined by Wiki)
 * For the STM32F103RB it is 1KB . When you assign page size, it should be kept as 0x0400
 * When you choose your FLASH_STORAGE location, make sure you choose it well over the 0x08000000 .This is where the flash memory starts.
 

 ## Acknowledgent and Credits
 I have followed the idea laid out by [Viktor Vano](https://github.com/viktorvano) .Althugh I've made a few changes here and there,the sole credit goes to him. Check him out too !
