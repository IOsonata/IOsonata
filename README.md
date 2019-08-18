# IOsonata
IOsonata multi-platform multi-architecture optimized software library for fast and easy iot products development

This is the new refactoring of the EHAL library (https://github.com/I-SYST/EHAL).

Although this refactoring includes supports for multiple IDE/Compilers.  The prefered IDE is still Eclipse/GCC.  GCC is the facto standard for embedded software development. Eclipse is 100% free and the most flexible IDE.  It could be little overwhelming for newbies at first (like any other IDE if you are new to it anyway).

For desktop pc version of the library, native compiler and IDE are used.  XCode for OSX, Visual Studio for Windows, Eclipse for Linux.

### IDE limiations :

Eclipse & GCC : Full C++ supports, full file io
IAR : Full C++ support, no system support for file io
uVision : Could not create library compilation properly
CrossWorks : GCC C++ is stripped down to bare bone, no file io supports
Segger Stusio : Strip down version of CrossWorks.  Even less functional.

