// INT16CXX.H:  saving and restoring registers during interrupt
/*
 IMPORTANT: CC5X will AUTOMATICALLY check that the registers W,
 STATUS, PCLATH and FSR are saved and restored during interrupt.

 CC5X also supports CUSTOM save and restore sequences. If you want
 to use your own register save and restore during interrupt, please
 read the following CUSTOM INTERRUPT SAVE AND RESTORE section.


// DEFAULT INTERRUPT STRUCTURE (RECOMMENDED)

   #include "int16CXX.h"

   #pragma origin 4

   interrupt serverX( void)
   {
       /* W and STATUS are saved by the next macro. PCLATH is also
          saved if necessary. The code produced is CPU-dependent. */

       int_save_registers    // W, STATUS (and PCLATH if required)

       //char sv_FSR = FSR;  // save FSR if required

       // handle the interrupt (insert application code here)

       //FSR = sv_FSR;       // restore FSR if saved

       int_restore_registers // W, STATUS (and PCLATH if required)

       /* IMPORTANT : GIE should normally NOT be set or cleared in
          the interrupt routine. GIE is AUTOMATICALLY cleared on
          interrupt entry by the CPU and set to 1 on exit (by
          RETFIE). Setting GIE to 1 inside the interrupt service
          routine will cause nested interrupts if an interrupt is
          pending. Too deep nesting may crash the program ! */

       /* IMPORTANT : It is NOT recommended to specify the interrupt
          save style (i.e. INT_xxx_style) in the application. This
          is done in the chip HEADER file (f.ex. 16F877.h). Wrong
          register save style may cause strange problems and is very
          difficult to trace. */
   }

 The compiler will detect if the FSR register is modified during
 interrupt processing without being saved and restored. The
 supplied macros for saving and restoring registers will not save
 FSR. This have to be done by user code when needed. If FSR is
 modified by a table or pointer access, or by direct writing, the
 compiler will check that FSR is saved and restored, also in
 nested function calls. Note that the FSR saving and restoring
 can be done in a local region surrounding the indexed access,
 and does not need to be done in the beginning and end of the
 interrupt routine.

 A warning is printed if FSR is saved but not changed.

 The error and warning messages printed can be removed:

  #pragma interruptSaveCheck  n  // no warning or error
  #pragma interruptSaveCheck  w  // warning only
  #pragma interruptSaveCheck  e  // error and warning (default)

 Note that the above pragma also change the checking done on W,
 STATUS and PCLATH.


// CUSTOM INTERRUPT SAVE AND RESTORE:

 It is not required to use the above save and restore macros. CC5X
 also supports custom interrupt structures.

  A) You may want to use your own save and restore sequence. This
     can be done by inline assembly. Take a look at the macros at
     the end of this file to get an idea on how to do this. If CC5X
     does not accept your code, just insert (on your own risk):

       #pragma interruptSaveCheck  n  // no warning or error

  B) No registers need to be saved when using the following
     instructions in the interrupt routine. The register save
     checking should NOT be disabled.

       btss(bx1);      // BTFSS 0x70,bx1  ; unbanked RAM/SFR only
       bx2 = 1;        // BSF   0x70,bx2  ; unbanked RAM/SFR only
       bx1 = 0;        // BCF   0x70,bx1  ; unbanked RAM/SFR only
       btsc(bx1);      // BTFSC 0x70,bx1  ; unbanked RAM/SFR only
       sleep();        // SLEEP
       vs = swap(vs);  // SWAPF vs,1      ; unbanked RAM/SFR only
       vs = incsz(vs); // INCFSZ vs,1     ; unbanked RAM/SFR only
       nop();          // NOP
       vs = decsz(vs); // DECFSZ vs,1     ; unbanked RAM/SFR only
       clrwdt();       // CLRWDT

  C) It is possible to enable interrupt only in special regions
     (wait loops) in such a way that W, STATUS, PCLATH and FSR can
     be modified during interrupt without disturbing the main
     program. Note that interrupt must ONLY be enabled in codepage
     0 when PCLATH is not saved. The register save can then be
     omitted and the save checking must be swithed off to avoid the
     error messages:

       #pragma interruptSaveCheck  n  // no warning or error
*/


#if defined _16C61  ||  defined _16C71  || defined _16C84 \
    ||  defined INT_min_style

 // CODE : Up to 2048 words (PCLATH not used)
 // RAM :  Located in bank 0 and mapped to bank 1, RP1 is unused

 #define  int_save_registers  \
    char s1, s2;  \
    s1 = W;  \
    s2 = swap( STATUS);

 #define  int_restore_registers \
    STATUS = swap( s2);  \
    s1 = swap( s1);  \
    W = swap( s1);


#elif defined _16C64  ||  defined _16C622  ||  defined INT_med_style

 // CODE : Up to 2048 words (PCLATH not used)
 // RAM :  Located in bank 0 and 1 (no mapping), RP1 is unused

 #define  int_save_registers  \
   #pragma char s1_save = 0x20  \
   #pragma char s1_savX = 0xA0  \
   #pragma rambank 0  \
    char s2; \
   #pragma update_RP 0  \
    s1_save = W;  \
    W = swap( STATUS);  \
    RP0 = 0; \
   #pragma update_RP 1  \
    s2 = W;  \


 #define  int_restore_registers \
    W = swap( s2);  \
   #pragma update_RP 0  \
    STATUS = W;  \
    s1_save = swap( s1_save);  \
    W = swap( s1_save); \
   #pragma update_RP 1


#elif defined _16C65  ||  defined _16C73  ||  defined _16C74 \
      ||  defined INT_max_style

 // CODE : Above 2048 words (PCLATH is saved)
 // RAM :  Located in bank 0 and 1 (no mapping between bank 0 and 1)

 #define  int_save_registers  \
   #pragma char s1_save = 0x20  \
   #pragma char s1_savX = 0xA0  \
   #pragma rambank 0  \
    char s2, s3; \
   #pragma update_RP 0  \
    s1_save = W;  \
    W = swap( STATUS);  \
    RP0 = 0; \
   #pragma update_RP 1  \
    s2 = W;  \
    s3 = PCLATH;  \
    PCLATH = 0;


 #define  int_restore_registers \
    PCLATH = s3;  \
    W = swap( s2);  \
   #pragma update_RP 0  \
    STATUS = W;  \
    s1_save = swap( s1_save);  \
    W = swap( s1_save); \
   #pragma update_RP 1


#elif defined _16C620  ||  defined _16C621  ||  defined INT_bank0_style

 // CODE : Up to 2048 words (PCLATH not used)
 // RAM :  Located in bank 0 only (no mapping either), RP1 is unused

 #define  int_save_registers  \
   #pragma rambank 0  \
    char s1, s2;  \
    bit RP0_save;  \
   #pragma update_RP 0  \
    if ( RP0)  {  \
        RP0 = 0;  \
        RP0_save = 1;  \
    }  \
    else  \
        RP0_save = 0;  \
   #pragma update_RP 1  \
    s1 = W;  \
    s2 = swap( STATUS);


 #define  int_restore_registers \
    W = swap( s2);  \
   #pragma update_RP 0  \
    STATUS = W;  \
    s1 = swap( s1);  \
    W = swap( s1);  \
    if ( RP0_save)  \
        RP0 = 1;  \
   #pragma update_RP 1


#elif defined INT_com_style

 // CODE : Up to 2048 words (PCLATH not used)
 // RAM :  Located in 2 banks, some locations are mapped, RP1 is unused

 #define  int_save_registers  \
   #pragma rambank 0 \
    char s1_save; \
   #pragma rambank INT_rambank  \
    char s2; \
   #pragma update_RP 0  \
    s1_save = W;  \
    W = swap( STATUS);  \
    RP0 = INT_rambank % 2; \
   #pragma update_RP 1  \
    s2 = W;  \


 #define  int_restore_registers \
    W = swap( s2);  \
   #pragma update_RP 0  \
    STATUS = W;  \
    s1_save = swap( s1_save);  \
    W = swap( s1_save); \
   #pragma update_RP 1


#elif defined INT_lgen_style

 // CODE : Up to 2048 words (PCLATH not used)
 // RAM :  Both RP0 and RP1 are active, some RAM locations are mapped

 #define  int_save_registers  \
   #pragma rambank - \
    char s1_save; \
   #pragma rambank INT_rambank  \
    char s2; \
   #pragma update_RP 0  \
    s1_save = W;  \
    W = swap( STATUS);  \
    RP0 = INT_rambank % 2; \
    RP1 = INT_rambank / 2; \
   #pragma update_RP 1  \
    s2 = W;


 #define  int_restore_registers \
    W = swap( s2);  \
   #pragma update_RP 0  \
    STATUS = W;  \
    s1_save = swap( s1_save);  \
    W = swap( s1_save); \
   #pragma update_RP 1


#elif defined INT_midgen_style

 // CODE : Above 2048 words (PCLATH is saved)
 // RAM :  Located in 2 banks, some locations are mapped, RP1 is unused

 #define  int_save_registers  \
   #pragma rambank - \
    char s1_save; \
   #pragma rambank INT_rambank  \
    char s2, s3; \
   #pragma update_RP 0  \
    s1_save = W;  \
    W = swap( STATUS);  \
    RP0 = INT_rambank % 2; \
   #pragma update_RP 1  \
    s2 = W;  \
    s3 = PCLATH;  \
    PCLATH = 0;


 #define  int_restore_registers \
    PCLATH = s3;  \
    W = swap( s2);  \
   #pragma update_RP 0  \
    STATUS = W;  \
    s1_save = swap( s1_save);  \
    W = swap( s1_save); \
   #pragma update_RP 1


#elif defined INT_gen_style

 // CODE : Above 2048 words (PCLATH is saved)
 // RAM :  Both RP0 and RP1 are active, some RAM locations are mapped

 #define  int_save_registers  \
   #pragma rambank - \
    char s1_save; \
   #pragma rambank INT_rambank  \
    char s2, s3; \
   #pragma update_RP 0  \
    s1_save = W;  \
    W = swap( STATUS);  \
    RP0 = INT_rambank % 2; \
    RP1 = INT_rambank / 2; \
   #pragma update_RP 1  \
    s2 = W;  \
    s3 = PCLATH;  \
    PCLATH = 0;


 #define  int_restore_registers \
    PCLATH = s3;  \
    W = swap( s2);  \
   #pragma update_RP 0  \
    STATUS = W;  \
    s1_save = swap( s1_save);  \
    W = swap( s1_save); \
   #pragma update_RP 1


#else  // an undefined chip type

 #error Interrupt save and restore macro's are undefined for this chip

 // The interrupt save type is normally defined in the header file
 // by selecting one of the above predefined types.

#endif
