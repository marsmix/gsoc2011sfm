#ifndef _SFM_MACRO_H_
#define _SFM_MACRO_H_

//Taken from opencv's precom.hpp
#if _MSC_VER >= 1200
#pragma warning( disable: 4127 4251 4521 4996 )
#endif

#if ( defined WIN32 || defined _WIN32 || defined WINCE )
  #if defined SFM_API_EXPORTS
      #define SFM_EXPORTS __declspec( dllexport )
  #else
      #define SFM_EXPORTS __declspec( dllimport )
  #endif
#else
    #define SFM_EXPORTS
#endif


#endif