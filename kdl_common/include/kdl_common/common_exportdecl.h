
#ifndef COMMON_EXPORTDECL_H
# define COMMON_EXPORTDECL_H

#if defined _WIN32 || defined __CYGWIN__
#  define COMMON_DLLIMPORT __declspec(dllimport)
#  define COMMON_DLLEXPORT __declspec(dllexport)
#  define COMMON_DLLLOCAL
#else 
#  if __GNUC__ >= 4
#   define COMMON_DLLIMPORT __attribute__ ((visibility("default")))
#   define COMMON_DLLEXPORT __attribute__ ((visibility("default")))
#   define COMMON_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define COMMON_DLLIMPORT
#   define COMMON_DLLEXPORT
#   define COMMON_DLLLOCAL
#  endif
#endif

#ifdef COMMON_STATIC
#  define COMMON_DLLAPI
#  define COMMON_LOCAL
#else
#  ifdef COMMON_EXPORTS
#    define COMMON_API COMMON_DLLEXPORT
#  else
#    define COMMON_API COMMON_DLLIMPORT
#  endif
#  define COMMON_LOCAL COMMON_DLLLOCAL
#endif

#endif //COMMON_EXPORTDECL_H
