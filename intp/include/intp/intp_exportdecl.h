
#ifndef INTP_EXPORTDECL_H
# define INTP_EXPORTDECL_H

#if defined _WIN32 || defined __CYGWIN__
#  define INTP_DLLIMPORT __declspec(dllimport)
#  define INTP_DLLEXPORT __declspec(dllexport)
#  define INTP_DLLLOCAL
#else 
#  if __GNUC__ >= 4
#   define INTP_DLLIMPORT __attribute__ ((visibility("default")))
#   define INTP_DLLEXPORT __attribute__ ((visibility("default")))
#   define INTP_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define INTP_DLLIMPORT
#   define INTP_DLLEXPORT
#   define INTP_DLLLOCAL
#  endif
#endif

#ifdef INTP_STATIC
#  define INTP_DLLAPI
#  define INTP_LOCAL
#else
#  ifdef INTP_EXPORTS
#    define INTP_API INTP_DLLEXPORT
#  else
#    define INTP_API INTP_DLLIMPORT
#  endif
#  define INTP_LOCAL INTP_DLLLOCAL
#endif

#endif //INTP_EXPORTDECL_H
