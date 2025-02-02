
#ifndef SCURVE_EXPORTDECL_H
# define SCURVE_EXPORTDECL_H

#if defined _WIN32 || defined __CYGWIN__
#  define SCURVE_DLLIMPORT __declspec(dllimport)
#  define SCURVE_DLLEXPORT __declspec(dllexport)
#  define SCURVE_DLLLOCAL
#else 
#  if __GNUC__ >= 4
#   define SCURVE_DLLIMPORT __attribute__ ((visibility("default")))
#   define SCURVE_DLLEXPORT __attribute__ ((visibility("default")))
#   define SCURVE_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define SCURVE_DLLIMPORT
#   define SCURVE_DLLEXPORT
#   define SCURVE_DLLLOCAL
#  endif
#endif

#ifdef SCURVE_STATIC
#  define SCURVE_DLLAPI
#  define SCURVE_LOCAL
#else
#  ifdef SCURVE_EXPORTS
#    define SCURVE_API SCURVE_DLLEXPORT
#  else
#    define SCURVE_API SCURVE_DLLIMPORT
#  endif
#  define SCURVE_LOCAL SCURVE_DLLLOCAL
#endif

#endif //SCURVE_EXPORTDECL_H
