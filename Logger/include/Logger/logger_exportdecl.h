
#ifndef LOGGER_EXPORTDECL_H
# define LOGGER_EXPORTDECL_H

#if defined _WIN32 || defined __CYGWIN__
#  define LOGGER_DLLIMPORT __declspec(dllimport)
#  define LOGGER_DLLEXPORT __declspec(dllexport)
#  define LOGGER_DLLLOCAL
#else 
#  if __GNUC__ >= 4
#   define LOGGER_DLLIMPORT __attribute__ ((visibility("default")))
#   define LOGGER_DLLEXPORT __attribute__ ((visibility("default")))
#   define LOGGER_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define LOGGER_DLLIMPORT
#   define LOGGER_DLLEXPORT
#   define LOGGER_DLLLOCAL
#  endif
#endif

#ifdef LOGGER_STATIC
#  define LOGGER_DLLAPI
#  define LOGGER_LOCAL
#else
#  ifdef LOGGER_EXPORTS
#    define LOGGER_API LOGGER_DLLEXPORT
#  else
#    define LOGGER_API LOGGER_DLLIMPORT
#  endif
#  define LOGGER_LOCAL LOGGER_DLLLOCAL
#endif

#endif //LOGGER_EXPORTDECL_H
