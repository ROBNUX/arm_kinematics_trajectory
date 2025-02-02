
#ifndef TRAJECTORY_EXPORTDECL_H
# define TRAJECTORY_EXPORTDECL_H

#if defined _WIN32 || defined __CYGWIN__
#  define TRAJECTORY_DLLIMPORT __declspec(dllimport)
#  define TRAJECTORY_DLLEXPORT __declspec(dllexport)
#  define TRAJECTORY_DLLLOCAL
#else 
#  if __GNUC__ >= 4
#   define TRAJECTORY_DLLIMPORT __attribute__ ((visibility("default")))
#   define TRAJECTORY_DLLEXPORT __attribute__ ((visibility("default")))
#   define TRAJECTORY_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define TRAJECTORY_DLLIMPORT
#   define TRAJECTORY_DLLEXPORT
#   define TRAJECTORY_DLLLOCAL
#  endif
#endif

#ifdef TRAJECTORY_STATIC
#  define TRAJECTORY_DLLAPI
#  define TRAJECTORY_LOCAL
#else
#  ifdef TRAJECTORY_EXPORTS
#    define TRAJECTORY_API TRAJECTORY_DLLEXPORT
#  else
#    define TRAJECTORY_API TRAJECTORY_DLLIMPORT
#  endif
#  define TRAJECTORY_LOCAL TRAJECTORY_DLLLOCAL
#endif

#endif //TRAJECTORY_EXPORTDECL_H
