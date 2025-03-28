
#ifndef KINEMATICS_EXPORTDECL_H
# define KINEMATICS_EXPORTDECL_H

#if defined _WIN32 || defined __CYGWIN__
#  define KINEMATICS_DLLIMPORT __declspec(dllimport)
#  define KINEMATICS_DLLEXPORT __declspec(dllexport)
#  define KINEMATICS_DLLLOCAL
#else 
#  if __GNUC__ >= 4
#   define KINEMATICS_DLLIMPORT __attribute__ ((visibility("default")))
#   define KINEMATICS_DLLEXPORT __attribute__ ((visibility("default")))
#   define KINEMATICS_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define KINEMATICS_DLLIMPORT
#   define KINEMATICS_DLLEXPORT
#   define KINEMATICS_DLLLOCAL
#  endif
#endif

#ifdef KINEMATICS_STATIC
#  define KINEMATICS_DLLAPI
#  define KINEMATICS_LOCAL
#else
#  ifdef KINEMATICS_EXPORTS
#    define KINEMATICS_API KINEMATICS_DLLEXPORT
#  else
#    define KINEMATICS_API KINEMATICS_DLLIMPORT
#  endif
#  define KINEMATICS_LOCAL KINEMATICS_DLLLOCAL
#endif

#endif //KINEMATICS_EXPORTDECL_H
