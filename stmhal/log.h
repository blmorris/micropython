
#include <stdio.h>

//#define LOG_NDEBUG
#ifndef LOG_NDEBUG
  #define LOG_INFO(msg)\
  {\
      if(msg)\
      {\
          printf("INFO: [%s:%d] %s() [%s]\n", __FILE__, __LINE__, __FUNCTION__, (#msg));\
      }\
      else\
      {\
        printf("INFO: [%s:%d] %s()\n", __FILE__, __LINE__, __FUNCTION__);\
      }\
  }

  #define LOG_ERR(msg)\
  {\
      if(msg)\
      {\
          printf("ERR: [%s:%d] %s() [%s]\n", __FILE__, __LINE__, __FUNCTION__, (#msg));\
      }\
      else\
      {\
        printf("ERR: [%s:%d] %s()\n", __FILE__, __LINE__, __FUNCTION__);\
      }\
  }

  // Log on a condition without a return, just continue execution
  #define LOG_COND_CONT(condition)\
  {\
      if(!(condition))\
      {\
          printf("Condition [%s] failed in [%s:%d] %s() \n", (#condition), __FILE__, __LINE__, __FUNCTION__);\
      }\
  }
  // Log on a condition with a return code
  #define LOG_COND_RET(condition, rc)\
  {\
      if(!(condition))\
      {\
          printf("Condition [%s] failed in [%s:%d] %s()\n", (#condition), __FILE__, __LINE__, __FUNCTION__);\
          return(rc);\
      }\
  }
#else
  #define LOG_INFO(msg)                  ((void)0)
  #define LOG_ERR(msg)                   ((void)0)
  #define LOG_COND_CONT(condition)       ((void)0)
  #define LOG_COND_RET(condition, rc)    ((void)0)
#endif


#define RAISE_EXCEP(assertion, type, msg) { if ((assertion) == 0) nlr_raise(mp_obj_new_exception_msg(&(type), (msg))); }

