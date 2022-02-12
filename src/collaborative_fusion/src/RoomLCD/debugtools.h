#ifndef DEBUGTOOLS_H_
#define DEBUGTOOLS_H_

#ifdef DEBUG
#define DBGVAR( os, var ) \
  (os) << "DBG: " << __FILE__ << "(" << __LINE__ << ") "\
       << #var << " = [" << (var) << "]" << std::endl
#else
#define DBGVAR( os, var ) do{}while(0)
#endif

#endif