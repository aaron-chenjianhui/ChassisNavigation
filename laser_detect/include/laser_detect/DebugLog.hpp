#ifndef _DEBUG_LOG_HPP
#define _DEBUG_LOG_HPP

#define DEBUG 1


#include <iostream>


#if DEBUG
#define DEBUGLOG(X) { std::cout << X << std::endl; }
#else
#define DEBUGLOG(X)
#endif



#endif
