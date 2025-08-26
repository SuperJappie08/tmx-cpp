#pragma once

#ifndef TMX_CPP_WINDOWS
#if defined(_WIN32)
#define TMX_CPP_WINDOWS
#elif defined(_WIN64)
#define TMX_CPP_WINDOWS
#elif defined(__CYGWIN__) && !defined(_WIN32)
#define TMX_CPP_WINDOWS
#endif
#endif
