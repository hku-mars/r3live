#ifndef __OS_COMPATIBLE_HPP__
#define __OS_COMPATIBLE_HPP__
#include <string>
#include <fstream>
#include <iostream>
#include <ostream>
#include <sstream>
#include <stdarg.h>     //need for such like printf(...)
#include <stdio.h>
#include <string>
#if defined _MSC_VER
#include <direct.h>
#elif defined __GNUC__
#include <sys/types.h>
#include <sys/stat.h>
#endif
namespace Common_tools
{
    inline void create_dir(std::string dir)
    {
#if defined _MSC_VER
        _mkdir(dir.data());
#elif defined __GNUC__
        mkdir(dir.data(), 0777);
#endif
    }

    // Using asprintf() on windows
    // https://stackoverflow.com/questions/40159892/using-asprintf-on-windows
#ifndef _vscprintf
/* For some reason, MSVC fails to honour this #ifndef. */
/* Hence function renamed to _vscprintf_so(). */
    inline int _vscprintf_so(const char * format, va_list pargs) {
        int retval;
        va_list argcopy;
        va_copy(argcopy, pargs);
        retval = vsnprintf(NULL, 0, format, argcopy);
        va_end(argcopy);
        return retval;
    }
#endif // 

#ifndef vasprintf
    inline int vasprintf(char **strp, const char *fmt, va_list ap) {
        int len = _vscprintf_so(fmt, ap);
        if (len == -1) return -1;
        char *str = (char*)malloc((size_t)len + 1);
        if (!str) return -1;
        int r = vsnprintf(str, len + 1, fmt, ap); /* "secure" version of vsprintf */
        if (r == -1) return free(str), -1;
        *strp = str;
        return r;
    }
#endif // vasprintf
}
#endif