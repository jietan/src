#ifndef SEHOON_MISC_H
#define SEHOON_MISC_H

// Define a macro which returns the "pretty" name of function
// It should include names of class and functions
//#ifdef __linux
//#define FUNCTION_NAME() (__PRETTY_FUNCTION__)
//#elif defined(__APPLE__)
//#define FUNCTION_NAME() (__PRETTY_FUNCTION__)
//#elif defined(__WINDOWS__)
//#error "Define your own FUNCTION_NAME() macro on Windows"
//#else
//#error "What's your operating system?"
//#endif

//#define FUNCTION_NAME() ""

// Safe Release Ptr
// Warning!!! You should put ";" after this macros!!!
#define SAFE_RELEASE_PTR(x) do{if(x) {delete x; x = NULL;}}while(0)


#endif // #ifndef SEHOON_MISC_H
