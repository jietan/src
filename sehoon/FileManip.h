#ifndef SEHOON_FILEMANIP_H
#define SEHOON_FILEMANIP_H

#include "boost/filesystem.hpp"
namespace bf = boost::filesystem;

namespace sehoon {
    namespace filemanip {

        bool exists(const char* const filename) {
            return bf::exists(filename);
        }

        string parent_path(const char* const filename) {
            bf::path p(filename);
            return p.parent_path().string();
        }

        string stem(const char* const filename) {
            bf::path p(filename);
            return p.stem();
      
        }

        string extension(const char* const filename) {
            bf::path p(filename);
            return p.extension();
        }

        string stemExt(const char* const filename) {
            bf::path p(filename);
            return p.filename();
        }

    } // namespace filemanip
} // namespace sehoon

#endif
