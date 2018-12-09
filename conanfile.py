from conans import ConanFile, CMake

class RexConan(ConanFile):
    generators = "cmake"
    settings = "os", "arch", "compiler", "build_type"

    def requirements(self):
        self.requires("zlib/1.2.9@conan/stable")
        self.requires("boost/1.64.0@conan/stable")
        self.requires("libxml2/2.9.8@bincrafters/stable")
        #self.requires("curses/6.1@conan/stable")
        #self.requires("llvm/5.0@conan/stable")
        #self.requires("clang/5.0@conan/stable")
