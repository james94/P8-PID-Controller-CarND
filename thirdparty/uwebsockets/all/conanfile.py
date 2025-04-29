from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout
from conan.tools.files import get, copy

class UWebSocketsConan(ConanFile):
    name = "uwebsockets"
    version = "e94b6e1"
    
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"
    exports_sources = "conandata.yml"
    
    def layout(self):
        cmake_layout(self)

    def source(self):
        get(self, **self.conan_data["sources"][self.version], strip_root=True)
        # self.tools.get(**self.conan_data["sources"][self.version])
        # self.tools.rename(f"uWebSockets-{self.version}", "src")

    def requirements(self):
        self.requires("openssl/3.1.1")
        self.requires("libuv/1.44.2", override=True)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
        self.copy("*.h", src="src", dst="include")
        self.copy("*.a", src="", dst="lib", keep_path=False)
        self.copy("*.so", src="", dst="lib", keep_path=False)

    def package_info(self):
        self.cpp_info.libs = ["uWS"]
