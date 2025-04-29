from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout

class PIDControllerRecipe(ConanFile):
    name = "pid_controller"
    version = "1.0.0"

    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"


    def requirements(self):
        self.requires("nlohmann_json/3.11.2")
        self.requires("openssl/3.1.1")
        self.requires("libuv/1.44.2")
        self.requires("gtest/1.11.0", options={"shared": True})
        # self.requires("uwebsockets/20.71.0")
        self.requires("uwebsockets/e94b6e1@udacity/stable")

    def build_requirements(self):
        self.tool_requires("cmake/3.25.3")

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
