from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps
try:
    from conan.tools.layout import cmake_layout
except ImportError:  # Conan < 2.0
    cmake_layout = None


class MADConan(ConanFile):
    name = "MAD"
    version = "0.1"
    license = "MIT"
    url = "https://example.com/MAD"
    description = "Magnetic anomaly detection framework"
    settings = "os", "compiler", "build_type", "arch"
    exports_sources = "*"

    def requirements(self):
        self.requires("glfw/3.4")
        self.requires("glew/2.2.0")
        self.requires("glu/system")
        self.requires("glm/cci.20230113")
        self.requires("imgui/cci.20230105+1.89.2.docking")
        self.requires("libcurl/8.9.1")
        self.requires("libjpeg/9f", override=True)
        self.requires("nanoflann/1.6.0")
        self.requires("opengl/system")
        self.requires("eigen/3.4.0")
        self.requires("fmt/10.2.1")
        self.requires("spdlog/1.12.0")
        self.requires("nlohmann_json/3.11.2")
        self.requires("glad/0.1.36")
        self.requires("gtest/cci.20210126")

    def layout(self):
        if cmake_layout:
            cmake_layout(self)
        else:
            self.folders.source = "."
            self.folders.build = "build"

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["CMAKE_CXX_STANDARD"] = "20"
        tc.generate()
        deps = CMakeDeps(self)
        deps.generate()
