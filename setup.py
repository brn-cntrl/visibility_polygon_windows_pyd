from setuptools import setup, Extension
from pybind11.setup_helpers import Pybind11Extension, build_ext
import sys

clipper2_sources = [
    "clipper.engine.cpp",
    "clipper.offset.cpp",
    "clipper.rectclip.cpp",
]

extra_link_args = []
if sys.platform == "darwin":
    extra_link_args = [
        "-Wl,-rpath,/usr/lib",  
        "-Wl,-rpath,/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/lib",
    ]

ext_modules = [
    Pybind11Extension(
        "visibility_polygon",
        sources=[
            "visibility_bindings.cpp",
            "VisibilityPolygon.cpp",
        ] + clipper2_sources,
        include_dirs=[
            ".",  
            "clipper2",  
        ],
        extra_compile_args=[
            "-std=c++17",
            "-O3",
            "-fPIC",
        ] + (["-arch", "arm64"] if sys.platform == "darwin" else []),
        extra_link_args=extra_link_args,  
        language="c++",
    ),
]

setup(
    name="visibility_polygon",
    version="0.1.0",
    author="Brian Cantrell",
    description="Visibility polygon computation with Clipper2",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
)