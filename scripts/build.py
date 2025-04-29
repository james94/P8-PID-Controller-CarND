#!/usr/bin/env python3
import subprocess
import shutil
import os
import argparse
from pathlib import Path

def setup_conan():
    # "--force"
    subprocess.run(["conan", "profile", "detect", "--force"], check=True)
    # subprocess.run(["conan", "install", ".", "--output-folder=build", "--build=missing",
    #                 "-s", "compiler.libcxx=libstdc++11"], check=True)

# "--settings=build_type=Debug"
def install_deps_via_conan(profile):
    subprocess.run(["conan", "install", ".", 
                    "--output-folder=build", 
                    "--build=missing",
                    "--profile=" + profile,
                    "--settings=build_type=Debug"], check=True)

def build_proj_via_conan(profile):
    subprocess.run(["conan", "build", ".", 
                    "--output-folder=build", 
                    "--build=missing",
                    "--profile=" + profile], check=True)

# def build_project(build_type="Debug", clean=False):
def build_project(profile="default"):
    build_dir = Path("build")
    if build_dir.exists():
        shutil.rmtree(build_dir)
    
    build_dir.mkdir(exist_ok=True)

    # profile = profile

    setup_conan()

    # cmake_cmd = [
    #     "cmake",
    #     f"-DCMAKE_BUILD_TYPE={build_type}",
    #     "-S", ".",
    #     "-B", "build",
    #     "-DCMAKE_TOOLCHAIN_FILE=build/build/Release/generators/conan_toolchain.cmake"
    # ]

    # build_cmd = [
    #     "cmake",
    #     "--build", "build",
    #     "--config", build_type,
    #     "-j", str(os.cpu_count())
    # ]

    try:
        # subprocess.run(cmake_cmd, check=True)
        # subprocess.run(build_cmd, check=True)

        install_deps_via_conan(profile=profile)

        build_proj_via_conan(profile=profile)

        print("Build successful!")
    except subprocess.CalledProcessError as e:
        print(f"\nBuild failed: {e}")
        exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # parser.add_argument("-c", "--clean", action="store_true", help="Clean build directory")
    # parser.add_argument("-r", "--release", action="store_true", help="Build in Release mode")
    parser.add_argument("--profile", default=os.path.expanduser("~/.conan2/profiles/default"), help="Conan profile name")
    args = parser.parse_args()

    # build_type = "Release" if args.release else "Debug"
    # build_project(build_type, args.clean)
    build_project(profile=args.profile)
