import os
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
RECIPE_PATH = SCRIPT_DIR.parent / "thirdparty/uwebsockets/all"

def create_package():
    # Export the recipe
    subprocess.run([
        "conan", "export", ".",
        "--user=udacity", "--channel=stable"
    ], check=True, cwd=RECIPE_PATH)
    
    # Create the package
    subprocess.run([
        "conan", "create", ".",
        f"--profile={os.path.expanduser('~/.conan2/profiles/default')}",
        "--build=missing"
    ], check=True, cwd=RECIPE_PATH)

# Assuming this script runs from project root folder
if __name__ == "__main__":
    create_package()
