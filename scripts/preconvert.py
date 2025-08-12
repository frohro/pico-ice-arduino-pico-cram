Import("env")

# Run convert_binaries.py before building to ensure headers exist
# Works both when run from project root or invoked by PlatformIO
import os
import subprocess

project_dir = env.get("PROJECT_DIR")
script = os.path.join(project_dir, "convert_binaries.py")

if os.path.exists(script):
    print("[preconvert] Converting .bin files to headers...")
    try:
        # Use the same Python used by PlatformIO
        python_exe = env.get("PYTHONEXE", "python3")
        subprocess.check_call([python_exe, script], cwd=project_dir)
    except subprocess.CalledProcessError as e:
        print("[preconvert] Conversion failed:", e)
else:
    print("[preconvert] convert_binaries.py not found at:", script)
