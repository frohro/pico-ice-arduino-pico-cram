Import("env")

# Assemble the PIO programs into headers before build
import os, subprocess, shutil, re

project_dir = env.get("PROJECT_DIR")
pio_srcs = [
    os.path.join(project_dir, "src", "pio", "syscfg_spi_tx.pio"),
    os.path.join(project_dir, "src", "pio", "clk_out.pio"),
]


def strip_pio_version_field(header_path: str):
    try:
        with open(header_path, "r", encoding="utf-8") as f:
            content = f.read()
        # Remove any designated initializer for .pio_version = ...,
        content = re.sub(r"\n\s*\.pio_version\s*=\s*[^,}\n]+,?\s*\n", "\n", content)
        # Clean up a trailing comma before closing brace of struct initializer
        content = re.sub(r",\s*\n\s*};", "\n};", content)
        with open(header_path, "w", encoding="utf-8") as f:
            f.write(content)
    except Exception as e:
        print("[pioasm] Warning: failed to sanitize header:", header_path, e)


for pio_src in pio_srcs:
    out_h  = os.path.join(project_dir, "include", os.path.basename(pio_src) + ".h")
    if os.path.exists(pio_src):
        os.makedirs(os.path.dirname(out_h), exist_ok=True)
        # Prefer the framework-bundled pioasm if available
        candidates = [
            shutil.which("pioasm"),
            os.path.expanduser("~/.platformio/packages/framework-arduino-pico/pico-sdk/tools/pioasm/build/pioasm"),
            os.path.expanduser("~/.platformio/packages/framework-arduino-pico/pico-sdk/tools/pioasm"),
        ]
        tool = next((c for c in candidates if c and os.path.isfile(c) and os.access(c, os.X_OK)), None)
        if not tool:
            print("[pioasm] Error: pioasm tool not found. Add it to PATH.")
            continue
        cmd = [tool, "-o", "c-sdk", pio_src, out_h]
        print("[pioasm]", " ".join(cmd))
        try:
            subprocess.check_call(cmd)
        except Exception as e:
            print("[pioasm] c-sdk failed, retrying with -o c:", e)
            cmd = [tool, "-o", "c", pio_src, out_h]
            print("[pioasm]", " ".join(cmd))
            subprocess.check_call(cmd)
        # Sanitize header to remove fields unsupported by the framework's pico-sdk
        strip_pio_version_field(out_h)
    else:
        print("[pioasm] No PIO source found:", pio_src)
