Import("env")
import os
import subprocess

def generate_uf2(source, target, env):
    firmware_hex = os.path.join(env.subst("$BUILD_DIR"), "firmware.hex")
    firmware_uf2 = os.path.join(env.subst("$BUILD_DIR"), "firmware.uf2")
    uf2conv = os.path.join(
        env.PioPlatform().get_package_dir("framework-arduinoadafruitnrf52"),
        "tools", "uf2conv", "uf2conv.py"
    )
    subprocess.check_call([
        env.subst("$PYTHONEXE"), uf2conv,
        firmware_hex, "-c", "-f", "0xADA52840", "-o", firmware_uf2
    ])
    print(f"UF2 created: {firmware_uf2}")

env.AddPostAction("$BUILD_DIR/firmware.hex", generate_uf2)
