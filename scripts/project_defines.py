"""Publish the handful of main.cpp settings that H4Plugins must also be built with.

Most settings in src/main.cpp only ever need to be seen by the project's own
code, so dl_config_map.h translating them into internal macro names is enough.
Two of them are different: H4Plugins compiles its own .cpp files as separate
translation units that never include a project header, so a setting defined
only in dl_config_map.h reaches our code and not the library.

That is not merely a missing feature. H4P_USE_WIFI_AP guards a member of class
H4P_WiFi (the captive-portal DNSServer) as well as whole functions, so a
mismatch gives the project and the library two different layouts for the same
class - undefined behaviour, and an access point that never starts because
_startAP() was never compiled.

Passing them as global -D flags from here keeps src/main.cpp the single place a
user edits, while guaranteeing both halves of the build agree.

CLI use (prints what would be defined, changes nothing):
    python scripts/project_defines.py
PlatformIO invokes this file as a pre-build script.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

try:
    Import("env")  # PlatformIO/SCons injects both Import and env.
except NameError:
    env = None

PROJECT_DIR = (
    Path(env.subst("$PROJECT_DIR")) if env is not None else Path(__file__).resolve().parent.parent
)
MAIN_CPP = PROJECT_DIR / "src" / "main.cpp"

# (setting in main.cpp, macro the library expects, default, is it a string)
# Keep this list as short as it can possibly be: every entry is a setting that
# has to be kept consistent in two places.
SETTINGS = (
    ("WIFI_ACCESS_POINT_MODE", "H4P_USE_WIFI_AP", "1", False),
    ("HOTSPOT_PASSWORD", "DATALOGGER_AP_PASSWORD", "datalogger", True),
)


def read_setting(source: str, name: str, is_string: bool):
    """Return the value of an active `#define <name> <value>` line, else None.

    Deliberately simple: it matches the one-define-per-line style main.cpp is
    written in, and ignores commented-out lines so that toggling a setting off
    with `//` behaves the way the file's own comments say it does.
    """
    pattern = re.compile(
        r"^[ \t]*#[ \t]*define[ \t]+" + re.escape(name) + r"[ \t]+(.+?)[ \t]*(?://.*)?$",
        re.MULTILINE,
    )
    match = pattern.search(source)
    if not match:
        return None
    value = match.group(1).strip()
    if is_string:
        if len(value) >= 2 and value[0] == '"' and value[-1] == '"':
            return value[1:-1]
        return None  # not a plain literal; fall back to the default
    return value if re.fullmatch(r"-?\d+", value) else None


def resolve():
    """Work out the (macro, value, is_string) triples this build needs."""
    try:
        source = MAIN_CPP.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        raise RuntimeError("cannot read %s: %s" % (MAIN_CPP, exc))

    resolved = []
    for setting, macro, default, is_string in SETTINGS:
        value = read_setting(source, setting, is_string)
        if value is None:
            value = default
            print(
                "[defines] %s not set in main.cpp, using default %s"
                % (setting, default)
            )
        resolved.append((macro, value, is_string))
    return resolved


def main():
    resolved = resolve()

    if env is None:
        for macro, value, is_string in resolved:
            shown = '"%s"' % value if is_string else value
            print("-D%s=%s" % (macro, shown))
        return

    defines = []
    for macro, value, is_string in resolved:
        # StringifyMacro handles the quoting a string literal needs to survive
        # being passed through the command line to the compiler.
        defines.append((macro, env.StringifyMacro(value) if is_string else value))

    # Append to the global environment, not the project one: the whole point is
    # that .pio/libdeps sources see these too.
    try:
        global_env = DefaultEnvironment()  # noqa: F821 - injected by SCons
    except NameError:
        global_env = env
    global_env.Append(CPPDEFINES=defines)
    env.Append(CPPDEFINES=defines)

    print(
        "[defines] shared with libraries: "
        + ", ".join("%s=%s" % (m, v) for m, v, _ in resolved)
    )


try:
    main()
except RuntimeError as exc:
    print("[defines] %s" % exc, file=sys.stderr)
    sys.exit(1)
