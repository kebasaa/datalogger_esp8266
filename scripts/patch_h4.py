"""Install the project's H4 hardening patches with standard git apply.

CLI use:
    python scripts/patch_h4.py --deps-root /path/to/libraries
PlatformIO invokes this file as a pre-build script and supplies the matching
.pio/libdeps/<environment> directory automatically.
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

try:
    Import("env")  # PlatformIO/SCons injects both Import and env.
except NameError:
    env = None

PATCHES = (
    ("H4Plugins", "3.5.3", "h4plugins.patch", "AP lifecycle and command-input hardening"),
    ("H4AsyncWebServer", "0.0.10", "h4asyncwebserver.patch", "bounded HTTP parser"),
    ("H4AsyncTCP", "0.0.23", "h4asynctcp.patch", "malformed URL guard"),
    ("H4AsyncMQTT", "1.0.0-rc11", "h4asyncmqtt.patch", "empty packet guard"),
    ("ArmadilloHTTP", "0.1.8", "armadillohttp.patch", "bounded TCP header parser"),
)
PATCH_DIR = (
    Path(env.subst("$PROJECT_DIR")) / "scripts" / "patches"
    if env is not None
    else Path(__file__).with_name("patches")
)

# The retired patch_h4plugins.py made three AP-related edits directly in the
# PlatformIO dependency cache, without the H4HARDEN markers used by the portable
# patches. They cannot be safely combined with the complete Git patch, so a
# positively identified cache is discarded and rebuilt by PlatformIO.
LEGACY_H4PLUGINS_SIGNATURES = (
    ("AP dashboard", "_uiAdd(h4pTag(),H4P_UI_TEXT", "_uiAdd(boardTag(),H4P_UI_TEXT", "_uiAdd(NBootsTag(),H4P_UI_TEXT"),
    ("AP password", "DATALOGGER_AP_PASSWORD", "WiFi.softAP(CSTR(h4p[deviceTag()]), DATALOGGER_AP_PASSWORD)"),
    ("fallback delay", "h4.once(", "_lostIP()"),
)


def git(target: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", "-C", str(target), *args],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def require_git_tree(target: Path) -> None:
    result = git(target, "rev-parse", "--is-inside-work-tree")
    if result.returncode or result.stdout.strip() != "true":
        raise RuntimeError("%s is not a Git working tree" % target)


def has_legacy_patch(target: Path, library: str) -> bool:
    """Return true only for the known pre-portable H4Plugins patch state."""
    if library != "H4Plugins":
        return False

    sources = (
        target / "src" / "H4P_WiFi.cpp",
        target / "src" / "H4P_WiFiAP.cpp",
    )
    try:
        text = "\n".join(source.read_text(encoding="utf-8") for source in sources)
    except OSError:
        return False

    # Never classify the current portable patch as legacy, even if a later
    # unrelated edit makes its reverse check fail.
    if "H4HARDEN:" in text:
        return False
    return all(all(needle in text for needle in signature[1:]) for signature in LEGACY_H4PLUGINS_SIGNATURES)


def remove_legacy_target(deps_root: Path, target: Path, library: str) -> None:
    """Delete only a manifest-listed direct child of the active libdeps root."""
    root = deps_root.resolve()
    resolved = target.resolve()
    if resolved.parent != root or resolved.name != library:
        raise RuntimeError("Refusing to remove unexpected dependency path: %s" % target)
    shutil.rmtree(resolved)


def install_patch(deps_root: Path, library: str, version: str, filename: str, reason: str) -> None:
    target = deps_root / library
    patch = PATCH_DIR / filename
    if not target.is_dir():
        raise RuntimeError("Pinned %s %s is missing at %s" % (library, version, target))
    if not patch.is_file():
        raise RuntimeError("Patch file is missing: %s" % patch)
    require_git_tree(target)

    # An exact reverse check is the idempotence test. It is intentionally first:
    # a patched tree is expected to be dirty from Git's perspective.
    reverse = git(target, "apply", "--reverse", "--check", str(patch))
    if reverse.returncode == 0:
        print("[h4] already applied: %s (%s)" % (filename, reason))
        return

    status = git(target, "status", "--porcelain")
    if status.returncode:
        raise RuntimeError("Cannot inspect %s: %s" % (target, status.stderr.strip()))
    if status.stdout.strip():
        if has_legacy_patch(target, library):
            remove_legacy_target(deps_root, target, library)
            environment = env.subst("$PIOENV") if env is not None else "<environment>"
            raise RuntimeError(
                "Removed legacy inline-patched %s cache at %s. "
                "Run 'pio run -e %s' again so PlatformIO re-downloads it and applies %s."
                % (library, target, environment, filename)
            )
        raise RuntimeError(
            "%s has unexpected local changes; refuse to mix them with %s"
            % (target, filename)
        )

    check = git(target, "apply", "--check", str(patch))
    if check.returncode:
        raise RuntimeError(
            "%s does not apply to pinned %s %s:\n%s"
            % (filename, library, version, check.stderr.strip())
        )
    apply = git(target, "apply", str(patch))
    if apply.returncode:
        raise RuntimeError("Failed to apply %s:\n%s" % (filename, apply.stderr.strip()))
    print("[h4] applied: %s (%s)" % (filename, reason))


def install_all(deps_root: Path) -> None:
    deps_root = deps_root.resolve()
    for patch in PATCHES:
        install_patch(deps_root, *patch)


def cli() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deps-root", required=True, type=Path)
    install_all(parser.parse_args().deps_root)


if env is not None:
    install_all(Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env.subst("$PIOENV"))
elif __name__ == "__main__":
    cli()
