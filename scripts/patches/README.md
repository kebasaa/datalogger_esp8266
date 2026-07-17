# H4 hardening patches

These are standard unified Git patches for the dependency revisions pinned in
\`platformio.ini\`. Apply one manually from a checkout of that library:

\`\`\`powershell
git -C C:\path\to\H4Plugins apply C:\path\to\project\scripts\patches\h4plugins.patch
\`\`\`

| Library | Pinned version | Patch |
| --- | --- | --- |
| H4Plugins | 3.5.3 | \`h4plugins.patch\` |
| H4AsyncWebServer | 0.0.10 | \`h4asyncwebserver.patch\` |
| H4AsyncTCP | 0.0.23 | \`h4asynctcp.patch\` |
| H4AsyncMQTT | 1.0.0-rc11 | \`h4asyncmqtt.patch\` |
| ArmadilloHTTP | 0.1.8 | \`armadillohttp.patch\` |

For all patches at once, use:

\`\`\`powershell
python scripts/patch_h4.py --deps-root C:\path\to\platformio\libdeps\d1_mini_pro
\`\`\`

The installer uses \`git apply --check\` before applying and
\`git apply --reverse --check\` to detect an already-applied patch. It refuses
to mix a new patch with unrelated local modifications.

## Upgrading from the retired inline patcher

If the installer recognizes the old unmarked H4Plugins AP dashboard,
fallback-delay, and AP-password edits, it removes only that generated
\`H4Plugins\` checkout and stops the current build. Run the build again to let
PlatformIO download the pristine pinned library and apply \`h4plugins.patch\`:

\`\`\`powershell
pio run -e d1_mini_pro
\`\`\`

The installer never removes a dependency with unknown local changes.
