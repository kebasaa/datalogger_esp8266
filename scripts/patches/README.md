# H4 hardening patches

These are standard unified Git patches for the dependency revisions pinned in
`platformio.ini`. Apply one manually from a checkout of that library:

```powershell
git -C C:\path\to\H4Plugins apply C:\path\to\project\scripts\patches\h4plugins.patch
```

| Library | Pinned version | Patch |
| --- | --- | --- |
| H4 | 4.0.10 | `h4.patch` |
| H4Plugins | 3.5.8 | `h4plugins.patch` |
| H4AsyncWebServer | 0.0.13 | `h4asyncwebserver.patch` |
| H4AsyncTCP | 0.0.25 | `h4asynctcp.patch` |
| H4AsyncMQTT | 1.0.0-rc12 | `h4asyncmqtt.patch` |
| ArmadilloHTTP | 0.2.0 | `armadillohttp.patch` |
| H4Tools | 0.0.16 | `h4tools.patch` |

`patch_h4.py` compares these against the checkout in the dependency cache and
refuses to run if they disagree, so a moved pin is reported as a moved pin rather
than as a mysterious context mismatch.

It reads the **Git tag**, not `library.properties`, because upstream repeatedly
ships releases without bumping that file: H4AsyncWebServer 0.0.11 declares
`0.0.10`, and ArmadilloHTTP 0.2.0 declares `0.1.9`. The file is only a fallback
for a checkout that is not a Git tree.

For all patches at once, use:

```powershell
python scripts/patch_h4.py --deps-root C:\path\to\platformio\libdeps\d1_mini_pro
```

The installer uses `git apply --check` before applying and
`git apply --reverse --check` to detect an already-applied patch. It refuses
to mix a new patch with unrelated local modifications.

Every hunk is marked with an `// H4HARDEN:<tag>` comment. Those markers are load
bearing: `has_legacy_patch()` in `scripts/patch_h4.py` keys on them to tell a
patched tree apart from one carrying edits by the retired inline patcher. Keep
them when re-deriving a patch.

## Upgrading from the retired inline patcher

If the installer recognizes the old unmarked H4Plugins AP dashboard,
fallback-delay, and AP-password edits, it removes only that generated
`H4Plugins` checkout and stops the current build. Run the build again to let
PlatformIO download the pristine pinned library and apply `h4plugins.patch`:

```powershell
pio run -e d1_mini_pro
```

The installer never removes a dependency with unknown local changes.

## Upgrading between patch revisions

When a patch file itself changes (e.g. new hardening or resource fixes are folded
into an existing `<lib>.patch`), a build machine whose libdeps still carry the
*previous* version of that patch will make the tree dirty in a way the installer
does not recognize as the new patch. It then refuses to mix the old edits with
the new patch and stops the build with an "unexpected local changes" error rather
than corrupting the tree.

To upgrade, delete the affected generated dependency folder so PlatformIO
re-downloads the pristine pinned library, then rebuild so the current patch
applies to a clean tree:

```powershell
Remove-Item -Recurse -Force .pio\libdeps\d1_mini_pro\H4AsyncTCP
pio run -e d1_mini_pro
```

Repeat for each library whose patch changed (`H4AsyncTCP`, `H4AsyncWebServer`,
`H4AsyncMQTT`, `H4Plugins`). A freshly cloned checkout is unaffected.

## Regenerating a patch

The installer only *applies* patches; it never produces them. To change what a
patch does, edit the library sources in place and diff them back out:

```powershell
# 1. edit .pio\libdeps\d1_mini_pro\H4Plugins\src\*.cpp, keeping // H4HARDEN: markers
# 2. diff the whole library tree back into the patch file
git -C .pio\libdeps\d1_mini_pro\H4Plugins diff > scripts\patches\h4plugins.patch
```

Verify both directions before committing. The reverse check is what makes the
installer idempotent, and the forward check is what makes it work on a fresh
clone:

```powershell
# reverse: must succeed against the tree you just diffed
git -C .pio\libdeps\d1_mini_pro\H4Plugins apply --reverse --check scripts\patches\h4plugins.patch

# forward: must succeed against a pristine checkout
git clone .pio\libdeps\d1_mini_pro\H4Plugins $env:TEMP\h4p_test
git -C $env:TEMP\h4p_test checkout .
git -C $env:TEMP\h4p_test apply --check ..\..\scripts\patches\h4plugins.patch
```

Then delete the generated dependency folder and rebuild, so the new patch is
exercised against a clean tree exactly as a fresh clone would see it.

## Upgrading the pinned upstream version

Distinct from the case above, and more involved: this is when the version in
`platformio.ini` itself moves (e.g. h4plugins 3.5.3 to 3.5.6), not just the patch
contents.

Bumping a pin without re-deriving the patches is caught up front:

```
H4Plugins is version 3.5.6 on disk but h4plugins.patch was derived against 3.5.3.
```

That check matters because the alternative is worse than a build failure: a
hunk whose context still matches can apply cleanly to code whose meaning has
changed underneath it.

Steps:

1. Clone the library **outside** `.pio`. The libdeps checkouts are shallow
   single-tag clones, so `git rev-parse <newtag>` fails there and you cannot
   diff releases in place.
2. Diff the old and new tags for the files this project patches, to see what you
   are walking into. For H4Plugins that is `src/H4P_WiFi.cpp` and
   `src/H4P_WiFiAP.cpp`; for H4AsyncWebServer, `src/H4AsyncHandlers.cpp`,
   `src/H4AsyncWebServer.cpp`, `src/H4AT_HTTPHandlerWS.cpp` and
   `src/H4AW_HTTPHandlerSSE.cpp`.
3. Update the pin in `platformio.ini` and delete the generated dependency folder
   so PlatformIO fetches the new tag pristine.
4. Re-derive the patch: `git apply --3way scripts\patches\<lib>.patch`, resolve
   any conflicts, keep the `// H4HARDEN:` markers, then regenerate as above.
5. Update **both** the version string in the `PATCHES` tuple in
   `scripts/patch_h4.py` and the table at the top of this file. They are the only
   record of which upstream release a patch was derived against.
6. Re-sync `data/h4/` if the library's own `data/h4/` assets changed, and
   `pio run -t uploadfs` — a stale `h4.js` against a newer plugin leaves the
   legacy dashboard blank.
