"""Minify and gzip the web UI sources into the LittleFS image.

Reads web/src/{index.html,app.css,app.js} and writes data/h4/{app.htm,app.css,
app.js}.gz. Only the compressed files are emitted: shipping both would just be
a second way for the device to serve something stale.

Why the size cap matters. H4AW_HTTPHandler::_serveFile pushes a whole file into
H4AsyncClient::TX in one synchronous loop, and everything lwIP will not accept
immediately is queued as a malloc'd copy. The TCP send buffer cannot refill
during that loop, because the ACKs that would drain it are processed by the H4
scheduler only after the loop returns. So roughly (file size - send buffer)
bytes sit in the heap at once, on a device that refuses all new connections
below about 13 KB free. Hence one small file per request, and a hard ceiling.

Only the standard library is used, matching scripts/patch_h4.py: no npm, no pip.
The minifier is deliberately conservative - it is not a parser, so it only does
transformations that are safe without understanding the grammar.

CLI use:
    python scripts/build_web.py
PlatformIO invokes this file as a pre-build script.
"""
from __future__ import annotations

import gzip
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
SRC_DIR = PROJECT_DIR / "web" / "src"
OUT_DIR = PROJECT_DIR / "data" / "h4"

# Ceiling for a single asset, derived from the TCP send buffer rather than
# picked round.
#
# platformio.ini builds with PIO_FRAMEWORK_ARDUINO_LWIP2_HIGHER_BANDWIDTH_LOW_FLASH,
# which selects the MSS-1460 lwIP2 variant, so TCP_SND_BUF = 2 * TCP_MSS = 2920
# bytes. Whatever does not fit in that buffer when _serveFile runs its loop is
# what ends up queued on the heap, so a file of size S costs roughly
# max(0, S - 2920) bytes of transient heap.
#
# The CSS and JS are now INLINED into the page, so there is exactly one asset and
# one request. That is not a size optimisation, it is a connection one: lwIP is
# built with MEMP_NUM_TCP_PCB 5, so the whole device has five TCP connections.
# Fetching the page, stylesheet, script, favicon and logo in parallel used most
# of them at once, and anything that then wanted a socket - including Android's
# captive-portal probe - found none. One file means one connection.
#
# The ceiling rises accordingly (the payload that used to be three files is now
# one), but the TOTAL budget does not move: that is the number that actually
# bounds transient heap, and it is unchanged at 10240.
MAX_GZ_BYTES = 9216

# Guard against the page creeping up over time. Unchanged from when the UI was
# split across three files - inlining must not become an excuse to grow.
MAX_TOTAL_GZ_BYTES = 10240

# Sources inlined into index.html before minifying, replacing the <link> and
# <script src> that used to fetch them separately.
INLINE_CSS = "app.css"
INLINE_JS = "app.js"

# (source, output, kind)
ASSETS = (
    ("index.html", "app.htm.gz", "html"),
)


def strip_block_comments(text: str) -> str:
    """Remove /* ... */ comments.

    Skips anything inside a quoted string so a URL or a regex containing the
    delimiters survives. Not a tokenizer, but it does track quoting, which is
    the failure mode a naive regex actually hits.
    """
    out = []
    i, n = 0, len(text)
    quote = None
    while i < n:
        c = text[i]
        if quote:
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(text[i + 1])
                i += 2
                continue
            if c == quote:
                quote = None
            i += 1
            continue
        if c in "\"'`":
            quote = c
            out.append(c)
            i += 1
            continue
        if c == "/" and i + 1 < n and text[i + 1] == "*":
            end = text.find("*/", i + 2)
            i = n if end == -1 else end + 2
            continue
        out.append(c)
        i += 1
    return "".join(out)


def minify_css(text: str) -> str:
    text = strip_block_comments(text)
    text = re.sub(r"\s+", " ", text)
    text = re.sub(r"\s*([{};:,>~])\s*", r"\1", text)
    text = text.replace(";}", "}")
    return text.strip()


def minify_js(text: str) -> str:
    """Drop comments and leading indentation.

    Line joining is NOT attempted: without a parser that is where automatic
    semicolon insertion turns working code into silently broken code. Gzip
    recovers most of what the whitespace would have cost anyway.
    """
    text = strip_block_comments(text)
    lines = []
    for line in text.split("\n"):
        stripped = line.strip()
        # Only drop a // comment that starts the line. Anything later could be
        # inside a string or a regex, and telling those apart needs a parser.
        if stripped.startswith("//"):
            continue
        if stripped:
            lines.append(stripped)
    return "\n".join(lines)


def minify_html(text: str) -> str:
    text = re.sub(r"<!--(?!\[if).*?-->", "", text, flags=re.DOTALL)
    # Collapse whitespace only between tags, so text content is left alone.
    text = re.sub(r">\s+<", "><", text)
    # A SPACE, not "". This line used to delete the newline and the indentation
    # around it, which welded the last word of one line to the first word of the
    # next anywhere a sentence wrapped: "A session\n keeps" shipped as
    # "A sessionkeeps", and likewise "ofknown", "highconcentration", "dewpoint".
    # The substitution above has already removed the whitespace that sits
    # between tags, so the only thing reaching here is prose, where the space is
    # exactly what belongs.
    return re.sub(r"[ \t]*\n[ \t]*", " ", text).strip()


MINIFIERS = {"html": minify_html, "css": minify_css, "js": minify_js}


def build():
    if not SRC_DIR.is_dir():
        raise RuntimeError("no web sources at %s" % SRC_DIR)
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    total = 0
    problems = []
    for name, out_name, kind in ASSETS:
        src = SRC_DIR / name
        if not src.is_file():
            problems.append("missing source %s" % src)
            continue

        raw = src.read_text(encoding="utf-8")
        text = MINIFIERS[kind](raw)

        if kind == "html":
            # Guard against the word-welding bug coming back. Any sentence in
            # index.html that wraps across a line break used to lose the space at
            # the join, which is invisible in the source and only shows up as a
            # typo on the rendered page. Take the words either side of every line
            # break in the original prose and check the pair still has a
            # separator after minification.
            for a, b in re.findall(r"(\w+)[ \t]*\n[ \t]*(\w+)", raw):
                if (a + b) in text:
                    problems.append(
                        "minify_html welded '%s' and '%s' into '%s%s' - the "
                        "newline substitution must produce a space" % (a, b, a, b)
                    )
                    break

            # Inline AFTER minifying the HTML, never before.
            #
            # minify_html collapses every newline, while minify_js deliberately
            # keeps them - without a real parser, joining JS lines is where
            # automatic semicolon insertion turns working code into silently
            # broken code (see minify_js). Inlining first would hand the script to
            # the HTML minifier and lose exactly those newlines. Both tags are
            # single-line and contain no whitespace runs, so they survive
            # minification intact and can be substituted afterwards.
            css_src = SRC_DIR / INLINE_CSS
            js_src = SRC_DIR / INLINE_JS
            if not css_src.is_file() or not js_src.is_file():
                problems.append("missing %s or %s to inline" % (css_src, js_src))
                continue
            css_raw = css_src.read_text(encoding="utf-8")
            js_raw = js_src.read_text(encoding="utf-8")

            # Refuse to ship a page whose script cannot parse.
            #
            # A stray "*/" left outside a comment block shipped once and cost a
            # full test round to find: the page rendered perfectly, but the whole
            # script failed to parse, so nothing ever polled the device and the UI
            # sat on "Connecting..." forever. Nothing in the build noticed - the
            # minifier is regex-based and happily minified the broken file, and
            # the device cannot tell a working page from a broken one.
            #
            # There is no JS parser here (no node on the build machine), so check
            # the two structural things that silently destroy a whole file.
            for name, src in (("app.css", css_raw), ("app.js", js_raw)):
                depth, i, stray = 0, 0, []
                while i < len(src) - 1:
                    two = src[i:i + 2]
                    if depth == 0 and two == "/*":
                        depth, i = 1, i + 2
                        continue
                    if depth == 1 and two == "*/":
                        depth, i = 0, i + 2
                        continue
                    if depth == 0 and two == "*/":
                        stray.append(src.count("\n", 0, i) + 1)
                    i += 1
                if depth:
                    problems.append("%s has an unclosed /* comment" % name)
                if stray:
                    problems.append(
                        "%s has a stray '*/' outside any comment at line %d - the"
                        " file will not parse and the page will load but do"
                        " nothing" % (name, stray[0])
                    )
            if js_raw.count("{") != js_raw.count("}"):
                problems.append(
                    "app.js brace mismatch: %d '{' vs %d '}'"
                    % (js_raw.count("{"), js_raw.count("}"))
                )
            if problems:
                continue

            css = MINIFIERS["css"](css_raw)
            js = MINIFIERS["js"](js_raw)

            css_tag = '<link rel="stylesheet" href="/app.css">'
            js_tag = '<script src="/app.js"></script>'
            if css_tag not in text or js_tag not in text:
                problems.append(
                    "index.html does not contain the exact tags build_web.py"
                    " inlines (%s / %s)" % (css_tag, js_tag)
                )
                continue
            text = text.replace(css_tag, "<style>%s</style>" % css)
            text = text.replace(js_tag, "<script>\n%s\n</script>" % js)

        minified = text.encode("utf-8")
        # mtime=0 keeps the output byte-identical between builds, so an
        # unchanged UI does not show up as a changed file in git.
        packed = gzip.compress(minified, compresslevel=9, mtime=0)

        if not packed:
            problems.append("%s produced an empty archive" % name)
            continue

        over = len(packed) > MAX_GZ_BYTES
        if over:
            problems.append(
                "%s is %d bytes gzipped, %d over the %d byte limit"
                % (out_name, len(packed), len(packed) - MAX_GZ_BYTES, MAX_GZ_BYTES)
            )

        state = "OVER LIMIT"
        if not over:
            dest = OUT_DIR / out_name
            # Only rewrite on change, so uploadfs is not tricked into thinking
            # the filesystem image is newer than it is.
            if not dest.is_file() or dest.read_bytes() != packed:
                dest.write_bytes(packed)
                state = "written"
            else:
                state = "unchanged"
            total += len(packed)

        # Always report every file before failing: seeing all three sizes is
        # what tells you which one to trim.
        print(
            "[web] %-12s %5d raw -> %5d min -> %5d gz  (%s)"
            % (out_name, len(raw.encode("utf-8")), len(minified), len(packed), state)
        )

    # Assets this script used to emit but no longer does. Left behind they would
    # still be uploaded by uploadfs, taking filesystem space and - worse - being
    # served by the library's file catch-all as a stale copy of a UI that has
    # since been inlined.
    for orphan in ("app.css.gz", "app.js.gz"):
        stale = OUT_DIR / orphan
        if stale.is_file():
            stale.unlink()
            print("[web] removed stale %s (now inlined into app.htm.gz)" % orphan)

    if total > MAX_TOTAL_GZ_BYTES:
        problems.append(
            "the UI totals %d bytes gzipped, over the %d byte budget"
            % (total, MAX_TOTAL_GZ_BYTES)
        )

    if problems:
        raise RuntimeError("; ".join(problems))

    print(
        "[web] total %d of %d bytes gzipped across %d files"
        % (total, MAX_TOTAL_GZ_BYTES, len(ASSETS))
    )


try:
    build()
except RuntimeError as exc:
    # Fail the build rather than let a device be flashed with a filesystem that
    # serves a truncated or missing UI.
    print("[web] ERROR: %s" % exc, file=sys.stderr)
    sys.exit(1)
