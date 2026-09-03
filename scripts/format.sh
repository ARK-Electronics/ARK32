#!/usr/bin/env bash
# Format (or check) AM32 application / MCU port C sources with clang-format.
#
# Usage:
#   scripts/format.sh              # rewrite files in place (make format)
#   scripts/format.sh --check      # fail if any file would change (CI)
#   scripts/format.sh --changed    # only files changed vs git merge-base/HEAD
#   scripts/format.sh --check --changed
#
# Style config: repo-root .clang-format
# Excludes vendor HAL (Drivers), CMSIS, generated DroneCAN DSDL, libcanard, etc.
#
# Requires clang-format 22.1.5 (same pin as CI). Distro packages (Ubuntu 18.x)
# produce a different AST and will fail the PR check. Override the binary with
# CLANG_FORMAT=... if it reports that version; otherwise a repo-local venv is
# bootstrapped under tools/clang-format-venv (gitignored).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

CLANG_FORMAT_PIN=22.1.5
CF_VENV="$ROOT/tools/clang-format-venv"

CHECK=0
CHANGED_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --check) CHECK=1 ;;
    --changed|--diff-only) CHANGED_ONLY=1 ;;
    -h|--help)
      sed -n '2,16p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown option: $arg" >&2
      echo "Usage: $0 [--check] [--changed]" >&2
      exit 2
      ;;
  esac
done

clang_format_version() {
  "$1" --version 2>/dev/null | head -1 || true
}

clang_format_is_pin() {
  [[ -x "$1" ]] && [[ "$(clang_format_version "$1")" == *"$CLANG_FORMAT_PIN"* ]]
}

ensure_clang_format() {
  local c cf
  if [[ -n "${CLANG_FORMAT:-}" ]]; then
    if clang_format_is_pin "$CLANG_FORMAT"; then
      printf '%s\n' "$CLANG_FORMAT"
      return 0
    fi
    echo "CLANG_FORMAT=$CLANG_FORMAT is not clang-format $CLANG_FORMAT_PIN ($(clang_format_version "$CLANG_FORMAT"))." >&2
    return 1
  fi
  for c in "$CF_VENV/bin/clang-format" "$HOME/.local/bin/clang-format" "$(command -v clang-format 2>/dev/null || true)"; do
    if clang_format_is_pin "$c"; then
      printf '%s\n' "$c"
      return 0
    fi
  done
  echo "clang-format $CLANG_FORMAT_PIN not found (CI pin). Bootstrapping $CF_VENV" >&2
  python3 -m venv "$CF_VENV"
  "$CF_VENV/bin/pip" install -q "clang-format==$CLANG_FORMAT_PIN"
  cf="$CF_VENV/bin/clang-format"
  if ! clang_format_is_pin "$cf"; then
    echo "error: $cf is not clang-format $CLANG_FORMAT_PIN ($(clang_format_version "$cf"))" >&2
    return 1
  fi
  printf '%s\n' "$cf"
}

CF="$(ensure_clang_format)" || exit 1

# Enable the repo pre-push hook (check_format) for this clone.
if git rev-parse --git-dir >/dev/null 2>&1 && [[ -d "$ROOT/.githooks" ]]; then
  git config --local core.hooksPath .githooks
fi

# Collect sources under application and MCU trees, pruning third-party /
# generated paths. Keep this list in sync with .clang-format-ignore.
collect_all() {
  find Src Inc Mcu \
    \( \
      -path '*/Drivers/*' -o \
      -path '*/CMSIS/*' -o \
      -path '*/dsdl_generated/*' -o \
      -path '*/libcanard/*' -o \
      -path '*/Startup/*' \
    \) -prune -o \
    -type f \( -name '*.c' -o -name '*.h' \) -print \
    | grep -Ev '(^|/)jsmn\.[ch]$' \
    | sort
}

if [[ "$CHANGED_ONLY" -eq 1 ]]; then
  # Prefer merge-base with origin/ark-release or origin/main when present.
  base=""
  for cand in origin/ark-release origin/main; do
    if git rev-parse --verify "$cand" >/dev/null 2>&1; then
      base="$(git merge-base HEAD "$cand" 2>/dev/null || true)"
      [[ -n "$base" ]] && break
    fi
  done
  if [[ -z "$base" ]]; then
    base="HEAD"
  fi
  mapfile -t ALL_FILES < <(collect_all)
  mapfile -t CHANGED < <(
    git diff --name-only --diff-filter=d "$base" -- '*.c' '*.h'
    git diff --name-only --diff-filter=d --cached -- '*.c' '*.h'
  )
  # Unique changed paths that are in our format set.
  declare -A want=()
  for f in "${CHANGED[@]}"; do
    [[ -n "$f" ]] && want["$f"]=1
  done
  FILES=()
  for f in "${ALL_FILES[@]}"; do
    [[ -n "${want[$f]+x}" ]] && FILES+=("$f")
  done
else
  mapfile -t FILES < <(collect_all)
fi

if [[ ${#FILES[@]} -eq 0 ]]; then
  echo "No files to format."
  exit 0
fi

echo "clang-format $(clang_format_version "$CF") ($CF)"
echo "Files: ${#FILES[@]}  mode: $([[ $CHECK -eq 1 ]] && echo check || echo fix)"

# Canonical form = clang-format, then drop trailing whitespace (clang-format
# can leave spaces at EOL inside comments). Used by both check and fix.
canonical_form() {
  "$CF" "$1" | sed 's/[[:space:]]*$//'
}

if [[ "$CHECK" -eq 1 ]]; then
  bad=()
  for f in "${FILES[@]}"; do
    if ! canonical_form "$f" | cmp -s "$f" -; then
      bad+=("$f")
    fi
  done
  if [[ ${#bad[@]} -ne 0 ]]; then
    echo "Format check failed for ${#bad[@]} file(s):" >&2
    printf '  %s\n' "${bad[@]}" >&2
    echo >&2
    echo 'Fix with:  make format' >&2
    echo 'Or only changed files:  make format_changed' >&2
    exit 1
  fi
  echo "Format checks passed."
  exit 0
fi

# In-place format. Run clang-format twice: some files with awkward comments
# need a second pass to stabilize (clang-format is not always idempotent).
# Then strip trailing whitespace so on-disk form matches --check.
format_batch() {
  printf '%s\0' "${FILES[@]}" | xargs -0 -n 32 -P "$(nproc 2>/dev/null || echo 4)" \
    "$CF" -i
}
format_batch
format_batch

# Portable EOL-whitespace strip.
if command -v perl >/dev/null 2>&1; then
  printf '%s\0' "${FILES[@]}" | xargs -0 -n 64 perl -pi -e 's/[ \t]+$//'
else
  for f in "${FILES[@]}"; do
    sed -i.bak 's/[[:space:]]*$//' "$f" && rm -f "$f.bak"
  done
fi

echo "Formatting done (${#FILES[@]} files)."
