# AM32 (ARK) project rules

## Formatting

Always run `make format` from the repo root **before committing** any code changes (C/C++/headers and other clang-format-managed sources). Include format-only diffs in the commit or a follow-up `style:` commit on the same PR. `scripts/format.sh` requires clang-format **22.1.5** (CI pin); Ubuntu 18.x will fail the PR check. `make format` bootstraps that pin into `tools/clang-format-venv` if needed and enables a pre-push hook that runs `make check_format`.
