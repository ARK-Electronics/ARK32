#!/usr/bin/env python3
"""Exercise formatter configuration and pre-push checks in disposable Git repos.

Run with: python3 -m unittest discover -s scripts/tests -v
"""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import textwrap
import unittest


REPO_ROOT = Path(__file__).resolve().parents[2]
FORMATTED_SOURCE = "int value = 0;\n"
UNFORMATTED_SOURCE = "int value=0;\n"


class FormatHookTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="ark32-format-hooks-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.repo = self.root / "checkout with spaces"
        self.repo.mkdir()
        self.remote = self.root / "remote.git"
        self.env = os.environ.copy()
        # Keep developer and CI Git configuration out of the fixture.
        for key in list(self.env):
            if key.startswith("GIT_"):
                del self.env[key]
        self.env.update(
            GIT_CONFIG_NOSYSTEM="1",
            GIT_CONFIG_GLOBAL=os.devnull,
            GIT_AUTHOR_NAME="Format Hook Test",
            GIT_AUTHOR_EMAIL="format-test@example.invalid",
            GIT_COMMITTER_NAME="Format Hook Test",
            GIT_COMMITTER_EMAIL="format-test@example.invalid",
            LC_ALL="C",
        )
        formatter = self.root / "fake clang-format"
        formatter.write_text(
            textwrap.dedent(
                """\
                #!/usr/bin/env python3
                from pathlib import Path
                import sys

                if sys.argv[1:] == ["--version"]:
                    print("clang-format version 22.1.5")
                elif sys.argv[1] == "-i":
                    for name in sys.argv[2:]:
                        path = Path(name)
                        path.write_text(path.read_text().replace("value=", "value = "))
                else:
                    print(Path(sys.argv[1]).read_text().replace("value=", "value = "), end="")
                """
            )
        )
        formatter.chmod(0o755)
        self.env["CLANG_FORMAT"] = str(formatter)
        for name in ("Src", "Inc", "Mcu", "scripts", ".githooks"):
            (self.repo / name).mkdir()
        for name in ("Inc", "Mcu"):
            (self.repo / name / ".gitkeep").touch()
        for name in ("scripts/format.sh", ".githooks/pre-push", ".clang-format"):
            shutil.copy2(REPO_ROOT / name, self.repo / name)
        (self.repo / ".githooks/pre-push").chmod(0o755)
        (self.repo / "Makefile").write_text(
            ".PHONY: format check_format\n"
            "format:\n\tbash scripts/format.sh\n"
            "check_format:\n\tbash scripts/format.sh --check\n"
        )
        self.source = self.repo / "Src/example.c"
        self.source.write_text(FORMATTED_SOURCE)
        self.git("init", "--quiet", "--initial-branch=clean")
        self.git("add", ".")
        self.git("commit", "--quiet", "-m", "Formatted baseline")
        self.git("init", "--quiet", "--bare", str(self.remote))
        self.git("remote", "add", "origin", str(self.remote))
        self.git("push", "--quiet", "--no-verify", "origin", "clean")
        self.git("config", "--local", "core.hooksPath", ".githooks")

    def run_command(self, *args, check=True, env=None):
        result = subprocess.run(
            args,
            cwd=self.repo,
            env=self.env if env is None else env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=30,
        )
        if check:
            self.assertEqual(result.returncode, 0, result.stdout)
        return result

    def git(self, *args, check=True, env=None):
        return self.run_command("git", *args, check=check, env=env)

    def remote_ref(self, ref):
        result = self.git("--git-dir", str(self.remote), "rev-parse", "--verify", ref, check=False)
        return result.stdout.strip() if result.returncode == 0 else None

    def make_bad_branch(self):
        self.git("checkout", "--quiet", "-b", "bad")
        self.source.write_text(UNFORMATTED_SOURCE)
        self.git("add", "Src/example.c")
        self.git("commit", "--quiet", "-m", "Unformatted source")
        self.git("checkout", "--quiet", "clean")

    def test_formatter_preserves_existing_hook_configuration(self):
        custom_hooks = self.root / "custom hooks"
        custom_hooks.mkdir()
        hook = custom_hooks / "pre-push"
        hook.write_text("#!/bin/sh\necho custom-validator-rejected >&2\nexit 1\n")
        hook.chmod(0o755)
        self.git("config", "--local", "core.hooksPath", str(custom_hooks))
        for mode in ("check_format", "format"):
            with self.subTest(mode=mode):
                self.run_command("make", mode)
                self.assertEqual(
                    self.git("config", "--local", "--get", "core.hooksPath").stdout.strip(),
                    str(custom_hooks),
                )
                result = self.git("push", "origin", "clean:refs/heads/custom-check", check=False)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertIn("custom-validator-rejected", result.stdout)
                self.assertIsNone(self.remote_ref("refs/heads/custom-check"))

    def test_formatter_does_not_install_hooks(self):
        self.git("config", "--local", "--unset", "core.hooksPath")
        for mode in ("check_format", "format"):
            with self.subTest(mode=mode):
                self.run_command("make", mode)
                result = self.git("config", "--local", "--get", "core.hooksPath", check=False)
                self.assertEqual(result.returncode, 1, result.stdout)

    def test_push_checks_branch_being_pushed(self):
        self.make_bad_branch()
        result = self.git("push", "origin", "bad:refs/heads/bad", check=False)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("Src/example.c", result.stdout)
        self.assertIsNone(self.remote_ref("refs/heads/bad"))
        self.assertEqual(self.source.read_text(), FORMATTED_SOURCE)

    def test_push_ignores_uncommitted_formatting_errors(self):
        self.source.write_text(UNFORMATTED_SOURCE)
        self.git("add", "Src/example.c")
        self.source.write_text("int value=1;\n")
        staged_before = self.git("diff", "--cached", "--binary").stdout
        unstaged_before = self.git("diff", "--binary").stdout
        self.git("push", "origin", "clean:refs/heads/clean-copy")
        self.assertEqual(self.remote_ref("refs/heads/clean-copy"), self.git("rev-parse", "clean").stdout.strip())
        self.assertEqual(self.source.read_text(), "int value=1;\n")
        self.assertEqual(self.git("diff", "--cached", "--binary").stdout, staged_before)
        self.assertEqual(self.git("diff", "--binary").stdout, unstaged_before)

    def test_multi_ref_push_rejects_unformatted_commit(self):
        self.make_bad_branch()
        result = self.git(
            "push", "origin", "clean:refs/heads/clean-copy", "bad:refs/heads/bad", check=False
        )
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIsNone(self.remote_ref("refs/heads/clean-copy"))
        self.assertIsNone(self.remote_ref("refs/heads/bad"))

    def test_deletion_skips_formatting(self):
        self.git("push", "--no-verify", "origin", "clean:refs/heads/disposable")
        self.source.write_text(UNFORMATTED_SOURCE)
        env = dict(self.env, CLANG_FORMAT=str(self.root / "missing-clang-format"))
        self.git("push", "origin", ":refs/heads/disposable", env=env)
        self.assertIsNone(self.remote_ref("refs/heads/disposable"))
        self.assertEqual(self.source.read_text(), UNFORMATTED_SOURCE)

    def test_annotated_tag_checks_its_commit(self):
        self.make_bad_branch()
        self.git("tag", "-a", "bad-release", "bad", "-m", "Unformatted release")
        result = self.git("push", "origin", "refs/tags/bad-release", check=False)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("Src/example.c", result.stdout)
        self.assertIsNone(self.remote_ref("refs/tags/bad-release"))

    def test_formatted_annotated_tag_ignores_dirty_worktree(self):
        self.git("tag", "-a", "good-release", "clean", "-m", "Formatted release")
        self.source.write_text(UNFORMATTED_SOURCE)
        self.git("push", "origin", "refs/tags/good-release")
        self.assertEqual(
            self.remote_ref("refs/tags/good-release"),
            self.git("rev-parse", "refs/tags/good-release").stdout.strip(),
        )
        self.assertEqual(self.source.read_text(), UNFORMATTED_SOURCE)

    def test_duplicate_commit_is_checked_once(self):
        self.git("tag", "-a", "good-release", "clean", "-m", "Formatted release")
        result = self.git("push", "origin", "clean:refs/heads/clean-copy", "refs/tags/good-release")
        self.assertEqual(result.stdout.count("Checking formatting for "), 1, result.stdout)

    def test_non_commit_tag_skips_formatting(self):
        blob = self.git("rev-parse", "clean:Src/example.c").stdout.strip()
        self.git("tag", "-a", "source-file", blob, "-m", "A tagged source blob")
        env = dict(self.env, CLANG_FORMAT=str(self.root / "missing-clang-format"))
        self.git("push", "origin", "refs/tags/source-file", env=env)
        self.assertEqual(
            self.remote_ref("refs/tags/source-file"),
            self.git("rev-parse", "refs/tags/source-file").stdout.strip(),
        )

    def test_commit_without_formatter_uses_installed_check(self):
        self.git("checkout", "--quiet", "-b", "historical")
        self.git("rm", "scripts/format.sh", "Makefile")
        self.source.write_text(UNFORMATTED_SOURCE)
        self.git("add", "Src/example.c")
        self.git("commit", "--quiet", "-m", "Tree predating formatter")
        self.git("checkout", "--quiet", "clean")
        result = self.git("push", "origin", "historical", check=False)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("Src/example.c", result.stdout)
        self.assertIsNone(self.remote_ref("refs/heads/historical"))

    def test_local_export_ignore_does_not_skip_source(self):
        self.make_bad_branch()
        (self.repo / ".git/info/attributes").write_text("Src/example.c export-ignore\n")
        result = self.git("push", "origin", "bad", check=False)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("Src/example.c", result.stdout)
        self.assertIsNone(self.remote_ref("refs/heads/bad"))

    def test_committed_export_ignore_does_not_skip_source(self):
        self.make_bad_branch()
        self.git("checkout", "--quiet", "bad")
        (self.repo / ".gitattributes").write_text("Src/example.c export-ignore\n")
        self.git("add", ".gitattributes")
        self.git("commit", "--quiet", "-m", "Exclude source from archives")
        self.git("checkout", "--quiet", "clean")
        result = self.git("push", "origin", "bad", check=False)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("Src/example.c", result.stdout)
        self.assertIsNone(self.remote_ref("refs/heads/bad"))


if __name__ == "__main__":
    unittest.main()
