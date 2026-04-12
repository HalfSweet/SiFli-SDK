from __future__ import annotations

import argparse
import os
import shutil
import sys
import tempfile
import textwrap
import unittest
from unittest import mock


ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import sdk_env  # noqa: E402


class CompatHashTests(unittest.TestCase):
    def make_profile_dir(self, lock_body: str, pyproject_body: str, uv_lock_body: str) -> str:
        temp_dir = tempfile.mkdtemp(prefix="sdk-env-test-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        os.makedirs(os.path.join(temp_dir, "profile"), exist_ok=True)
        with open(os.path.join(temp_dir, "profile", "lock.json"), "w", encoding="utf-8") as f:
            f.write(lock_body)
        with open(os.path.join(temp_dir, "profile", "pyproject.toml"), "w", encoding="utf-8") as f:
            f.write(pyproject_body)
        with open(os.path.join(temp_dir, "profile", "uv.lock"), "w", encoding="utf-8") as f:
            f.write(uv_lock_body)
        return temp_dir

    def test_compat_hash_ignores_comments_whitespace_and_key_order(self) -> None:
        lock_a = textwrap.dedent(
            """
            {
              "schema_version": 1,
              "profile": "default",
              "python": {"version": "3.12.0", "project_dir": "tools/locks/default", "lock_file": "tools/locks/default/uv.lock"},
              "defaults": {"targets": ["all"]},
              "tools": {"sftool": "0.1.16"},
              "path_order": ["sftool"],
              "conan": {"config_id": "sdk.conan-config.v2.4", "remote_name": "artifactory", "remote_url": "https://example.com", "home_subdir": "default"}
            }
            """
        ).strip()
        lock_b = textwrap.dedent(
            """
            {
              "profile": "default",
              "schema_version": 1,
              "conan": {"remote_url": "https://example.com", "home_subdir": "changed", "remote_name": "artifactory", "config_id": "sdk.conan-config.v2.4"},
              "path_order": ["sftool"],
              "tools": {"sftool": "0.1.16"},
              "defaults": {"targets": ["all"]},
              "python": {"lock_file": "tools/locks/default/uv.lock", "project_dir": "tools/locks/default", "version": "3.12.0"}
            }
            """
        ).strip()
        pyproject_a = textwrap.dedent(
            """
            # comment
            [project]
            name = "demo"
            version = "0.1.0"
            dependencies = ["click", "requests"]
            """
        ).strip()
        pyproject_b = textwrap.dedent(
            """
            [project]
            version = "0.1.0"
            name = "demo"
            dependencies = [
              "click",
              "requests",
            ]
            # trailing comment
            """
        ).strip()
        uv_lock_a = textwrap.dedent(
            """
            version = 1

            [[package]]
            name = "click"
            version = "8.1.7"
            """
        ).strip()
        uv_lock_b = textwrap.dedent(
            """
            version = 1

            # comment
            [[package]]
            version = "8.1.7"
            name = "click"
            """
        ).strip()

        root_a = self.make_profile_dir(lock_a, pyproject_a, uv_lock_a)
        root_b = self.make_profile_dir(lock_b, pyproject_b, uv_lock_b)
        hash_a = sdk_env.compute_env_compat_sha256(
            os.path.join(root_a, "profile", "lock.json"),
            os.path.join(root_a, "profile", "pyproject.toml"),
            os.path.join(root_a, "profile", "uv.lock"),
        )
        hash_b = sdk_env.compute_env_compat_sha256(
            os.path.join(root_b, "profile", "lock.json"),
            os.path.join(root_b, "profile", "pyproject.toml"),
            os.path.join(root_b, "profile", "uv.lock"),
        )
        self.assertEqual(hash_a, hash_b)

    def test_compat_hash_changes_on_semantic_change(self) -> None:
        lock_body = textwrap.dedent(
            """
            {
              "schema_version": 1,
              "profile": "default",
              "python": {"version": "3.12.0", "project_dir": "tools/locks/default", "lock_file": "tools/locks/default/uv.lock"},
              "defaults": {"targets": ["all"]},
              "tools": {"sftool": "0.1.16"},
              "path_order": ["sftool"],
              "conan": {"config_id": "sdk.conan-config.v2.4", "remote_name": "artifactory", "remote_url": "https://example.com", "home_subdir": "default"}
            }
            """
        ).strip()
        pyproject_a = textwrap.dedent(
            """
            [project]
            name = "demo"
            version = "0.1.0"
            dependencies = ["click"]
            """
        ).strip()
        pyproject_b = textwrap.dedent(
            """
            [project]
            name = "demo"
            version = "0.1.0"
            dependencies = ["requests"]
            """
        ).strip()
        uv_lock_body = textwrap.dedent(
            """
            version = 1
            """
        ).strip()
        root_a = self.make_profile_dir(lock_body, pyproject_a, uv_lock_body)
        root_b = self.make_profile_dir(lock_body, pyproject_b, uv_lock_body)
        hash_a = sdk_env.compute_env_compat_sha256(
            os.path.join(root_a, "profile", "lock.json"),
            os.path.join(root_a, "profile", "pyproject.toml"),
            os.path.join(root_a, "profile", "uv.lock"),
        )
        hash_b = sdk_env.compute_env_compat_sha256(
            os.path.join(root_b, "profile", "lock.json"),
            os.path.join(root_b, "profile", "pyproject.toml"),
            os.path.join(root_b, "profile", "uv.lock"),
        )
        self.assertNotEqual(hash_a, hash_b)


class StateAndPathTests(unittest.TestCase):
    def make_args(self, **overrides: object) -> argparse.Namespace:
        defaults = {
            "cache_dir": None,
            "staging_dir": None,
            "offline": False,
            "mirror": None,
            "from_bundle": None,
            "profile": "default",
            "shell": "bash",
        }
        defaults.update(overrides)
        return argparse.Namespace(**defaults)

    def test_write_profile_state_round_trip(self) -> None:
        temp_dir = tempfile.mkdtemp(prefix="sdk-env-state-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        state_path = os.path.join(temp_dir, "sifli-sdk-env.json")
        installed = {"python": {"version": "3.12.0", "env_path": "/tmp/env"}}
        sdk_env.write_profile_state(
            state_path,
            "/repo",
            "default",
            installed=installed,
            auto_reconcile="always",
        )
        loaded = sdk_env.read_profile_state(state_path, "/repo", "default")
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded["installed"], installed)
        self.assertEqual(loaded["preferences"]["auto_reconcile"], "always")

    def test_merge_managed_paths_replaces_previous_paths_and_dedupes(self) -> None:
        merged = sdk_env.merge_managed_paths(
            current_path=os.pathsep.join(["/old/a", "/usr/bin", "/keep"]),
            old_managed_paths=["/old/a", "/old/b"],
            new_managed_paths=["/new/a", "/usr/bin"],
        )
        self.assertEqual(merged, os.pathsep.join(["/new/a", "/usr/bin", "/keep"]))

    def test_runtime_config_ignores_install_root_in_config_json(self) -> None:
        temp_home = tempfile.mkdtemp(prefix="sdk-env-home-")
        self.addCleanup(lambda: shutil.rmtree(temp_home, ignore_errors=True))
        install_root = os.path.join(temp_home, ".sifli")
        os.makedirs(install_root, exist_ok=True)
        with open(os.path.join(install_root, "config.json"), "w", encoding="utf-8") as f:
            f.write(
                textwrap.dedent(
                    """
                    {
                      "install_root": "/tmp/should-be-ignored",
                      "cache_root": "/tmp/cache-root",
                      "staging_root": "/tmp/staging-root",
                      "offline": true
                    }
                    """
                ).strip()
            )

        with mock.patch.dict(
            os.environ,
            {"HOME": temp_home, "SIFLI_SDK_TOOLS_PATH": install_root},
            clear=True,
        ):
            config = sdk_env.RuntimeConfig.load(self.make_args())

        self.assertEqual(config.install_root, os.path.realpath(install_root))
        self.assertEqual(config.cache_root, os.path.realpath("/tmp/cache-root"))
        self.assertEqual(config.staging_root, os.path.realpath("/tmp/staging-root"))
        self.assertTrue(config.offline)

    def test_current_env_pointer_round_trip(self) -> None:
        temp_install_root = tempfile.mkdtemp(prefix="sdk-env-install-root-")
        self.addCleanup(lambda: shutil.rmtree(temp_install_root, ignore_errors=True))
        config = sdk_env.RuntimeConfig(
            install_root=temp_install_root,
            cache_root=os.path.join(temp_install_root, "cache"),
            staging_root=os.path.join(temp_install_root, "staging"),
            offline=False,
            python_default_index="https://pypi.org/simple",
            python_indexes=[],
            python_index_strategy="first-index",
            sources=[],
        )
        env_path = os.path.join(temp_install_root, "python_env", "default", "py3.12.0")
        sdk_env.write_current_env_pointer(config, "default", env_path)
        loaded = sdk_env.read_current_env_pointer(config, "default")
        self.assertEqual(loaded, os.path.realpath(env_path))

    def test_export_reexec_argv_uses_target_env_python(self) -> None:
        temp_install_root = tempfile.mkdtemp(prefix="sdk-env-install-root-")
        self.addCleanup(lambda: shutil.rmtree(temp_install_root, ignore_errors=True))
        config = sdk_env.RuntimeConfig(
            install_root=temp_install_root,
            cache_root=os.path.join(temp_install_root, "cache"),
            staging_root=os.path.join(temp_install_root, "staging"),
            offline=False,
            python_default_index="https://pypi.org/simple",
            python_indexes=[],
            python_index_strategy="first-index",
            sources=[],
        )
        lock = sdk_env.ProfileLock(
            path="/tmp/lock.json",
            schema_version=1,
            profile="default",
            python_version="3.12.0",
            python_project_dir="tools/locks/default",
            python_lock_file="tools/locks/default/uv.lock",
            default_targets=["all"],
            tools={"sftool": "0.1.16"},
            path_order=["sftool"],
            conan_config_id="sdk.conan-config.v2.4",
            conan_remote_name="artifactory",
            conan_remote_url="https://example.com",
            conan_home_subdir="default",
        )
        args = self.make_args(profile="default", shell="bash", offline=True, mirror="https://mirror.example")
        argv = sdk_env.export_reexec_argv(args, config, lock)

        self.assertEqual(argv[0], sdk_env.python_executable(sdk_env.python_env_path(config, lock)))
        self.assertEqual(argv[1:5], [os.path.join(ROOT, "tools", "sdk_env.py"), "export", "--profile", "default"])
        self.assertIn("--offline", argv)
        self.assertIn("https://mirror.example", argv)


class TargetParsingTests(unittest.TestCase):
    def test_install_target_conflict_is_rejected(self) -> None:
        lock = sdk_env.ProfileLock(
            path="/tmp/lock.json",
            schema_version=1,
            profile="default",
            python_version="3.12.0",
            python_project_dir="tools/locks/default",
            python_lock_file="tools/locks/default/uv.lock",
            default_targets=["all"],
            tools={"sftool": "0.1.16"},
            path_order=["sftool"],
            conan_config_id="sdk.conan-config.v2.4",
            conan_remote_name="artifactory",
            conan_remote_url="https://example.com",
            conan_home_subdir="default",
        )
        with self.assertRaises(sdk_env.SDKEnvError):
            sdk_env.parse_install_targets(lock, "sf32lb52", ["sf32lb58"])


if __name__ == "__main__":
    unittest.main()
