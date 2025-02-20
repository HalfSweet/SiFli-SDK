#!/usr/bin/env python
# coding=utf-8
#
# SPDX-FileCopyrightText: 2025 SiFli
#
# SPDX-License-Identifier: Apache-2.0
#
# This script helps installing tools required to use the SiFli-SDK, and updating PATH
# to use the installed tools. It can also create a Python virtual environment,
# and install Python requirements into it.
# It does not install OS dependencies. It does install tools such as the
# arm-none-eabi-gcc compiler, cmake, ninja, etc.
#
# By default, downloaded tools will be installed under $HOME/.sifli directory
# (%USERPROFILE%/.sifli on Windows). This path can be modified by setting
# SIFLI_SDK_PATH variable prior to running this tool.
#
# Users do not need to interact with this script directly. In SiFli-SDK root directory,
# install.sh (.bat) and export.sh (.bat) scripts are provided to invoke this script.
#
# Usage:
#
# * To install the tools, run `sifli_sdk_tools.py install`.
#
# * To install the Python environment, run `sifli_sdk_tools.py install-python-env`.
#
# * To start using the tools, run `eval "$(sifli_sdk_tools.py export)"` — this will update
#   the PATH to point to the installed tools and set up other environment variables
#   needed by the tools.

import argparse
import sys


def main(argv: List[str]):
    parser = argparse.ArgumentParser()
    parser.add_argument("--sifli-sdk-path", help="SiFli-SDK to use")

    subparsers = parser.add_subparsers(dest="action")

    install = subparsers.add_parser(
        "install", help="Download and install tools into the tools directory"
    )
    install.add_argument(
        "tools",
        metavar="TOOL",
        nargs="*",
        default=["required"],
        help=(
            "Tools to install.\n"
            "To install a specific version use <tool_name>@<version> syntax. "
            "To install tools by pattern use wildcards in <tool_name_pattern>. "
            "Use empty or 'required' to install required tools, not optional ones. "
            "Use 'all' to install all tools, including the optional ones."
        ),
    )
    install.add_argument(
        "--targets",
        default="all",
        help=(
            "A comma separated list of desired chip targets for installing. "
            "It defaults to installing all supported targets."
        ),
    )

    install_python_env = subparsers.add_parser(
        "install-python-env",
        help=(
            "Create Python virtual environment and install the "
            "required Python packages"
        ),
    )
    install_python_env.add_argument(
        "--reinstall",
        help="Discard the previously installed environment",
        action="store_true",
    )
    install_python_env.add_argument(
        "--extra-wheels-dir",
        help=("Additional directories with wheels " "to use during installation"),
    )
    # install_python_env.add_argument(
    #     "--extra-wheels-url",
    #     help="Additional URL with wheels",
    #     default=SIFLI_SDK_PIP_WHEELS_URL,
    # )
    install_python_env.add_argument(
        "--no-index", help="Work offline without retrieving wheels index"
    )
    install_python_env.add_argument(
        "--features",
        default="core",
        help=(
            "A comma separated list of desired features for installing. "
            "It defaults to installing just the core functionality."
        ),
    )
    # install_python_env.add_argument(
    #     "--no-constraints",
    #     action="store_true",
    #     default=no_constraints_default,
    #     help=(
    #         "Disable constraint settings. Use with care and only when you want to manage "
    #         "package versions by yourself. It can be set with the SIFLI_SDK_PYTHON_CHECK_CONSTRAINTS "
    #         "environment variable."
    #     ),
    # )

    args = parser.parse_args(argv)
    if args.action is None:
        parser.print_help()
        parser.exit(1)


if __name__ == "__main__":
    main(sys.argv[1:])
