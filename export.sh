# This script should be sourced, not executed.

# shellcheck disable=SC2128,SC2169,SC2039,SC3054 # ignore array expansion warning
if [ -n "${BASH_SOURCE-}" ] && [ "${BASH_SOURCE[0]}" = "${0}" ]
then
    echo "This script should be sourced, not executed:"
    # shellcheck disable=SC2039,SC3054  # reachable only with bash
    echo ". ${BASH_SOURCE[0]}"
    exit 1
fi

sdk_path="."
shell_type="sh"

# shellcheck disable=SC2128,SC2169,SC2039,SC3054,SC3028
if [ -n "${BASH_SOURCE-}" ]
then
    # shellcheck disable=SC3028,SC3054
    sdk_path=$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd -P)
    shell_type="bash"
elif [ -n "${ZSH_VERSION-}" ]
then
    sdk_path=$(cd "$(dirname "${(%):-%x}")"; pwd -P)
    shell_type="zsh"
fi

export SIFLI_SDK_PATH="${sdk_path}"

show_help() {
    cat <<'EOF'
usage: . ./export.sh [--profile PROFILE]

Activate the installed SiFli-SDK environment in the current shell.

options:
  --profile PROFILE   profile to export, defaults to "default"
  -h, --help          show this help message and exit
EOF
}

profile="default"
prev=""
for arg in "$@"
do
    if [ "${prev}" = "--profile" ]
    then
        profile="${arg}"
        prev=""
        continue
    fi
    case "${arg}" in
        -h|--help)
            show_help
            unset sdk_path
            return 0
            ;;
        --profile)
            prev="--profile"
            ;;
        --profile=*)
            profile="${arg#--profile=}"
            ;;
    esac
done

if [ "${prev}" = "--profile" ]
then
    echo "ERROR: --profile requires a value." >&2
    unset sdk_path
    return 1
fi

install_root="${SIFLI_SDK_TOOLS_PATH:-$HOME/.sifli}"
pointer_path="${install_root}/python_env/${profile}/current-env.txt"

if [ ! -f "${pointer_path}" ]
then
    echo "ERROR: profile '${profile}' is not installed. Missing ${pointer_path}. Run ./install.sh first." >&2
    unset sdk_path
    return 1
fi

env_path=$(tr -d '\r\n' < "${pointer_path}")
if [ -z "${env_path}" ]
then
    echo "ERROR: profile '${profile}' has an empty environment pointer at ${pointer_path}. Run ./install.sh first." >&2
    unset sdk_path
    return 1
fi

python_path="${env_path}/bin/python"
if [ ! -f "${python_path}" ]
then
    echo "ERROR: installed python for profile '${profile}' was not found at ${python_path}. Run ./install.sh again." >&2
    unset sdk_path
    return 1
fi

export_output=$("${python_path}" "${sdk_path}/tools/sdk_env.py" export --shell "${shell_type}" "$@")
export_status=$?
if [ ${export_status} -ne 0 ]
then
    unset sdk_path
    return ${export_status}
fi

script_path=$(printf '%s\n' "${export_output}" | tail -n 1)
if [ ! -f "${script_path}" ]
then
    echo "ERROR: export helper did not return a valid script path." >&2
    unset sdk_path
    return 1
fi

. "${script_path}"
unset sdk_path
