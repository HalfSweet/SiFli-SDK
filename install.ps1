#!/usr/bin/env pwsh

$SIFLI_SDK_PATH = $PSScriptRoot
$env:SIFLI_SDK_PATH = $SIFLI_SDK_PATH

if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    Write-Error "uv was not found in PATH. Please install uv before running install.ps1."
    exit 1
}

& uv run --with rich --python 3.12.0 --no-project "$SIFLI_SDK_PATH/tools/sdk_env.py" install @args
exit $LASTEXITCODE
