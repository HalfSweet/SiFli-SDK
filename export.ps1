#!/usr/bin/env pwsh

$sifli_sdk_path = "$PSScriptRoot"
$env:SIFLI_SDK_PATH = $sifli_sdk_path

function Show-ExportHelp {
    Write-Output "usage: .\export.ps1 [--profile PROFILE]"
    Write-Output ""
    Write-Output "Activate the installed SiFli-SDK environment in the current shell."
    Write-Output ""
    Write-Output "options:"
    Write-Output "  --profile PROFILE   profile to export, defaults to `"default`""
    Write-Output "  -h, --help          show this help message and exit"
}

$profile = "default"
for ($i = 0; $i -lt $args.Count; $i++) {
    $arg = [string]$args[$i]
    if ($arg -eq "-h" -or $arg -eq "--help") {
        Show-ExportHelp
        exit 0
    }
    if ($arg -eq "--profile") {
        if ($i + 1 -ge $args.Count) {
            Write-Error "--profile requires a value."
            exit 1
        }
        $profile = [string]$args[$i + 1]
        $i++
        continue
    }
    if ($arg.StartsWith("--profile=")) {
        $profile = $arg.Substring("--profile=".Length)
    }
}

$installRoot = if ($env:SIFLI_SDK_TOOLS_PATH) { $env:SIFLI_SDK_TOOLS_PATH } else { Join-Path $HOME ".sifli" }
$pointerPath = Join-Path $installRoot "python_env/$profile/current-env.txt"
if (-not (Test-Path $pointerPath)) {
    Write-Error "profile '$profile' is not installed. Missing $pointerPath. Run .\install.ps1 first."
    exit 1
}

$envPath = (Get-Content -Raw $pointerPath).Trim()
if (-not $envPath) {
    Write-Error "profile '$profile' has an empty environment pointer at $pointerPath. Run .\install.ps1 first."
    exit 1
}

$pythonPath = Join-Path $envPath "Scripts/python.exe"
if (-not (Test-Path $pythonPath)) {
    Write-Error "installed python for profile '$profile' was not found at $pythonPath. Run .\install.ps1 again."
    exit 1
}

$output = & $pythonPath "$sifli_sdk_path/tools/sdk_env.py" export --shell powershell @args
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$scriptPath = $output | Select-Object -Last 1
if (-not $scriptPath -or -not (Test-Path $scriptPath)) {
    Write-Error "export helper did not return a valid script path."
    exit 1
}

. $scriptPath
