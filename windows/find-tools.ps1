# Locates the Python interpreter and VcXsrv used by the workshop launcher.
# Dot-source this file: . "$PSScriptRoot\find-tools.ps1"

function Test-PythonExe {
    param([string]$Path)
    if (-not $Path -or -not (Test-Path $Path)) { return $false }
    if ($Path -like '*\WindowsApps\*') { return $false }   # Microsoft Store stub
    try {
        $version = & $Path -c "import sys; print('%d.%d' % sys.version_info[:2])" 2>$null
        if ($LASTEXITCODE -ne 0 -or -not $version) { return $false }
        return ([version]$version -ge [version]'3.8')
    } catch { return $false }
}

function Find-Python {
    param([string]$Preferred)
    $candidates = @()
    if ($Preferred) { $candidates += $Preferred }
    $launcher = Get-Command py.exe -ErrorAction SilentlyContinue
    if ($launcher) {
        $fromLauncher = & $launcher.Source -3 -c "import sys; print(sys.executable)" 2>$null
        if ($LASTEXITCODE -eq 0 -and $fromLauncher) { $candidates += $fromLauncher.Trim() }
    }
    foreach ($name in @('python.exe', 'python3.exe')) {
        Get-Command $name -All -ErrorAction SilentlyContinue | ForEach-Object { $candidates += $_.Source }
    }
    $roots = @("$env:LOCALAPPDATA\Programs\Python", "$env:ProgramFiles", "${env:ProgramFiles(x86)}", 'C:\')
    foreach ($root in $roots) {
        if ($root -and (Test-Path $root)) {
            Get-ChildItem -Path $root -Directory -Filter 'Python3*' -ErrorAction SilentlyContinue |
                Sort-Object Name -Descending |
                ForEach-Object { $candidates += (Join-Path $_.FullName 'python.exe') }
        }
    }
    foreach ($candidate in $candidates | Select-Object -Unique) {
        if (Test-PythonExe $candidate) { return (Resolve-Path $candidate).Path }
    }
    return $null
}

function Find-VcXsrv {
    param([string]$Preferred)
    $candidates = @()
    if ($Preferred) { $candidates += $Preferred }
    $candidates += "$env:ProgramFiles\VcXsrv\vcxsrv.exe"
    $candidates += "${env:ProgramFiles(x86)}\VcXsrv\vcxsrv.exe"
    $command = Get-Command vcxsrv.exe -ErrorAction SilentlyContinue
    if ($command) { $candidates += $command.Source }
    $uninstallKeys = @(
        'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*',
        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*',
        'HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*')
    Get-ItemProperty $uninstallKeys -ErrorAction SilentlyContinue |
        Where-Object { $_.DisplayName -like 'VcXsrv*' -and $_.InstallLocation } |
        ForEach-Object { $candidates += (Join-Path $_.InstallLocation 'vcxsrv.exe') }
    foreach ($candidate in $candidates | Select-Object -Unique) {
        if ($candidate -and (Test-Path $candidate)) { return (Resolve-Path $candidate).Path }
    }
    return $null
}

function Install-WithWinget {
    param([string]$Id, [string]$Name)
    $winget = Get-Command winget.exe -ErrorAction SilentlyContinue
    if (-not $winget) {
        throw "$Name is not installed and winget is not available. Install $Name manually and run setup again."
    }
    Write-Host "Installing $Name with winget ..."
    & $winget.Source install --id $Id --exact --source winget --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -ne 0) { throw "winget could not install $Name (exit code $LASTEXITCODE)." }
    # Newly installed programs are not on this session's PATH yet; rebuild it from the registry.
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path', 'User')
}
