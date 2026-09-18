# One-time workshop setup: finds (or installs) Python and VcXsrv and writes settings.json.
$ErrorActionPreference = 'Stop'
$base = $PSScriptRoot
. "$base\find-tools.ps1"

$examplePath = Join-Path $base 'settings.example.json'
$settingsPath = Join-Path $base 'settings.json'
$settings = Get-Content $examplePath -Raw | ConvertFrom-Json
if (Test-Path $settingsPath) {
    # Keep values the user already changed (sensor IP, lidar_type, ...); only fill blanks.
    $existing = Get-Content $settingsPath -Raw | ConvertFrom-Json
    foreach ($property in $existing.PSObject.Properties) {
        if ($null -ne $property.Value -and "$($property.Value)" -ne '') {
            $settings | Add-Member -NotePropertyName $property.Name -NotePropertyValue $property.Value -Force
        }
    }
}

$python = Find-Python -Preferred $settings.python_exe
if (-not $python) {
    Install-WithWinget -Id 'Python.Python.3.12' -Name 'Python 3.12'
    $python = Find-Python
    if (-not $python) { throw 'Python was installed but could not be found. Open a new PowerShell and run setup again.' }
}
$pythonVersion = & $python -c "import sys; print(sys.version.split()[0])"
Write-Host "Python : $python ($pythonVersion)"

$xserver = Find-VcXsrv -Preferred $settings.vcxsrv_exe
if (-not $xserver) {
    Install-WithWinget -Id 'marha.VcXsrv' -Name 'VcXsrv'
    $xserver = Find-VcXsrv
    if (-not $xserver) { throw 'VcXsrv was installed but could not be found. Open a new PowerShell and run setup again.' }
}
Write-Host "VcXsrv : $xserver"

$settings.python_exe = $python -replace '\\', '/'
$settings.vcxsrv_exe = $xserver -replace '\\', '/'
$json = $settings | ConvertTo-Json -Depth 3
[IO.File]::WriteAllText($settingsPath, $json + "`n", (New-Object Text.UTF8Encoding($false)))
Write-Host "Saved  : $settingsPath"
Write-Host ("Sensor : {0}:{1} -> PC {2}:{3} (lidar_type {4})" -f $settings.sensor_ip, $settings.sensor_port, $settings.pc_ip, $settings.pc_port, $settings.lidar_type)

$docker = Get-Command docker.exe -ErrorAction SilentlyContinue
if ($docker) {
    $osType = & $docker.Source info --format '{{.OSType}}' 2>$null
    if ($osType -eq 'linux') { Write-Host 'Docker : ready (linux engine)' }
    else { Write-Warning 'Docker Desktop is installed but not running, or not in Linux container mode.' }
} else {
    Write-Warning 'Docker Desktop was not found. Install it from https://www.docker.com/products/docker-desktop/ before building the image.'
}
