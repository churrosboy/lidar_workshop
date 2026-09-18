$ErrorActionPreference = 'Stop'
$base = $PSScriptRoot
. "$base\find-tools.ps1"
$settingsPath = Join-Path $base 'settings.json'
if (-not (Test-Path $settingsPath)) { throw 'settings.json not found. Run windows\setup.cmd first.' }
$s = Get-Content $settingsPath -Raw | ConvertFrom-Json
$lidarType = if ($s.lidar_type) { [string]$s.lidar_type } else { 'GL5' }
if ($lidarType -notin @('GL5', 'GL3')) { throw "lidar_type must be GL5 or GL3: $lidarType" }
foreach ($key in @('sensor_ip', 'pc_ip')) {
    if ([System.Net.IPAddress]::Parse($s.$key).AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork) { throw 'IPv4 required.' }
}
foreach ($key in @('sensor_port', 'pc_port', 'relay_port', 'container_port')) {
    $value = [int]$s.$key
    if ($value -lt 1 -or $value -gt 65535) { throw "Invalid port: $key" }
}
$docker = (Get-Command docker.exe -ErrorAction SilentlyContinue).Source
if (-not $docker) { $docker = "$env:ProgramFiles\Docker\Docker\resources\bin\docker.exe" }
if (-not (Test-Path $docker)) { throw 'Docker CLI not found. Add it to PATH.' }
function Invoke-Docker {
    & $docker @args
    if ($LASTEXITCODE -ne 0) { throw "Docker command failed: $($args[0])" }
}
$osType = Invoke-Docker info --format '{{.OSType}}'
if ($osType -ne 'linux') { throw 'Start the Docker Desktop Linux engine.' }
$python = Find-Python -Preferred $s.python_exe
$xserver = Find-VcXsrv -Preferred $s.vcxsrv_exe
if (-not $python) { throw 'Python 3.8+ was not found. Run windows\setup.cmd to install it.' }
if (-not $xserver) { throw 'VcXsrv was not found. Run windows\setup.cmd to install it.' }
Invoke-Docker image inspect $s.image | Out-Null
$runtime = Join-Path $base '.local'
New-Item -ItemType Directory -Path $runtime -Force | Out-Null
$authPath = Join-Path $runtime 'rviz.Xauthority'
if (-not (Test-Path $authPath)) {
    $cookie = New-Object byte[] 16
    $rng = [Security.Cryptography.RandomNumberGenerator]::Create()
    try { $rng.GetBytes($cookie) } finally { $rng.Dispose() }
    $auth = [byte[]]@(255,255,0,0,0,1,48,0,18) + [Text.Encoding]::ASCII.GetBytes('MIT-MAGIC-COOKIE-1') + [byte[]]@(0,16) + $cookie
    [IO.File]::WriteAllBytes($authPath, $auth)
}
# Reuse only a screen server launched by this checkout.
$xprocess = Get-CimInstance Win32_Process -Filter "Name='vcxsrv.exe'"
if ($xprocess -and -not ($xprocess | Where-Object { $_.CommandLine -like "*$authPath*" })) {
    throw 'Another VcXsrv session exists. Close it before starting this configuration.'
}
if (-not $xprocess) {
    Start-Process -FilePath $xserver -WindowStyle Hidden -ArgumentList ":0 -multiwindow -clipboard -nowgl -listen tcp -auth `"$authPath`" -logfile `"$runtime\vcxsrv.log`""
}
$names = @(Invoke-Docker container ls -a --format '{{.Names}}')
if ($names -contains $s.container) {
    $existing = Invoke-Docker container inspect $s.container
    $container = ($existing | ConvertFrom-Json)[0]
    if ($container.Config.Labels.'soslab.windows.workspace' -ne $runtime) {
        throw 'An existing container has a different configuration. Preserve its work and migrate it manually; this launcher does not delete containers.'
    }
} else {
    Invoke-Docker create --name $s.container --label "soslab.windows.workspace=$runtime" -it --platform linux/amd64 -e DISPLAY=host.docker.internal:0 -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1 -e XAUTHORITY=/windows-runtime/rviz.Xauthority -e GL5_PARAMS_FILE=/windows-runtime/gl5.yaml -e GL5_REGION_FILE=/windows-runtime/gl5_region.json --mount "type=bind,source=$runtime,target=/windows-runtime" $s.image bash | Out-Null
}
Invoke-Docker start $s.container | Out-Null
& $docker exec $s.container pgrep -x gl5_node | Out-Null
if ($LASTEXITCODE -eq 0) { Write-Host 'LiDAR is already running.'; exit 0 }
$hostLine = @(Invoke-Docker exec $s.container getent ahostsv4 host.docker.internal)[0]
$hostIP = ($hostLine -split '\s+')[0]
[void][System.Net.IPAddress]::Parse($hostIP)
# Explicit floating-point YAML values are required by the ROS parameter types.
$yaml = "gl5_node:`n  ros__parameters:`n    lidar_type: '$lidarType'`n    sensor_ip: '$hostIP'`n    sensor_port: $($s.relay_port)`n    pc_ip: '0.0.0.0'`n    pc_port: $($s.container_port)`n    frame_id: laser`n    range_min: 0.0`n    range_max: 60.0`n    angle_offset: 0.0`n"
[IO.File]::WriteAllText((Join-Path $runtime 'gl5.yaml'), $yaml, (New-Object Text.UTF8Encoding($false)))
$relayRunning = $false
$pidPath = Join-Path $runtime 'relay.pid'
if (Test-Path $pidPath) {
    $relayId = [int](Get-Content $pidPath)
    $p = Get-CimInstance Win32_Process -Filter "ProcessId=$relayId"
    $relayRunning = $p -and $p.CommandLine -like "*$base\lidar_udp_relay.py*"
}
if (-not $relayRunning) {
    $relay = Start-Process -FilePath $python -WindowStyle Hidden -ArgumentList "-u `"$base\lidar_udp_relay.py`" --sensor-ip $($s.sensor_ip) --sensor-port $($s.sensor_port) --pc-ip $($s.pc_ip) --pc-port $($s.pc_port) --relay-port $($s.relay_port)" -RedirectStandardOutput "$runtime\relay.log" -RedirectStandardError "$runtime\relay-error.log" -PassThru
    $relay.Id | Set-Content $pidPath
    Start-Sleep -Seconds 1
    if ($relay.HasExited) { throw (Get-Content "$runtime\relay-error.log" -Raw) }
}
Invoke-Docker exec -it $s.container /ros_entrypoint.sh bash scripts/run.sh
