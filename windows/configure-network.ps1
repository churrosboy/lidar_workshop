param([int]$InterfaceIndex = 0)
$ErrorActionPreference = 'Stop'
$settings = Get-Content (Join-Path $PSScriptRoot 'settings.json') -Raw | ConvertFrom-Json
# Physical wired adapters only (no Wi-Fi = NdisPhysicalMedium 9, no virtual adapters).
$wired = Get-NetAdapter | Where-Object { $_.HardwareInterface -eq $true -and $_.NdisPhysicalMedium -ne 9 }
if ($InterfaceIndex -gt 0) {
    $adapter = $wired | Where-Object ifIndex -eq $InterfaceIndex
    if (-not $adapter) { throw "Interface $InterfaceIndex is not a physical wired adapter. Candidates: $(($wired | ForEach-Object { "$($_.ifIndex)=$($_.Name)" }) -join ', ')" }
} else {
    # Without an index, pick the wired adapter that has link (the sensor cable is plugged in).
    $connected = @($wired | Where-Object Status -eq 'Up')
    if ($connected.Count -eq 1) { $adapter = $connected[0] }
    elseif ($connected.Count -eq 0) { throw "No wired adapter is connected. Plug in the sensor cable and retry. Wired adapters: $(($wired | ForEach-Object { "$($_.ifIndex)=$($_.Name) [$($_.Status)]" }) -join ', ')" }
    else { throw "Several wired adapters are connected; pass -InterfaceIndex <n>: $(($connected | ForEach-Object { "$($_.ifIndex)=$($_.Name)" }) -join ', ')" }
}
$InterfaceIndex = $adapter.ifIndex
Write-Host "Using adapter $($adapter.Name) (ifIndex $InterfaceIndex)"
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
if (-not ([Security.Principal.WindowsPrincipal]$identity).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Run this script in an administrator PowerShell.'
}
$address = [System.Net.IPAddress]::Parse($settings.pc_ip)
if ($address.AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork) { throw 'IPv4 required.' }
$prefix = [int]$settings.prefix_length
if ($prefix -lt 1 -or $prefix -gt 32) { throw 'Invalid prefix length.' }
# No default gateway or DNS is added; the Wi-Fi route is retained.
$mask = ((0..3 | ForEach-Object {
    $bits = [Math]::Min(8, [Math]::Max(0, $prefix - 8 * $_))
    [int](256 - [Math]::Pow(2, 8 - $bits))
}) -join '.')
& netsh interface ipv4 set address "name=$InterfaceIndex" source=static "address=$address" "mask=$mask" gateway=none
if ($LASTEXITCODE -ne 0) { throw 'Ethernet configuration failed.' }
$rule = "SOSLAB-GL5-UDP-$InterfaceIndex"
if (Get-NetFirewallRule -Name $rule -ErrorAction SilentlyContinue) {
    Remove-NetFirewallRule -Name $rule
}
New-NetFirewallRule -Name $rule -DisplayName 'SOSLAB GL5 sensor UDP' -Direction Inbound -Action Allow -Protocol UDP -LocalPort $settings.pc_port -RemoteAddress $settings.sensor_ip -InterfaceAlias $adapter.Name -Profile Any | Out-Null
Get-NetIPAddress -InterfaceIndex $InterfaceIndex -AddressFamily IPv4
