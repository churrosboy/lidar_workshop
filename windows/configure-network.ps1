param([Parameter(Mandatory=$true)][int]$InterfaceIndex)
$ErrorActionPreference = 'Stop'
$settings = Get-Content (Join-Path $PSScriptRoot 'settings.json') -Raw | ConvertFrom-Json
$adapter = Get-NetAdapter | Where-Object ifIndex -eq $InterfaceIndex
if (-not $adapter) { throw 'Network adapter not found.' }
if ($adapter.HardwareInterface -ne $true -or $adapter.NdisPhysicalMedium -eq 9) {
    throw 'Select the physical USB Ethernet adapter, not Wi-Fi or a virtual adapter.'
}
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
