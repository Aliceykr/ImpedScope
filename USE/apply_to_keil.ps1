# Re-attaches USE/spi_receiver.c and the ../USE include path to the MDK-ARM
# project after STM32CubeMX regenerates the .uvprojx (idempotent).
#
# Run from the project root with:
#   powershell -NoProfile -ExecutionPolicy Bypass -File USE\apply_to_keil.ps1

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$mdk  = Join-Path $root 'MDK-ARM'
$uv   = Get-ChildItem $mdk -Filter *.uvprojx | Select-Object -First 1

if (-not $uv) { Write-Error 'No .uvprojx found under MDK-ARM.'; exit 1 }

$text = [IO.File]::ReadAllText($uv.FullName)
$nl = "`r`n"
if (-not $text.Contains("`r`n")) { $nl = "`n" }
$changed = $false

# 1) Add the Application/User/USE group with spi_receiver.c
if (-not $text.Contains('../USE/spi_receiver.c')) {
    $group =
        "        <Group>$nl" +
        "          <GroupName>Application/User/USE</GroupName>$nl" +
        "          <Files>$nl" +
        "            <File>$nl" +
        "              <FileName>spi_receiver.c</FileName>$nl" +
        "              <FileType>1</FileType>$nl" +
        "              <FilePath>../USE/spi_receiver.c</FilePath>$nl" +
        "            </File>$nl" +
        "          </Files>$nl" +
        "        </Group>$nl"
    $anchor = "        <Group>$nl          <GroupName>Drivers/STM32H7xx_HAL_Driver</GroupName>"
    if ($text.Contains($anchor)) {
        $text = $text.Replace($anchor, $group + $anchor)
        $changed = $true
    } else {
        Write-Warning 'Driver group anchor not found; close the project in Keil and retry.'
    }
}

# 2) Add ../USE to the compiler include path
if (-not $text.Contains('../USE;')) {
    $inc = '<IncludePath>../Core/Inc;'
    if ($text.Contains($inc)) {
        $text = $text.Replace($inc, '<IncludePath>../USE;../Core/Inc;')
        $changed = $true
    }
}

if ($changed) {
    $enc = New-Object Text.UTF8Encoding($false)
    [IO.File]::WriteAllText($uv.FullName, $text, $enc)
    Write-Host 'Keil project updated: USE group and include path applied.'
} else {
    Write-Host 'Keil project already configured; nothing to do.'
}
