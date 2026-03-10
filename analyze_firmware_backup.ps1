param(
    [string]$BackupDir = ".\firmware_backup",
    [int]$MinStringLength = 6,
    [switch]$UseLatestOnly,
    [int]$MaxMatchesPerFile = 250
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

function Get-PrintableStrings {
    param(
        [byte[]]$Bytes,
        [int]$MinLength = 6
    )

    $sb = New-Object System.Text.StringBuilder
    $results = New-Object System.Collections.Generic.List[string]

    foreach ($byte in $Bytes) {
        if (($byte -ge 32 -and $byte -le 126) -or $byte -eq 9) {
            [void]$sb.Append([char]$byte)
        }
        else {
            if ($sb.Length -ge $MinLength) {
                $results.Add($sb.ToString())
            }
            [void]$sb.Clear()
        }
    }

    if ($sb.Length -ge $MinLength) {
        $results.Add($sb.ToString())
    }

    return $results
}

function Get-TargetBins {
    param(
        [string]$Dir,
        [switch]$LatestOnly
    )

    $bins = Get-ChildItem -Path $Dir -Filter "*.bin" -File | Sort-Object LastWriteTime
    if ($bins.Count -eq 0) {
        return @()
    }

    if (-not $LatestOnly) {
        return $bins
    }

    $selected = @()
    $latestFull = $bins | Where-Object { $_.Name -match "full" } | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    $latestApp = $bins | Where-Object { $_.Name -match "app" } | Sort-Object LastWriteTime -Descending | Select-Object -First 1

    if ($null -ne $latestFull) { $selected += $latestFull }
    if ($null -ne $latestApp) { $selected += $latestApp }

    if ($selected.Count -eq 0) {
        $selected += ($bins | Select-Object -Last 1)
    }

    return $selected
}

if (-not (Test-Path -Path $BackupDir)) {
    throw "Backup folder not found: $BackupDir"
}

$targetBins = Get-TargetBins -Dir $BackupDir -LatestOnly:$UseLatestOnly
if ($targetBins.Count -eq 0) {
    throw "No .bin files found in: $BackupDir"
}

$analysisDir = Join-Path $BackupDir "analysis"
if (-not (Test-Path -Path $analysisDir)) {
    New-Item -ItemType Directory -Path $analysisDir | Out-Null
}

$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$summaryPath = Join-Path $analysisDir "analysis_summary_$timestamp.txt"

$keywords = @(
    "ssid", "pass", "password", "wifi", "wlan", "http", "https", "mqtt", "api", "token", "key", "ota", "espota",
    "st7789", "ili9341", "tft", "display", "bmp280", "qmi", "imu", "i2c", "spi", "gpio", "boot", "reset",
    "arduino", "esp32", "tenstar", "feather"
)
$keywordPattern = ($keywords -join "|")
$urlPattern = 'https?://[^\s"''<>]+'
$macPattern = "\b([0-9A-Fa-f]{2}[:-]){5}[0-9A-Fa-f]{2}\b"
$pinPattern = "(GPIO\s*\d+|TFT_[A-Z0-9_]+|PIN\s*\d+)"

$summary = New-Object System.Collections.Generic.List[string]
$summary.Add("Firmware backup analysis")
$summary.Add("Timestamp: $timestamp")
$summary.Add("BackupDir: $BackupDir")
$summary.Add("Files analyzed: $($targetBins.Count)")
$summary.Add("")

foreach ($bin in $targetBins) {
    Write-Host "Analyzing $($bin.Name)..." -ForegroundColor Cyan

    $bytes = [System.IO.File]::ReadAllBytes($bin.FullName)
    $strings = Get-PrintableStrings -Bytes $bytes -MinLength $MinStringLength

    $stringsPath = Join-Path $analysisDir "$($bin.BaseName).strings.txt"
    $strings | Set-Content -Path $stringsPath -Encoding UTF8

    $keywordMatches = @($strings | Where-Object { $_ -match $keywordPattern } | Select-Object -First $MaxMatchesPerFile)
    $keywordPath = Join-Path $analysisDir "$($bin.BaseName).matches.txt"
    $keywordMatches | Set-Content -Path $keywordPath -Encoding UTF8

    $urls = @($strings |
        Select-String -Pattern $urlPattern -AllMatches |
        ForEach-Object { $_.Matches.Value } |
        Sort-Object -Unique)
    $urlsPath = Join-Path $analysisDir "$($bin.BaseName).urls.txt"
    $urls | Set-Content -Path $urlsPath -Encoding UTF8

    $macs = @($strings |
        Select-String -Pattern $macPattern -AllMatches |
        ForEach-Object { $_.Matches.Value } |
        Sort-Object -Unique)
    $macPath = Join-Path $analysisDir "$($bin.BaseName).macs.txt"
    $macs | Set-Content -Path $macPath -Encoding UTF8

    $pinHints = @($strings | Where-Object { $_ -match $pinPattern } | Select-Object -First $MaxMatchesPerFile)
    $pinPath = Join-Path $analysisDir "$($bin.BaseName).pins.txt"
    $pinHints | Set-Content -Path $pinPath -Encoding UTF8

    $summary.Add("File: $($bin.Name)")
    $summary.Add("  Raw size: $([math]::Round($bin.Length / 1KB, 2)) KB")
    $summary.Add("  Strings extracted: $($strings.Count)")
    $summary.Add("  Keyword matches: $($keywordMatches.Count)")
    $summary.Add("  URL hits: $($urls.Count)")
    $summary.Add("  MAC hits: $($macs.Count)")
    $summary.Add("  Pin hints: $($pinHints.Count)")
    $summary.Add("  Output:")
    $summary.Add("    - $stringsPath")
    $summary.Add("    - $keywordPath")
    $summary.Add("    - $urlsPath")
    $summary.Add("    - $macPath")
    $summary.Add("    - $pinPath")
    $summary.Add("")
}

$summary | Set-Content -Path $summaryPath -Encoding UTF8

Write-Host "" 
Write-Host "Analysis complete." -ForegroundColor Green
Write-Host "Summary: $summaryPath" -ForegroundColor Green
Write-Host ""
Write-Host "Quick start:" -ForegroundColor Yellow
Write-Host "  1) Open the summary file" -ForegroundColor White
Write-Host "  2) Check *.matches.txt for config hints" -ForegroundColor White
Write-Host "  3) Check *.pins.txt for display/sensor pin clues" -ForegroundColor White
Write-Host "  4) Check *.urls.txt for library/docs/endpoints" -ForegroundColor White
