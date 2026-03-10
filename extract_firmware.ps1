# TENSTAR ESP32-S3 Firmware Extraction Script
# Extracts the existing firmware from the board for analysis

$PORT = "COM4"
$OUTPUT_DIR = ".\firmware_backup"

Write-Host "`n=== ESP32-S3 Firmware Extraction ===" -ForegroundColor Cyan
Write-Host "Port: $PORT" -ForegroundColor Yellow

# Create output directory
if (-not (Test-Path $OUTPUT_DIR)) {
    New-Item -ItemType Directory -Path $OUTPUT_DIR | Out-Null
    Write-Host "Created backup directory: $OUTPUT_DIR" -ForegroundColor Green
}

# Check if esptool is available via Python
Write-Host "`nChecking for esptool..." -ForegroundColor Yellow
$esptoolCheck = python -m esptool version 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: esptool not found!" -ForegroundColor Red
    Write-Host "Install with: pip install esptool" -ForegroundColor Yellow
    exit 1
}

Write-Host "esptool found!" -ForegroundColor Green

# Extract full flash (4MB)
Write-Host "`nExtracting full flash (4MB) - this takes ~2 minutes..." -ForegroundColor Cyan
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$outputFile = Join-Path $OUTPUT_DIR "firmware_full_$timestamp.bin"

python -m esptool --chip esp32s3 --port $PORT --baud 921600 read_flash 0x0 0x400000 $outputFile

if ($LASTEXITCODE -eq 0) {
    Write-Host "`nSUCCESS! Firmware saved to:" -ForegroundColor Green
    Write-Host "  $outputFile" -ForegroundColor White
    
    $fileSize = (Get-Item $outputFile).Length
    Write-Host "  Size: $([math]::Round($fileSize/1MB, 2)) MB" -ForegroundColor White
    
    # Also extract just the app partition
    Write-Host "`nExtracting app partition only..." -ForegroundColor Cyan
    $appFile = Join-Path $OUTPUT_DIR "firmware_app_$timestamp.bin"
    python -m esptool --chip esp32s3 --port $PORT --baud 921600 read_flash 0x10000 0x200000 $appFile
    
    Write-Host "`nBackup complete! Files:" -ForegroundColor Green
    Get-ChildItem $OUTPUT_DIR -Filter "*.bin" | Format-Table Name, @{Label="Size (MB)"; Expression={[math]::Round($_.Length/1MB, 2)}}
    
    Write-Host "`nTo analyze the firmware:" -ForegroundColor Yellow
    Write-Host "  - Use 'strings' command to find text/config" -ForegroundColor White
    Write-Host "  - Search for WiFi credentials, API keys, etc." -ForegroundColor White
    Write-Host "  - Check for library versions and configs" -ForegroundColor White
    
} else {
    Write-Host "`nERROR: Extraction failed!" -ForegroundColor Red
    Write-Host "Make sure the board is connected and not in use." -ForegroundColor Yellow
}
