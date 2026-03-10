$pio = "C:\Users\lpf\.platformio\penv\Scripts\platformio.exe"

& $pio run --target upload -e tenstar_esp32s3_tft
if ($LASTEXITCODE -eq 0) {
    Write-Host "Waiting for board to restart..." -ForegroundColor Yellow
    Start-Sleep -Seconds 5
    & $pio device monitor
}
