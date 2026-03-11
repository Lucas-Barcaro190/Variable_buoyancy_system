# Serial Port Logger for Pico
# Captures output from COM5 and saves to a file
# IMPORTANT: Close VS Code's serial monitor before running this script!

$comPort = "COM5"
$baudRate = 115200
$outputFile = "C:\VBS\vbs_cpp\potentiometer_log_$(Get-Date -Format 'yyyy-MM-dd_HHmmss').txt"

# Check if port is already in use
Write-Host "Checking if $comPort is available..." -ForegroundColor Yellow

# Create serial port object
$serial = New-Object System.IO.Ports.SerialPort
$serial.PortName = $comPort
$serial.BaudRate = $baudRate
$serial.Parity = "None"
$serial.DataBits = 8
$serial.StopBits = 1
$serial.ReadTimeout = 500
$serial.WriteTimeout = 500

try {
    $serial.Open()
    Write-Host "Connected to $comPort at $baudRate baud" -ForegroundColor Green
    Write-Host "Logging to: $outputFile" -ForegroundColor Green
    Write-Host "Press Ctrl+C to stop..." -ForegroundColor Yellow
    
    # Create/clear output file with header
    "Potentiometer Data Capture - $(Get-Date)" | Out-File -FilePath $outputFile -Encoding UTF8
    "===============================================" | Out-File -FilePath $outputFile -Encoding UTF8 -Append
    "" | Out-File -FilePath $outputFile -Encoding UTF8 -Append
    
    $lineCount = 0
    
    # Read from serial port continuously
    while ($true) {
        try {
            if ($serial.BytesToRead -gt 0) {
                $line = $serial.ReadLine()
                Write-Host $line  # Display in console
                $line | Out-File -FilePath $outputFile -Encoding UTF8 -Append  # Save to file
                $lineCount++
            }
        }
        catch {
            # Timeout or no data - just continue
        }
        Start-Sleep -Milliseconds 50
    }
}
catch {
    Write-Host "Error: $_" -ForegroundColor Red
    Write-Host "Make sure COM5 is not in use by VS Code!" -ForegroundColor Red
}
finally {
    if ($serial -and $serial.IsOpen) {
        $serial.Close()
        $serial.Dispose()
    }
    Write-Host "Connection closed. Logged $lineCount lines." -ForegroundColor Yellow
    Write-Host "Output saved to: $outputFile" -ForegroundColor Green
}
