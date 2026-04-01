$comPort = "COM5"
$baudRate = 115200
$outputFile = "C:\VBS\vbs_cpp\pressure_log_$(Get-Date -Format 'yyyy-MM-dd_HHmmss').txt"

function vbsMove {
    param([int]$repeat, [int]$speed, [int]$value, [int]$waitTime)
    $list = @()
    1..$repeat | ForEach-Object { $list += ,@("move $value $speed", $waitTime) }
    return $list
}

$sequence = @()
$sequence += ,@("", 180000)
$sequence += vbsMove -repeat 27 -speed 2 -value 1720 -waitTime 20000
$sequence += vbsMove -repeat 1 -speed 2 -value 860 -waitTime 20000
$sequence += vbsMove -repeat 27 -speed 2 -value -1720 -waitTime 20000
$sequence += vbsMove -repeat 1 -speed 2 -value 46440 -waitTime 20000

$serial = New-Object System.IO.Ports.SerialPort($comPort, $baudRate, "None", 8, "One")
$serial.ReadTimeout = 100
$serial.WriteTimeout = 500

try {
    $serial.Open()
    Write-Host "Conectado em $comPort. Gravando em: $outputFile" -ForegroundColor Green
    
    # Cabeçalho do Log
    "PC_Timestamp,Pico_Timestamp,Pressure,Temperature,Encoder_Degrees" | Out-File -FilePath $outputFile -Encoding UTF8
    
    $step = 0
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    $nextActionTime = 0 # Momento em ms para a próxima ação

    while ($true) {
        # 1. LEITURA (Prioridade 1: Não perder dados do Pico)
        while ($serial.BytesToRead -gt 0) {
            try {
                $picoData = $serial.ReadLine().Trim()
                if ($picoData) {
                    # Adicionamos o tempo do PC no início para correlacionar eventos
                    $logLine = "$(Get-Date -Format 'HH:mm:ss.fff'),$picoData"
                    $logLine | Out-File $outputFile -Encoding UTF8 -Append
                    Write-Host $logLine
                }
            } catch { break }
        }
        # 2. ESCRITA (Lógica da Sequência)
        if ($step -lt $sequence.Count) {
            $currentTime = $timer.ElapsedMilliseconds
            
            if ($currentTime -ge $nextActionTime) {
                $msg = $sequence[$step][0]
                $delay = $sequence[$step][1]
                
                $serial.WriteLine($msg)
                Write-Host ">>> ENVIADO [$step]: $msg (Próximo em: $($delay/1000)s)" -ForegroundColor Cyan
                
                $nextActionTime = $currentTime + $delay
                $step++
            }
        }

        # Pequena pausa para alívio de CPU (não afeta a precisão de 100ms do Pico)
        Start-Sleep -Milliseconds 10
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
