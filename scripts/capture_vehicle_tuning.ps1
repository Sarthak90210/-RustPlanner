param([int]$Round = 12)
$ErrorActionPreference = 'Stop'
$workspaceRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$captureRoot = Join-Path $workspaceRoot ('target\judging\round{0:D2}' -f $Round)
$captureExe = Join-Path $workspaceRoot 'target\debug\missionplanner.exe'
New-Item -ItemType Directory -Force -Path $captureRoot | Out-Null
$desktopData = Get-ItemProperty -LiteralPath 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Explorer\VirtualDesktops'
$env:MP_DESKTOP = ([Guid]::new([byte[]]$desktopData.VirtualDesktopIDs[0..15])).ToString()
$env:MP_AUTOSIM = '1'
# listeners on 127.0.0.1 only: no Windows Firewall prompt for each new build
$env:MP_LOOPBACK_ONLY = '1'
$env:MP_VIEW = 'config'
$env:MP_CAPTURE_AFTER = '7'
$env:MP_DATA_DIR = Join-Path $captureRoot 'vehicle-tuning-profile'
$binaryHash = (Get-FileHash -LiteralPath $captureExe -Algorithm SHA256).Hash
foreach ($case in @(@('plane','1','ArduPlane'), @('rover','1','ArduRover'), @('tracker','2','ArduTracker'))) {
    $env:MP_SIM_KIND = $case[0]
    $env:MP_PAGE = $case[1]
    $env:MP_CAPTURE = Join-Path $captureRoot ('config_' + $case[0] + '.png')
    $started = [DateTime]::UtcNow
    $process = Start-Process -FilePath $captureExe -WindowStyle Hidden -PassThru -RedirectStandardError (Join-Path $captureRoot ('config_' + $case[0] + '.stderr.log')) -RedirectStandardOutput (Join-Path $captureRoot ('config_' + $case[0] + '.stdout.log'))
    try {
        if (-not $process.WaitForExit(45000)) { throw 'Vehicle tuning capture exceeded its deadline' }
        $file = Get-Item -LiteralPath $env:MP_CAPTURE
        if ($file.LastWriteTimeUtc -lt $started) { throw 'Capture is stale' }
        $receipt = Get-Content -LiteralPath ([IO.Path]::ChangeExtension($env:MP_CAPTURE, 'capture.json')) -Raw | ConvertFrom-Json
        if ($receipt.status -ne 'ok' -or -not $receipt.runtime.params_complete -or $receipt.runtime.firmware -ne $case[2]) { throw 'Capture did not reach the intended firmware with complete parameters' }
        [ordered]@{ binary_sha256=$binaryHash; pid=$process.Id; captured_utc=$file.LastWriteTimeUtc.ToString('o'); receipt=$receipt } |
            ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $captureRoot ('config_' + $case[0] + '.manifest.json')) -Encoding UTF8
        Write-Output ($case[2] + ' saved, parameters=' + $receipt.runtime.parameters)
    } finally {
        if (-not $process.HasExited) { Stop-Process -Id $process.Id -ErrorAction SilentlyContinue }
    }
}
