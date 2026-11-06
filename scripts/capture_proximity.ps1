param([int]$Round = 34)
$ErrorActionPreference = 'Stop'
$taskRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$taskOutput = Join-Path $taskRoot "target/judging/round$Round"
[IO.Directory]::CreateDirectory($taskOutput) | Out-Null
Push-Location $taskRoot
try {
    cargo build -p mp-app --example proximity_capture
    if ($LASTEXITCODE -ne 0) { throw 'Radar capture build failed' }
    $taskExe = Join-Path $taskRoot 'target/debug/examples/proximity_capture.exe'
    $taskHash = (Get-FileHash -LiteralPath $taskExe -Algorithm SHA256).Hash
    foreach ($taskCase in @(@('normal',430,391,'copter'), @('small',280,260,'copter'), @('plane',650,500,'plane'))) {
        $taskPng = Join-Path $taskOutput ("rust-" + $taskCase[0] + '.png')
        # The fixture exports only its own rendered content; no desktop capture/input.
        $taskProcess = Start-Process -FilePath $taskExe -ArgumentList @(('"' + $taskPng + '"'),$taskCase[1],$taskCase[2],$taskCase[3]) -WindowStyle Hidden -PassThru
        if (-not $taskProcess.WaitForExit(25000)) { throw 'Radar capture timed out' }
        if ($taskProcess.ExitCode -ne 0) { throw 'Radar capture failed' }
        $taskReceipt = Get-Content -LiteralPath ([IO.Path]::ChangeExtension($taskPng, 'capture.json')) -Raw | ConvertFrom-Json
        if ($taskReceipt.width -ne $taskCase[1] -or $taskReceipt.height -ne $taskCase[2]) { throw 'Unexpected capture dimensions' }
        $taskRef = "target/judging/round$Round/reference-" + $taskCase[0] + '.png'
        if ($taskCase[3] -eq 'plane') {
            & "$PSScriptRoot/render_proximity_reference.ps1" -Width $taskCase[1] -Height $taskCase[2] -Plane -Output $taskRef
        } else {
            & "$PSScriptRoot/render_proximity_reference.ps1" -Width $taskCase[1] -Height $taskCase[2] -Output $taskRef
        }
        [ordered]@{ executable_sha256=$taskHash; rust_receipt=$taskReceipt; reference=$taskRef } | ConvertTo-Json -Depth 5 |
            Set-Content -LiteralPath (Join-Path $taskOutput ($taskCase[0] + '.manifest.json')) -Encoding UTF8
    }
} finally { Pop-Location }
