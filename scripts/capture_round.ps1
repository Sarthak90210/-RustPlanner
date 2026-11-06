param([int]$Round = 7, [int]$From = 1, [int]$Count = 50)
$ErrorActionPreference = 'Stop'
$workspaceRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$captureRoot = Join-Path $workspaceRoot ('target\judging\round{0:D2}' -f $Round)
$captureExe = Join-Path $workspaceRoot 'target\debug\missionplanner.exe'
New-Item -ItemType Directory -Force -Path $captureRoot | Out-Null
$desktopData = Get-ItemProperty -LiteralPath 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Explorer\VirtualDesktops'
$desktopOne = ([Guid]::new([byte[]]$desktopData.VirtualDesktopIDs[0..15])).ToString()
$binaryHash = (Get-FileHash -LiteralPath $captureExe -Algorithm SHA256).Hash
# Each process exports its own renderer output and exits. No desktop input, shell
# screenshots, external capture helper, background watcher, or hardware connection.
$cases = @(
 @('01_data_disconnected','data',0,0,0,0,4),
 @('02_data_quick_flying','data',1,1,0,0,20),
 @('03_data_actions','data',1,0,1,0,7),
 @('04_data_messages','data',1,1,2,0,10),
 @('05_data_status','data',1,0,6,0,7),
 @('06_data_tlogs','data',1,0,11,0,7),
 @('07_plan_mission','plan',1,1,0,0,12),
 @('08_plan_disconnected','plan',0,0,0,0,4),
 @('09_setup_firmware','setup',0,0,0,1,7),
 @('10_setup_accel','setup',1,0,0,7,7),
 @('11_setup_radio','setup',1,0,0,9,7),
 @('12_setup_flightmodes','setup',1,0,0,13,7),
 @('13_setup_rtk','setup',0,0,0,19,4),
 @('14_config_fullparams','config',1,0,0,8,8),
 @('15_config_basictuning','config',1,0,0,1,7),
 @('16_config_extendedtuning','config',1,0,0,2,7),
 @('17_config_geofence','config',1,0,0,0,7),
 @('18_config_planner','config',0,0,0,10,4),
 @('19_simulation','sim',0,0,0,0,4),
 @('20_help','help',0,0,0,0,4),
 @('21_plan_survey_simple','plan',0,0,0,0,4,'MP_SURVEY','0'),
 @('22_plan_survey_camera','plan',0,0,0,0,4,'MP_SURVEY','2'),
 @('23_setup_frametype','setup',1,0,0,6,7),
 @('24_setup_compass','setup',1,0,0,8,7),
 @('25_setup_servo','setup',1,0,0,10,7),
 @('26_setup_esc','setup',1,0,0,12,7),
 @('27_setup_failsafe','setup',1,0,0,14,7),
 @('28_setup_hwid','setup',1,0,0,16,7),
 @('29_setup_battery','setup',1,0,0,23,7),
 @('30_setup_advanced','setup',0,0,0,41,4),
 @('31_mavlink_inspector','data',1,0,0,0,7,'MP_INSPECTOR','1'),
 @('32_ctrl_f_temp','data',0,0,0,0,4,'MP_TEMP','1'),
 @('33_config_mavftp','config',1,0,0,6,8),
 @('34_setup_sikradio','setup',0,0,0,21,4),
 @('35_setup_camera_gimbal','setup',1,0,0,33,7),
 @('36_setup_initial_params','setup',1,0,0,15,7),
 @('37_setup_joystick','setup',0,0,0,26,4),
 @('38_data_swarm','data',0,0,0,0,10,'MP_SWARM','1'),
 @('39_data_transponder','data',1,0,5,0,7),
 @('40_data_payload','data',1,0,10,0,7),
 @('41_setup_secure','setup',0,0,0,3,4),
 @('42_script_repl','setup',1,0,0,43,7),
 @('43_onboard_osd','config',1,0,0,5,7),
 @('44_setup_serial','setup',1,0,0,11,7),
 @('45_hardware_osd','setup',1,0,0,32,7),
 @('46_gps_order','setup',1,0,0,22,7),
 @('47_adsb','setup',1,0,0,17,7),
 @('48_cubeid','setup',1,0,0,20,7),
 @('49_dronecan','setup',0,0,0,25,4),
 @('50_antenna_tracker','setup',0,0,0,39,4),
 @('51_plan_utm','plan',1,1,0,0,12,'MP_COORDS','UTM'),
 @('52_plan_mgrs','plan',1,1,0,0,12,'MP_COORDS','MGRS'),
 @('53_swarm_formation','data',0,0,0,0,14,'MP_SWARM','1','MP_TOOL','Swarm'),
 @('54_swarm_sequence','data',0,0,0,0,14,'MP_SWARM','1','MP_TOOL','Sequence Swarm'),
 @('55_swarm_follow_leader','data',0,0,0,0,14,'MP_SWARM','1','MP_TOOL','Follow the leader'),
 @('56_swarm_follow_swarm','data',0,0,0,0,14,'MP_SWARM','1','MP_TOOL','Follow Swarm')
)
$last = [Math]::Min($cases.Count, $From - 1 + $Count) - 1
for ($ci = $From - 1; $ci -le $last; $ci++) {
    $case = $cases[$ci]
    foreach ($name in @('MP_AUTOSIM','MP_DEMO','MP_SURVEY','MP_INSPECTOR','MP_TEMP','MP_SWARM','MP_COORDS','MP_TOOL')) {
        Remove-Item -LiteralPath "Env:$name" -ErrorAction SilentlyContinue
    }
    $env:MP_COORDS = 'GEO'
    $env:MP_DESKTOP = $desktopOne
    $env:MP_DATA_DIR = Join-Path $captureRoot 'session-data'
    $env:MP_VIEW = $case[1]
    if ($case[2]) { $env:MP_AUTOSIM = '1' }
    if ($case[3]) { $env:MP_DEMO = '1' }
    $env:MP_FDTAB = [string]$case[4]
    $env:MP_PAGE = [string]$case[5]
    $env:MP_CAPTURE_AFTER = [string]$case[6]
    $env:MP_CAPTURE = Join-Path $captureRoot ($case[0] + '.png')
    if ($case.Count -gt 7) { Set-Item -Path ('Env:' + $case[7]) -Value $case[8] }
    if ($case.Count -gt 9) { Set-Item -Path ('Env:' + $case[9]) -Value $case[10] }
    for ($attempt = 1; $attempt -le 2; $attempt++) {
      try {
        $started = [DateTime]::UtcNow
        $process = $null
        try {
            $process = Start-Process -FilePath $captureExe -WindowStyle Hidden -PassThru -RedirectStandardError (Join-Path $captureRoot ($case[0] + '.stderr.log')) -RedirectStandardOutput (Join-Path $captureRoot ($case[0] + '.stdout.log'))
            if (-not $process.WaitForExit(($case[6] + 40) * 1000)) { throw 'Capture did not exit before its deadline' }
            $file = Get-Item -LiteralPath $env:MP_CAPTURE
            if ($file.LastWriteTimeUtc -lt $started) { throw 'Capture is stale' }
            $receipt = Get-Content -LiteralPath ([IO.Path]::ChangeExtension($env:MP_CAPTURE,'capture.json')) -Raw | ConvertFrom-Json
            if ($receipt.status -ne 'ok') { throw 'Capture receipt did not report success' }
            if (($case[2] -or $env:MP_SWARM) -and -not $receipt.runtime.params_complete) { throw 'Simulator parameters were not complete' }
            [ordered]@{ name=$case[0]; binary_sha256=$binaryHash; desktop=$desktopOne; pid=$process.Id; exit_code=$process.ExitCode; captured_utc=$file.LastWriteTimeUtc.ToString('o'); receipt=$receipt } |
                ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $captureRoot ($case[0] + '.manifest.json')) -Encoding UTF8
            Write-Output ($case[0] + ' saved, parameters=' + $receipt.runtime.parameters)
        } finally {
            if ($process -and -not $process.HasExited) { Stop-Process -Id $process.Id -ErrorAction SilentlyContinue }
        }
        break
      } catch {
        if ($attempt -ge 2) { throw }
        Write-Output ($case[0] + ' retry: ' + $_.Exception.Message)
      }
    }
}
