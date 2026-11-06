param([int]$Width = 430, [int]$Height = 391, [switch]$Plane,
      [string]$Output = 'target/judging/round34/reference.png')
$ErrorActionPreference = 'Stop'
$taskRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$taskSourcePath = Join-Path $taskRoot 'reference/MissionPlanner/Controls/ProximityControl.cs'
$taskSource = [IO.File]::ReadAllText($taskSourcePath)
$taskStart = $taskSource.IndexOf('        private void Temp_Paint(')
$taskEnd = $taskSource.IndexOf('        public new void Show()', $taskStart)
if ($taskStart -lt 0 -or $taskEnd -lt 0) { throw 'Reference paint method boundaries changed' }
# Compile the unchanged reference paint method with data/type adapters only. Width and
# Height are explicitly the content fixture size, with no native form chrome. This
# verifies source drawing content, not a full Mission Planner form or application.
$taskPaint = $taskSource.Substring($taskStart, $taskEnd - $taskStart)
$taskSupport = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'proximity_reference_support.cs'))
Add-Type -ReferencedAssemblies System.Drawing,System.Windows.Forms -TypeDefinition ($taskSupport.Replace('/*REFERENCE_PAINT_METHOD*/', $taskPaint))
$taskFixture = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'fixtures/proximity.json') -Raw | ConvertFrom-Json
$taskHarness = New-Object ProximityPaintHarness
$taskHarness.Width = $Width
$taskHarness.Height = $Height
$taskHarness.screenradius = $taskFixture.radius_cm
$taskHarness.mavsize = $taskFixture.vehicle_cm
$taskHarness.Copter = -not $Plane
for ($taskIndex = 0; $taskIndex -lt $taskFixture.sensors_cm.Count; $taskIndex++) {
    $taskHarness.Add($taskIndex, 0, 45, $taskFixture.sensors_cm[$taskIndex])
}
foreach ($taskArray in $taskFixture.arrays) { $taskHarness.Add(100, $taskArray.angle, $taskArray.width, $taskArray.distance_cm) }
$taskOutput = [IO.Path]::GetFullPath((Join-Path $taskRoot $Output))
[IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($taskOutput)) | Out-Null
$taskHarness.Render($taskOutput, (Join-Path $taskRoot 'reference/MissionPlanner/Resources/quad2.png'))
[ordered]@{ source='unchanged C# Temp_Paint, isolated content harness'; source_sha256=(Get-FileHash -LiteralPath $taskSourcePath).Hash; width=$Width; height=$Height; copter=(-not $Plane); dpi=96; native_form_chrome=$false; default_font=[Drawing.SystemFonts]::DefaultFont.Name; fixture='scripts/fixtures/proximity.json' } |
    ConvertTo-Json | Set-Content -LiteralPath ([IO.Path]::ChangeExtension($taskOutput, 'json')) -Encoding UTF8
