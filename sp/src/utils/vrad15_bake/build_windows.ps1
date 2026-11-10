# Builds vrad15_bake.exe and copies it and its DLLs to the folder holding vrad.exe.
# Needs CMake and Visual Studio with the x64 C++ tools.
# Usage: powershell -ExecutionPolicy Bypass -File build_windows.ps1 -BinDir "C:\path\to\game\bin"
param([Parameter(Mandatory=$true)][string]$BinDir)
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$deps = Join-Path $here 'deps'
New-Item -ItemType Directory -Force $deps | Out-Null
$packages = @{
  embree = 'https://github.com/RenderKit/embree/releases/download/v4.3.3/embree-4.3.3.x64.windows.zip'
  oidn   = 'https://github.com/RenderKit/oidn/releases/download/v2.3.3/oidn-2.3.3.x64.windows.zip'
}
$roots = @()
foreach ($name in $packages.Keys) {
  $dest = Join-Path $deps $name
  if (-not (Test-Path $dest)) {
    $zip = "$dest.zip"
    Invoke-WebRequest $packages[$name] -OutFile $zip
    Expand-Archive $zip $dest
    Remove-Item $zip
  }
  # Some archives wrap everything in one folder; use whichever level holds bin\.
  $root = Get-ChildItem $dest -Recurse -Directory -Filter bin | Select-Object -First 1 | ForEach-Object { $_.Parent.FullName }
  $roots += $root
}
$build = Join-Path $here 'build'
cmake -S $here -B $build -A x64 "-DCMAKE_PREFIX_PATH=$($roots -join ';')"
if ($LASTEXITCODE) { throw 'CMake configure failed' }
cmake --build $build --config Release
if ($LASTEXITCODE) { throw 'Build failed' }
Copy-Item (Join-Path $build 'Release\vrad15_bake.exe') $BinDir -Force
foreach ($root in $roots) { Copy-Item (Join-Path $root 'bin\*.dll') $BinDir -Force }
Write-Host "Installed vrad15_bake.exe to $BinDir"
