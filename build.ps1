param(
    [Parameter(Mandatory = $false)]
    [string]$KspDir = $env:KSPDIR,

    [Parameter(Mandatory = $false)]
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release"
)

if ([string]::IsNullOrWhiteSpace($KspDir)) {
    throw "KSP directory is required. Pass -KspDir or set the KSPDIR environment variable."
}

$managed = Join-Path $KspDir "KSP_x64_Data\Managed"
if (-not (Test-Path (Join-Path $managed "Assembly-CSharp.dll"))) {
    $managed = Join-Path $KspDir "KSP_Data\Managed"
}
if (-not (Test-Path (Join-Path $managed "Assembly-CSharp.dll"))) {
    throw "Could not find KSP managed assemblies under '$managed'. Check -KspDir."
}

$localDotnet = Join-Path $PSScriptRoot ".dotnet\dotnet.exe"
$localLinuxDotnet = Join-Path $PSScriptRoot ".dotnet-linux-net8\dotnet"
$dotnet = if (Test-Path $localDotnet) { $localDotnet } else { "dotnet" }
if ($IsLinux -and (Test-Path $localLinuxDotnet)) {
    $dotnet = $localLinuxDotnet
}

$env:DOTNET_CLI_HOME = Join-Path $PSScriptRoot ".dotnet_home"
$env:NUGET_PACKAGES = Join-Path $PSScriptRoot ".nuget\packages"
$env:DOTNET_SKIP_FIRST_TIME_EXPERIENCE = "1"
$env:DOTNET_NOLOGO = "1"

$projectPath = Join-Path $PSScriptRoot "Source\KerbalLiDAR\KerbalLiDAR.csproj"
$assetsPath = Join-Path $PSScriptRoot "Source\KerbalLiDAR\obj\project.assets.json"
$nugetConfigPath = Join-Path $PSScriptRoot "NuGet.Config"
$restoreArgs = @()
if ((Test-Path $assetsPath) -and
    (Get-Item $assetsPath).LastWriteTimeUtc -ge (Get-Item $projectPath).LastWriteTimeUtc -and
    (Get-Item $assetsPath).LastWriteTimeUtc -ge (Get-Item $nugetConfigPath).LastWriteTimeUtc) {
    $restoreArgs += "--no-restore"
    Write-Host "Using cached NuGet assets: $assetsPath"
}

& $dotnet build $projectPath -c $Configuration -p:KSPDIR="$KspDir" -p:KSPManagedDir="$managed" @restoreArgs --configfile $nugetConfigPath
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$pluginDir = Join-Path $PSScriptRoot "GameData\KerbalLiDAR\Plugins"
New-Item -ItemType Directory -Force -Path $pluginDir | Out-Null

$pluginDll = Join-Path $pluginDir "KerbalLiDAR.dll"
$pluginPdb = Join-Path $pluginDir "KerbalLiDAR.pdb"
$objDir = Join-Path $PSScriptRoot "Source\KerbalLiDAR\obj\$Configuration"
$objDll = Join-Path $objDir "KerbalLiDAR.dll"
$objPdb = Join-Path $objDir "KerbalLiDAR.pdb"

if (-not (Test-Path $pluginDll) -and (Test-Path $objDll)) {
    Copy-Item -Force $objDll $pluginDll
}

if (-not (Test-Path $pluginPdb) -and (Test-Path $objPdb)) {
    Copy-Item -Force $objPdb $pluginPdb
}

Write-Host "KerbalLiDAR mod folder is ready at: $((Join-Path $PSScriptRoot 'GameData\KerbalLiDAR'))"
exit 0
