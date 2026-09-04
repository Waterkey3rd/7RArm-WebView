$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'web\public\wasm'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

$emxx = Join-Path $projectRoot '.emscripten-env\Library\bin\em++.bat'
if (-not (Test-Path -LiteralPath $emxx)) {
    throw 'Project-local Emscripten is missing. Create .emscripten-env first.'
}
$node = (Get-Command node -ErrorAction Stop).Source
$env:EM_NODE_JS = $node.Replace('\', '/')
$env:EM_CACHE = Join-Path $projectRoot '.emscripten-cache'

& $emxx `
    (Join-Path $PSScriptRoot 'deploy_ik_api.cpp') `
    '-I' $projectRoot `
    '-std=c++20' '-O0' `
    '-sMODULARIZE=1' '-sEXPORT_ES6=1' '-sENVIRONMENT=web,worker,node' `
    '-sFILESYSTEM=0' '-sALLOW_MEMORY_GROWTH=1' `
    '-sEXPORTED_FUNCTIONS=["_malloc","_free","_deploy_ik_solve","_deploy_fk","_deploy_fk_chain","_deploy_get_joint_limits","_deploy_model_version","_deploy_trajectory_duration","_deploy_trajectory_sample"]' `
    '-sEXPORTED_RUNTIME_METHODS=["HEAPF64"]' `
    '-o' (Join-Path $outputDir 'deploy_ik.js')

if ($LASTEXITCODE -ne 0) { throw "Emscripten failed with exit code $LASTEXITCODE" }
Write-Host "WASM generated in $outputDir"
