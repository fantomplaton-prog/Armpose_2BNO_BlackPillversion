# STM32 Toolchain installation script for Windows
# Run with admin rights: Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process; .\install-toolchain.ps1

Write-Host "=== STM32 Toolchain Installation ===" -ForegroundColor Cyan

# 1. CMake
Write-Host "`n[1/5] Installing CMake..." -ForegroundColor Yellow
winget install Kitware.CMake --accept-source-agreements -e 2>$null
if ($LASTEXITCODE -ne 0) {
    choco install cmake -y 2>$null
}

# 2. Ninja
Write-Host "[2/5] Installing Ninja..." -ForegroundColor Yellow
winget install Ninja-build.Ninja --accept-source-agreements -e 2>$null
if ($LASTEXITCODE -ne 0) {
    choco install ninja -y 2>$null
}

# 3. ARM GNU Toolchain
Write-Host "[3/5] Installing ARM GNU Toolchain..." -ForegroundColor Yellow
winget install xPack.ARMToolchain --accept-source-agreements -e 2>$null
if ($LASTEXITCODE -ne 0) {
    choco install gcc-arm-embedded -y 2>$null
}

# 4. STM32CubeCLT (ST-LINK GDB Server + STM32_Programmer_CLI)
Write-Host "[4/5] STM32CubeCLT installation..." -ForegroundColor Yellow
Write-Host "  [!] Download STM32CubeCLT from https://www.st.com/en/development-tools/stm32cubeclt.html" -ForegroundColor Magenta
Write-Host "  [!] Extract and add to PATH: C:\Program Files\STM32CubeCLT\tools\bin" -ForegroundColor Magenta

# 5. VS Code Extensions
Write-Host "[5/5] Installing VS Code extensions..." -ForegroundColor Yellow
code --install-extension marus25.cortex-debug 2>$null
code --install-extension ms-vscode.cmake-tools 2>$null
code --install-extension ms-vscode.cpptools 2>$null

Write-Host "`n=== Checking Installation ===" -ForegroundColor Cyan
$tools = @('cmake', 'ninja', 'arm-none-eabi-gcc', 'arm-none-eabi-gdb')
foreach ($tool in $tools) {
    $found = Get-Command $tool -ErrorAction SilentlyContinue
    if ($found) {
        Write-Host "  [OK] ${tool}: $($found.Source)" -ForegroundColor Green
    } else {
        Write-Host "  [--] ${tool}: not found" -ForegroundColor Yellow
    }
}

Write-Host "`n=== Adding Paths to PATH ===" -ForegroundColor Cyan
$paths = @(
    'C:\Program Files\CMake\bin',
    'C:\Program Files\ninja',
    'C:\Program Files (x86)\GNU Arm Embedded Toolchain\bin',
    'C:\ProgramData\xpack\xpack-arm-toolchain\xpack\arm-none-eabi\bin',
    "$env:LOCALAPPDATA\xpack\xpack-arm-toolchain\xpack\arm-none-eabi\bin"
)

foreach ($path in $paths) {
    if (Test-Path $path) {
        Write-Host "  [+] Adding: $path" -ForegroundColor Green
        $env:PATH = "${path};$env:PATH"
    }
}

Write-Host "`n[OK] Installation complete!" -ForegroundColor Green
Write-Host "  Reload VS Code: File > Reload Window" -ForegroundColor Cyan
