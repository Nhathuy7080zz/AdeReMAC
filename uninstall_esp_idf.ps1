# =========================================================================================
# SCRIPT DỌN SẠCH TRIỆT ĐỂ ESP-IDF TRÊN WINDOWS (PowerShell)
# Cách chạy: Mở PowerShell -> chạy: powershell -ExecutionPolicy Bypass -File uninstall_esp_idf.ps1
# =========================================================================================

Write-Host "=== BAT DAU GO BO VA DON SACH ESP-IDF ===" -ForegroundColor Cyan

# 1. Xoa cac thu muc cai dat va cache cua ESP-IDF / Espressif
$folders = @(
    "C:\Espressif",
    "$env:USERPROFILE\.espressif",
    "$env:USERPROFILE\esp",
    "$env:LOCALAPPDATA\Espressif",
    "$env:APPDATA\Espressif",
    "$env:USERPROFILE\.cache\Espressif"
)

foreach ($dir in $folders) {
    if (Test-Path $dir) {
        Write-Host "Dang xoa thu muc: $dir ..." -ForegroundColor Yellow
        Remove-Item -Path $dir -Recurse -Force -ErrorAction SilentlyContinue
    }
}

# 2. Xoa cac bien moi truong IDF_* trong User va System
$envVars = @("IDF_PATH", "IDF_TOOLS_PATH", "IDF_PYTHON_ENV_PATH", "ESP_IDF_VERSION", "OPENOCD_SCRIPTS")
foreach ($v in $envVars) {
    [Environment]::SetEnvironmentVariable($v, $null, "User")
    [Environment]::SetEnvironmentVariable($v, $null, "Machine")
}

# 3. Loc bo cac duong dan Espressif / .espressif khoi bien PATH cua User
$userPath = [Environment]::GetEnvironmentVariable("Path", "User")
if ($userPath) {
    $cleanPath = ($userPath -split ";" | Where-Object {
        $_ -notmatch "Espressif" -and $_ -notmatch "\.espressif"
    }) -join ";"
    [Environment]::SetEnvironmentVariable("Path", $cleanPath, "User")
    Write-Host "Da lam sach bien PATH cua User!" -ForegroundColor Green
}

# 4. Xoa Registry Keys cua Espressif
$regPaths = @(
    "HKCU:\Software\Espressif",
    "HKLM:\SOFTWARE\Espressif",
    "HKLM:\SOFTWARE\WOW6432Node\Espressif"
)
foreach ($rp in $regPaths) {
    if (Test-Path $rp) {
        Write-Host "Dang xoa Registry: $rp ..." -ForegroundColor Yellow
        Remove-Item -Path $rp -Recurse -Force -ErrorAction SilentlyContinue
    }
}

# 5. Xoa Shortcut tren Desktop va Start Menu
$shortcuts = @(
    "$env:USERPROFILE\Desktop\ESP-IDF*",
    "$env:USERPROFILE\Desktop\Espressif*",
    "$env:APPDATA\Microsoft\Windows\Start Menu\Programs\ESP-IDF*",
    "$env:ProgramData\Microsoft\Windows\Start Menu\Programs\ESP-IDF*"
)
foreach ($sc in $shortcuts) {
    Remove-Item -Path $sc -Recurse -Force -ErrorAction SilentlyContinue
}

Write-Host "=== HOAN TAT! MAY TINH DA SACH HOAN TOAN ESP-IDF ===" -ForegroundColor Green
