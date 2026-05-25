$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent $scriptDir

$uvprojx = Get-ChildItem "$projectRoot\MDK-ARM" -Filter "*.uvprojx" | Select-Object -First 1

if ($null -eq $uvprojx) {
    Write-Host "No .uvprojx file found in MDK-ARM."
    exit 1
}

# 先备份一份，防止再次损坏
$backup = $uvprojx.FullName + ".bak"
Copy-Item $uvprojx.FullName $backup -Force

# 读取工程文件
$content = [System.IO.File]::ReadAllText($uvprojx.FullName)

# 只替换 FreeRTOS portable 路径，不改其他 XML 结构
$content = $content.Replace("FreeRTOS/Source/portable/RVDS/", "FreeRTOS/Source/portable/GCC/")
$content = $content.Replace("FreeRTOS\Source\portable\RVDS\", "FreeRTOS\Source\portable\GCC\")

# 用 UTF-8 无 BOM 写回，Keil 更稳定
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText($uvprojx.FullName, $content, $utf8NoBom)

Write-Host "FreeRTOS port path has been changed from RVDS to GCC for AC6."