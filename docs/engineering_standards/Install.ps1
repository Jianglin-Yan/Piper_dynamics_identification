<#
.SYNOPSIS
安装个人工程规范，并合并接收端 Agent 的规则入口。
.DESCRIPTION
仅替换 engineering-global-default 标记块，保留其他约定。
所有校验在写入前完成；变更前备份已存在文件。重复安装相同版本不重复追加。
Windows PowerShell 5.1 / PowerShell 7；Linux 使用 PowerShell 7 或按 README 手动安装。
#>
[CmdletBinding()]
param(
    [string]$RulesDirectory,
    [string]$EntryFile,
    [switch]$Preview
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$utf8 = [Text.UTF8Encoding]::new($false)
$codexDirectory = if ($env:CODEX_HOME) {
    $env:CODEX_HOME
} else {
    Join-Path ([Environment]::GetFolderPath('UserProfile')) '.codex'
}
if (-not $RulesDirectory) {
    $RulesDirectory = Join-Path $codexDirectory 'standards/engineering'
}
if (-not $EntryFile) {
    $EntryFile = Join-Path $codexDirectory 'AGENTS.md'
}
$rulesDestination = [IO.Path]::GetFullPath($RulesDirectory)
$entryDestination = [IO.Path]::GetFullPath($EntryFile)
$packageRoot = [IO.Path]::GetFullPath($PSScriptRoot)
$packagePrefix = $packageRoot.TrimEnd([IO.Path]::DirectorySeparatorChar) + [IO.Path]::DirectorySeparatorChar
$comparison = if ([IO.Path]::DirectorySeparatorChar -eq '\') {
    [StringComparison]::OrdinalIgnoreCase
} else {
    [StringComparison]::Ordinal
}
if ($rulesDestination.StartsWith($packagePrefix, $comparison) -or $rulesDestination.Equals($packageRoot, $comparison)) {
    throw 'Install outside the extracted package directory.'
}

# 校验包内规则和入口模板，避免路径越界及损坏文件被写入接收端。
$manifest = Get-Content -LiteralPath (Join-Path $packageRoot 'manifest.json') -Raw -Encoding UTF8 | ConvertFrom-Json
foreach ($item in $manifest.files) {
    $sourcePath = [IO.Path]::GetFullPath((Join-Path $packageRoot $item.path))
    if (-not $sourcePath.StartsWith($packagePrefix, $comparison)) {
        throw "Invalid package path: $($item.path)"
    }
    if ((Get-FileHash -LiteralPath $sourcePath -Algorithm SHA256).Hash -ne $item.sha256) {
        throw "Package checksum mismatch: $($item.path)"
    }
}

$templatePath = Join-Path $packageRoot 'AGENT_RULES.template.md'
$block = [IO.File]::ReadAllText($templatePath).Replace('{{RULES_DIRECTORY}}', $rulesDestination.Replace('\', '/'))
$startMarker = '<!-- engineering-global-default:start -->'
$endMarker = '<!-- engineering-global-default:end -->'
$existingEntry = if (Test-Path -LiteralPath $entryDestination) {
    [IO.File]::ReadAllText($entryDestination)
} else {
    ''
}
$startCount = [regex]::Matches($existingEntry, [regex]::Escape($startMarker)).Count
$endCount = [regex]::Matches($existingEntry, [regex]::Escape($endMarker)).Count
if ($startCount -ne $endCount -or $startCount -gt 1) {
    throw 'Existing rules markers are incomplete or duplicated; merge manually before installing.'
}
if ($startCount -eq 1) {
    $startOffset = $existingEntry.IndexOf($startMarker)
    $endOffset = $existingEntry.IndexOf($endMarker) + $endMarker.Length
    if ($endOffset -le $startOffset) {
        throw 'Existing rules marker order is invalid.'
    }
    $nextEntry = $existingEntry.Substring(0, $startOffset) + $block.TrimEnd() + $existingEntry.Substring($endOffset)
} else {
    $nextEntry = if ($existingEntry.Length) { $existingEntry.TrimEnd() + "`n`n" + $block } else { $block }
}

$changes = @()
foreach ($item in $manifest.files) {
    if (-not $item.path.StartsWith('rules/')) {
        continue
    }
    $relativePath = $item.path.Substring(6)
    $targetPath = [IO.Path]::GetFullPath((Join-Path $rulesDestination $relativePath))
    $targetPrefix = $rulesDestination.TrimEnd([IO.Path]::DirectorySeparatorChar) + [IO.Path]::DirectorySeparatorChar
    if (-not $targetPath.StartsWith($targetPrefix, $comparison) -or $targetPath.Equals($entryDestination, $comparison)) {
        throw "Invalid rules destination: $relativePath"
    }
    if (-not (Test-Path -LiteralPath $targetPath) -or (Get-FileHash -LiteralPath $targetPath -Algorithm SHA256).Hash -ne $item.sha256) {
        $changes += [pscustomobject]@{ Source = (Join-Path $packageRoot $item.path); Target = $targetPath }
    }
}
$entryChanged = $existingEntry -cne $nextEntry
Write-Output "Rules: $rulesDestination"
Write-Output "Agent entry: $entryDestination"
Write-Output "Changed rule files: $($changes.Count); entry changed: $entryChanged"
if ($Preview) {
    Write-Output 'Preview only; no files written.'
    return
}
if ($changes.Count -eq 0 -and -not $entryChanged) {
    Write-Output 'Already installed; no changes.'
    return
}

# 先备份全部受影响的旧文件。备份包含原路径，失败时可逐项恢复；不删除现有额外规则。
$backupDirectory = Join-Path ([IO.Path]::GetDirectoryName($entryDestination)) ('engineering-backup-' + [Guid]::NewGuid().ToString('N'))
[IO.Directory]::CreateDirectory($backupDirectory) | Out-Null
$restoreIndex = @()
$targets = @($changes | ForEach-Object { $_.Target })
if ($entryChanged) {
    $targets += $entryDestination
}
foreach ($targetPath in $targets) {
    $backupName = $null
    if (Test-Path -LiteralPath $targetPath) {
        $backupName = '{0:D3}.bak' -f $restoreIndex.Count
        Copy-Item -LiteralPath $targetPath -Destination (Join-Path $backupDirectory $backupName)
    }
    $restoreIndex += [pscustomobject]@{ target = $targetPath; backup = $backupName; existed = ($null -ne $backupName) }
}
[IO.File]::WriteAllText((Join-Path $backupDirectory 'restore-index.json'), (ConvertTo-Json -InputObject $restoreIndex -Depth 4), $utf8)
Write-Output "Backup: $backupDirectory"
foreach ($change in $changes) {
    [IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($change.Target)) | Out-Null
    Copy-Item -LiteralPath $change.Source -Destination $change.Target -Force
}
if ($entryChanged) {
    [IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($entryDestination)) | Out-Null
    [IO.File]::WriteAllText($entryDestination, $nextEntry, $utf8)
}
Write-Output 'Installed. Start a new agent session and ask it to confirm the loaded rules path.'
