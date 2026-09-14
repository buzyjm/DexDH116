# Release 发布与镜像同步指南

本文说明如何从本地 `packages/` 发布 LHandProLib Release，并把允许公开的
附件同步到 `sorrowfeng/LHandProLib-Releases`。

## 为什么必须从本地发布

`packages/` 已被 `.gitignore` 忽略，打包产物只存在于构建机器，GitHub
Actions checkout 无法取得这些文件。因此发布脚本在本地运行，Actions 只负责
在 `release.published` 后派发镜像同步任务。

本地发布还会使用当前用户的 `gh` 凭据创建 Release，使
`release.published` 能正常触发后续 workflow。不要恢复使用仓库
`GITHUB_TOKEN` 发布 Release 的旧 workflow。

## 发布前准备

1. 安装并登录 GitHub CLI：

   ```powershell
   gh auth login
   gh auth status
   ```

2. 确认待发布目录名是合法的 `YYYYMMDD` 日期，并且包含以下两个文件：

   ```text
   packages/<tag>/LHandProLib-API-Linux-<tag>.tar.gz
   packages/<tag>/LHandProLib-API-Windows-<tag>.7z
   ```

3. 确认仓库中存在两份发布手册：

   ```text
   docs/LHandProLib_SDK_Manual.md
   docs/LHandProLib_SDK_功能手册.md
   ```

发布脚本只上传上述两个平台包和两份手册。缺少任意文件都会终止发布，
`packages/<tag>` 中的其他文件不会上传。
Release 正文的 Downloads 会固定保留飞书镜像链接，不展示本地
`Package directory`。

## 本地发布

发布 `packages/` 中日期最新的目录：

```powershell
cd D:\Project\HandProject\LHandProLib
python .github\scripts\publish_latest_package_release.py
```

发布指定日期的历史包：

```powershell
python .github\scripts\publish_latest_package_release.py 20260710
```

脚本会先检查远端同名 tag。新 tag 读取当前 `HEAD` 的 Git 历史；重发已有 tag
时先取得远端 tag 的准确提交快照，避免后续 `HEAD` 变化影响本次重试。脚本
排除 merge commit，并把提交者时间换算为东八区日期，随后按
`上一有效 package 日期 < commit 日期 <= 当前 package 日期` 筛选、去重，优先
选择最近的有效提交生成最多 5 条 highlights。这里的“有效 package”要求两个
精确命名的平台归档都存在；首个有效 package 没有日期下界。只有窗口内不存在
有效提交时才使用维护类提交，窗口完全为空时使用 `Package update.`。该过程
不依赖本地 `CHANGELOG.md` 是否存在或是否及时更新。

本地 Git 历史必须完整包含待发布日期范围；脚本会拒绝浅克隆。Git 查询或记录
解析失败时发布会直接终止，不会用空白说明继续发布。可在发布前运行回归测试：

```powershell
python -B .github\scripts\test_publish_latest_package_release.py
```

完成 highlights 后，脚本会依次执行以下操作：

1. 远端同名 tag 不存在时，在当前提交创建并推送 tag。若本地已有同名 tag
   但它不指向当前提交，脚本会拒绝推送。
2. 新 Release 使用一次 `gh release create ... --verify-tag` 调用上传四个
   附件并发布。
3. 同 tag Release 已存在时，脚本会原地更新说明，并用一次
   `gh release upload ... --clobber` 更新四个附件，不会删除 Release 或改变
   其 URL。上传成功后会清除非公开附件和重复附件。若上次失败留下 draft，
   本次会在四项附件收敛后将其正式发布；prerelease 也会规范化为正式版本。

创建新 Release 时，`gh` 会在附件上传完成后才发布，因此镜像 workflow
收到 `release.published` 时可以看到完整附件集合。安全重发已有 Release
不会产生新的 `release.published` 事件，脚本会在更新和清理全部成功后直接
通过本地 `gh` 派发镜像同步。恢复并发布已有 draft 时会重新产生
`release.published`，因此该分支不额外重复派发。

## 自动镜像同步

源仓库 `.github/workflows/trigger-release-mirror.yml` 监听
`release.published`。它只接受八位日期 tag，并把本次 `release_tag` 派发给
`sorrowfeng/LHandProLib-Releases` 的 `sync-releases.yml`。客户专用或其他格式
的 tag 会被跳过。已有 Release 的原地重发则由本地发布脚本直接派发同一个
目标 workflow，避免依赖不会再次产生的 `release.published` 事件。

跨仓库派发和读取私有 Release 需要配置两个 Actions secret：

| 仓库 | Secret | 用途 |
| --- | --- | --- |
| `sorrowfeng/LHandProLib` | `MIRROR_REPO_TOKEN` | 对镜像仓库具有 Actions 写权限，用于派发 `sync-releases.yml` |
| `sorrowfeng/LHandProLib-Releases` | `SRC_REPO_TOKEN` | 对私有源仓库具有 Contents 读权限，用于读取 Release 与下载附件 |

本地重发已有 Release 时，直接派发使用当前 `gh` 登录用户的权限；该用户也
必须有权运行镜像仓库的 Actions workflow。

## 手动补同步

自动派发失败或需要重新校验历史 Release 时，运行：

```powershell
gh workflow run sync-releases.yml `
  --repo sorrowfeng/LHandProLib-Releases `
  --ref main `
  --field release_tag=20260710
```

如果镜像 workflow 因长期无活动被禁用，先重新启用：

```powershell
gh workflow enable sync-releases.yml --repo sorrowfeng/LHandProLib-Releases
```

## 检查与故障排查

```powershell
# 查看源仓库 Release
gh release view 20260710 --repo sorrowfeng/LHandProLib

# 查看镜像同步运行记录
gh run list --repo sorrowfeng/LHandProLib-Releases --workflow sync-releases.yml
```

- 提示发布文件缺失：检查文件名是否与 tag 完全匹配，且两份手册位于
  `docs/`。
- `gh` 鉴权失败：运行 `gh auth status`，或为当前进程提供有效的
  `GITHUB_TOKEN`。
- 自动派发失败：检查 `MIRROR_REPO_TOKEN` 是否存在且可对镜像仓库运行
  workflow。
- 镜像下载失败：检查目标仓库的 `SRC_REPO_TOKEN` 是否可读取私有源仓库。
