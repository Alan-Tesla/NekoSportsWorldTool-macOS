# NekoSportsWorldTool macOS

基于 [YanamiNeko/NekoSportsWorldTool](https://github.com/YanamiNeko/NekoSportsWorldTool) 与 [zxxhz 二开版本](https://github.com/zxxhz/NekoSportsWorldTool) 整理的 macOS 改进版。保留原项目历史与 CC BY-NC 4.0 许可证。原项目说明见 [README.upstream.md](README.upstream.md)。

## 2026-10-17 更新

- 合并路线方案，保留基于主干道的随机路线生成、覆盖和里程校验；配置保存在本机。
- 随机结果保留选中状态，生成失败显示具体原因；修正转弯几何与新增立即折返。
- 配速叠加缓慢随机变化，在总用时60%后逐渐减速；“换一版”刷新随机种子。
- 修复图表配速单位（分钟/公里）和起点时间标记；历史记录不会自动修改。
- 本机release构建与125项测试通过；第三方详情图显示仍待新记录验证。

## 本版改进

- macOS 应用目录与可写数据目录适配，内置中文字体。
- 按纬度计算经纬度距离，改善路网显示、距离与点位半径的一致性。
- 距离、时间、步数与分段统计统一计算，修正不足十秒的末窗及分段时间。
- 配速模型保留“macOS 平稳配速版”：平滑热身、缓慢随机变化及后程渐进减速。
- 增加数据一致性检查与本地模拟测试。模拟曲线不是实测运动数据。
- macOS 禁用上游自动覆盖更新，避免替换掉本版修改。

## 在 Mac 上构建

安装 Xcode Command Line Tools（`xcode-select --install`）和 Rust stable 工具链后运行：

```sh
cargo test --locked --workspace --lib --tests
cargo build --release --locked
./target/release/nekosportsworldtool
```

默认构建包含图形界面。Apple Silicon 本机已验证 release 构建；Intel Mac 需要在对应机器或交叉编译环境另行验证。

已有 macOS 应用包在本机通过启动验证；应用包同级的 `NekoSportsWorldTool-data` 目录用于保存配置与会话，请保持可写。

## 发布内容

仓库不包含个人账号、会话、学校打卡点缓存或个人路线。测试路线是以海上虚构坐标构造的圆环，不对应校园或真实跑步记录。运行时文件已加入 `.gitignore`，不要强制提交本地配置或数据目录。

## 来源与许可

原作者：YanamiNeko；二开来源：zxxhz/NekoSportsWorldTool。合并基线：`ecafd0d93a006a58fa08fe06d9215bc92721f21f`（完整准确版本以 Git 历史为准）。本版保留 [LICENSE](LICENSE)，仅限 CC BY-NC 4.0 许可范围内使用。字体与 vendored 依赖保留各自许可证。
