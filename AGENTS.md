# AGENTS.md

## VS Code Clickable File Links

- Always output file references as Markdown links with an absolute filesystem path and optional line number using this format:
  - `[label](/<absolute-path>:line)`
- On Windows, use forward slashes and include the drive letter in the path, for example:
  - `[file.h](/c:/project/src/file.h:100)`
- Prefer this format for every file reference so links open directly in this VS Code chat environment.
- Do not use `vscode://` links for file references.

## Project context
- 本项目基于 PX4 Autopilot，主要开发内容涉及飞控功能扩展、驱动适配、控制分配、uORB 通信、参数系统、以及与外部执行器/传感器的接口联调。
- 修改代码时，优先保持与 PX4 现有架构、命名风格和模块边界一致。
- 非必要不要引入新的第三方依赖、不要大范围重构与当前任务无关的模块。

## Working style
- 先阅读相关调用链和模块入口，再修改实现。
- 优先做最小可验证修改（minimal diff）。
- 修改前先确认影响范围：模块启动、uORB topic、参数、定时调度、驱动接口、控制链路、日志输出。
- 若发现任务需求与当前实现不一致，先说明差异，再给出修改方案。
- 不要猜测协议字段、单位或坐标系；不确定时先在代码中定位定义来源。

## Build and test
- 修改后优先进行最小范围编译验证，避免无关全量构建。
- 常用构建命令：
  - `make px4_sitl_default`
  - `make px4_fmu-v5_default`
- 若修改仅限某个模块，优先确认该模块能通过编译并且依赖完整。
- 若修改启动脚本、参数、airframe、ROMFS、驱动注册、uORB 消息，必须提醒检查运行时加载链路。

## PX4-specific rules
- 涉及控制逻辑修改时，说明控制输入、约束、分配结果和输出路径。
- 涉及 uORB 时，明确：
  - topic 定义位置
  - 发布者/订阅者
  - 发布频率或调度触发方式
  - 是否需要同步更新消息定义、CMake、日志或文档
- 涉及参数时，明确：
  - 参数定义位置
  - 默认值
  - 单位
  - 生效路径
  - 是否需要重启/热更新
- 涉及 WorkQueue / ScheduledWorkItem / 定时器时，明确运行频率、触发条件和线程上下文。
- 涉及驱动时，优先查明：
  - probe/init/start/Run/stop 路径
  - 串口或总线配置
  - DMA / 中断 / 轮询机制
  - 错误恢复逻辑
- 涉及 mixer / control allocation / actuator outputs 时，明确控制量来源、变换关系和最终输出通道。

## Communication and protocol changes
- 如果任务涉及串口、CAN、I2C、SPI、MAVLink、DDS、RTPS 或自定义协议：
  - 明确输入帧格式、字段含义、字节序、校验方式、长度和刷新频率
  - 说明发送和接收链路分别在哪些函数中处理
  - 标出缓存、DMA、中断、状态机或解析逻辑的位置
- 不要在不了解上下行完整数据流的情况下直接修改协议字段。
- 如果修改协议打包/拆包逻辑，必须同步检查日志、调试输出和异常分支。

## Files to inspect first
- 控制分配相关：
  - `src/modules/control_allocator/`
- 执行器输出相关：
  - `src/drivers/`
  - `src/modules/`
- 启动与机型配置相关：
  - `ROMFS/px4fmu_common/`
- 参数与消息相关：
  - `src/lib/parameters/`
  - `msg/`
- 平台与板级相关：
  - `boards/`
  - `platforms/`

## Code conventions
- 保持与 PX4 现有代码风格一致。
- 新增函数应职责单一，避免过长。
- 新增状态变量时说明生命周期、线程上下文和并发访问风险。
- 关键协议处理、状态机分支、容错逻辑必须写清楚注释。
- 不要随意重命名已有接口、topic、参数名、日志关键字，除非任务明确要求。
- 新加入的代码都必须以"// add by jayjie"，以“// end”结尾来区分是我开发的代码

## Validation expectations
- 回答时优先给出：
  1. 修改了什么
  2. 为什么这样改
  3. 影响哪些模块/话题/参数/接口
  4. 如何验证
- 如果无法完整验证，明确说明：
  - 哪些已验证
  - 哪些未验证
  - 风险点在哪里
- 若修改影响飞控安全、执行器输出或任务关键闭环，必须提示潜在风险。

## Preferred response format
- 先给结论，再给调用链或数据流说明。
- 涉及 PX4 源码分析时，优先按“入口函数 → 中间处理 → 输出结果”展开。
- 涉及驱动/通信时，优先按“初始化 → 收发机制 → 解析/封装 → 状态更新 → 异常处理”展开。
- 涉及控制链路时，优先按“输入 → 变换/分配 → 输出 → 执行器反馈”展开。

## Notion代码笔记规则
当任务是“将代码整理为笔记并输出到Notion”时：

- 必须使用 Notion 兼容的标准 Markdown
- 标题只用 `#` / `##` / `###`
- 使用短段落、短项目符号
- 代码使用 fenced code block，并标注语言
- 公式使用 LaTeX，公式都不要加$$，这会导致无法渲染为latex公式
- 不要使用 HTML、脚注、Mermaid、折叠块、自定义提示块、自动目录

### 输出目标
笔记用于：
- 理解模块职责
- 梳理调用链和数据流
- 明确输入输出、状态变化
- 支持后续开发交接

不要写成泛泛总结、教程或宣传性描述。

## 代码编译
- 更新完代码优先使用 make px4_fmu-v5_default 进行编译

## Do not
- 不要跳过调用链分析直接改代码。
- 不要在未确认单位、坐标系、通道映射、消息来源前做控制相关修改。
- 不要为了通过编译而删除关键逻辑、绕过错误处理或硬编码协议字段。
- 不要修改与当前任务无关的大量文件。
