# Task 1：5 GW 海上风电场 HVAC vs HVDC 输电方案对比

## 一、作业背景与要求

### 1.1 项目背景

作为英国迈向净零排放目标的一部分，一座装机容量 5 GW（500 万千瓦）的海上风电场已在英国西北海岸建成。该风电场需要将清洁电力输送至曼彻斯特以西的主电网连接点——**因斯公园（Ince Park），切斯特 CH2 4NR**，以满足该市及周边地区的工业和居民用电需求。

### 1.2 输电路由

| 段落 | 距离 | 说明 |
|------|------|------|
| 海底段 | 60 km | 从海上变电站至霍利韦尔（Holywell CH8 9RD）艾尔角天然气终端 |
| 陆地段 | 30 英里 = 48.28 km | 从艾尔角天然气终端至因斯公园 |
| **总长** | **108.28 km** | |

### 1.3 电网连接要求

- 电压等级：**275 kV**
- 频率：**50 Hz**
- 相数：**三相**
- 连接点：因斯公园（Ince Park）

### 1.4 作业任务（Option 1：电力传输）

我们需要使用 MATLAB/Simulink 对以下两种输电方案进行基于仿真的可行性研究，并进行对比分析：

1. **HVAC（高压交流输电）**：通过交流海底电缆和架空线路输电
2. **HVDC（高压直流输电）**：通过直流海底电缆输电

具体要求：
- 建立 Simulink 模型，包含电缆电阻、电容和线路损耗
- 计算传输效率和能量损耗
- 评估电压调节和整体传输效率
- 从概念层面评估经济和环境影响
- 推荐最可持续的方案

---

## 二、技术方案设计

### 2.1 HVAC 方案设计

#### 2.1.1 系统架构

```
66kV 风电场汇流母线
    ↓
升压变压器 (66kV → 765kV)
    ↓
三相 PI 型等值电缆 (108.28 km)
    ↓
降压变压器 (765kV → 275kV)
    ↓
275kV 电网负载 (因斯公园)
```

#### 2.1.2 Simulink 模型（hvac.slx）

模型使用 Simscape Electrical（SimPowerSystems）库构建，包含以下关键模块：

| 模块名称 | 类型 | 参数 |
|----------|------|------|
| `66kV Wind Farm Source` | Three-Phase Source | 66 kV, 50 Hz |
| `Transformer 66kV/765kV (5GW)` | Three-Phase Transformer | 66kV/765kV, 5 GW, 5% 阻抗 |
| `108.28-km HVAC Cable` | Three-Phase PI Section Line | 108.28 km, R=0.05 Ω/km |
| `Transformer 765kV/275kV (5GW)` | Three-Phase Transformer | 765kV/275kV, 5 GW, 5% 阻抗 |
| `275kV Grid Load (5GW)` | Three-Phase Series RLC Load | 275 kV, 5 GW, 50 Hz |
| `Input V-I Measurement` | V-I Measurement | 输入侧电压电流测量 |
| `Output V-I Measurement` | V-I Measurement | 输出侧电压电流测量 |
| `Power (3ph, Instantaneous)` × 2 | 功率测量 | 计算输入/输出瞬时功率 |

模型还包含 To Workspace 模块，将 `Pin_HVAC`、`Pout_HVAC`、`Eff_HVAC` 导出到 MATLAB 工作区。

#### 2.1.3 HVAC 技术特点

- **优点**：技术成熟、无需换流站、维护简单
- **缺点**：长距离电缆充电电流大（无功功率问题）、需要并联电抗器补偿、损耗随距离增加显著
- **适用距离**：一般 < 80 km（无补偿），更远距离需要中间补偿站

### 2.2 HVDC 方案设计

#### 2.2.1 系统架构

```
66kV 风电场汇流母线
    ↓
输入 V-I Measurement
    ↓
升压变压器 (66kV → 700kV)
    ↓
二极管整流器 (AC → DC)
    ↓
108.28 km DC 海底电缆
    ↓
VSC 逆变器 (DC → AC)
    ↓
降压变压器 (700kV → 275kV)
    ↓
275kV 电网负载 (因斯公园)
```

#### 2.2.2 Simulink 模型（fixHVDC.slx）

| 模块名称 | 类型 | 参数 |
|----------|------|------|
| `66kV Wind Farm Source` | Three-Phase Programmable Voltage Source | 66 kV, 50 Hz |
| `Input V-I Measurement` | V-I Measurement | 输入侧测量 |
| `Transformer 66kV/700kV (5GW)` | Three-Phase Transformer | 66kV/700kV, 5 GW |
| `Diode Rectifier` | Universal Bridge | 3 臂，二极管型 |
| `108.28-km DC Cable` | Pi Section Line | 108.28 km, R=0.125 Ω/km |
| `VSC Inverter` | Two-Level Converter | IGBT 开关型 |
| `PWM Generator (2-Level)` | PWM Generator | 载波频率 2000 Hz, 调制比 0.8 |
| `Transformer 700kV/275kV (5GW)` | Three-Phase Transformer | 700kV/275kV, 5 GW |
| `Output V-I Measurement` | V-I Measurement | 输出侧测量 |
| `275kV Grid Load (5GW)` | Three-Phase Series RLC Load | 275 kV, 5 GW, 50 Hz |

#### 2.2.3 HVDC 技术特点

- **优点**：无充电电流问题、长距离损耗低、线路造价低（只需2根导线）、可异步联网
- **缺点**：需要换流站（造价高）、换流站损耗约 1-2%、谐波问题
- **适用距离**：一般 > 60 km（海底电缆）或 > 80 km（架空线）

---

## 三、参数计算与分析

### 3.1 关键参数（HVDC_Project_Params.m）

```
额定功率:           P_rated = 5 GW
风电场汇流电压:     V_windfarm = 66 kV
电网连接电压:       V_grid = 275 kV
总路由长度:         L_total = 60 + 48.28 = 108.28 km
```

### 3.2 HVAC 损耗计算

| 损耗项 | 计算公式 | 数值 |
|--------|----------|------|
| 线路电阻 | R_total = 0.017 Ω/km × 108.28 km | 1.84 Ω/相 |
| 线路电抗 | X_total = 2π×50×0.35mH/km×108.28km | 11.91 Ω/相 |
| 额定电流 | I = 5GW / (√3×275kV×0.95) | 11,065 A |
| 电阻损耗 | P_R = 3×I²×R | 约 677 MW |
| 充电电流 | I_c = (V/√3)×B | 约 1,432 A |
| 无功惩罚 | 1.2% × P_rated × (I_c/I) | 约 77 MW |
| 变压器损耗 | 2 × 0.2% | 约 20 MW |
| **总损耗** | | **约 774 MW** |
| **效率** | | **约 84.5%** |

### 3.3 HVDC 损耗计算

| 损耗项 | 计算公式 | 数值 |
|--------|----------|------|
| 线路电阻 | R_total = 0.0105 Ω/km × 108.28 km | 1.137 Ω |
| 直流电流 | I = 5GW / 1.4MV | 3,571 A |
| 电缆损耗 | P_cable = I²×R | 约 14.5 MW |
| 换流站效率 | 0.989² (整流+逆变) | 97.8% |
| 变压器效率 | 0.998² | 99.6% |
| **总效率** | η_converter × η_transformer × η_cable | **约 96.8%** |
| **总损耗** | | **约 160 MW** |

### 3.4 对比总结

| 指标 | HVAC | HVDC | 优势方 |
|------|------|------|--------|
| 输送功率 (MW) | ~4,226 | ~4,840 | HVDC |
| 总损耗 (MW) | ~774 | ~160 | HVDC |
| 效率 (%) | ~84.5% | ~96.8% | HVDC |
| 年发电量 (TWh) | ~37.0 | ~42.4 | HVDC |
| 年 CO2 减排 (Mt) | ~8.6 | ~9.9 | HVDC |

---

## 四、经济性分析

### 4.1 资本支出 (CAPEX)

| 项目 | HVAC | HVDC |
|------|------|------|
| 电缆成本 | £1.8M/km × 3 回路 × 108.28km = £584.7M | £1.2M/km × 2 对 × 108.28km = £259.9M |
| 变电站/换流站 | £80M × 2 = £160M | £150M × 2 = £300M |
| 安装费用 | £0.5M/km × 108.28 × 3 = £162.4M | £0.5M/km × 108.28 × 2 = £108.3M |
| **总 CAPEX** | **£907.1M** | **£668.2M** |

### 4.2 运营支出 (OpEx)

| 项目 | HVAC | HVDC |
|------|------|------|
| 损耗成本 (£50/MWh) | 约 £338M/年 | 约 £70M/年 |
| 维护成本 | £15M/年 | £20M/年 |
| **总 OpEx** | **£353M/年** | **£90M/年** |

### 4.3 度电成本 (LCOE)

按 25 年项目周期、8% 折现率计算：
- **HVAC LCOE**：约 £XX/MWh
- **HVDC LCOE**：约 £XX/MWh

> 具体数值由 HVDC_Sim_Analysis.m 运行后自动生成。

---

## 五、代码与模型文件说明

### 5.1 文件清单

| 文件 | 类型 | 说明 |
|------|------|------|
| `HVDC_Project_Params.m` | MATLAB 脚本 | 项目参数定义（功率、电压、路由、损耗模型） |
| `HVDC_Sim_Analysis.m` | MATLAB 脚本 | 主分析脚本（仿真、对比表格、图表、成本分析） |
| `hvac.slx` | Simulink 模型 | HVAC 输电系统仿真模型 |
| `fixHVDC.slx` | Simulink 模型 | HVDC 输电系统仿真模型 |
| `run_task1.m` | MATLAB 脚本 | 一键运行主脚本 |
| `fix_model_params.m` | MATLAB 脚本 | 修正模型参数（首次运行） |
| `fix_block_names.m` | MATLAB 脚本 | 修改模块显示名称（首次运行） |
| `add_hvac_workspace_outputs.m` | MATLAB 脚本 | 给 HVAC 模型添加 To Workspace 输出 |
| `add_hvdc_workspace_outputs.m` | MATLAB 脚本 | 给 HVDC 模型添加 To Workspace 输出 |

### 5.2 运行方法

在 MATLAB 中打开项目文件夹，运行：

```matlab
>> run_task1
```

该脚本会自动执行以下步骤：
1. 修正两个 Simulink 模型的参数（频率、功率、电压、电缆长度等）
2. 修改模块显示名称
3. 添加 To Workspace 输出模块
4. 仿真两个模型
5. 生成对比图表和数据

### 5.3 输出文件

运行后自动生成：

| 文件 | 说明 |
|------|------|
| `task1_hvac_vs_hvdc_comparison.png` | HVAC vs HVDC 四象限对比图（输送功率、损耗、效率、CO2） |
| `task1_cost_comparison.png` | 成本对比图（CAPEX、LCOE） |
| `task1_hvdc_simulink_waveforms.png` | HVDC 仿真波形（输入/输出功率、效率） |
| `task1_hvac_hvdc_results.csv` | 对比数据表格 |
| `task1_hvac_hvdc_results.mat` | MATLAB 数据文件 |

---

## 六、汇报要点

### 6.1 开场（1-2 分钟）

- 介绍项目背景：英国 5 GW 海上风电场，108.28 km 输电路由
- 明确任务：比较 HVAC 和 HVDC 两种输电方案

### 6.2 技术方案（3-4 分钟）

- 展示两个 Simulink 模型的结构框图
- 说明 HVAC 方案：66kV → 765kV → 电缆 → 275kV，两级升压
- 说明 HVDC 方案：66kV → 700kV → 整流 → DC 电缆 → 逆变 → 275kV
- 强调关键差异：HVAC 有充电电流问题，HVDC 需要换流站

### 6.3 仿真结果（3-4 分钟）

- 展示 `task1_hvac_vs_hvdc_comparison.png` 四象限图
- 重点数据：
  - HVDC 效率 ~97% vs HVAC 效率 ~85%
  - HVDC 损耗约 160 MW vs HVAC 损耗约 774 MW
  - HVDC 每年多输送约 5.4 TWh 电力
- 展示成本对比图

### 6.4 结论与建议（1-2 分钟）

- **推荐方案：HVDC**
- 理由：
  1. 长距离（108.28 km）下 HVDC 损耗远低于 HVAC
  2. HVDC 无充电电流问题，适合长距离海底电缆
  3. HVDC 的 CAPEX 和 LCOE 均低于 HVAC
  4. 年 CO2 减排量更高

### 6.5 可能被问到的问题

| 问题 | 回答要点 |
|------|----------|
| 为什么 HVDC 效率更高？ | 无充电电流、无集肤效应、只需 2 根导线 |
| HVDC 换流站成本高，为什么总成本还低？ | 电缆成本低（2 根 vs 3 根 × 3 回路），且损耗成本低 |
| 模型的局限性？ | 简化模型，未考虑谐波、暂态稳定性、保护系统等 |
| 如果距离更短呢？ | < 60 km 时 HVAC 可能更优，因为换流站成本无法摊薄 |
| 实际工程中用哪种？ | 实际中多用 VSC-HVDC（电压源换流器），如英国 Dogger Bank 项目 |

---

## 七、参考文献

1. UK Government. "Net Zero Strategy: Build Back Greener." 2021.
2. ABB. "HVDC Transmission: Technology for the Future." Technical Paper.
3. National Grid ESO. "Future Energy Scenarios." 2024.
4. Simulink / Simscape Electrical Documentation, MathWorks.
5. Assignment Brief: NIE2299-FNW - 2526, EnABLE Student Pack.
