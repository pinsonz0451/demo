# 支持高精度计算的存算一体单元设计

## 项目简介

本项目以 Yan et al.（IEEE JSSC 2024）提出的**密集-CIM 稀疏数字处理器**为参考，聚焦于其 Intensive-CIM 数据通路的 RTL 实现。完整通路覆盖从 FP16 激活输入到 FP16 输出的全过程：

```
FP16 激活向量
    │
    ▼
┌─────────────┐
│   Aligner   │  FP16 → bit-serial 二补码流（MSB 先出）
└─────────────┘
    │ bs_valid / bs_done / act_bs
    ▼
┌─────────────┐
│  CIM_Macro  │  1-bit SRAM 权重 × bit-serial 激活 → 有符号整数 MAC
└─────────────┘
    │ mac_result_flat / mac_valid
    ▼
┌──────────────────┐
│  int_to_fp_acc   │  INT32 → FP21 → 累加 → FP16（含指数偏移修正）
└──────────────────┘
    │ fp16_result_flat / fp16_valid
    ▼
FP16 输出
```

核心数学流程遵循论文 Section IV 的描述：exponent 对齐 → 整数矩阵乘累加 → 浮点归一化。

---

## 与论文的差异（已知简化）

本实现有意忽略论文中提升复杂度的创新点，专注于核心数据通路的正确性：

| 论文特性 | 本实现 |
|---|---|
| Ping-Pong 权重更新 | 不含，权重写入与 MAC 分时进行 |
| Low-MACV 截断加法树 | 不含，使用全精度 128 输入加法树 |
| 稀疏加速（Sparse Digital Core） | 不含 |
| 权重精度 INT8 / INT16 | 简化为 1-bit |
| 128 通道 × 64 列 | 参数化设计，仿真使用 8 通道 × 4 列 |

---


## FP21 自定义格式说明

归一化单元使用论文定义的 21-bit 中间浮点格式进行累加，以避免中间精度损失：

```
Bit 20     : 符号位 S
Bits 19:15 : 绝对指数 E（无偏置，取值 0–30）
Bits 14:0  : 尾数 M（15 位，比 FP16 多 5 位保护位）

值 = (−1)^S × 2^E × (1 + M / 2^15)
零 = 21'b0（唯一零表示）
```

write-back 时通过 `e_bias = emin − MAN_W − 15` 修正指数后截断为 FP16。

---

## 仿真运行

需要 [Icarus Verilog](https://github.com/steveicarus/iverilog)（`-g2001` 模式）。

**单元测试 — Aligner**

```bash
iverilog -g2001 -o sim_aligner.out \
  rtl/Aligner/aligner_channel.v \
  rtl/Aligner/aligner.v \
  sim/tb/tb_aligner.v
vvp sim_aligner.out
```

**单元测试 — CIM_Macro（含有符号数验证）**

```bash
iverilog -g2001 -o sim_cim.out \
  rtl/CIM_Macro/adder_tree_128.v \
  rtl/CIM_Macro/cim_array.v \
  rtl/CIM_Macro/cim_macro.v \
  sim/tb/tb_cim_macro.v
vvp sim_cim.out
```

**全链路集成测试**

```bash
iverilog -g2001 -o sim_top.out \
  rtl/Aligner/aligner_channel.v \
  rtl/Aligner/aligner.v \
  rtl/CIM_Macro/adder_tree_128.v \
  rtl/CIM_Macro/cim_array.v \
  rtl/CIM_Macro/cim_macro.v \
  rtl/Normalization/int_to_fp21.v \
  rtl/Normalization/fp21_adder.v \
  rtl/Normalization/fp21_to_fp16.v \
  rtl/Normalization/int_to_fp_acc.v \
  rtl/CIM_TOP.v \
  sim/tb/tb_top.v
vvp sim_top.out
gtkwave tb_top.vcd   # 可选，查看波形
```

集成测试覆盖 8 个测试用例（3 种权重模式 × 多种激活模式），期望输出示例：

```
=================================================================
  Intensive-CIM Full Datapath Integration Testbench
  NUM_CH=8  NUM_COL=4  PSUM_W=32
=================================================================
[PASS] TC1 col=0  expected=4500  got=4500  diff=0 ULP
[PASS] TC1 col=1  expected=4500  got=4500  diff=0 ULP
...
=================================================================
  RESULTS: 32 PASSED / 0 FAILED / 32 total
  >>> ALL TESTS PASSED <<<
=================================================================
```

---

## 参考文献

S. Yan et al., "A 28-nm Floating-Point Computing-in-Memory Processor Using Intensive-CIM Sparse-Digital Architecture," *IEEE Journal of Solid-State Circuits*, vol. 59, no. 8, pp. 2630–2643, Aug. 2024.

---

## 许可

本项目仅用于学术研究与毕业设计，不用于商业目的。