<div align="center">
  <h1>VibeStick 液态沙漏</h1>
  <p><strong>一个装进口袋的 StickS3 液态细沙沙漏。</strong></p>
  <p>
    在 M5Stack StickS3 上呈现细密粒子、重力感应、灵活计时，<br>
    并用柔和提示音结束每一次专注。
  </p>
  <p>
    <a href="#项目概览">项目概览</a> ·
    <a href="#设备体验">设备体验</a> ·
    <a href="#编译与刷入">编译与刷入</a> ·
    <a href="#按键操作">按键操作</a> ·
    <a href="README.md">English</a>
  </p>
  <p>
    <img alt="硬件：M5Stack StickS3" src="https://img.shields.io/badge/hardware-M5Stack%20StickS3-EA1D2C">
    <img alt="ESP-IDF：5.5" src="https://img.shields.io/badge/ESP--IDF-5.5-E7352C">
    <img alt="屏幕：135 × 240" src="https://img.shields.io/badge/display-135%20%C3%97%20240-22A6B3">
    <img alt="粒子数：300" src="https://img.shields.io/badge/particles-300-F2B84B">
  </p>
  <br>
  <img src="docs/images/liquid-hourglass-hero.png" alt="VibeStick 液态沙漏产品概念效果图" width="760">
</div>

## 项目概览

VibeStick 液态沙漏把 StickS3 变成一个有触感的专注计时器。固定的玻璃外框中运行着 300 颗模拟沙粒：它们会响应设备倾斜，按照倒计时进度流动，并在 `00:00` 时让最后的真实沙粒刚好落定。

| | 能力 | 作用 |
| --- | --- | --- |
| **01** | 液态细沙 | 用暖黄色与青蓝色细沙和短促自然的流束呈现流动层次。 |
| **02** | 重力感应 | 使用经过滤波的 IMU 数据，让沙粒平滑响应倾斜，玻璃外框保持固定。 |
| **03** | 灵活计时 | 提供 `1 / 5 / 10 / 15 / 25` 分钟预设，并支持自定义分钟。 |
| **04** | 嵌入式性能 | 对物理计算和局部绘制设定明确边界，在 ESP32-S3 上保持流畅。 |

## 设备体验

<table>
  <tr>
    <td width="50%" align="center">
      <img src="docs/images/liquid-hourglass-closeup.png" alt="液态沙漏界面概念特写" width="100%">
    </td>
    <td width="50%" align="center">
      <img src="docs/images/liquid-hourglass-hero.png" alt="VibeStick 液态沙漏外观概念效果图" width="100%">
    </td>
  </tr>
  <tr>
    <td valign="top">
      <strong>细密液态沙粒</strong><br>
      细小沙粒、克制光晕和水滴式流束带来层次感，不依赖实时模糊或桌面级特效。
    </td>
    <td valign="top">
      <strong>有触感的专注计时器</strong><br>
      通过 StickS3 的实体按键与 IMU 完成开始、暂停、重置、选时和翻转操作。
    </td>
  </tr>
</table>

> [!NOTE]
> 以上图片为产品概念效果图。固件实际运行在 StickS3 的 `135 × 240` 屏幕上，个别视觉细节可能与实机略有差异。

## 开始前的准备

- [ ] 一台 M5Stack StickS3 和一根 USB-C 数据线。
- [ ] 已安装 ESP32-S3 支持的 ESP-IDF 5.5.x。
- [ ] 已确认 StickS3 的串口，以下统一用 `<PORT>` 表示。
- [ ] 如果只更新双应用设备中的沙漏程序，请保持分区表和 OTA 数据不变。

<p align="center"><strong>准备 → 编译 → 刷入 → 重启 → 使用</strong></p>

## 编译与刷入

### 从源码编译

加载 ESP-IDF，然后在仓库根目录执行：

```sh
. /path/to/esp-idf/export.sh
idf.py -C firmware/sticks3 build
```

编译完成后的应用镜像位于：

```text
firmware/sticks3/build/vibestick_hourglass_liquid.bin
```

### 刷入已验证镜像

双应用布局把沙漏程序放在 `0x20000` 的 `ota_0` 分区。只写入应用镜像，可保留另一 OTA 分区中的固件：

```sh
python -m esptool --chip esp32s3 -p <port> \
  write_flash 0x20000 \
  firmware/sticks3/releases/liquid-style-v1/vibestick_hourglass_liquid.bin
```

仓库已包含[经过验证的应用镜像](firmware/sticks3/releases/liquid-style-v1/vibestick_hourglass_liquid.bin)，其[版本说明和 SHA-256](firmware/sticks3/releases/liquid-style-v1/README.md)也一并保存在仓库中。

> [!WARNING]
> 只更新这个应用分区时，请不要重写分区表或 OTA 数据，否则可能移除另一固件，或改变预期的启动布局。

## 按键操作

| 操作 | 功能 |
| --- | --- |
| 正面蓝键单击 | 开始或暂停；到达 `00:00` 后开始完整重置的新一轮。 |
| 正面蓝键双击 | 重置倒计时和全部沙粒。 |
| 侧键单击 | 循环选择预设时间。 |
| 侧键长按 | 进入自定义分钟模式。 |
| 自定义模式下侧键单击 / 双击 | 增加 1 / 5 分钟。 |
| 自定义模式下蓝键单击 / 双击 | 确认 / 取消。 |
| 侧键三按 | 在双应用设备上切换固件；详见[双固件操作流程](docs/MULTI_FIRMWARE.zh-CN.md)。 |
| 设备旋转 180° | 按新的重力方向重置并重新开始。 |

## 实现方式

- 屏幕：`135 × 240` 竖屏。
- 显示刷新目标：`25–30 FPS`。
- 固定物理更新：`20 Hz`。
- 粒子池：固定 300 颗粒子，活动粒子数量受控。
- 碰撞粗筛：固定 4 像素均匀网格。
- 玻璃碰撞：预计算左右边界与壁面法线。
- 局部刷新：仅重绘 `115 × 118` 沙漏画布。
- 视觉效果：固定像素遮罩，不使用实时模糊、滤镜或全屏缓冲。
- 沙量守恒：`上层 + 下落中 + 下层 = 总数`。
- 完成反馈：通过 ES8311 播放柔和、非阻塞的三音提示。

倒计时进度同时控制时间与沙粒转移，因此最后一批真实沙粒会在倒计时到达 `00:00` 时落定。

## 项目结构

```text
VibeStick-Hourglass-Liquid/
  README.md
  README.zh-CN.md
  docs/images/
  firmware/sticks3/
    include/
    src/
    third_party/bmi270/
    releases/liquid-style-v1/
```

## 当前限制

- 固件仅面向 M5Stack StickS3。
- 仓库中的二进制文件是适用于本项目双应用分区布局的应用镜像，不是完整的出厂烧录镜像。
- 概念效果图用于表达目标视觉；实机的颜色、亮度和细小粒子细节可能有所不同。
- 侧键三按切换固件，要求另一 OTA 分区中已存在兼容的第二个应用。

## 隐私

本固件完全在设备本地运行：不会连接 Wi-Fi、调用远程服务、录音或收集用户数据。
但编译产物仍可能包含本地开发信息，因此公开前应检查二进制、日志、截图、图片元数据
以及 Git 提交身份。详见[隐私说明](docs/PRIVACY.zh-CN.md)。

## 许可证

项目原创代码和文档使用 [MIT License](LICENSE)。Bosch BMI270 源码继续遵循其
随附的 BSD-3-Clause 许可证，详见 [NOTICE](NOTICE)。
