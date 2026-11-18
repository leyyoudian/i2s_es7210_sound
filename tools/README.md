# 主机测试与音频诊断工具

当前工具面向 **Windows、Python 3.11/3.12**。播放使用`winsound`，音频转换使用`audioop`（Python 3.13已移除）。主机C测试和C代码重放需MSVC Build Tools；网页测试需Node.js。扩展语音素材转换还需FFmpeg加入PATH。开发板固件仍使用ESP-IDF v5.4.2构建。

```powershell
python -m pip install -r tools/requirements.txt
python -m pytest tests -q
```

运行时把`COM25`替换成你的串口，确认板子与电脑音箱位置、系统音量在前后对照中保持一致。测试会真实播放声音。脚本会排空串口，等待DOA任务就绪，优先按板端`frame_ms`对齐播放时间。诊断日志不等于相机看到的灯光，也不等于独立实测方位。

## 工具入口

| 工具 | 用途 |
|---|---|
| `serial_capture.py` | 抓取串口文本，默认输出到被忽略的日志文件 |
| `real_audio_bench.py` | 下载/缓存15段真实录音，播放并记录同一窗口内的DOA |
| `prepare_extended_audio.py` | 独立准备26段扩展素材；可指定`--esc50-dir`复用已有公开数据集 |
| `analyze_audio_suite.py` | 按采样时间/播放窗口汇总响应、波动和离群，不把单次输出称为准确率 |
| `summarize_doa_bench.py` | 汇总真实录音测试产生的JSON |
| `long_audio_bench.py` | 播放一个完整WAV，按30秒分段记录运行状态 |
| `capture_doa_stream.py` | 显式诊断固件的连续四通道PCM、CRC/包序/溢出校验 |
| `capture_doa_pcm.py` | 旧十六进制快照诊断，保存离散帧，不是连续录音 |
| `replay_doa_pipeline.py` | 在主机上编译当前实际C处理段，重放无丢包的四通道WAV |
| `check_fractional_doa.py` | 对比小数采样延时下的相关插值误差，仅算法回归 |
| `doa_bench.py` | 早期可复现的合成噪声/啁啾/短声基准，不作为真实素材的替代 |
| `upload_server.py` | 板级自检录音接收及本机浏览器回放 |
| `package_source.py` | 从指定Git提交生成源码ZIP，检查构建产物/私有配置并生成SHA256 |

`playback_*`和`sim_doa.py`为保留的早期工具，包含旧参数/固定端口，不能认为它们始终与当前固件等价。正式对照优先用上述显式参数脚本及实际C代码重放。

## 真实录音播放

```powershell
python tools/real_audio_bench.py --prepare
python tools/real_audio_bench.py --port COM25 --count 15 --gain 2 --compact --out tools/recheck.log
python tools/prepare_extended_audio.py
python tools/real_audio_bench.py --manifest .fetch/extended-audio/manifest.json --port COM25 --count 26 --gain 2 --compact --out tools/extended_recheck.log
python tools/analyze_audio_suite.py tools/recheck.log tools/extended_recheck.log --out .fetch/summary.json
```

峰值先归一化到0.30，`--gain 1`为0.30、`--gain 2`为0.60，并不是声压级标定；实际声压还受系统音量和扬声器影响。可用`--repeat 2`重复选定素材，`--channel left/right`分别测试两个扬声器，避免把多个物理发声点误当作单源。

完整音乐测试接受用户自己的PCM WAV，不自动生成/循环片段：

```powershell
python tools/long_audio_bench.py --port COM25 --wav C:/path/to/music.wav --out tools/music_recheck.log
```

## 连续四通道PCM与重放

仅在确实需要原始采样时启用诊断构建；此时正常定位、灯效和网络处理暂停。

```powershell
idf.py -D DOA_PCM_STREAM=ON -D DOA_PCM_DUMP=OFF build
idf.py -p COM25 flash
python tools/capture_doa_stream.py --port COM25 --manifest .fetch/extended-audio/manifest.json --count 4 --gain 1 --out .fetch/capture.wav
idf.py -D DOA_PCM_STREAM=OFF -D DOA_PCM_DUMP=OFF build
idf.py -p COM25 flash
python tools/replay_doa_pipeline.py .fetch/capture.wav --out .fetch/replay.json
```

必须在CRC错误、序号缺失、DMA溢出均为0时才把保存的WAV当成连续采样分析。重放编译主机C后端；实机用S3优化FFT，有微小浮点差异。采集可能包含现场其它人的声音，分享源码时不包含这些本地录音。

## 数据与许可

下载和派生音频在`.fetch/`缓存；源URL、SHA256和来源说明在manifest中。ESC-50素材查看[数据集](https://github.com/karolpiczak/ESC-50)与对应Freesound录音的许可；LibriSpeech例子保留朗读者署名并按CC BY 4.0来源说明使用。代码/SDK/模型许可与录音素材许可分开，下载成功不等于可随产品重新分发。

不要把`.fetch/`、本地私有Wi-Fi配置、测试录音、视频、日志、构建目录和带密码的固件装进网盘源码包。

提交后可生成不含Git历史的分享包：

```powershell
python tools/package_source.py --ref HEAD --out C:/path/to/i2s_es7210_sound-source.zip
```

包内包含SDK、模型、源码、测试、文档及来源版本说明；旁边生成`.zip.sha256`。打包器只读取指定提交，不会把未提交文件或本机录音悄悄带进去。远程Git历史中曾经公开的内容不会因此被清除，本次不强制改写仓库历史。
