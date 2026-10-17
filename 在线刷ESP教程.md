

# win11下用git 上传版本

```
cd "C:\Users\JimYu\DoubaoWork\chats\2026-10-14\new-chat\pavo20-led-controller"
git add .
git commit -m "开机日志修复"
git push
```

# 在线编译

git上传后，打开actions

[All workflows](https://github.com/jimyu0628/pavo20-led-controller/actions)

在中间部分，你上传的版本，点标题。

滚屏到最底部，Artifacts , 点右侧下载。



# 网页刷写工具（之前你用过的）：

- **ESP Web Flasher**（刷固件）：`https://esptool.spacehuhn.com/`
- **Web Serial Monitor**（串口监视器，看日志）：`https://serial.huhn.me`

刷机步骤回顾（针对 ESP32-C3 SuperMini）：

1. 打开 ESP Web Flasher → **Connect** → 选择 ESP32-C3 的 USB 串口（会弹系统权限框，点允许）；
2. 选 **merged.bin**（用 Actions 最新构件里的），**地址填 `0x0`，只留这一个槽**（其余 0x1000/0x8000 等空槽删除或留空）；
3. 点 **Program / Erase** → 等待写入完成 → 自动复位。

> 注意：网页刷写用的是浏览器 WebSerial，**需 Chrome/Edge**，且连接时如果提示权限要允许；C3 刷 merged.bin 必须从 0x0 开始，不能按 ESP32 classic 的四段地址来。

