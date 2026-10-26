#pragma once
// ============================================================
// 内嵌网页（v32.21 · A+ 界面：灯光 / 连接 / 设备 三个功能区）
//
// 页面与固件的接口：
//   GET  /api/state   → 一条 JSON 给齐三页所有数值（字段见 wifi_cfg.cpp:sendState）
//   POST /api/set     → 一次提交五项参数（mode&color&spd&bright&count），回同样 JSON
//   POST /api/ota     → multipart 字段名 fw，写固件后设备自重启
//
// 页面里没有任何硬编码的假数据：设备名、SSID、IP、倒计时、运行时长、复位原因、
// 飞控帧数全部来自 /api/state，所以固件里不再需要 @@DEVNAME@@ 文本替换。
// ============================================================
static const char HTML_PAGE[] = R"rawliteral(<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0b0b0d">
<title>幻彩灯带控制板</title>
<style>
  :root{
    --bg:#0b0b0d; --card:#17171a; --card2:#1e1e22; --line:#2a2a30;
    --txt:#ececee; --dim:#8a8a92; --dim2:#5e5e66;
    --amber:#ffb454; --amber-d:#3a2a10; --blue:#2d7ff9; --green:#3ddc84; --red:#e8734a;
  }
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
  /* 滚动条常驻占位：三页内容长短不同，不加这个的话有没有滚动条会让整页宽度差 15px，
     切页时文字会横向抖一下。overflow-y:scroll 才是真正让它常驻的那一句，
     scrollbar-gutter 在根元素上单独用不生效。 */
  html{overflow-y:scroll;scrollbar-gutter:stable;scrollbar-width:thin;
       scrollbar-color:#33333d #0b0b0d}
  ::-webkit-scrollbar{width:9px;height:9px}
  ::-webkit-scrollbar-track{background:#0b0b0d}
  ::-webkit-scrollbar-thumb{background:#33333d;border-radius:9px;border:2px solid #0b0b0d}
  ::-webkit-scrollbar-thumb:hover{background:#43434e}
  body{margin:0;background:var(--bg);color:var(--txt);
       font-family:system-ui,-apple-system,"Segoe UI","Microsoft YaHei",sans-serif;
       padding:14px 14px 30px;max-width:560px;margin:0 auto;font-size:15px}
  body.light{padding-bottom:112px}

  /* 顶栏 */
  header{display:flex;align-items:center;gap:11px;padding:2px 2px 10px}
  .logo{width:38px;height:38px;border-radius:11px;background:linear-gradient(150deg,#2a1c06,#181818);
        border:1px solid #3a2a10;display:flex;align-items:center;justify-content:center;flex:0 0 auto}
  .logo svg{width:21px;height:21px}
  .htxt{flex:1;min-width:0}
  .htxt h1{font-size:15.5px;margin:0;font-weight:650;letter-spacing:.2px}
  .htxt .m{font-size:12px;color:var(--dim);margin-top:3px;display:flex;gap:7px;align-items:center}
  .chip{font-size:10.5px;padding:2px 6px;border-radius:5px;background:#22222a;color:#b9b9c2;border:1px solid var(--line)}
  .live{display:flex;align-items:center;gap:5px;font-size:11.5px;color:var(--green);
        background:#0f2418;border:1px solid #1c4530;padding:4px 9px;border-radius:20px;flex:0 0 auto}
  .live.off{color:var(--red);background:#241008;border-color:#4a2410}
  .dot{width:6px;height:6px;border-radius:50%;background:var(--green);box-shadow:0 0 8px var(--green)}
  .live.off .dot{background:var(--red);box-shadow:0 0 8px var(--red)}
  .iconbtn{width:34px;height:34px;flex:0 0 auto;border-radius:10px;border:1px solid var(--line);
           background:#191920;color:#b9b9c2;font-family:inherit;font-size:15px;font-weight:700;
           display:flex;align-items:center;justify-content:center;cursor:pointer;line-height:1;
           transition:background .12s,transform .1s}
  .iconbtn:active{transform:scale(.93);background:#23232b}

  /* ================= 一级：功能区标签（灯光 / 连接 / 设备） ================= */
  .ftabs{position:sticky;top:0;z-index:8;display:flex;gap:2px;
         margin:0 -14px 12px;padding:0 14px;
         background:rgba(11,11,13,.96);backdrop-filter:blur(14px);
         border-bottom:1px solid var(--line)}
  .ftabs button{flex:1;background:none;border:0;color:var(--dim);font-family:inherit;
                font-size:14.5px;font-weight:600;letter-spacing:.5px;padding:12px 0 13px;
                cursor:pointer;position:relative;transition:color .15s}
  .ftabs button.on{color:#fff}
  .ftabs button.on::after{content:"";position:absolute;left:50%;transform:translateX(-50%);
                          bottom:-1px;width:34px;height:2.5px;border-radius:3px 3px 0 0;
                          background:var(--amber);box-shadow:0 0 10px rgba(255,180,84,.5)}
  .ftabs .badge{position:absolute;top:9px;right:calc(50% - 26px);width:5px;height:5px;border-radius:50%;
                background:var(--amber);box-shadow:0 0 7px rgba(255,180,84,.9);display:none}
  .ftabs button.dirty .badge{display:block}

  .fpage{display:none}
  .fpage.on{display:block;animation:fade .18s ease-out}
  @keyframes fade{from{opacity:0;transform:translateY(5px)}to{opacity:1;transform:none}}

  /* ================= 吸顶预览（只在灯光页） =================
     吸在一级标签栏下方（--ftab-h 由脚本实测后写入），短屏滚动时也能看到预演 */
  .preview{position:sticky;top:var(--ftab-h,44px);z-index:5;
           background:var(--bg);margin:0 0 10px;padding-top:3px}
  .preview canvas{display:block;width:100%;height:62px;background:#07070a;
                  border-radius:10px;border:1px solid #23232a;
                  box-shadow:inset 0 0 26px rgba(0,0,0,.9)}
  .pvi{display:flex;justify-content:space-between;align-items:baseline;gap:8px;margin-top:7px;min-height:18px}
  .pvi .pm{color:var(--amber);font-weight:700;font-size:13px;letter-spacing:.2px;white-space:nowrap}
  .pvi .pd{font-size:11px;color:var(--dim2);font-variant-numeric:tabular-nums;
           white-space:nowrap;overflow:hidden;text-overflow:ellipsis;text-align:right}
  .pvi .pd b{color:#c08a3e;font-weight:600}

  /* ================= 二级：灯光页内的五格参数标签 ================= */
  .tabbar{display:grid;grid-template-columns:repeat(5,1fr);gap:6px;margin-bottom:10px}
  .tab{position:relative;display:flex;flex-direction:column;align-items:center;justify-content:center;
       gap:3px;min-height:54px;padding:7px 3px 8px;border-radius:11px;
       background:var(--card);border:1px solid var(--line);color:inherit;font-family:inherit;
       cursor:pointer;transition:background .13s,border-color .13s,transform .1s}
  .tab:active{transform:scale(.975)}
  .tab .tk{font-size:10px;color:var(--dim2);letter-spacing:.2px;white-space:nowrap;line-height:1}
  .tab .tv{font-size:13.5px;font-weight:700;line-height:1.15;color:#d9d9e2;
           font-variant-numeric:tabular-nums;white-space:nowrap;
           max-width:100%;overflow:hidden;text-overflow:ellipsis;padding:0 1px}
  /* 模式名较长（Betaflight）时缩一点字号，避免被省略号截成「Betaflig…」 */
  .tab .tv.sm{font-size:10.5px;letter-spacing:-.3px}
  .tab.on{background:var(--amber-d);border-color:#8a6320;
          box-shadow:inset 0 0 0 1px rgba(255,180,84,.18)}
  .tab.on .tk{color:#c9974a}
  .tab.on .tv{color:var(--amber)}
  .tab .d2{position:absolute;top:5px;right:6px;width:5px;height:5px;border-radius:50%;
           background:var(--amber);box-shadow:0 0 7px rgba(255,180,84,.9);display:none}
  .tab.dirty .d2{display:block}

  /* 五个调整页叠在同一格里，取最高页高度，切换不跳动 */
  .panelbox{background:var(--card);border:1px solid var(--line);border-radius:14px;
            padding:11px 13px 13px;margin-bottom:10px;
            display:grid;grid-template-columns:1fr;align-content:start}
  .panel{grid-area:1/1;display:flex;flex-direction:column;justify-content:center;
         visibility:hidden;opacity:0;pointer-events:none;transition:opacity .16s}
  .panel.show{visibility:visible;opacity:1;pointer-events:auto}
  .phead{display:flex;align-items:baseline;justify-content:space-between;gap:8px;margin:2px 0 4px}
  .phead .pt{font-size:12.5px;color:var(--dim);font-weight:600;letter-spacing:.6px}
  .phead .pv{font-size:13.5px;color:var(--amber);font-weight:700;letter-spacing:0;
             font-variant-numeric:tabular-nums;white-space:nowrap}

  /* ================= 卡片（连接页 / 设备页） ================= */
  .card{background:var(--card);border:1px solid var(--line);border-radius:14px;
        padding:12px 14px 13px;margin-bottom:10px}
  .card h2{font-size:11.5px;margin:0 0 5px;color:var(--dim2);font-weight:700;letter-spacing:.9px;
           display:flex;justify-content:space-between;align-items:baseline;gap:8px}
  .card h2 span{font-weight:400;letter-spacing:0;font-size:10.5px;color:var(--dim2);
                white-space:nowrap;flex:0 0 auto}
  .kv{display:flex;justify-content:space-between;align-items:center;gap:12px;
      padding:8px 0;font-size:13.5px}
  .kv + .kv{border-top:1px solid #202026}
  .kv .k{color:var(--dim);white-space:nowrap;flex:0 0 auto}
  .kv .v{font-weight:600;font-variant-numeric:tabular-nums;text-align:right;
         min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;color:#dcdce4}
  .kv .v.ok{color:var(--green)}
  .kv .v.amber{color:var(--amber)}
  .kv .v.mono{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:12px;font-weight:500}
  /* v32.23：飞控灯色字节顺序 —— 点一下换一个的小药丸按钮 */
  .ordbtn{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:12px;font-weight:700;
          letter-spacing:.5px;color:var(--amber);background:var(--amber-d);
          border:1px solid #8a6320;border-radius:999px;padding:3px 11px;cursor:pointer;
          -webkit-tap-highlight-color:transparent}
  .ordbtn:active{transform:scale(.96)}
  .ordbtn:disabled{opacity:.5;color:var(--dim2);background:var(--card2);border-color:var(--line)}
  .prog{height:4px;background:#2b2b33;border-radius:4px;overflow:hidden;margin:9px 0 2px}
  .prog i{display:block;height:100%;width:0;background:var(--amber);border-radius:4px;
          transition:width .9s linear}
  .note{font-size:11px;color:var(--dim2);line-height:1.75;margin-top:9px}
  .note b{color:#b9b9c2;font-weight:600}

  /* 页内大按钮 */
  .pbtn{display:flex;align-items:center;justify-content:center;gap:8px;width:100%;min-height:50px;
        margin-top:0;border-radius:12px;border:1px solid #2c2c34;background:var(--card2);
        color:#d6d6de;font-family:inherit;font-size:14.5px;font-weight:650;
        text-decoration:none;cursor:pointer}
  .pbtn svg{width:18px;height:18px;flex:0 0 auto}
  .pbtn.pri{background:linear-gradient(180deg,#3488ff,#2069e0);border-color:#4f97ff;color:#fff;
            box-shadow:0 6px 18px rgba(45,127,249,.26)}
  .pbtn:active{transform:scale(.99)}
  .stack{display:grid;gap:9px;margin-bottom:10px}

  /* 日志开关 + 日志框 */
  .toggle{display:flex;align-items:center;justify-content:space-between;font-size:13.5px;
          color:var(--dim);cursor:pointer;padding:6px 0 2px}
  .switch{width:44px;height:26px;border-radius:20px;background:#2a2a33;position:relative;
          transition:background .18s;flex:0 0 auto}
  .switch::after{content:"";position:absolute;top:3px;left:3px;width:20px;height:20px;
                 border-radius:50%;background:#8a8a92;transition:left .18s,background .18s}
  .switch.on{background:#5a3f14}
  .switch.on::after{left:21px;background:var(--amber)}
  /* v32.33：仿 iPhone 开关（51×31，白色滑块 27px，打开=系统绿 #34c759）。
     用在「连接」页的蓝牙广播行 —— 那一行整行可点，开关贴最右。 */
  .switch.ios{width:51px;height:31px;border-radius:16px;background:#39393d;
              transition:background .22s}
  .switch.ios::after{width:27px;height:27px;top:2px;left:2px;background:#fff;
                     box-shadow:0 2px 5px rgba(0,0,0,.38);transition:left .22s,background .22s}
  .switch.ios.on{background:#34c759}
  .switch.ios.on::after{left:22px;background:#fff}
  /* 日志：开关打开后才有工具条（暂停/复制）。
     日志每秒重写一次，手机上一滑动就被刷新，根本没法选中复制 —— 所以
     复制前先自动暂停，再抽全部内容。 */
  .logwrap{display:none;margin-top:9px}
  .logwrap.show{display:block}
  .logtools{display:flex;gap:7px;justify-content:flex-end;align-items:center;margin-bottom:7px}
  .mini{height:28px;padding:0 12px;border-radius:8px;border:1px solid #2a2a31;
        background:var(--card2);color:#b9b9c4;font-family:inherit;font-size:12px;
        font-weight:600;cursor:pointer;display:inline-flex;align-items:center;gap:5px;
        -webkit-tap-highlight-color:transparent}
  .mini svg{width:13px;height:13px;flex:0 0 auto}
  .mini:active{transform:scale(.96)}
  .mini.on{border-color:#8a6320;color:var(--amber);background:var(--amber-d)}
  .logbox{margin-top:0;background:#08080b;border:1px solid #202026;border-radius:9px;
          padding:8px 10px;font-family:ui-monospace,Menlo,Consolas,monospace;
          font-size:11px;line-height:1.85;color:#7f9a8a;white-space:pre-wrap;
          max-height:220px;overflow:auto;-webkit-user-select:text;user-select:text}

  /* 快捷档（大目标，戴手套好点） */
  .chips{display:grid;grid-template-columns:repeat(5,1fr);gap:6px;margin-top:10px}
  .chips.four{grid-template-columns:repeat(4,1fr)}
  .chips.three{grid-template-columns:repeat(3,1fr)}
  .chip2{min-height:34px;border-radius:8px;background:var(--card2);border:1px solid #26262c;
         color:#c0c0ca;font-family:inherit;font-size:12px;font-weight:600;cursor:pointer;
         font-variant-numeric:tabular-nums;padding:0 2px;transition:background .12s,color .12s}
  .chip2:active{transform:scale(.96)}
  .chip2.on{background:var(--amber-d);border-color:#8a6320;color:var(--amber)}

  /* 模式网格 */
  .modes{display:grid;grid-template-columns:1fr 1fr;gap:7px;margin-top:8px}
  .mode{display:flex;align-items:center;gap:8px;min-height:42px;padding:0 9px;border-radius:10px;
        background:var(--card2);border:1px solid #26262c;color:#c7c7d0;font-size:13px;
        text-align:left;font-family:inherit;cursor:pointer;
        transition:background .12s,border-color .12s,color .12s}
  .mode .n{width:18px;height:18px;border-radius:5px;background:#2c2c34;color:#7b7b85;font-size:10.5px;
           display:flex;align-items:center;justify-content:center;flex:0 0 auto;font-variant-numeric:tabular-nums}
  .mode:active{transform:scale(.985)}
  .mode.on{background:var(--amber-d);border-color:#8a6320;color:var(--amber);font-weight:600}
  .mode.on .n{background:#8a6320;color:#1a1000}

  /* ===== 通用轨道：点方块直选 + 按住拖动无级 ===== */
  .rail{position:relative;padding:15px 10px 4px;touch-action:none;cursor:pointer;
        user-select:none;-webkit-user-select:none}
  .rail .rt{position:relative;height:34px;background:#101014;border:1px solid #26262c;
            border-radius:9px;overflow:hidden;transition:border-color .14s,box-shadow .14s}
  .rail.dragging .rt{border-color:#8a6320;box-shadow:0 0 0 3px rgba(255,180,84,.09)}
  .rail .rf{position:absolute;left:0;top:0;bottom:0;width:0;pointer-events:none;
            background:linear-gradient(90deg,rgba(255,180,84,.06),rgba(255,180,84,.22))}
  .rail .rticks{position:absolute;inset:0;pointer-events:none}
  .rail .tick{position:absolute;top:50%;width:5px;height:15px;border-radius:2px;
              background:#33333d;transition:background .12s,box-shadow .12s}
  .rail .tick.on{background:var(--amber);box-shadow:0 0 9px rgba(255,180,84,.8)}
  .rail .rk{position:absolute;top:4px;bottom:4px;width:3px;transform:translateX(-50%);
            background:#fff;border-radius:2px;pointer-events:none;
            box-shadow:0 0 8px rgba(255,255,255,.55)}
  .rail .rb{position:absolute;top:-12px;left:0;transform:translateX(-50%);
            background:#f0f0f4;color:#111;font-size:12.5px;font-weight:700;
            padding:4px 9px;border-radius:11px;white-space:nowrap;
            opacity:0;transition:opacity .14s;pointer-events:none;z-index:4;
            font-variant-numeric:tabular-nums;box-shadow:0 3px 12px rgba(0,0,0,.5)}
  .rail.dragging .rb{opacity:1}
  .rail.dragging .rk{box-shadow:0 0 14px rgba(255,255,255,.8)}
  .rail .ends{display:flex;justify-content:space-between;margin-top:5px;
              font-size:10.5px;color:var(--dim2);font-variant-numeric:tabular-nums}

  /* 灯带长度：左右加减 */
  .stepwrap{display:flex;align-items:center;gap:9px}
  .stepwrap .rail{flex:1;min-width:0}
  .stp{width:46px;height:46px;flex:0 0 auto;border-radius:11px;background:var(--card2);
       border:1px solid #2c2c34;color:#d6d6de;font-family:inherit;font-size:23px;font-weight:600;
       line-height:1;display:flex;align-items:center;justify-content:center;cursor:pointer;
       transition:background .1s,transform .1s,color .1s}
  .stp:active{background:#2b2b35;transform:scale(.95);color:#fff}
  .stp.off{opacity:.3;pointer-events:none}

  /* 颜色：只点选 */
  .colors{display:grid;grid-template-columns:repeat(4,1fr);gap:11px 9px;margin-top:12px}
  .sw{background:none;border:0;padding:0;display:flex;flex-direction:column;align-items:center;
      gap:6px;cursor:pointer;font-family:inherit}
  .sw i{display:block;width:100%;aspect-ratio:1;border-radius:50%;border:2px solid #3a3a44;
        box-shadow:inset 0 0 0 1px rgba(0,0,0,.35);transition:transform .12s,box-shadow .12s,border-color .12s}
  .sw:active i{transform:scale(.93)}
  .sw.on i{border-color:#fff;transform:scale(1.04);box-shadow:0 0 0 3px rgba(255,255,255,.15)}
  .sw span{font-size:10.5px;color:var(--dim2);line-height:1}
  .sw.on span{color:var(--amber);font-weight:600}

  /* 提示 */
  .tip{display:flex;gap:6px;align-items:flex-start;font-size:11.5px;color:#8d8d97;
       background:#131317;border-radius:8px;padding:7px 9px;line-height:1.55;margin-top:10px}
  .tip b{color:var(--amber);font-weight:600}

  /* ===== 固件升级入口：设备页主视觉，一眼就要看到 ===== */
  .otacard{position:relative;display:flex;align-items:center;gap:12px;width:100%;
           margin:0 0 10px;padding:13px;border-radius:14px;cursor:pointer;
           font-family:inherit;text-align:left;overflow:hidden;
           border:1.5px solid #7c5618;
           background:linear-gradient(135deg,#2f2109 0%,#1f1607 52%,#2d200b 100%);
           box-shadow:0 8px 22px rgba(240,160,60,.15),inset 0 1px 0 rgba(255,214,150,.12);
           transition:border-color .15s,box-shadow .15s,transform .12s}
  .otacard::before{content:"";position:absolute;left:-30%;top:-130%;width:170%;height:230%;
                   background:radial-gradient(closest-side,rgba(255,196,110,.17),transparent 72%);
                   pointer-events:none}
  .otacard:active{transform:scale(.985)}
  .otacard .oi{position:relative;width:44px;height:44px;flex:0 0 auto;border-radius:13px;
               display:flex;align-items:center;justify-content:center;
               background:linear-gradient(180deg,#ffca7d,#ee9a2c);color:#231402;
               box-shadow:0 5px 14px rgba(255,170,60,.4)}
  .otacard .oi svg{width:23px;height:23px}
  .otacard .ot{position:relative;flex:1;min-width:0}
  .otacard .ot b{display:block;font-size:15.5px;font-weight:750;color:#ffd79a;letter-spacing:.3px}
  .otacard .ot em{display:block;font-style:normal;font-size:11.5px;line-height:1.55;
                  color:#a98a58;margin-top:3px}
  .otacard .ot em i{font-style:normal;color:#d9a964;font-variant-numeric:tabular-nums}
  .otacard .oa{position:relative;flex:0 0 auto;display:flex;align-items:center;color:#8d6d3a;
               transition:transform .18s}
  .otacard .oa svg{width:18px;height:18px}
  .otacard.on{border-color:#ffc06a;
              box-shadow:0 8px 26px rgba(255,180,80,.32),inset 0 1px 0 rgba(255,220,160,.2)}
  .otacard.on .oa{transform:rotate(90deg);color:#ffc06a}

  /* ===== 固件升级面板（设备页内展开） ===== */
  .otabox{display:none;margin-top:10px}
  .otabox.show{display:block;animation:fade .18s ease-out}
  .filepick{display:flex;align-items:center;justify-content:center;flex-direction:column;gap:4px;
            min-height:70px;padding:12px;border:1.5px dashed #3a3a44;border-radius:12px;
            background:#141418;color:#a9a9b3;font-size:13px;font-weight:600;
            text-align:center;cursor:pointer;transition:border-color .15s,background .15s,color .15s}
  .filepick small{display:block;font-size:10.5px;color:var(--dim2);font-weight:400;
                  line-height:1.5;margin-top:3px;word-break:break-all}
  .filepick.has{border-style:solid;border-color:#8a6320;background:#1a1409;color:var(--amber)}
  .filepick.has small{color:#9a7a44}
  .pbar{height:8px;background:#232329;border-radius:8px;overflow:hidden;margin:12px 0 0;display:none}
  .pbar.show{display:block}
  .pbar i{display:block;height:100%;width:0;border-radius:8px;
          background:linear-gradient(90deg,#f0a03c,#ffc06a);transition:width .18s ease-out}
  .pmeta{display:flex;justify-content:space-between;gap:8px;margin-top:6px;font-size:11px;
         color:var(--dim2);font-variant-numeric:tabular-nums;min-height:14px}
  .pmeta b{color:var(--amber);font-weight:600}
  .pbtn.amber{background:linear-gradient(180deg,#ffc06a,#f0a03c);border-color:#ffcf8a;color:#241503;
              box-shadow:0 6px 18px rgba(255,180,84,.26);min-height:54px;font-size:15.5px;
              font-weight:750;letter-spacing:.4px}
  /* 选好文件、可以点了 → 呼吸光，提醒你按这里 */
  .pbtn.amber:not([disabled]){animation:otaPulse 1.8s ease-in-out infinite}
  @keyframes otaPulse{
    0%,100%{box-shadow:0 6px 18px rgba(255,180,84,.26)}
    50%    {box-shadow:0 10px 30px rgba(255,198,112,.55)}
  }
  .otabox h2{font-size:15px}
  .otabox h2::before{content:"";display:inline-block;width:4px;height:14px;border-radius:2px;
                     background:linear-gradient(180deg,#ffc06a,#ef9c33);margin-right:8px;
                     vertical-align:-2px}
  .pbtn[disabled]{opacity:.45;pointer-events:none}
  .otaerr{font-size:11.5px;color:var(--red);margin-top:7px;line-height:1.6;display:none}
  .otaerr.show{display:block}

  /* ================= 使用手册（全屏浮层） ================= */
  .sheet{position:fixed;inset:0;z-index:30;display:none;flex-direction:column;
         background:var(--bg);max-width:560px;margin:0 auto}
  .sheet.show{display:flex;animation:slin .22s ease-out}
  @keyframes slin{from{opacity:0;transform:translateY(22px)}to{opacity:1;transform:none}}
  .shead{display:flex;align-items:center;gap:10px;padding:14px 14px 12px;
         border-bottom:1px solid var(--line);flex:0 0 auto}
  .shead h2{margin:0;font-size:16px;font-weight:650;flex:1}
  .shead .sub{font-size:11px;color:var(--dim2);font-weight:400;margin-left:6px}
  .sbody{flex:1;overflow-y:auto;-webkit-overflow-scrolling:touch;
         padding:12px 14px calc(22px + env(safe-area-inset-bottom))}
  .sec{background:var(--card);border:1px solid var(--line);border-radius:12px;
       margin-bottom:9px;overflow:hidden}
  .sec>summary{list-style:none;cursor:pointer;padding:13px 14px;font-size:14px;font-weight:650;
               display:flex;align-items:center;gap:9px;color:#e4e4ea}
  .sec>summary::-webkit-details-marker{display:none}
  .sec>summary::marker{display:none}
  .sec>summary::after{content:"";margin-left:auto;width:7px;height:7px;flex:0 0 auto;
                      border-right:2px solid var(--dim2);border-bottom:2px solid var(--dim2);
                      transform:rotate(-45deg) translate(-2px,-2px);transition:transform .18s}
  .sec[open]>summary::after{transform:rotate(45deg) translate(-2px,-2px)}
  .sec>summary .sn{width:21px;height:21px;border-radius:6px;background:#2c2214;color:var(--amber);
                   font-size:11px;font-weight:700;display:flex;align-items:center;
                   justify-content:center;flex:0 0 auto;font-variant-numeric:tabular-nums}
  .sec .sc{padding:0 14px 13px;font-size:13px;line-height:1.8;color:#b6b6bf}
  .sec .sc p{margin:0 0 9px}
  .sec .sc p:last-child{margin-bottom:0}
  .sec .sc b{color:#e8e8ee;font-weight:650}
  .sec .sc em{color:var(--amber);font-style:normal;font-weight:600}
  .sec .sc ol,.sec .sc ul{margin:0 0 9px;padding-left:19px}
  .sec .sc li{margin-bottom:5px}
  .sec .sc code{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:11.5px;
                background:#0d0d11;border:1px solid #26262c;border-radius:5px;
                padding:1px 5px;color:#d8b47a;white-space:nowrap}
  .snote{font-size:11.5px;color:var(--dim2);line-height:1.8;padding:4px 4px 0}
  body.lock{overflow:hidden}

  /* ================= 底部应用条（只在灯光页出现） ================= */
  .applybar{display:none;position:fixed;left:0;right:0;bottom:0;z-index:12;max-width:560px;margin:0 auto;
            background:rgba(13,13,16,.97);backdrop-filter:blur(12px);border-top:1px solid var(--line);
            padding:9px 14px calc(9px + env(safe-area-inset-bottom));
            align-items:center;gap:11px}
  body.light .applybar{display:flex}
  .abinfo{flex:1;min-width:0}
  .abinfo .l1{font-size:12.5px;font-weight:600;color:var(--dim);white-space:nowrap;
              overflow:hidden;text-overflow:ellipsis}
  .abinfo .l1 b{color:var(--amber);font-variant-numeric:tabular-nums}
  .abinfo .l2{font-size:10.5px;color:var(--dim2);margin-top:3px;white-space:nowrap;
              overflow:hidden;text-overflow:ellipsis;font-variant-numeric:tabular-nums}
  .apply{flex:0 0 auto;min-width:118px;min-height:46px;border-radius:12px;border:1px solid #3a3a44;
         background:#22222a;color:#7d7d88;font-family:inherit;font-size:14.5px;font-weight:700;
         letter-spacing:.5px;cursor:default;transition:background .15s,color .15s,box-shadow .15s,transform .1s}
  .apply.hot{background:linear-gradient(180deg,#ffc06a,#f0a03c);border-color:#ffcf8a;color:#241503;
             cursor:pointer;box-shadow:0 6px 18px rgba(255,180,84,.3)}
  .apply.hot:active{transform:scale(.97)}
  .apply.busy{background:#2b2b35;color:#c9c9d2;border-color:#3a3a44}

  #toast{position:fixed;left:50%;bottom:86px;transform:translateX(-50%) translateY(16px);
         background:#f0f0f4;color:#111;font-size:12.5px;font-weight:600;padding:8px 15px;border-radius:20px;
         opacity:0;transition:.2s;pointer-events:none;white-space:nowrap;z-index:20}
  #toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
</style>
</head>
<body class="light">

<header>
  <div class="logo">
    <svg viewBox="0 0 24 24" fill="none" stroke="#ffb454" stroke-width="1.8" stroke-linecap="round">
      <path d="M2 12h3M8 12h1M13 12h1M18 12h1M22 12h.01"/>
      <circle cx="5" cy="12" r="1.6" fill="#ffb454" stroke="none"/>
      <circle cx="11" cy="12" r="1.6" fill="#ffb454" stroke="none"/>
      <circle cx="16" cy="12" r="1.6" fill="#ffb454" stroke="none"/>
      <circle cx="20.5" cy="12" r="1.6" fill="#ffb454" stroke="none"/>
    </svg>
  </div>
  <div class="htxt">
    <h1>幻彩灯带控制板</h1>
    <div class="m"><span id="devName">连接中…</span><span class="chip" id="verChip">—</span></div>
  </div>
  <button class="iconbtn" id="helpOpen" aria-label="使用手册" title="使用手册">?</button>
  <div class="live" id="live"><span class="dot"></span><span id="liveN">在线</span></div>
</header>

<!-- ===== 一级：三个功能区 ===== -->
<nav class="ftabs" id="ftabs">
  <button class="on" data-f="light">灯光<i class="badge"></i></button>
  <button data-f="conn">连接</button>
  <button data-f="dev">设备</button>
</nav>

<!-- ============ 灯光 ============ -->
<section class="fpage on" data-f="light">
  <div class="preview">
    <canvas id="cv"></canvas>
    <div class="pvi">
      <span class="pm" id="pvMode">—</span>
      <span class="pd" id="pvDesc"></span>
    </div>
  </div>

  <div class="tabbar" id="tabbar"></div>

  <div class="panelbox">
    <!-- 模式 -->
    <section class="panel" data-k="mode">
      <div class="phead"><span class="pt">灯光模式</span><span class="pv" id="vMode">—</span></div>
      <div class="modes" id="modes"></div>
    </section>

    <!-- 亮度 -->
    <section class="panel" data-k="bright">
      <div class="phead"><span class="pt">亮度</span><span class="pv" id="vBright">—</span></div>
      <div class="rail" id="railBright">
        <div class="rt"><div class="rf"></div><div class="rticks"></div><div class="rk"></div></div>
        <div class="rb">—</div>
        <div class="ends"><span>0 关灯</span><span>100 最亮</span></div>
      </div>
      <div class="chips" id="chBright"></div>
      <div class="tip"><span>·</span><span><b>点方块</b>直选档位，<b>按住左右拖</b>无级微调。</span></div>
    </section>

    <!-- 速度 -->
    <section class="panel" data-k="spd">
      <div class="phead"><span class="pt">速度</span><span class="pv" id="vSpd">—</span></div>
      <div class="rail" id="railSpd">
        <div class="rt"><div class="rf"></div><div class="rticks"></div><div class="rk"></div></div>
        <div class="rb">—</div>
        <div class="ends"><span>很慢</span><span>很快</span></div>
      </div>
      <div class="chips three" id="chSpd"></div>
      <div class="tip"><span>·</span><span>常亮模式不用速度；呼吸、跑马、彩虹类才看得出差别。</span></div>
    </section>

    <!-- 颜色（只点选） -->
    <section class="panel" data-k="color">
      <div class="phead"><span class="pt">颜色</span><span class="pv" id="vColor">—</span></div>
      <div class="colors" id="colors"></div>
      <div class="tip"><span>·</span><span>彩虹类模式自己循环变色，不受这里影响；选「关」等于关灯。</span></div>
    </section>

    <!-- 灯带长度 -->
    <section class="panel" data-k="count">
      <div class="phead"><span class="pt">灯带长度（颗数）</span><span class="pv" id="vCount">—</span></div>
      <div class="stepwrap">
        <button class="stp" id="cntMinus" aria-label="减少一颗">&minus;</button>
        <div class="rail" id="railCount">
          <div class="rt"><div class="rf"></div><div class="rticks"></div><div class="rk"></div></div>
          <div class="rb">—</div>
          <div class="ends"><span>1</span><span class="cmax">64</span></div>
        </div>
        <button class="stp" id="cntPlus" aria-label="增加一颗">+</button>
      </div>
      <div class="chips four" id="chCount"></div>
      <div class="tip"><span>·</span><span>轨道粗调，<b>长按 &minus; / +</b> 一颗一颗精调。设多了不会坏，多出来的不点亮。</span></div>
    </section>
  </div>
</section>

<!-- ============ 连接 ============ -->
<section class="fpage" data-f="conn">
  <div class="card">
    <h2>WiFi 热点<span>AP 模式</span></h2>
    <div class="kv"><span class="k">网络名称</span><span class="v mono" id="cSsid">—</span></div>
    <div class="kv"><span class="k">密码</span><span class="v mono">12345678</span></div>
    <div class="kv"><span class="k">访问地址</span><span class="v mono" id="cIp">—</span></div>
    <div class="kv"><span class="k">在线设备</span><span class="v ok" id="cSta">—</span></div>
    <div class="kv"><span class="k">空闲自动关闭</span><span class="v amber" id="cIdle">—</span></div>
    <div class="prog"><i id="idlebar"></i></div>
    <div class="note" id="cIdleNote">连续 60 秒没有设备连接、没有网页操作、没有蓝牙连接时，WiFi 与蓝牙会自动关闭，只保留灯带功能。重新开启需断电重启。</div>
  </div>

  <div class="card">
    <h2>蓝牙调参<span>与热点同名</span></h2>
    <div class="kv" id="bleRow" style="cursor:pointer">
      <span class="k">蓝牙广播</span>
      <span class="v"><span class="switch ios" id="bleSw" role="switch" aria-checked="false"></span></span>
    </div>
    <div class="kv"><span class="k">广播名称</span><span class="v mono" id="bSsid">—</span></div>
    <div class="kv"><span class="k">状态</span><span class="v" id="bState">—</span></div>
    <div class="kv"><span class="k">透传通道</span><span class="v" id="bDiag">—</span></div>
    <div class="note" id="bNote">广播已关（默认）：手机搜不到本机，WiFi / USB 调参照常。</div>
    <div class="note">手机系统蓝牙里连这台设备，再在 Betaflight App 的端口列表里选它即可，不需要配对码。</div>
  </div>

  <div class="card">
    <h2>Betaflight 链路<span>UART1</span></h2>
    <div class="kv"><span class="k">波特率</span><span class="v">115200</span></div>
    <div class="kv"><span class="k">LED 数据帧</span><span class="v" id="fcFrames">—</span></div>
    <div class="kv"><span class="k">灯色字节顺序</span><span class="v"><button class="ordbtn" id="fcOrder" type="button">—</button></span></div>
    <div class="kv"><span class="k">网络调参端口</span><span class="v mono" id="mspUrl">tcp://192.168.4.1:5761</span></div>
    <div class="kv"><span class="k">TCP 客户端</span><span class="v mono" id="mspState">—</span></div>
    <div class="note">电脑端 Betaflight 配置器选 <b>Manual Selection</b>，填 <b>tcp://192.168.4.1:5761</b>。</div>
    <div class="note">灯色不对（设成<b>红</b>显示成<b>绿</b>、设成<b>蓝</b>显示成<b>红</b>）就点一下上面的顺序按钮换下一个，换到红的就是红的为止 —— 会自动记住，不用重启。</div>
  </div>

  <div class="stack">
    <a class="pbtn pri" href="https://app.betaflight.com" target="_blank" rel="noopener">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">
        <path d="M3 17l6-6 4 4 8-8"/><path d="M21 7v5h-5"/>
      </svg>
      打开 Betaflight 调参
    </a>
  </div>
  <div class="note">
    连着本热点时这个按钮打不开：本热点不联外网，所有域名都会被指回 192.168.4.1。
    要离线用，请先断开热点、用移动数据打开 app.betaflight.com 并「添加到主屏幕」。
  </div>
</section>

<!-- ============ 设备 ============ -->
<section class="fpage" data-f="dev">
  <div class="card">
    <h2>设备信息</h2>
    <div class="kv"><span class="k">设备名</span><span class="v mono" id="dName">—</span></div>
    <div class="kv"><span class="k">固件版本</span><span class="v mono" id="dVer">—</span></div>
    <div class="kv"><span class="k">构建时间</span><span class="v mono" id="dBld">—</span></div>
    <div class="kv"><span class="k">灯带长度</span><span class="v" id="dCount">—</span></div>
    <div class="kv"><span class="k">复位原因</span><span class="v" id="dRst">—</span></div>
    <div class="kv"><span class="k">运行时长</span><span class="v mono" id="uptime">—</span></div>
  </div>

  <button type="button" class="otacard" id="otaOpen">
    <span class="oi">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round">
        <path d="M12 16.5V4.2"/><path d="M7 8.6l5-4.4 5 4.4"/><path d="M4.5 19.6h15"/>
      </svg>
    </span>
    <span class="ot">
      <b>固件升级</b>
      <em>WiFi OTA · 不用插线 · 当前 <i id="otaCur">—</i></em>
    </span>
    <span class="oa">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round">
        <path d="M9 5.5l6.5 6.5L9 18.5"/>
      </svg>
    </span>
  </button>

  <div class="stack">
    <button class="pbtn" id="helpOpen2">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">
        <circle cx="12" cy="12" r="9"/><path d="M9.2 9.3a2.9 2.9 0 1 1 4 2.7c-.8.4-1.2 1-1.2 1.8v.3"/><path d="M12 17.2v.01"/>
      </svg>
      使用手册
    </button>
  </div>

  <div class="card otabox" id="otabox">
    <h2>固件升级<span>WiFi OTA</span></h2>
    <label class="filepick" id="fwPick">
      <input type="file" id="fwFile" accept=".bin" hidden>
      <span id="fwLabel">选择 firmware.bin
        <small>从 GitHub Actions 构件里下，别超过 1.5 MB</small>
      </span>
    </label>
    <div class="pbar" id="pbar"><i id="pfill"></i></div>
    <div class="pmeta"><span id="pml">未开始</span><span id="pmr"></span></div>
    <div class="otaerr" id="otaerr"></div>
    <div class="stack" style="margin:11px 0 0">
      <button class="pbtn amber" id="otaGo" disabled>开始升级</button>
    </div>
    <div class="note">
      升级大约 10~30 秒，<b style="color:#e8734a">期间绝对不能断电</b>。写入失败会自动回滚到旧固件，不会变砖。
      完成后设备自动重启，热点会消失一两秒，需要重新连接。
    </div>
  </div>

  <div class="card">
    <h2>调试信息<span>可复制</span></h2>
    <div class="toggle" id="dbgRow">
      <span>显示运行日志</span>
      <span class="switch" id="dbgSw"></span>
    </div>
    <div class="logwrap" id="logwrap">
      <div class="logtools">
        <button type="button" class="mini" id="logPause">暂停刷新</button>
        <button type="button" class="mini" id="logCopy">
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2"
               stroke-linecap="round" stroke-linejoin="round">
            <rect x="9" y="9" width="11" height="11" rx="2.5"/>
            <path d="M6.5 15H5.5A1.5 1.5 0 0 1 4 13.5v-8A1.5 1.5 0 0 1 5.5 4h8A1.5 1.5 0 0 1 15 5.5v1"/>
          </svg>
          复制
        </button>
      </div>
      <div class="logbox" id="logbox"></div>
    </div>
  </div>
</section>

<!-- ===== 底部：待应用状态 + 应用按钮（仅灯光页） ===== -->
<div class="applybar">
  <div class="abinfo">
    <div class="l1" id="abL1">读取中…</div>
    <div class="l2" id="abL2">—</div>
  </div>
  <button class="apply" id="applyBtn">应用</button>
</div>

<div id="toast"></div>

<!-- ================= 使用手册 ================= -->
<div class="sheet" id="sheet">
  <div class="shead">
    <h2>使用手册<span class="sub" id="sheetName">PavoLED</span></h2>
    <button class="iconbtn" id="helpClose" aria-label="关闭">&times;</button>
  </div>
  <div class="sbody">

    <details class="sec" open>
      <summary><span class="sn">1</span>第一次连接设备</summary>
      <div class="sc">
        <ol>
          <li>给飞控通电，灯带会先闪两下<b>青绿色自检</b>，然后进入当前模式。</li>
          <li>手机 WiFi 列表里找 <code>PavoLED-XX</code>（XX 是本机 MAC 末字节，每台不同），密码 <code>12345678</code>。</li>
          <li>连上后一般会自动弹出控制台；没弹就用浏览器打开 <code>192.168.4.1</code>。</li>
        </ol>
        <p>华为 / 荣耀手机只会在通知栏弹一条「已连接，需登录」，点那条通知才会进页面；点不动就直接用浏览器开上面那个地址。</p>
        <p>本热点<em>不联外网</em>，所有域名都会被指回 192.168.4.1。手机如果弹出「此网络无法上网，是否切换」，选<b>保持连接</b>，否则会悄悄切回移动数据、网页就打不开了。</p>
      </div>
    </details>

    <details class="sec">
      <summary><span class="sn">2</span>调灯光</summary>
      <div class="sc">
        <p>最上面三格是功能区：<b>灯光 / 连接 / 设备</b>。灯光页里再分五格参数：</p>
        <ul>
          <li><b>当前模式</b> —— 10 种，点一下就选中（还没写进灯带）。</li>
          <li><b>亮度 / 速度</b> —— 点方块直选档位，按住轨道左右拖是无级微调。亮度 0 等于关灯。</li>
          <li><b>颜色</b> —— 只能点选，8 个颜色，戴手套也好按；最后一个「关」是灭灯。</li>
          <li><b>灯带长度</b> —— 轨道粗调，两边的 <b>&minus; / +</b> 一颗一颗精调，长按会连发。范围 1~<span class="cmax">64</span>。</li>
        </ul>
        <p>改完点底部<b>「应用」</b>才真正写进灯带。改过没应用的格子右上角有<em>小圆点</em>，顶上「灯光」标签也会亮一个，切到别的页也看得见。此时预览条上写着「预演中」——灯带还是旧的。</p>
        <p>几种不能调的：常亮模式不用速度；彩虹类（07~09）自己循环变色，颜色设置对它们无效。</p>
        <p>末尾有几颗不亮不是坏了：灯带实际颗数比设置的多，多出来的不点亮而已，把「灯带长度」调大即可。</p>
      </div>
    </details>

    <details class="sec">
      <summary><span class="sn">3</span>用手机调 Betaflight（蓝牙）</summary>
      <div class="sc">
        <p>蓝牙广播名和热点同名（<code>PavoLED-XX</code>），不需要配对码。</p>
        <ol>
          <li>手机系统蓝牙里连它。</li>
          <li>打开 Betaflight App，端口列表里选这台设备，正常调参。</li>
        </ol>
        <p>蓝牙是这台板子<em>透传</em>给飞控的，所以 App 里看到的参数就是飞控真实的参数。「连接」页能看到当前是广播中还是已连接。</p>
      </div>
    </details>

    <details class="sec">
      <summary><span class="sn">4</span>电脑上调参（网络 / TCP）</summary>
      <div class="sc">
        <p>电脑连上本热点后，Betaflight 配置器选 <b>Manual Selection</b>，地址填 <code>tcp://192.168.4.1:5761</code>。</p>
        <p>手机网页版的 app.betaflight.com 也能用，但<b>连着本热点时打不开</b>——它要外网。做法是：先断开热点，用移动数据打开网页版，「添加到主屏幕」，之后再连热点从主屏幕图标进。</p>
      </div>
    </details>

    <details class="sec">
      <summary><span class="sn">5</span>固件升级（OTA）</summary>
      <div class="sc">
        <ol>
          <li>设备页 → <b>固件升级（WiFi OTA）</b> → 选择 <code>firmware.bin</code>。</li>
          <li>点「开始升级」，进度条走完后设备自己重启。期间<em>绝对不能断电</em>。</li>
          <li>重启时热点会断一下，重新连接即可。</li>
        </ol>
        <p>写入失败会自动回滚旧固件，不会变砖。文件大小要在 1.5 MB 以内，从 GitHub Actions 的构件里拿，别用源码压缩包。</p>
      </div>
    </details>

    <details class="sec">
      <summary><span class="sn">6</span>为什么热点会自己消失</summary>
      <div class="sc">
        <p>为了省电、也为了飞行时不给飞控添干扰：连续 <b>60 秒</b>既没有设备连接、也没有网页操作、也没有蓝牙连接，WiFi 和蓝牙就会一起关掉，只留灯带继续工作。网页开着不动不算闲——页面每秒都在跟设备说话。</p>
        <p>这是<em>一次性</em>的——关了之后不能再从网页打开，要断电重启才会恢复。所以调参时别让它闲着太久。</p>
      </div>
    </details>

    <details class="sec">
      <summary><span class="sn">7</span>遇到问题</summary>
      <div class="sc">
        <p><b>网页打不开</b> —— 先确认手机还连着这个热点（状态栏没有 WiFi 图标就是切走了）；再确认没超过 60 秒闲置。</p>
        <p><b>灯带完全不亮</b> —— 亮度是不是拖到 0 了、颜色是不是选了「关」；再看飞控里有没有打开 <code>LED_STRIP</code> 功能。</p>
        <p><b>只有前几颗亮</b> —— 「灯带长度」设小了，调大。</p>
        <p><b>飞行器不动但灯带在跑</b> —— 正常，灯带逻辑独立于飞控状态。</p>
        <p><b>Betaflight 模式颜色不对</b> —— 设成红显示成绿、设成蓝显示成红，就是灯色字节顺序跟飞控不一致：到「连接」页，点一下<b>灯色字节顺序</b>那个小药丸，换一个再看，换到红的就是红的为止（会自动记住，不用重启）。</p>
        <p><b>切到灯效模式灯却没变</b> —— 只有「Betaflight」模式才由飞控接管灯带；其余模式彻底本地渲染。若切了完全没反应，先看「连接」页的 LED 帧数是不是在涨。</p>
        <p><b>电脑配置器连不上</b> —— 检查端口填的是 <code>tcp://192.168.4.1:5761</code>，不是 192.168.4.1:5761（少了 tcp:// 会被当网页地址）。</p>
        <p><b>设备重启 / 看日志</b> —— 本页不提供重启按钮（防止误触），断电重新上电即可。</p>
      </div>
    </details>

    <div class="snote">这份手册内置在固件里，断网也能看。版本 <span id="sheetVer">—</span>。</div>
  </div>
</div>

<script>
/* ================= 固件侧的常量与状态 ================= */
var MODES = ["Betaflight","常亮","呼吸","跑马","双边跑马","导向","增长","彩虹呼吸","彩虹渐变","双边彩虹"];

/* 顺序必须与 effects.cpp 的 COLORS[] 一致：白红黄蓝绿青品红、关闭 */
var COLORS = [
  {n:"白",   h:-1,  s:0,   css:"#ffffff"},
  {n:"红",   h:0,   s:100, css:"#ff3b30"},
  {n:"黄",   h:60,  s:100, css:"#ffd60a"},
  {n:"蓝",   h:240, s:100, css:"#3b82f6"},
  {n:"绿",   h:120, s:100, css:"#22c55e"},
  {n:"青",   h:180, s:100, css:"#22d3ee"},
  {n:"品红", h:300, s:100, css:"#e879f9"},
  {n:"关",   h:-1,  s:0,   css:"#0b0b0d"}
];

/* 灯带缓冲上限的默认值（须与 config.h 的 STRIP_MAX_LEDS 一致）。
   真正以 /api/state 的 maxc 为准；拿到后若不同会重建一次轨道，见 ensureCountRail()。 */
var COUNT_MAX = 128;

/* base = 灯带上正在跑的那套；st = 界面里正在改的那套 */
var base = {mode:1, color:0, spd:55, bright:50, count:55};
var st   = {mode:1, color:0, spd:55, bright:50, count:55, ford:3, fordn:"RBG"};
var dev  = null;      // 最近一次 /api/state 的原始 JSON
var MAXC = COUNT_MAX;

var TABS = [
  {k:"mode",   n:"当前模式"},
  {k:"bright", n:"亮度"},
  {k:"spd",    n:"速度"},
  {k:"color",  n:"颜色"},
  {k:"count",  n:"灯带长度"}
];

/* ================= 工具 ================= */
function $(id){ return document.getElementById(id); }
var INIT_HASH = (location.hash || "").replace("#","");
function clamp(v,a,b){ return v<a?a:(v>b?b:v); }
function spdLabel(v){
  if(v < 20) return "很慢";
  if(v < 40) return "慢";
  if(v < 65) return "中";
  if(v < 85) return "快";
  return "很快";
}
function tabVal(k){
  if(k === "mode")   return MODES[st.mode] || "—";
  if(k === "bright") return st.bright + "%";
  if(k === "spd")    return st.spd + "";
  if(k === "color")  return COLORS[st.color] ? COLORS[st.color].n : "—";
  return st.count + "";
}
function isDirty(k){
  return st[k] !== base[k];
}
function dirtyList(){
  var out = [];
  for(var i=0;i<TABS.length;i++) if(isDirty(TABS[i].k)) out.push(TABS[i]);
  return out;
}
function shake(ms){ if(navigator.vibrate){ try{ navigator.vibrate(ms); }catch(e){} } }

/* ================= 一级：功能区切换 ================= */
var curFeature = "light";
var featureBtns = $("ftabs").querySelectorAll("button");
function goFeature(f){
  curFeature = f;
  var pages = document.querySelectorAll(".fpage");
  for(var i=0;i<pages.length;i++) pages[i].classList.toggle("on", pages[i].getAttribute("data-f") === f);
  for(var k=0;k<featureBtns.length;k++) featureBtns[k].classList.toggle("on", featureBtns[k].getAttribute("data-f") === f);
  document.body.classList.toggle("light", f === "light");
  window.scrollTo(0, 0);
  if(f === "light"){ for(var r in rails){ if(rails[r] && rails[r]._render) rails[r]._render(); } }
}
for(var fi=0;fi<featureBtns.length;fi++){
  (function(b){ b.onclick = function(){ goFeature(b.getAttribute("data-f")); }; })(featureBtns[fi]);
}

/* ================= 二级：五格参数标签 ================= */
var tabEls = {};
function buildTabs(){
  var box = $("tabbar");
  box.innerHTML = "";
  for(var i=0;i<TABS.length;i++){
    (function(t){
      var b = document.createElement("button");
      b.className = "tab";
      b.innerHTML = '<span class="tk">' + t.n + '</span><span class="tv"></span><i class="d2"></i>';
      b.onclick = function(){ selectTab(t.k); };
      box.appendChild(b);
      tabEls[t.k] = b;
    })(TABS[i]);
  }
}

var curTab = "mode";
function selectTab(k){
  curTab = k;
  var panelBox = document.querySelectorAll(".panel");
  for(var i=0;i<panelBox.length;i++){
    panelBox[i].classList.toggle("show", panelBox[i].getAttribute("data-k") === k);
  }
  for(var key in tabEls) tabEls[key].classList.toggle("on", key === k);
  for(var r in rails){ if(rails[r] && rails[r]._render) rails[r]._render(); }
}

/* ================= 通用轨道 ================= */
var rails = {};
function makeRail(cfg){
  var box    = $(cfg.id);
  var track  = box.querySelector(".rt");
  var fill   = box.querySelector(".rf");
  var knob   = box.querySelector(".rk");
  var bubble = box.querySelector(".rb");
  var tickBox= box.querySelector(".rticks");

  var span = cfg.max - cfg.min;
  function pct(v){ return span === 0 ? 0 : (v - cfg.min) / span; }
  function fromPct(p){ return Math.round(cfg.min + p * span); }

  var ticks = [];
  if(cfg.ticks){
    for(var k=0;k<cfg.ticks.length;k++){
      var v = cfg.ticks[k];
      var d = document.createElement("i");
      d.className = "tick";
      var lp = pct(v) * 100;
      d.style.left = lp.toFixed(3) + "%";
      /* 贴两端的方块不再 -50% 位移，否则一半被 overflow 裁掉 */
      d.style.transform = "translate(" + (lp < 2 ? "0" : (lp > 98 ? "-100%" : "-50%")) + ",-50%)";
      tickBox.appendChild(d);
      ticks.push({v:v, el:d});
    }
  }

  function render(){
    var v = cfg.get();
    var p = clamp(pct(v), 0, 1);
    if(fill) fill.style.width = (p*100).toFixed(2) + "%";
    knob.style.left = (p*100).toFixed(2) + "%";
    if(bubble) bubble.textContent = cfg.fmt(v);
    var best = -1, bd = 1e9;
    for(var i=0;i<ticks.length;i++){
      var dd = Math.abs(ticks[i].v - v);
      if(dd < bd){ bd = dd; best = i; }
    }
    for(var i=0;i<ticks.length;i++) ticks[i].el.classList.toggle("on", i === best);
  }

  var dragging = false;
  function valueAt(clientX){
    var r = track.getBoundingClientRect();
    var p = (clientX - r.left) / r.width;
    return fromPct(clamp(p, 0, 1));
  }
  function put(v){
    if(v !== cfg.get()){
      cfg.set(v);
      shake(6);
    }
    render();
    syncAll();
  }

  box.addEventListener("pointerdown", function(e){
    if(e.pointerType === "mouse" && e.button !== 0) return;
    dragging = true;
    box.classList.add("dragging");
    try{ box.setPointerCapture(e.pointerId); }catch(err){}
    put(valueAt(e.clientX));
    e.preventDefault();
  });
  box.addEventListener("pointermove", function(e){
    if(!dragging) return;
    put(valueAt(e.clientX));
    e.preventDefault();
  });
  function stop(){
    if(!dragging) return;
    dragging = false;
    box.classList.remove("dragging");
  }
  box.addEventListener("pointerup", stop);
  box.addEventListener("pointercancel", stop);

  box._render = render;
  render();
  rails[cfg.id] = box;
  return box;
}

/* ================= 颜色（只点选） ================= */
function buildColors(){
  var box = $("colors");
  box.innerHTML = "";
  for(var i=0;i<COLORS.length;i++){
    (function(c, idx){
      var b = document.createElement("button");
      b.className = "sw";
      b.innerHTML = '<i style="background:' + c.css + '"></i><span>' + c.n + '</span>';
      b.onclick = function(){
        st.color = idx;
        shake(6);
        syncAll();
      };
      box.appendChild(b);
    })(COLORS[i], i);
  }
}
function renderColors(){
  var box = $("colors");
  for(var i=0;i<box.children.length;i++){
    box.children[i].classList.toggle("on", i === st.color);
  }
}

/* ================= 模式 ================= */
function buildModes(){
  var box = $("modes");
  box.innerHTML = "";
  for(var i=0;i<MODES.length;i++){
    (function(i){
      var b = document.createElement("button");
      b.className = "mode";
      b.innerHTML = '<span class="n">' + ("0"+i).slice(-2) + '</span><span>' + MODES[i] + '</span>';
      b.onclick = function(){
        st.mode = i;
        shake(6);
        syncAll();
      };
      box.appendChild(b);
    })(i);
  }
}
function renderModes(){
  var bs = $("modes").children;
  for(var k=0;k<bs.length;k++) bs[k].classList.toggle("on", k === st.mode);
}

/* ================= 灯带长度：− / + 长按连发 ================= */
function stepCount(d){
  var v = clamp(st.count + d, 1, MAXC);
  if(v === st.count) return;
  st.count = v;
  shake(5);
  syncAll();
}
function holdBtn(el, fn){
  var t1 = null, t2 = null;
  function start(e){
    e.preventDefault();
    fn();
    t1 = setTimeout(function(){ t2 = setInterval(fn, 110); }, 430);
  }
  function end(){ clearTimeout(t1); clearInterval(t2); }
  el.addEventListener("pointerdown", start);
  el.addEventListener("pointerup", end);
  el.addEventListener("pointercancel", end);
  el.addEventListener("pointerleave", end);
}

/* ================= 常用档（chips） ================= */
var CHIPS = {
  chBright: { get:function(){return st.bright;}, set:function(v){st.bright=v;},
              items:[[0,"关"],[25,"25"],[50,"50"],[75,"75"],[100,"100"]] },
  chSpd:    { get:function(){return st.spd;},    set:function(v){st.spd=v;},
              items:[[10,"慢"],[50,"中"],[90,"快"]] },
  /* 只有「灯带长度」需要按本机上限裁掉画不下的档位（max:true），
     亮度/速度的上限就是 100，别拿 MAXC 去比 —— 踩过：亮度 75/100 被吞掉 */
  chCount:  { get:function(){return st.count;},  set:function(v){st.count=v;}, max:true,
              items:[[32,"32"],[55,"55"],[64,"64"]] }
};
function buildChips(){
  for(var id in CHIPS){
    (function(cfg){
      var box = $(id);
      if(!box) return;
      box.innerHTML = "";
      for(var i=0;i<cfg.items.length;i++){
        (function(v, l){
          if(cfg.max && v > MAXC) return;   // 只有长度档要按上限裁
          var b = document.createElement("button");
          b.className = "chip2";
          b.textContent = l;
          b.dataset.v = v;
          b.onclick = function(){
            cfg.set(v);
            shake(6);
            syncAll();
          };
          box.appendChild(b);
        })(cfg.items[i][0], cfg.items[i][1]);
      }
    })(CHIPS[id]);
  }
}
function renderChips(){
  for(var id in CHIPS){
    var box = $(id);
    if(!box) continue;
    var cur = CHIPS[id].get();
    for(var i=0;i<box.children.length;i++){
      box.children[i].classList.toggle("on", Number(box.children[i].dataset.v) === cur);
    }
  }
}

/* ================= 统一刷新 ================= */
var dirtyCache = [];
function syncAll(){
  var i, t;

  dirtyCache = dirtyList();
  for(i=0;i<TABS.length;i++){
    t = TABS[i];
    var el = tabEls[t.k];
    if(!el) continue;
    var tvEl = el.querySelector(".tv");
    var tvTxt = tabVal(t.k);
    tvEl.textContent = tvTxt;
    tvEl.classList.toggle("sm", tvTxt.length > 4);   // 「Betaflight」这种长名字缩字号
    el.classList.toggle("on", t.k === curTab);
    el.classList.toggle("dirty", isDirty(t.k));
  }
  featureBtns[0].classList.toggle("dirty", dirtyCache.length > 0);

  $("pvMode").textContent = MODES[st.mode] || "—";
  $("pvDesc").innerHTML = (dirtyCache.length ? "<b>预演中</b> · " : "") +
      "亮度 " + st.bright + "% · 速度 " + st.spd + " · " + st.count + " 颗 · " +
      (COLORS[st.color] ? COLORS[st.color].n : "—");

  $("vMode").textContent   = MODES[st.mode] || "—";
  $("vBright").textContent = st.bright + "%";
  $("vSpd").textContent    = spdLabel(st.spd) + " " + st.spd;
  $("vColor").textContent  = COLORS[st.color] ? COLORS[st.color].n : "—";
  $("vCount").textContent  = st.count + " 颗";

  var dc = $("dCount");
  dc.textContent = st.count + " 颗";
  dc.className = "v" + (st.count !== base.count ? " amber" : "");

  renderModes();
  renderColors();
  renderChips();
  if(rails.railBright) rails.railBright._render();
  if(rails.railSpd)    rails.railSpd._render();
  if(rails.railCount)  rails.railCount._render();

  $("cntMinus").classList.toggle("off", st.count <= 1);
  $("cntPlus").classList.toggle("off", st.count >= MAXC);

  var btn = $("applyBtn");
  if(!applying){
    if(dirtyCache.length){
      var msg = '待应用 <b>' + dirtyCache.length + '</b> 项改动';
      if(dirtyCache.length <= 2){
        var names = [];
        for(i=0;i<dirtyCache.length;i++) names.push(dirtyCache[i].n);
        msg = '待应用 <b>' + dirtyCache.length + '</b> 项：' + names.join("、");
      }
      $("abL1").innerHTML = msg;
      btn.textContent = "应用";
      btn.className = "apply hot";
    }else{
      $("abL1").textContent = "灯带已是最新";
      btn.textContent = "应用";
      btn.className = "apply";
    }
  }
}

/* ================= 应用：一次 POST 全量提交 ================= */
var applying = false;
function doApply(){
  if(applying || !dirtyCache.length) return;
  applying = true;
  var btn = $("applyBtn");
  btn.className = "apply busy";
  btn.textContent = "写入中…";
  $("abL1").textContent = "正在写入灯带…";
  shake([10,40,10]);

  var body = "mode="   + st.mode   + "&color=" + st.color +
             "&spd="   + st.spd    + "&bright=" + st.bright +
             "&count=" + st.count;
  var req = new XMLHttpRequest();
  req.open("POST", "/api/set", true);
  req.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  req.timeout = 6000;
  req.onload = function(){
    var j = null;
    try{ j = JSON.parse(req.responseText); }catch(e){}
    if(j && typeof j.mode === "number"){
      absorb(j);                 // 以设备回读的值为准（含边界钳制后的结果）
      applying = false;
      syncAll();
      toast("已写入灯带");
    }else{
      applying = false;
      syncAll();
      toast("设备没回话，稍后再试");
    }
  };
  req.onerror = req.ontimeout = function(){
    applying = false;
    syncAll();
    toast("上传失败：确认手机还连着本热点");
  };
  req.send(body);
}

/* ================= 飞控灯色字节顺序（v32.23） ================= */
/* 板上 WS2812 常见 GRB，但飞控侧（ledstrip_grb_rgb 设置 / LED 芯片批次）可能不同，
   顺序错就表现为「设红显示绿、设蓝显示红」。这里点一下换下一个，设备立即按新顺序
   还原、并写进 NVS（断电不忘），不用重新刷固件。 */
var ORDER_NAMES = ["GRB","RGB","BRG","RBG","GBR","BGR"];
function cycleFcOrder(){
  var btn = $("fcOrder");
  if(btn.disabled) return;
  var next = ((st.ford|0) + 1) % 6;
  btn.disabled = true;
  btn.textContent = "…";
  var req = new XMLHttpRequest();
  req.open("POST", "/api/set", true);
  req.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  req.timeout = 6000;
  req.onload = function(){
    btn.disabled = false;
    var j = null;
    try{ j = JSON.parse(req.responseText); }catch(e){}
    if(j && typeof j.ford === "number"){
      st.ford  = j.ford;
      st.fordn = j.fordn || ORDER_NAMES[j.ford] || "";
      renderFcOrder();
      toast("灯色顺序 " + st.fordn);
    }else{
      renderFcOrder();
      toast("设备没回话，稍后再试");
    }
  };
  req.onerror = req.ontimeout = function(){
    btn.disabled = false;
    renderFcOrder();
    toast("上传失败：确认手机还连着本热点");
  };
  req.send("ford=" + next);
}
function renderFcOrder(){
  var b = $("fcOrder");
  if(!b) return;
  b.textContent = (st.fordn || ORDER_NAMES[st.ford|0] || "—") + " ⇄";
}

/* ================= BLE 广播开关（v32.32） ================= */
/* 默认关。C3 是单射频，BLE 广播和 WiFi 分时抢同一个射频，广播事件会打断灯带
   数据流（灯带偶发频闪）。只有要用手机 App 蓝牙调参时才打开，调完关掉。
   开关即时生效、断电记忆，不需要重启（协议栈一直在，只是不发广播包）。 */
function renderBleSw(){
  var sw = $("bleSw");
  if(sw) sw.classList.toggle("on", (st.bleen|0) === 1);
  var n = $("bNote");
  if(n) n.textContent = (st.bleen|0) === 1
    ? "广播中：手机蓝牙里能搜到本机。调完参数建议关掉，飞行时能少一层干扰。"
    : "广播已关（默认）：手机搜不到本机，WiFi 调参（tcp://192.168.4.1:5761）与 USB 调参照常。";
}
function toggleBle(){
  var on = ((st.bleen|0) === 1) ? 0 : 1;
  var req = new XMLHttpRequest();
  req.open("POST", "/api/set", true);
  req.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  req.timeout = 8000;
  req.onload = function(){
    var j = null;
    try{ j = JSON.parse(req.responseText); }catch(e){}
    if(j && typeof j.bleen === "number"){
      st.bleen = j.bleen;
      renderBleSw();
      toast(j.bleen ? "蓝牙广播已打开，手机可以搜到本机了"
                    : "蓝牙广播已关闭，灯带干扰最小");
    }else{
      renderBleSw();
      toast("设备没回话，稍后再试");
    }
  };
  req.onerror = req.ontimeout = function(){
    renderBleSw();
    toast("上传失败：确认手机还连着本热点");
  };
  req.send("bleen=" + on);
}

/* ================= 从设备状态同步 ================= */
var upBase = 0, upAt = 0;
var RST = {1:"上电复位",2:"外部复位",3:"软件重启",4:"内核崩溃",5:"中断看门狗",
           6:"任务看门狗",7:"看门狗",8:"深睡唤醒",9:"电压跌落",10:"SDIO",
           11:"USB",12:"JTAG",13:"EFUSE",14:"电源抖动",15:"CPU 锁死"};
/* v32.33：崩溃落点（固件里 stageMark 打的标记），重启后仍能读出来 */
var STAGE = {1:"启动初始化",2:"主循环空闲",3:"灯效渲染",4:"飞控映射",
             5:"灯带输出",6:"无线轮询",7:"按键处理"};
var upTick = 0;

function absorb(j){
  /* 关键顺序：先记住「改之前有没有未应用改动」，再覆盖 base。
     反过来的话刚进页面时 st(默认值) 会跟新的 base 一比就"脏了"，
     界面永远停在默认值、不跟设备走 —— 这个 bug 实测踩过一次。 */
  var wasDirty = dirtyList().length > 0;
  dev = j;
  if(typeof j.maxc === "number" && j.maxc > 0){
    MAXC = j.maxc;
    ensureCountRail();   // 上限与本地默认值不同时重建「灯带长度」轨道
  }
  base = {mode:j.mode, color:j.color, spd:j.spd, bright:j.bright, count:j.count};
  /* 没有待应用改动时，界面跟着设备走；有改动就保留用户正在改的值 */
  if(!wasDirty){
    st = {mode:base.mode, color:base.color, spd:base.spd, bright:base.bright,
          count:base.count, ford:st.ford, fordn:st.fordn};
  }
  /* 灯色顺序不属于「待应用」的 5 项，单独跟设备同步 */
  if(typeof j.ford === "number"){
    st.ford  = j.ford;
    st.fordn = j.fordn || "";
  }
  /* v32.32：BLE 广播开关（同样不属于「待应用」的 5 项，直接跟设备同步） */
  if(typeof j.bleen === "number") st.bleen = j.bleen;
  renderBleSw();
  if(typeof j.up === "number"){ upBase = j.up; upAt = Date.now(); }
  renderDev(j);
}

function hhmmss(s){
  var h = Math.floor(s/3600), m = Math.floor(s%3600/60), q = Math.floor(s%60);
  function p2(v){ return (v<10?"0":"") + v; }
  return p2(h)+":"+p2(m)+":"+p2(q);
}
function bleText(s){
  if(!s) return "—";
  if(s.indexOf("off") >= 0)  return {t:"已关闭", c:""};
  if(s.indexOf("idle") >= 0) return {t:"未广播（已关）", c:""};
  if(s.indexOf("err") >= 0 || s.indexOf("fail") >= 0) return {t:"异常", c:"amber"};
  if(s.indexOf("conn") >= 0) return {t:s.indexOf("no sub") >= 0 ? "已连接（未订阅）" : "已连接", c:"ok"};
  return {t:"广播中", c:"ok"};
}

function renderDev(j){
  $("devName").textContent = j.ssid || "PavoLED";
  $("verChip").textContent = j.v || "—";
  $("sheetName").textContent = j.ssid || "PavoLED";
  $("sheetVer").textContent  = j.v || "—";
  document.title = (j.ssid || "幻彩灯带") + " · 控制台";

  /* 连接页 */
  $("cSsid").textContent = j.ssid || "—";
  $("cIp").textContent   = j.ip || "—";
  $("cSta").textContent  = (j.sta|0) + " 台";
  $("bSsid").textContent = j.ssid || "—";
  var bl = bleText(j.ble);
  $("bState").textContent = bl.t;
  $("bState").className = "v" + (bl.c ? " " + bl.c : "");
  $("bDiag").textContent = j.bdiag || "—";
  $("fcFrames").textContent = (typeof j.fc === "number")
      ? (j.fc + " 帧" + (typeof j.lend === "number" && j.lend ? " · 末帧 " + j.lend + " 颗" : "")
              + (j.bad ? " · 丢弃 " + j.bad : ""))
      : "—";
  renderFcOrder();
  $("mspUrl").textContent   = "tcp://" + (j.ip || "192.168.4.1") + ":5761";
  $("mspState").textContent = j.msp || "—";

  /* 倒计时：idle < 0 表示有人在用，不会关 */
  var idle = (typeof j.idle === "number") ? j.idle : -1;
  if(idle < 0){
    $("cIdle").textContent = "不会关闭";
    $("cIdle").className = "v ok";
    $("idlebar").style.width = "100%";
    $("abL2").textContent = "本机在线，不会自动关闭";
  }else{
    $("cIdle").textContent = "00:" + (idle<10?"0":"") + idle;
    $("cIdle").className = "v amber";
    $("idlebar").style.width = (idle/60*100).toFixed(0) + "%";
    $("abL2").textContent = "无线空闲 00:" + (idle<10?"0":"") + idle + " 后自动关闭";
  }

  /* 设备页 */
  $("dName").textContent = j.ssid || "—";
  $("dVer").textContent  = j.v || "—";
  $("dBld").textContent  = j.bld || "—";
  if($("otaCur")) $("otaCur").textContent = j.v || "—";
  $("dRst").textContent  = RST[j.rs] || ("代码 " + j.rs);

  /* 调试日志 */
  var lend = j.lend|0, need = j.need|0, board = j.count|0, ratio = j.ratio|0;
  var needTxt = "";
  if(need > 0){
    needTxt = " · 飞控应发 " + need + " 颗（本板 " + board + " 颗 · 比例 1:" + ratio + "）";
    if(Math.abs(lend - need) > 1){
      needTxt += " · 对不上：差 " + Math.abs(lend - need) + " 颗" +
                 (lend > need ? "，多出的被丢掉" : "，末端会缺色") +
                 "（BF 里 LED 数量改成 " + need + "）";
    }
  }
  var rsn = RST[j.rs] || ("代码 " + j.rs);
  var lastSt = (j.lstg|0) > 0
      ? " · 上次跑到「" + (STAGE[j.lstg|0] || ("#" + (j.lstg|0))) + "」时中断（已跑 " + (j.lup|0) + "s）"
      : "";
  logText = "[固件] " + (j.v||"?") + " · build " + (j.bld||"?") + "\n" +
            "[WiFi] " + (j.ssid||"?") + " " + (j.ip||"?") + " · 在线 " + (j.sta|0) +
                     " · 空闲 " + (idle < 0 ? "在用" : idle + "s") + "\n" +
            "[DNS ] 劫持 " + (j.dns ? "ok" : "fail") + "\n" +
            "[BLE ] " + (j.ble === "ble idle"
                         ? "ble idle · 协议栈在跑、按设置不广播（要手机调参就打开上面的开关）"
                         : (j.ble||"?")) + "\n" +
            "[BLE ] " + (j.bdiag||"?") + "\n" +
            "[MSP ] " + (j.msp||"?") + "\n" +
            "[LED ] 发送 " + (j.tx|0) + "/s · 失败 " + (j.txf|0) +
                     " · 循环卡顿 " + (j.lpm|0) + "ms\n" +
            "[RX  ] LED 帧 " + (j.fc|0) + " · 字节顺序 " + (j.fordn||"?") +
                     " · 位周期 " + (j.per|0) + "ns\n" +
            // v32.33：「解出」= 本帧收到的位数 ÷ 24，不是飞控配置；
            // 「本板」用的是灯带长度（v32.32 这里错拿了标定状态位，永远显示 0 颗）
            "[RX  ] 解出 " + lend + " 颗 / " + (j.bits|0) + " 位" + needTxt + "\n" +
            "[RX  ] 帧头缺 " + (j.dropb|0) + " 位 · 断流 " + (j.gap|0) +
                     " · 毛刺 " + (j.spk|0) + " · 合并 " + (j.mrg|0) +
                     " · 临界位 " + (j.near|0) + "\n" +
            "[RX  ] 存疑帧 " + (j.mis|0) + "（帧内出过断流/毛刺，那几帧颜色会错位）\n" +
            "[RX  ] 高位 " + (j.hmin|0) + "~" + (j.hmax|0) + "ns · 阈值 " + (j.thr|0) +
                     "ns · 最长位 " + (j.gmax|0) + "ns · 捕获 " + (j.cap|0) + "µs\n" +
            "[RX  ] 残片 " + (j.frag|0) + " · 幽灵边沿 " + (j.ghost|0) + " · 静默 " + (j.mute|0) +
                     " · 自愈 " + (j.rxs|0) + " · 窗口超时 " + (j.capd|0) +
                     " · 飞控静默 " + (j.sil|0) + "s" +
                     ((j.sil|0) > 1 ? "（收不到飞控信号！）" : "") + "\n" +
            "[RX  ] 线路原始 前4颗 " + (j.head || "-") + "\n" +
            "[BOOT] " + (j.boot||"?") + "\n" +
            "[BOOT] 复位 " + rsn + lastSt;
  if(dbgOn && !logPaused) $("logbox").textContent = logText;
  if(dbgOn && !logPaused){
    var lb0 = $("logbox");
    lb0.scrollTop = lb0.scrollHeight;   // 跟到最新一行，省得手动拖
  }
}

/* ================= 轮询 /api/state ================= */
var logText = "", dbgOn = false, logPaused = false;
function setLive(on, txt){
  $("live").classList.toggle("off", !on);
  $("liveN").textContent = txt;
}
function pull(){
  if(otaRunning) return;   // OTA 上传时别抢带宽
  var req = new XMLHttpRequest();
  req.open("GET", "/api/state", true);
  req.timeout = 4000;
  req.onload = function(){
    var j = null;
    try{ j = JSON.parse(req.responseText); }catch(e){}
    if(!j || typeof j.mode !== "number"){ setLive(false, "异常"); return; }
    setLive(true, "在线 " + (j.sta|0));
    absorb(j);
    syncAll();
  };
  req.onerror = req.ontimeout = function(){ setLive(false, "断开"); };
  req.send();
}
setInterval(pull, 1000);

/* 运行时长按秒自走，不等下一次轮询，看起来才顺 */
setInterval(function(){
  var s = upBase + (upAt ? (Date.now() - upAt)/1000 : 0);
  $("uptime").textContent = hhmmss(s);
}, 1000);

/* ================= 灯带预览（Canvas） ================= */
function px(i, n, t){
  var per  = Math.max(300, 2400 - st.spd * 21);
  var ph   = (t % per) / per;
  var d    = Math.max(1, n - 1);
  var c    = COLORS[st.color] || COLORS[0];
  var h = c.h, s = c.s, v = 1;

  switch(st.mode){
    case 0: {
      var q = ((t / 2600) % 1);
      var dd0 = Math.abs(i / d - q);
      v = 0.14 + 0.86 * Math.exp(-dd0 * dd0 * 90);
      break;
    }
    case 1: v = 1; break;
    case 2: v = 0.06 + 0.94 * (0.5 + 0.5 * Math.sin(ph * 6.2832 - 1.5708)); break;
    case 3: {
      var p3 = ph * n;
      var dd3 = Math.abs(i - p3); dd3 = Math.min(dd3, n - dd3);
      v = Math.exp(-dd3 * dd3 * 0.9);
      break;
    }
    case 4: {
      var p1 = ph * n, p2 = (1 - ph) * n;
      var a1 = Math.abs(i - p1), a2 = Math.abs(i - p2);
      v = Math.max(Math.exp(-a1*a1*0.9), Math.exp(-a2*a2*0.9));
      break;
    }
    case 5: {
      var p5 = ph * n;
      var dd5 = (i - p5 + n) % n;
      var tail = n * 0.45;
      v = dd5 < tail ? Math.pow(1 - dd5 / tail, 2.0) : 0;
      break;
    }
    case 6: {
      var k = ph < 0.55 ? ph / 0.55 : 1 - (ph - 0.55) / 0.45;
      v = (i / d) <= k ? 1 : 0.02;
      break;
    }
    case 7:
      h = i / d * 360;
      v = 0.08 + 0.92 * (0.5 + 0.5 * Math.sin(ph * 6.2832));
      break;
    case 8:
      h = (i / d * 360 + ph * 360) % 360;
      v = 1;
      break;
    case 9:
      h = Math.abs(2 * i / d - 1) * 360;
      v = 1;
      break;
  }

  v *= st.bright / 100;
  if(st.color === 7) v = 0;          // 颜色「关」= 全灭
  if(st.mode >= 7) s = 100;
  return [h, s, v];
}

function drawStrip(t){
  var cv = $("cv");
  if(!cv) return;
  var dpr = window.devicePixelRatio || 1;
  var w = cv.clientWidth, h = cv.clientHeight;
  if(!w || !h) return;
  if(cv.width !== Math.round(w*dpr) || cv.height !== Math.round(h*dpr)){
    cv.width  = Math.round(w*dpr);
    cv.height = Math.round(h*dpr);
  }
  var g = cv.getContext("2d");
  g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,w,h);

  var n   = st.count;
  var pad = 7;
  var span= w - pad*2;
  var gap = span / n;
  var r   = Math.max(1.0, Math.min(gap * 0.42, h * 0.40));
  var cy  = h / 2;

  for(var i=0;i<n;i++){
    var x = pad + gap * (i + 0.5);
    var c = px(i, n, t);
    var v = c[2];

    g.beginPath();
    g.arc(x, cy, Math.max(r, 1), 0, 6.2832);
    g.fillStyle = "rgba(255,255,255,0.075)";
    g.fill();

    if(v > 0.015){
      var l = 13 + 47 * Math.min(1, v);
      var col = "hsl(" + c[0].toFixed(0) + "," + c[1].toFixed(0) + "%," + l.toFixed(1) + "%)";
      g.fillStyle = col;
      g.shadowColor = col;
      g.shadowBlur = 4 + 11 * v;
      g.beginPath();
      g.arc(x, cy, r, 0, 6.2832);
      g.fill();
      g.shadowBlur = 0;
    }
  }
}

var t0 = 0;
function loop(ts){
  if(!t0) t0 = ts;
  if(curFeature === "light") drawStrip(ts - t0);
  requestAnimationFrame(loop);
}

/* ================= toast ================= */
var tt = null;
function toast(t){
  var d = $("toast");
  d.textContent = t;
  d.classList.add("show");
  clearTimeout(tt);
  tt = setTimeout(function(){ d.classList.remove("show"); }, 1600);
}

/* ================= 连接页：BLE 广播开关 ================= */
var bleRow = $("bleRow");
if(bleRow) bleRow.addEventListener("click", toggleBle);

/* ================= 设备页：日志开关 ================= */
$("dbgRow").addEventListener("click", function(){
  dbgOn = !dbgOn;
  $("dbgSw").classList.toggle("on", dbgOn);
  var lw = $("logwrap");
  lw.classList.toggle("show", dbgOn);
  if(dbgOn) $("logbox").textContent = logText || "还没收到设备状态…";
  shake(6);
});

/* ================= 设备页：日志暂停 / 复制（v32.33） =================
   日志每秒重写一次，手机上一滑动就被刷掉，没法选中更没法复制。
   这里给两个按钮：暂停刷新（冻住当前内容慢慢看）、复制（抽全文到剪贴板，
   并自动暂停 —— 复制到一半内容变了会很莫名其妙）。 */
$("logPause").addEventListener("click", function(){
  logPaused = !logPaused;
  this.textContent = logPaused ? "继续刷新" : "暂停刷新";
  this.classList.toggle("on", logPaused);
  if(!logPaused) $("logbox").textContent = logText;
});
function copyText(txt, cb){
  if(navigator.clipboard && navigator.clipboard.writeText){
    navigator.clipboard.writeText(txt).then(function(){ cb(true); },
                                            function(){ cb(fallback()); });
  }else{
    cb(fallback());
  }
  function fallback(){
    // 本机是 http://192.168.4.1，不属于安全上下文，navigator.clipboard 多半没有，
    // 只能走老办法：临时 textarea + execCommand（iOS 要先 select 再 setSelectionRange）
    var ta = document.createElement("textarea");
    ta.value = txt;
    ta.setAttribute("readonly", "");
    ta.style.cssText = "position:fixed;top:0;left:0;width:1px;height:1px;opacity:0;";
    document.body.appendChild(ta);
    ta.select();
    try{ ta.setSelectionRange(0, txt.length); }catch(e){}
    var ok = false;
    try{ ok = document.execCommand("copy"); }catch(e){ ok = false; }
    document.body.removeChild(ta);
    return ok;
  }
}
$("logCopy").addEventListener("click", function(){
  if(!dbgOn){
    dbgOn = true;
    $("dbgSw").classList.add("on");
    $("logwrap").classList.add("show");
    $("logbox").textContent = logText || "";
  }
  logPaused = true;                    // 复制期间冻住，免得边复制边变
  var bp = $("logPause");
  bp.textContent = "继续刷新";
  bp.classList.add("on");
  copyText(logText || "", function(ok){
    toast(ok ? "日志已复制（刷新已暂停）"
             : "复制被浏览器拦了，请长按日志框手动选择");
  });
});

/* ================= 使用手册 ================= */
var sheet = $("sheet");
function openHelp(){
  sheet.classList.add("show");
  document.body.classList.add("lock");
  sheet.querySelector(".sbody").scrollTop = 0;
}
function closeHelp(){
  sheet.classList.remove("show");
  document.body.classList.remove("lock");
}
$("helpOpen").addEventListener("click", openHelp);
$("helpOpen2").addEventListener("click", openHelp);
$("helpClose").addEventListener("click", closeHelp);

/* ================= 固件升级：真·POST /api/ota =================
   multipart 字段名必须是 fw（见 src/ota.cpp）。
   进度用 XHR 的上传进度，比 fetch 靠谱（fetch 没有上传进度）。
   ========================================================= */
var OTA_MAX = 1.5 * 1024 * 1024;
var otaFile = null, otaRunning = false;

function fmtSize(b){
  if(b < 1024) return b + " B";
  if(b < 1048576) return (b / 1024).toFixed(1) + " KB";
  return (b / 1048576).toFixed(2) + " MB";
}
function otaErr(msg){
  var e = $("otaerr");
  if(msg){ e.textContent = msg; e.classList.add("show"); }
  else e.classList.remove("show");
}

$("otaOpen").addEventListener("click", function(e){
  e.preventDefault();
  var box = $("otabox");
  var on = !box.classList.contains("show");
  box.classList.toggle("show", on);
  $("otaOpen").classList.toggle("on", on);
  if(on) setTimeout(function(){ box.scrollIntoView({ block:"center", behavior:"smooth" }); }, 60);
});

$("fwFile").addEventListener("change", function(){
  var f = this.files && this.files[0];
  otaErr("");
  if(!f) return;
  if(!/\.bin$/i.test(f.name)){
    otaErr("这不是 .bin 固件。请选 GitHub Actions 构件里的 firmware.bin，不是源码压缩包。");
    return;
  }
  if(f.size > OTA_MAX){
    otaErr("文件 " + fmtSize(f.size) + "，超过 1.5 MB 上限。检查是不是下错了文件。");
    return;
  }
  if(f.size < 100 * 1024){
    otaErr("文件只有 " + fmtSize(f.size) + "，可能不完整。固件正常在 700 KB 以上。");
    return;
  }
  otaFile = f;
  $("fwPick").classList.add("has");
  $("fwLabel").innerHTML = f.name + "<small>" + fmtSize(f.size) + " · 点这里可以重选</small>";
  $("otaGo").disabled = false;
  $("otaGo").textContent = "开始升级";
  $("pfill").style.width = "0%";
  $("pbar").classList.remove("show");
  $("pml").textContent = "准备好了";
  $("pmr").textContent = "";
});

$("otaGo").addEventListener("click", function(){
  if(otaRunning || !otaFile) return;
  otaRunning = true;
  var go = $("otaGo");
  go.disabled = true;
  go.textContent = "上传中…";
  otaErr("");
  $("pbar").classList.add("show");
  $("pfill").style.width = "0%";
  $("pml").textContent = "正在上传";
  $("pmr").textContent = "0%";

  var fd = new FormData();
  fd.append("fw", otaFile);

  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/api/ota", true);
  xhr.upload.onprogress = function(e){
    if(!e.lengthComputable) return;
    var p = clamp(e.loaded / e.total * 100, 0, 100);
    $("pfill").style.width = p.toFixed(1) + "%";
    $("pmr").textContent = Math.round(p) + "%";
    if(p >= 100){
      $("pml").innerHTML = "已上传，<b>设备写入中…</b>";
    }
  };
  xhr.onload = function(){
    otaRunning = false;
    var j = null;
    try{ j = JSON.parse(xhr.responseText); }catch(e){}
    if(j && j.ok){
      $("pfill").style.width = "100%";
      $("pmr").textContent = "100%";
      $("pml").innerHTML = "写入完成，<b>设备重启中…</b>";
      go.textContent = "已完成";
      toast("升级完成，设备正在重启");
      shake([12,60,12,60,12]);
      setTimeout(function(){
        $("pml").textContent = "重启后热点会消失几秒，请重新连接";
        $("pmr").textContent = "";
      }, 2400);
    }else{
      go.textContent = "重试";
      go.disabled = false;
      $("pml").textContent = "升级失败";
      $("pmr").textContent = "";
      otaErr("设备返回：" + ((j && j.msg) || xhr.responseText || "无响应") + "（旧固件仍在，可重试）");
    }
  };
  xhr.onerror = function(){
    /* 上传到一半断线是正常的：设备写完就重启，连接会被掐掉 */
    otaRunning = false;
    $("pfill").style.width = "100%";
    $("pmr").textContent = "100%";
    $("pml").innerHTML = "连接中断，<b>设备可能正在重启</b>";
    go.textContent = "已完成";
    toast("连接中断，请等 10 秒后重连热点");
  };
  xhr.send(fd);
});

/* ================= 初始化 ================= */
buildTabs();
buildModes();
buildColors();
buildChips();

makeRail({
  id:"railBright", min:0, max:100,
  ticks:[0,10,20,30,40,50,60,70,80,90,100],
  get:function(){ return st.bright; },
  set:function(v){ st.bright = v; },
  fmt:function(v){ return v + "%"; }
});

makeRail({
  id:"railSpd", min:1, max:100,
  ticks:[1,25,50,75,100],
  get:function(){ return st.spd; },
  set:function(v){ st.spd = v; },
  fmt:function(v){ return spdLabel(v) + " " + v; }
});

/* 灯带长度轨道：刻度按上限均分。上限以设备报的 maxc 为准 ——
   改 config.h 的 STRIP_MAX_LEDS，页面不用动。 */
function buildCountRail(){
  var tickBox = document.querySelector("#railCount .rticks");
  tickBox.innerHTML = "";          // 重建前先清空，否则旧刻度会叠着留下来
  var t = [1];
  for(var i=1;i<=6;i++){
    var v = Math.round(MAXC * i / 7);
    if(v > 1 && t[t.length-1] !== v) t.push(v);
  }
  if(t[t.length-1] !== MAXC) t.push(MAXC);
  makeRail({
    id:"railCount", min:1, max:MAXC, ticks:t,
    get:function(){ return st.count; },
    set:function(v){ st.count = v; },
    fmt:function(v){ return v + " 颗"; }
  });
  var cms = document.querySelectorAll(".cmax");
  for(var ci=0;ci<cms.length;ci++) cms[ci].textContent = MAXC;
}
var builtMax = 0;
function ensureCountRail(){
  if(builtMax === MAXC) return;
  builtMax = MAXC;
  if(st.count   > MAXC) st.count   = MAXC;
  if(base.count > MAXC) base.count = MAXC;
  buildCountRail();
  buildChips();                    // 长度的快捷档也按新上限重画
}
ensureCountRail();

holdBtn($("cntMinus"), function(){ stepCount(-1); });
holdBtn($("cntPlus"),  function(){ stepCount(+1); });
$("applyBtn").addEventListener("click", doApply);
$("fcOrder").addEventListener("click", cycleFcOrder);   // v32.23：灯色顺序点一下换一个

selectTab("mode");
syncAll();
goFeature(INIT_HASH === "conn" || INIT_HASH === "dev" ? INIT_HASH : "light");
requestAnimationFrame(loop);
pull();                 // 立刻拉一次，别等第一个 1 秒

/* 一级标签栏实测高度 → 预览吸顶位置（不写死，字号或系统字体变了也不会错位） */
function syncTabH(){
  var h = $("ftabs").offsetHeight;
  if(h) document.documentElement.style.setProperty("--ftab-h", h + "px");
}
syncTabH();
window.addEventListener("resize", syncTabH);
window.addEventListener("orientationchange", function(){ setTimeout(syncTabH, 200); });
</script>
</body>
</html>)rawliteral";
