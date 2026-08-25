# Joy-Con SteamVR Bridge

[English](#english) | [日本語](#日本語)

## English

Experimental Windows prototype for exposing two Nintendo Joy-Cons as tracked
SteamVR controllers.

### Credits

- Created and implemented by **OpenAI Codex**
- Direction, requirements, and hardware testing by **gaemu-jp**

The project combines:

- Joy-Con rotation, buttons, and sticks from Bluetooth HID
- hand positions and slow rotation-drift correction from XR Animator VMC
- a Python UDP bridge
- a native OpenVR server driver that registers left and right controllers

This is an archived prototype, not a finished replacement for a tracked VR
controller. Position quality depends on XR Animator camera tracking, Joy-Con
yaw can drift, and the orientation mapping was tuned experimentally.

> [!WARNING]
> This project installs an experimental third-party driver into SteamVR and
> starts background bridge processes. Review the scripts before running them,
> close SteamVR before installing or replacing the driver, and use this project
> at your own risk. It may cause SteamVR startup failures, controller conflicts,
> tracking errors, or require manual driver removal. It is not affiliated with
> or endorsed by Nintendo, Valve, or XR Animator.

### Data Flow

```text
Joy-Con Bluetooth HID ---- rotation/buttons/sticks ---+
                                                       +--> Python bridge
XR Animator VMC :39771 -- hand position/correction ---+         |
                                                                 v
                                                        UDP :39772
                                                                 |
                                                                 v
                                                        OpenVR driver
                                                                 |
                                                                 v
                                                              SteamVR
```

### Repository Layout

- `work/joycon_vr_bridge.py`: input parsing, pose fusion, calibration, and UDP output
- `work/steamvr_driver/`: native OpenVR driver source and input profile
- `work/setup_joycon_steamvr.ps1`: builds, installs, and registers the driver
- `work/start_joycon_vr.ps1`: starts exactly one fusion bridge process
- `work/set_xr_animator_vmc_port.ps1`: updates an XR Animator config backup safely

The OpenVR SDK, build output, installed driver, generated DLLs, caches, and
runtime logs are intentionally excluded from Git.

### Requirements

- Windows 10 or 11
- Steam and SteamVR
- Python 3.12
- paired left and right Joy-Cons
- Visual Studio C++ Build Tools with CMake and Ninja
- XR Animator configured to send VMC data to UDP port `39771`

Install Python dependencies:

```powershell
python -m pip install -r requirements.txt
```

### Build And Install

The setup script clones Valve's OpenVR SDK when it is missing, builds
`driver_joycon.dll`, copies the driver into SteamVR, and registers it with
`vrpathreg`.

```powershell
powershell -ExecutionPolicy Bypass -File .\work\setup_joycon_steamvr.ps1
```

The script was written for the original machine's default SteamVR location and
Visual Studio 2026 Build Tools layout. Adjust the paths near the top of the
script for a different installation.

Start SteamVR, start XR Animator motion capture/VMC manually, then run:

```powershell
powershell -ExecutionPolicy Bypass -File .\work\start_joycon_vr.ps1
```

### Runtime Ports

- `26760/UDP`: optional BetterJoy DSU fallback
- `39771/UDP`: XR Animator VMC input
- `39772/UDP`: Python bridge to native SteamVR driver

### Controller Recenter

Direct HID starts with an arbitrary physical orientation. Hold each controller
in its intended neutral pose before recentering:

- right Joy-Con: ABXY face up, R edge forward, press `PLUS + right stick click`
- left Joy-Con: button face up, L edge forward, press `MINUS + left stick click`

Recenter also resets the VMC drift-correction reference for that hand.

### Current Limitations

- Camera-derived positions are not equivalent to lighthouse or inside-out controller tracking.
- Joy-Con IMUs have no absolute yaw reference and require correction/recentering.
- The VMC correction is deliberately slow so camera rotation does not replace Joy-Con motion.
- Input bindings use Vive compatibility and may need per-game customization.
- The orientation conversion contains empirical sign corrections from the original hardware tests.

### Privacy And Repository Hygiene

Runtime logs can contain timestamps, controller input, tracking poses, local
paths, and device diagnostics. Do not attach or commit logs without reviewing
them first. The repository ignores logs, generated DLLs, SDK downloads, build
directories, caches, and installed-driver output by default.

### License

Released under the [MIT License](LICENSE). The software is provided as-is,
without warranty. Third-party projects and dependencies remain subject to
their own licenses and trademarks.

---

## 日本語

Nintendo Joy-Con 2台を、トラッキング対応SteamVRコントローラーとして認識させるための
Windows向け実験的プロトタイプです。

### クレジット

- 作成・実装: **OpenAI Codex**
- 指示・要件定義・実機テスト: **gaemu-jp**

このプロジェクトは、次の情報を組み合わせて動作します。

- Bluetooth HIDから取得したJoy-Conの回転、ボタン、スティック
- XR AnimatorのVMCから取得した手の位置と緩やかな回転ドリフト補正
- Python製UDPブリッジ
- 左右のコントローラーを登録するネイティブOpenVRサーバードライバー

これは完成されたVRコントローラーの代替品ではなく、開発を終了した実験的プロトタイプです。
位置精度はXR Animatorのカメラトラッキングに依存し、Joy-Conのヨーはドリフトします。
姿勢変換も実機テストをもとに実験的に調整されています。

> [!WARNING]
> このプロジェクトはSteamVRへ実験的なサードパーティードライバーをインストールし、
> バックグラウンドでブリッジプロセスを起動します。実行前にスクリプトを確認し、
> ドライバーのインストールまたは交換時にはSteamVRを終了してください。
> 使用は自己責任です。SteamVRの起動失敗、コントローラー競合、トラッキング異常が発生し、
> ドライバーの手動削除が必要になる可能性があります。
> Nintendo、Valve、XR Animatorの公式プロジェクトではなく、承認も受けていません。

### データフロー

```text
Joy-Con Bluetooth HID ---- 回転/ボタン/スティック ---+
                                                       +--> Pythonブリッジ
XR Animator VMC :39771 ---- 手の位置/回転補正 --------+         |
                                                                 v
                                                        UDP :39772
                                                                 |
                                                                 v
                                                        OpenVRドライバー
                                                                 |
                                                                 v
                                                              SteamVR
```

### リポジトリ構成

- `work/joycon_vr_bridge.py`: 入力解析、姿勢合成、キャリブレーション、UDP出力
- `work/steamvr_driver/`: ネイティブOpenVRドライバーのソースと入力プロファイル
- `work/setup_joycon_steamvr.ps1`: ドライバーのビルド、インストール、登録
- `work/start_joycon_vr.ps1`: 姿勢合成ブリッジを1プロセスだけ起動
- `work/set_xr_animator_vmc_port.ps1`: バックアップを作成してXR Animator設定を更新

OpenVR SDK、ビルド結果、インストール済みドライバー、生成DLL、キャッシュ、実行ログは
意図的にGitの対象外にしています。

### 必要環境

- Windows 10または11
- SteamおよびSteamVR
- Python 3.12
- ペアリング済みの左右Joy-Con
- CMakeとNinjaを含むVisual Studio C++ Build Tools
- UDPポート`39771`へVMCを送信するよう設定したXR Animator

Python依存パッケージをインストールします。

```powershell
python -m pip install -r requirements.txt
```

### ビルドとインストール

セットアップスクリプトは、Valve OpenVR SDKが存在しない場合に取得し、
`driver_joycon.dll`をビルドしてSteamVRへコピーし、`vrpathreg`で登録します。

```powershell
powershell -ExecutionPolicy Bypass -File .\work\setup_joycon_steamvr.ps1
```

このスクリプトは、開発時のPCにおけるSteamVRの標準パスとVisual Studio 2026 Build Toolsの
構成に合わせて作られています。異なる環境では、スクリプト上部のパスを調整してください。

SteamVRを起動し、XR Animator側でモーションキャプチャとVMC送信を手動で開始してから、
次を実行します。

```powershell
powershell -ExecutionPolicy Bypass -File .\work\start_joycon_vr.ps1
```

### 使用ポート

- `26760/UDP`: BetterJoy DSUの任意フォールバック
- `39771/UDP`: XR Animator VMC入力
- `39772/UDP`: PythonブリッジからネイティブSteamVRドライバーへの出力

### コントローラーの再センタリング

直接HID入力では、起動時の物理姿勢が任意の基準になります。
各コントローラーを意図した正面姿勢で持ってから再センタリングしてください。

- 右Joy-Con: ABXY面を上、R側を前にして`PLUS + 右スティック押し込み`
- 左Joy-Con: ボタン面を上、L側を前にして`MINUS + 左スティック押し込み`

再センタリングすると、その手のVMCドリフト補正基準もリセットされます。

### 現在の制限

- カメラ由来の位置情報は、Lighthouseやインサイドアウト方式のコントローラートラッキングと同等ではありません。
- Joy-ConのIMUには絶対的なヨー基準がなく、補正または再センタリングが必要です。
- カメラ回転がJoy-Conの動きを置き換えないよう、VMC補正は意図的に遅く設定されています。
- 入力バインドはVive互換を使用しており、ゲームごとの調整が必要になる場合があります。
- 姿勢変換には、開発時の実機テストで得た経験的な符号補正が含まれています。

### プライバシーとリポジトリ管理

実行ログには、タイムスタンプ、コントローラー入力、トラッキング姿勢、ローカルパス、
デバイス診断情報が含まれる可能性があります。内容を確認せずログを添付・コミットしないでください。
ログ、生成DLL、SDK取得物、ビルドディレクトリ、キャッシュ、インストール済みドライバー出力は、
標準でGitの対象外になっています。

### ライセンス

[MIT License](LICENSE)で公開しています。本ソフトウェアは無保証で提供されます。
外部プロジェクト、依存パッケージ、商標には、それぞれのライセンスと条件が適用されます。
