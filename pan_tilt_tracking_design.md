# パン・チルトAI追跡カメラ基本設計書

## 文書番号: 12-001-DESIGN-001
## 作成日: 2025-07-22
## バージョン: 3.0
## 更新内容: Hailo-8L対応デュアル検出システム追加

---

## 1. 概要

### 1.1 システム概要
YOLOv8による犬猫検出機能と既存のサーボテストプログラムを統合し、検出された対象を自動追跡するパン・チルトカメラシステムを構築する。

### 1.2 設計方針
- 既存の実装済みコンポーネント（`servo_test.py`, `camera_detection_test.py`）を最大限活用
- **デュアル検出システム**: CPU版（学習用）とHailo-8L版（実用版）の両対応
- Simple P制御による分かりやすい追跡制御
- 初心者にも理解しやすいシンプルなアーキテクチャ
- **段階的学習**: 基礎理解から実践運用への自然な移行
- 安定性と教育効果を重視した設計

---

## 2. システムアーキテクチャ

### 2.1 システム構成図
```
┌─────────────────────────────────────────────────────────────┐
│                    メインプロセス                            │
│  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────┐ │
│  │  カメラ入力      │  │   YOLO検出      │  │ トラッキング │ │
│  │  モジュール     │→│ (CPU/Hailo選択) │→│  制御モジュール│ │
│  └─────────────────┘  └─────────────────┘  └──────────────┘ │
│                                                ↓             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────┐ │
│  │  Slack通知      │  │   画像保存      │  │ サーボ制御   │ │
│  │  モジュール     │  │   モジュール    │  │  モジュール   │ │
│  └─────────────────┘  └─────────────────┘  └──────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 コンポーネント設計

#### 2.2.1 メインコントローラー (`PanTiltTracker`)
- システム全体の制御と調整
- 各モジュール間のデータフロー管理
- エラーハンドリングとリソース管理

#### 2.2.2 デュアル検出システム

**CPU版検出モジュール (`YOLODetector`)**
- PyTorch/Ultralyticsによる汎用的な推論
- 学習・プロトタイピング・デバッグ用途
- 1-3 FPS での安定動作

**Hailo-8L版検出モジュール (`HailoYOLODetector`)**  
- Raspberry Pi AI Kit による高速推論
- GStreamerパイプラインでのリアルタイム処理
- 30-60 FPS での高性能動作

**検出器ファクトリー (`DetectorFactory`)**
- 環境自動判定による最適検出器選択
- 統一インターフェースでの透明な切り替え
- 性能比較情報の提供

#### 2.2.3 サーボ制御モジュール (`ServoController`)
- 既存の`servo_test.py`の機能を統合
- Adafruit PCA9685による精密なサーボ制御
- 安全範囲内での動作保証

#### 2.2.4 トラッキング制御モジュール (`TrackingController`)
- Simple P制御による分かりやすい追跡制御
- 対象の中心座標計算
- パン・チルト角度の計算と補正

---

## 3. カメラアングル制御アルゴリズム

### 3.1 制御方式の選定

#### 3.1.1 Simple P制御の採用

本ペット見守りシステムでは**Simple P制御（比例制御）を採用**する。

**採用理由：**

**1. 初心者向け教育効果**
- **理解のしやすさ**: 「誤差 × ゲイン = 補正量」というシンプルな計算
- **直感的な動作**: 目標から離れるほど大きく補正する分かりやすい制御
- **少ないパラメータ**: 調整が必要なのは1つのゲイン値のみ
- **デバッグの容易さ**: 動作が予測しやすく問題の特定が簡単

**2. 用途適合性**
- **低頻度動作**: 数分～数十分間隔での位置補正が主目的
- **静止画撮影**: 高速な連続追跡ではなく適切な画角への調整
- **見守り用途**: 複雑な制御よりも確実性と安定性を重視

**3. 技術的優位性**
- **実装の簡潔性**: 約10行のコードで実装可能
- **計算負荷軽減**: Raspberry Piのリソースを効率的に活用
- **安全性**: 予期しない振動や発散が起こりにくい

### 3.2 Simple P制御アルゴリズム

#### 3.2.1 基本設計方針

**制御フロー:**
1. **検出**: YOLOv8でペット（犬猫）を検出
2. **誤差計算**: 検出中心と画像中心の偏差を算出
3. **比例制御**: 誤差に比例ゲイン(Kp)を乗算
4. **角度更新**: サーボに角度指令を送信
5. **撮影**: 適切な画角で静止画を撮影

**制御周期:**
- **メイン周期**: 60秒～1800秒（1分～30分）の可変設定
- **サーボ応答**: 即座に実行（応答性重視）
- **安定化待機**: 角度変更後2-3秒の安定化時間

#### 3.2.2 制御アルゴリズムの実装

```python
class SimpleProportionalController:
    """一般的なSimple P制御器"""
    
    def __init__(self, 
                 image_width: int = 640,
                 image_height: int = 480,
                 pan_gain: float = 0.0156,    # 10.0/640 (一般的な推奨値)
                 tilt_gain: float = 0.0208):  # 10.0/480 (一般的な推奨値)
        
        self.image_center = (image_width // 2, image_height // 2)
        self.pan_gain = pan_gain    # Kp_pan
        self.tilt_gain = tilt_gain  # Kp_tilt
    
    def calculate_correction(self, detection_center: tuple) -> tuple:
        """検出中心から角度補正値を計算"""
        # 誤差計算（画像座標系）
        x_error = detection_center[0] - self.image_center[0]
        y_error = detection_center[1] - self.image_center[1]
        
        # Simple P制御（比例制御）
        pan_correction = x_error * self.pan_gain
        tilt_correction = -y_error * self.tilt_gain  # Y軸反転
        
        return (pan_correction, tilt_correction)
```

#### 3.2.3 パラメータ設定指針

**比例ゲイン(Kp)の設定:**

一般的なパン・チルトシステムでの推奨設定：
```python
# 基本設定（一般的な推奨値）
Kp_pan = 10.0 / 640 ≈ 0.0156    # パン制御ゲイン
Kp_tilt = 10.0 / 480 ≈ 0.0208   # チルト制御ゲイン

# 制御特性
画面端（320pixel誤差）→ 約5度補正
画面1/4（160pixel誤差）→ 約2.5度補正
```

**調整方針:**
- **応答性重視**: Kp値を1.5～2倍に増加
- **安定性重視**: Kp値を0.5～0.7倍に減少
- **微調整**: ±20%程度の範囲で調整

#### 3.2.4 実装上の考慮事項

**角度制限:**
```python
# サーボ保護のための制限
pan_limits = (-90, 90)      # パン可動範囲
tilt_limits = (-45, 45)     # チルト可動範囲
max_correction = 15         # 1回の最大補正角度
```

**AdafruitサーボKit連携:**
```python
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
kit.servo[0].angle = current_pan + pan_correction    # Pan制御
kit.servo[1].angle = current_tilt + tilt_correction  # Tilt制御
```

### 3.3 座標変換と角度計算

#### 3.3.1 画像座標から制御誤差への変換
```python
def calculate_tracking_error(detection_bbox, image_center):
    """
    検出結果から制御誤差を計算
    
    Args:
        detection_bbox: (x1, y1, x2, y2) バウンディングボックス
        image_center: (cx, cy) 画像中心座標
    
    Returns:
        (pan_error, tilt_error): パン・チルト制御誤差
    """
    # バウンディングボックスの中心を計算
    bbox_center_x = (detection_bbox[0] + detection_bbox[2]) / 2
    bbox_center_y = (detection_bbox[1] + detection_bbox[3]) / 2
    
    # 画像中心からのずれを計算（制御誤差）
    pan_error = bbox_center_x - image_center[0]   # X軸方向の誤差
    tilt_error = bbox_center_y - image_center[1]  # Y軸方向の誤差
    
    return pan_error, tilt_error
```

#### 3.2.2 制御出力から角度への変換
```python
def control_to_angle(control_output, max_angle=90):
    """
    PID制御出力を角度に変換
    
    Args:
        control_output: PID制御器の出力
        max_angle: 最大角度（度）
    
    Returns:
        angle: サーボ角度（-max_angle ~ +max_angle）
    """
    # 制御出力を角度範囲にマッピング
    angle = np.clip(control_output, -max_angle, max_angle)
    return angle
```

### 3.4 制御の特徴と制限

#### 3.4.1 Simple P制御の特徴

**利点:**
- **分かりやすさ**: 比例関係の単純な計算
- **安定性**: 発散しにくく安全
- **調整の簡単さ**: ゲイン1つだけの調整
- **学習効果**: 制御の基本原理が理解できる

**制限:**
- **定常偏差**: 完全に中央に合わせることが困難な場合がある
- **応答特性**: 微調整には時間がかかる
- **外乱対応**: 風などの外乱には弱い

#### 3.4.2 ペット見守りでの適用

本システムでは以下の理由でSimple P制御が最適：

- **目的**: 完璧な追跡よりも「だいたい画面に入る」ことが重要
- **頻度**: 数分おきの調整で十分
- **学習**: 制御の基本を理解する教育効果
- **保守**: トラブル時の原因特定が容易

---

## 4. モジュール詳細仕様

### 4.1 ディレクトリ構成
```
modules/
├── __init__.py
├── servo_controller.py         # サーボ制御モジュール
├── yolo_detector.py            # CPU版YOLO検出モジュール
├── yolo_detector_hailo.py      # Hailo-8L版YOLO検出モジュール
├── detector_factory.py         # 検出器ファクトリー
├── simple_p_controller.py      # Simple P制御モジュール
└── tracking_coordinator.py     # 全体調整モジュール

tests/
├── __init__.py
├── test_servo_controller.py    # サーボ制御テスト
├── test_yolo_detector.py       # CPU版YOLO検出テスト
├── test_detector_factory.py    # 検出器ファクトリーテスト
├── test_simple_p_controller.py # Simple P制御テスト
├── test_tracking_coordinator.py # 統合制御テスト
└── integration_test.py         # 統合テスト

main.py                         # メインアプリケーション
example_hailo_usage.py          # Hailo-8L使用例・性能比較デモ
```

### 4.2 ServoController クラス仕様

#### 4.2.1 概要
- **目的**: Adafruit PCA9685を使用したサーボ制御の抽象化
- **依存**: 既存の`servo_test.py`の機能を活用・拡張
- **責務**: パン・チルトサーボの精密制御と安全管理

#### 4.2.2 クラス定義
```python
class ServoController:
    def __init__(self, 
                 i2c_address: int = 0x40,
                 pwm_frequency: int = 50,
                 pan_channel: int = 0,
                 tilt_channel: int = 1,
                 pan_range: tuple = (-90, 90),
                 tilt_range: tuple = (-45, 45)):
        """
        サーボコントローラー初期化
        
        Args:
            i2c_address: PCA9685のI2Cアドレス
            pwm_frequency: PWM周波数（Hz）
            pan_channel: パンサーボのチャンネル番号
            tilt_channel: チルトサーボのチャンネル番号
            pan_range: パン動作範囲（度）
            tilt_range: チルト動作範囲（度）
        """
```

#### 4.2.3 主要メソッド
```python
def initialize(self) -> bool:
    """サーボドライバの初期化"""
    
def set_pan_angle(self, angle: float) -> bool:
    """パン角度設定"""
    
def set_tilt_angle(self, angle: float) -> bool:
    """チルト角度設定"""
    
def set_angles(self, pan_angle: float, tilt_angle: float) -> bool:
    """パン・チルト同時設定"""
    
def get_current_angles(self) -> tuple[float, float]:
    """現在の角度取得 (pan, tilt)"""
    
def move_to_center(self) -> bool:
    """中央位置への移動"""
    
def test_movement(self) -> bool:
    """動作テスト（既存servo_test.pyの機能）"""
    
def is_angle_safe(self, pan: float, tilt: float) -> bool:
    """角度の安全性チェック"""
    
def emergency_stop(self) -> None:
    """緊急停止"""
    
def cleanup(self) -> None:
    """リソース解放"""
```

#### 4.2.4 サーボ安全範囲とパラメータ設定

**SG90サーボモーター仕様:**
- **物理可動域**: 0° - 180°（理論値）
- **推奨パルス幅**: 500μs - 2400μs
- **PWM周波数**: 50Hz

**安全範囲設定の考慮事項:**

| 用途 | パン範囲 | チルト範囲 | 目的 |
|------|----------|------------|------|
| **テスト用** | 0° - 180° | 0° - 180° | サーボ性能確認・パラメータ検証 |
| **実用設定** | -90° - 90° | 30° - 150° | 機械的安全性・ケーブル保護 |
| **保守設定** | -45° - 45° | 45° - 135° | 初心者向け・最大安全性 |

**パラメータ設定例:**

```python
# SG90最適化設定（テスト用）
servo.Servo(
    pca.channels[0], 
    min_pulse=500,      # SG90推奨最小パルス
    max_pulse=2400,     # SG90推奨最大パルス
    actuation_range=180 # フル可動域
)

# 実用設定（安全重視）
servo.Servo(
    pca.channels[0], 
    min_pulse=600,      # 保守的最小パルス
    max_pulse=2300,     # 保守的最大パルス
    actuation_range=160 # 安全マージン付き
)
```

**設定変更時の注意事項:**
1. **段階的テスト**: 小範囲→中範囲→フル範囲の順でテスト
2. **物理的確認**: カメラマウント、ケーブル、周囲障害物の確認
3. **機械的制限**: 実際のハードウェア構成に合わせた制限値設定
4. **安全復帰**: テスト後は実用安全範囲への設定復帰

#### 4.2.5 エラーハンドリング
- 角度範囲外指定時の例外処理
- I2C通信エラーの検出・再試行
- サーボ応答異常の検出
- 安全範囲外動作の防止

### 4.3 YOLODetector クラス仕様

#### 4.3.1 概要
- **目的**: YOLOv8による犬猫検出機能の提供
- **依存**: 既存の`camera_detection_test.py`の機能を活用・拡張
- **責務**: リアルタイム検出と座標情報提供

#### 4.3.2 クラス定義
```python
class YOLODetector:
    def __init__(self, 
                 model_path: str = "yolov8n.pt",
                 confidence_threshold: float = 0.5,
                 target_classes: list = [15, 16],  # cat, dog
                 image_size: tuple = (640, 480)):
        """
        YOLO検出器初期化
        
        Args:
            model_path: YOLOv8モデルファイルパス
            confidence_threshold: 信頼度閾値
            target_classes: 対象クラスID（cat=15, dog=16）
            image_size: 処理画像サイズ
        """
```

#### 4.3.3 主要メソッド
```python
def load_model(self) -> bool:
    """YOLOv8モデルの読み込み"""
    
def detect_pets(self, frame: np.ndarray) -> list[dict]:
    """
    犬猫検出実行
    
    Returns:
        [{'class_id': int, 'confidence': float, 'bbox': (x1,y1,x2,y2), 'class_name': str}]
    """
    
def get_best_detection(self, detections: list) -> dict:
    """最も信頼度の高い検出結果を取得"""
    
def calculate_center(self, bbox: tuple) -> tuple[float, float]:
    """バウンディングボックスの中心座標計算"""
    
def calculate_tracking_error(self, detection: dict, image_center: tuple) -> tuple[float, float]:
    """画像中心からの誤差計算"""
    
def draw_detections(self, frame: np.ndarray, detections: list) -> np.ndarray:
    """検出結果の描画（デバッグ用）"""
    
def get_detection_statistics(self) -> dict:
    """検出統計情報取得"""
    
def set_confidence_threshold(self, threshold: float) -> None:
    """信頼度閾値の動的変更"""
```

#### 4.3.4 戻り値仕様
```python
# detect_pets()の戻り値例
[
    {
        'class_id': 16,
        'confidence': 0.87,
        'bbox': (150, 120, 350, 280),
        'class_name': 'Dog',
        'center': (250, 200)
    }
]
```

### 4.4 SimpleProportionalController クラス仕様

#### 4.4.1 概要
- **目的**: Simple P制御による分かりやすい追跡制御の実装
- **責務**: 誤差から補正角度の直接計算
- **特徴**: 初心者にも理解しやすいシンプルな実装

#### 4.4.2 クラス定義
```python
class SimpleProportionalController:
    def __init__(self, 
                 image_width: int = 640,
                 image_height: int = 480,
                 pan_gain: float = 0.0156,
                 tilt_gain: float = 0.0208,
                 max_correction: float = 15.0,
                 deadband: float = 5.0):
        """
        Simple P制御器初期化
        
        Args:
            image_width: 画像幅（pixel）
            image_height: 画像高さ（pixel）
            pan_gain: パン制御ゲイン
            tilt_gain: チルト制御ゲイン
            max_correction: 1回の最大補正角度（度）
            deadband: 不感帯（pixel）
        """
```

#### 4.4.3 主要メソッド
```python
def calculate_correction(self, detection_center: tuple) -> tuple:
    """
    検出中心から角度補正値を計算
    
    Args:
        detection_center: 検出対象の中心座標 (x, y)
        
    Returns:
        tuple: (pan_correction, tilt_correction) 角度補正値（度）
    """
    
def calculate_tracking_error(self, detection_bbox: tuple) -> tuple:
    """バウンディングボックスから制御誤差を計算"""
    
def set_gains(self, pan_gain: float, tilt_gain: float) -> None:
    """制御ゲインの動的変更"""
    
def get_parameters(self) -> dict:
    """制御パラメータの取得"""
    
def reset(self) -> None:
    """制御器状態のリセット"""
```

#### 4.4.4 制御アルゴリズム詳細
```python
# Simple P制御の実装（とてもシンプル！）
def calculate_correction(self, detection_center):
    # 1. 誤差を計算
    x_error = detection_center[0] - self.image_center[0]
    y_error = detection_center[1] - self.image_center[1]
    
    # 2. 比例制御で補正量を計算
    pan_correction = x_error * self.pan_gain
    tilt_correction = -y_error * self.tilt_gain  # Y軸反転
    
    # 3. 安全のため補正量を制限
    pan_correction = np.clip(pan_correction, -15, 15)
    tilt_correction = np.clip(tilt_correction, -15, 15)
    
    return (pan_correction, tilt_correction)
```

### 4.5 TrackingCoordinator クラス仕様

#### 4.5.1 概要
- **目的**: 各モジュールの統合制御
- **責務**: 動作モード管理、フロー制御、エラーハンドリング

#### 4.5.2 クラス定義
```python
class TrackingCoordinator:
    def __init__(self,
                 servo_controller: ServoController,
                 yolo_detector: YOLODetector,
                 simple_p_controller: SimpleProportionalController):
        """統合制御クラス初期化"""
```

#### 4.5.3 主要メソッド
```python
def initialize_system(self) -> bool:
    """システム全体の初期化"""
    
def start_tracking(self) -> None:
    """追跡モードの開始"""
    
def stop_tracking(self) -> None:
    """追跡モードの停止"""
    
def process_frame(self, frame: np.ndarray) -> dict:
    """1フレームの処理実行"""
    
def update_tracking_mode(self, detections: list) -> None:
    """動作モードの更新"""
    
def execute_scan_pattern(self) -> None:
    """スキャンパターンの実行"""
    
def handle_emergency(self, error_type: str) -> None:
    """緊急時処理"""
    
def get_system_status(self) -> dict:
    """システム状態の取得"""
```

### 4.6 モジュール間インターフェース

#### 4.6.1 データフロー
```
カメラフレーム → YOLODetector.detect_pets() → 検出結果
                     ↓
検出結果 → TrackingCoordinator.process_frame() → 制御判定
                     ↓
制御判定 → SimpleProportionalController.calculate_correction() → 角度補正値
                     ↓
角度補正値 → ServoController.set_angles() → サーボ制御
```

#### 4.6.2 エラー伝播
- 各モジュールは独立してエラーハンドリング
- TrackingCoordinatorが全体のエラー状態を監視
- 致命的エラー時は安全停止モードに移行

### 4.7 単体テスト仕様

#### 4.7.1 ServoController テスト
```python
def test_servo_initialization()          # 初期化テスト
def test_angle_setting()                # 角度設定テスト
def test_safety_limits()               # 安全制限テスト
def test_emergency_stop()              # 緊急停止テスト
```

#### 4.7.2 YOLODetector テスト
```python
def test_model_loading()               # モデル読み込みテスト
def test_pet_detection()               # 検出機能テスト
def test_confidence_filtering()        # 信頼度フィルタリングテスト
def test_coordinate_calculation()      # 座標計算テスト
```

#### 4.7.3 SimpleProportionalController テスト
```python
def test_simple_p_calculation()       # Simple P計算テスト
def test_gain_adjustment()             # ゲイン調整テスト
def test_correction_limits()           # 補正制限テスト
def test_deadband_functionality()      # 不感帯機能テスト
```

---

## 5. 動作モード設計

### 5.1 動作モード一覧

#### 5.1.1 追跡モード (TRACKING)
- **条件**: 犬または猫が検出された場合
- **動作**: 検出対象を画面中央に維持するよう追跡
- **制御**: Simple P制御による分かりやすい制御

#### 5.1.2 スキャンモード (SCANNING)
- **条件**: 検出対象が見つからない場合
- **動作**: 予定されたパターンでランダムまたは順次スキャン
- **制御**: 事前定義された角度シーケンス

#### 5.1.3 待機モード (STANDBY)
- **条件**: システム起動時、エラー時
- **動作**: 中央位置で待機
- **制御**: 固定角度（Pan=0°, Tilt=0°）

### 5.2 モード遷移図
```
    [待機モード]
         ↓ システム起動
    [スキャンモード] ←→ [追跡モード]
         ↑                    ↓
         └── エラー発生 ────┘
         
モード遷移条件:
- スキャン → 追跡: 対象検出時
- 追跡 → スキャン: 対象ロスト時（3秒間未検出）
- 任意 → 待機: エラー発生時
```

---

## 6. パフォーマンス要件

### 6.1 リアルタイム性能
- **フレーム処理速度**: 20fps以上
- **制御ループ周期**: 50ms以下
- **追跡応答時間**: 200ms以下

### 6.2 精度要件
- **追跡精度**: 画像中心±20pixel以内
- **角度精度**: ±2度以内
- **安定性**: 静止対象で振動幅±1度以内

### 6.3 安全要件

#### 6.3.1 動作範囲制限
**運用環境別の推奨設定:**

| 環境 | パン範囲 | チルト範囲 | 適用場面 |
|------|----------|------------|----------|
| **テスト環境** | 0° - 180° | 0° - 180° | サーボ性能確認、パラメータ調整時 |
| **開発環境** | -90° - 90° | 30° - 150° | 開発・デバッグ時の実用設定 |
| **本番環境** | -45° - 45° | 45° - 135° | 実際の運用時の安全設定 |

#### 6.3.2 その他安全要件
- **速度制限**: 最大角速度 30°/秒
- **緊急停止**: 異常検出時の安全停止
- **段階的テスト**: 可動域拡張時の段階的確認
- **物理的安全確認**: カメラマウント・ケーブル干渉チェック

---

## 7. 実装計画

### 7.1 Phase 1: 基本統合 (1週間)
- [x] Simple P制御コントローラークラス実装
- [ ] TrackingControllerクラス実装  
- [ ] 既存モジュールの統合
- [ ] 基本的な追跡機能の動作確認

### 7.2 Phase 2: 制御調整 (1週間)  
- [ ] Simple P制御ゲインの調整とチューニング
- [ ] スキャンモードの実装
- [ ] モード遷移ロジックの実装
- [ ] 動作の安定化

### 7.3 Phase 3: 機能拡張 (1週間)
- [ ] Slack通知機能の統合
- [ ] 画像保存機能の統合  
- [ ] エラーハンドリングの強化
- [ ] ログ機能の実装

### 7.4 Phase 4: テスト・調整 (1週間)
- [ ] 総合テストの実施
- [ ] パフォーマンス測定と調整
- [ ] ドキュメント整備
- [ ] 最終検収

---

## 8. 技術的制約・前提条件

### 8.1 ハードウェア制約
- Raspberry Pi 5の処理能力限界
- SG90サーボの応答速度・精度限界
- カメラモジュールv3の性能特性

### 8.2 ソフトウェア制約
- Python GIL によるマルチスレッド制限
- OpenCV の処理遅延
- YOLOv8 の推論時間

### 8.3 環境制約
- 照明条件による検出精度の変動
- 対象サイズによる検出性能の差
- 背景の複雑さによる誤検出

---

## 9. 依存関係・ライブラリ仕様

### 9.1 Adafruit Servo HAT ライブラリ要件

#### 9.1.1 対象ハードウェア
- **製品名**: Adafruit 16-Channel PWM/Servo HAT
- **SKU**: 2327
- **チップ**: PCA9685 PWMコントローラー
- **URL**: https://www.adafruit.com/product/2327

#### 9.1.2 推奨ライブラリ（2025年現在）

**選択肢1: PCA9685 + Motor（現在採用）**
```bash
# インストール
pip3 install adafruit-circuitpython-pca9685
pip3 install adafruit-circuitpython-motor

# 使用例
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
```

**選択肢2: ServoKit（シンプル版）**
```bash
# インストール
pip3 install adafruit-circuitpython-servokit

# 使用例
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
kit.servo[0].angle = 90
```

#### 9.1.3 ライブラリ選定理由

**PCA9685 + Motor を採用**
- ✅ **教育効果**: 制御の詳細が理解できる
- ✅ **拡張性**: PID制御等の高度な機能への発展が容易
- ✅ **デバッグ性**: トラブルシューティングが簡単
- ✅ **カスタマイズ性**: 細かい制御パラメータの調整が可能

**ServoKit との比較**
| 項目 | PCA9685 + Motor | ServoKit |
|------|-----------------|----------|
| 学習効果 | 高（制御理論の理解） | 低（ブラックボックス） |
| 実装複雑度 | 中（適度な複雑さ） | 低（1行でサーボ制御） |
| カスタマイズ性 | 高（詳細制御可能） | 低（固定API） |
| デバッグ性 | 高（段階的確認可能） | 低（内部処理が不明） |

#### 9.1.4 非推奨ライブラリ

**Adafruit_Python_PCA9685（廃止予定）**
- ❌ **ステータス**: アーカイブ済み・非推奨
- ❌ **理由**: CircuitPython移行により保守終了
- ❌ **URL**: https://github.com/adafruit/Adafruit_Python_PCA9685

### 9.2 その他の主要依存関係

#### 9.2.1 AI・画像処理ライブラリ
```bash
# YOLOv8 物体検出
pip3 install ultralytics

# OpenCV 画像処理
pip3 install opencv-python

# NumPy 数値計算
pip3 install numpy
```

#### 9.2.2 通信・設定管理ライブラリ
```bash
# Slack通知
pip3 install requests

# 環境変数管理
pip3 install python-dotenv
```

#### 9.2.3 システム依存ライブラリ
```bash
# CircuitPython基盤（Raspberry Pi専用）
pip3 install adafruit-blinka
```

### 9.3 バージョン管理・互換性

#### 9.3.1 推奨バージョン
- **Python**: 3.9以上
- **adafruit-circuitpython-pca9685**: 最新版
- **adafruit-circuitpython-motor**: 最新版
- **ultralytics**: 最新版

#### 9.3.2 互換性マトリックス
| OS | Python Ver | PCA9685 Lib | 動作確認 |
|----|------------|-------------|----------|
| Raspberry Pi OS | 3.9+ | 3.x | ✅ |
| Ubuntu 22.04 | 3.10+ | 3.x | ✅ |
| Windows 10/11 | 3.9+ | 3.x | ❌ (I2C未サポート) |

### 9.4 インストール手順

#### 9.4.1 仮想環境での推奨インストール
```bash
# 仮想環境作成
python3 -m venv venv
source venv/bin/activate

# 基本ライブラリ
pip install adafruit-circuitpython-pca9685
pip install adafruit-circuitpython-motor
pip install ultralytics
pip install opencv-python
pip install requests
pip install python-dotenv

# システム設定（Raspberry Pi）
sudo raspi-config  # I2C有効化
```

#### 9.4.2 動作確認コマンド
```bash
# ライブラリ確認
python3 -c "import adafruit_pca9685; print('PCA9685 OK')"
python3 -c "import adafruit_motor; print('Motor OK')"

# ハードウェア確認
sudo i2cdetect -y 1  # I2C デバイス確認
```

---

## 10. リスクと対策

### 10.1 技術的リスク

| リスク | 影響度 | 発生確率 | 対策 |
|--------|--------|----------|------|
| ゲインパラメータ調整困難 | 低 | 中 | 段階的調整手順の確立、テスト環境での確認 |
| リアルタイム性能不足 | 高 | 中 | プロファイリング、最適化、解像度調整 |
| サーボ精度不足 | 中 | 低 | キャリブレーション、フィードバック制御 |
| 検出精度不安定 | 中 | 中 | 複数フレーム判定、信頼度フィルタリング |

### 10.2 運用リスク

| リスク | 影響度 | 発生確率 | 対策 |
|--------|--------|----------|------|
| 機械的故障 | 高 | 低 | 定期メンテナンス、予備部品準備 |
| ネットワーク障害 | 中 | 中 | ローカル動作モード、再接続ロジック |
| 電源障害 | 高 | 低 | UPS導入、安全シャットダウン |

---

## 11. 参考文献・調査結果

### 11.1 学術論文
- "Predictive tracking of an object by a pan–tilt camera of a robot" (Nonlinear Dynamics, 2023)
- "Design and implementation of Pan-Tilt control for face tracking" (IEEE, 2017)

### 11.2 実装参考事例
- PyImageSearch: "Pan/tilt face tracking with a Raspberry Pi and OpenCV"
- GitHub: face-track-demo (Raspberry Pi PiCamera, OpenCV Face and Motion Tracking)
- Technology Tutorials: "Using a Pan/Tilt Camera Servo to Track an Object"

### 11.3 技術資料
- Adafruit PCA9685 16-Channel Servo Driver documentation
- YOLOv8 Ultralytics documentation
- OpenCV camera calibration and servo control examples

---

## 12. 変更履歴

| バージョン | 日付 | 変更内容 | 変更者 |
|------------|------|----------|--------|
| 1.0 | 2025-07-22 | 初版作成 | Claude |
| 1.1 | 2025-07-22 | モジュール詳細仕様を追加 | Claude |
| 3.0 | 2025-01-21 | Hailo-8L対応デュアル検出システム追加、性能比較表作成 | Claude |
| 3.1 | 2025-07-26 | 依存関係・ライブラリ仕様セクション追加、Adafruit Servo HATライブラリ調査結果反映 | Claude |
| 3.2 | 2025-07-26 | サーボ安全範囲とパラメータ設定仕様追加、運用環境別設定ガイド追加 | Claude |

---

## 13. 承認

| 役割 | 氏名 | 署名 | 日付 |
|------|------|------|------|
| 作成者 | Claude | - | 2025-07-22 |
| 確認者 | - | - | - |
| 承認者 | - | - | - |

---
*本文書は機密情報を含む可能性があります。関係者以外への開示は禁止されています。*