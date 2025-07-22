# パン・チルトAI追跡カメラ基本設計書

## 文書番号: 12-001-DESIGN-001
## 作成日: 2025-07-22
## バージョン: 1.0

---

## 1. 概要

### 1.1 システム概要
YOLOv8による犬猫検出機能と既存のサーボテストプログラムを統合し、検出された対象を自動追跡するパン・チルトカメラシステムを構築する。

### 1.2 設計方針
- 既存の実装済みコンポーネント（`servo_test.py`, `camera_detection_test.py`）を最大限活用
- PIDコントローラーによる安定した追跡制御
- シンプルで保守性の高いアーキテクチャ
- リアルタイム性能の確保

---

## 2. システムアーキテクチャ

### 2.1 システム構成図
```
┌─────────────────────────────────────────────────────────────┐
│                    メインプロセス                            │
│  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────┐ │
│  │  カメラ入力      │  │   YOLO検出      │  │ トラッキング │ │
│  │  モジュール     │→│   モジュール    │→│  制御モジュール│ │
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

#### 2.2.2 カメラ検出モジュール (`CameraDetector`)
- 既存の`camera_detection_test.py`をベースとして活用
- YOLOv8による犬猫検出機能
- バウンディングボックス情報の提供

#### 2.2.3 サーボ制御モジュール (`ServoController`)
- 既存の`servo_test.py`の機能を統合
- Adafruit PCA9685による精密なサーボ制御
- 安全範囲内での動作保証

#### 2.2.4 トラッキング制御モジュール (`TrackingController`)
- PIDコントローラーによる追跡制御
- 対象の中心座標計算
- パン・チルト角度の計算と補正

---

## 3. カメラアングル制御アルゴリズム

### 3.1 制御方式の選定

#### 3.1.1 PIDコントローラーの採用理由
調査結果に基づき、以下の理由からPIDコントローラーを採用：

1. **実績のある制御方式**: 多くのパン・チルト追跡システムで採用
2. **安定性**: オーバーシュートを抑制し滑らかな動作を実現
3. **調整の容易さ**: パラメータ調整により様々な応答特性に対応
4. **リアルタイム性**: 軽量な計算で高速な制御応答

#### 3.1.2 制御アルゴリズムの構成

```python
class PIDController:
    def __init__(self, kP=1.0, kI=0.0, kD=0.0):
        self.kP = kP  # 比例ゲイン
        self.kI = kI  # 積分ゲイン  
        self.kD = kD  # 微分ゲイン
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
    
    def update(self, error):
        current_time = time.time()
        delta_time = current_time - self.prev_time
        
        # P項: 現在の誤差に比例
        proportional = self.kP * error
        
        # I項: 誤差の積分（定常偏差の除去）
        self.integral += error * delta_time
        integral = self.kI * self.integral
        
        # D項: 誤差の微分（オーバーシュート抑制）
        derivative = self.kD * (error - self.prev_error) / delta_time
        
        output = proportional + integral + derivative
        
        self.prev_error = error
        self.prev_time = current_time
        
        return output
```

### 3.2 座標変換と角度計算

#### 3.2.1 画像座標から制御誤差への変換
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

### 3.3 PIDパラメータ調整指針

#### 3.3.1 初期調整手順
1. **P制御のみで開始** (kI=0, kD=0)
   - kPを徐々に増加
   - 振動が始まる直前の値の50%程度に設定

2. **I制御の追加**
   - 定常偏差が残る場合にkIを小さな値から増加
   - 一般的には kI = kP/10 程度から開始

3. **D制御の追加**
   - オーバーシュートが問題となる場合にkDを追加
   - 一般的には kD = kP/100 程度から開始

#### 3.3.2 推奨初期値
```python
# 保守的な初期パラメータ（安定性重視）
PAN_PID_PARAMS = {
    'kP': 0.8,
    'kI': 0.1,
    'kD': 0.05
}

TILT_PID_PARAMS = {
    'kP': 0.8,
    'kI': 0.1,
    'kD': 0.05
}
```

---

## 4. モジュール詳細仕様

### 4.1 ディレクトリ構成
```
modules/
├── __init__.py
├── servo_controller.py      # サーボ制御モジュール
├── yolo_detector.py         # YOLO検出モジュール  
├── pid_controller.py        # PID制御モジュール
└── tracking_coordinator.py  # 全体調整モジュール

tests/
├── __init__.py
├── test_servo_controller.py # サーボ制御テスト
├── test_yolo_detector.py    # YOLO検出テスト
├── test_pid_controller.py   # PID制御テスト
└── integration_test.py      # 統合テスト

main.py                      # メインアプリケーション
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

#### 4.2.4 エラーハンドリング
- 角度範囲外指定時の例外処理
- I2C通信エラーの検出・再試行
- サーボ応答異常の検出

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

### 4.4 PIDController クラス仕様

#### 4.4.1 概要
- **目的**: PID制御による追跡制御の実装
- **責務**: 誤差から補正角度の計算
- **特徴**: パン・チルト独立制御対応

#### 4.4.2 クラス定義
```python
class PIDController:
    def __init__(self, 
                 kP: float = 1.0, 
                 kI: float = 0.0, 
                 kD: float = 0.0,
                 output_limits: tuple = (-90, 90),
                 sample_time: float = 0.01):
        """
        PIDコントローラー初期化
        
        Args:
            kP: 比例ゲイン
            kI: 積分ゲイン
            kD: 微分ゲイン
            output_limits: 出力制限（度）
            sample_time: サンプリング時間（秒）
        """
```

#### 4.4.3 主要メソッド
```python
def update(self, error: float) -> float:
    """
    PID制御更新
    
    Args:
        error: 制御誤差
        
    Returns:
        control_output: 制御出力（角度補正値）
    """
    
def reset(self) -> None:
    """PID内部状態のリセット"""
    
def set_parameters(self, kP: float, kI: float, kD: float) -> None:
    """PIDパラメータの動的変更"""
    
def set_output_limits(self, min_output: float, max_output: float) -> None:
    """出力制限の設定"""
    
def get_components(self) -> dict:
    """P、I、D各成分の取得（デバッグ用）"""
    
def is_stable(self, tolerance: float = 1.0) -> bool:
    """制御安定性の判定"""
```

#### 4.4.4 制御アルゴリズム詳細
```python
# PID制御式の実装
def update(self, error: float) -> float:
    current_time = time.time()
    delta_time = current_time - self.prev_time
    
    # P項: 現在の誤差に比例
    self.proportional = self.kP * error
    
    # I項: 誤差の積分（定常偏差除去）
    self.integral += error * delta_time
    self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
    integral_term = self.kI * self.integral
    
    # D項: 誤差の微分（オーバーシュート抑制）
    if delta_time > 0:
        self.derivative = (error - self.prev_error) / delta_time
    else:
        self.derivative = 0
    derivative_term = self.kD * self.derivative
    
    # 制御出力の計算
    output = self.proportional + integral_term + derivative_term
    output = np.clip(output, self.output_limits[0], self.output_limits[1])
    
    # 状態更新
    self.prev_error = error
    self.prev_time = current_time
    
    return output
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
                 pan_pid: PIDController,
                 tilt_pid: PIDController):
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
制御判定 → PIDController.update() → 角度補正値
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

#### 4.7.3 PIDController テスト
```python
def test_pid_calculation()             # PID計算テスト
def test_parameter_tuning()            # パラメータ調整テスト
def test_output_limits()              # 出力制限テスト
def test_stability_detection()         # 安定性判定テスト
```

---

## 5. 動作モード設計

### 5.1 動作モード一覧

#### 5.1.1 追跡モード (TRACKING)
- **条件**: 犬または猫が検出された場合
- **動作**: 検出対象を画面中央に維持するよう追跡
- **制御**: PIDコントローラーによる連続制御

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
- **動作範囲制限**: Pan ±90°, Tilt ±45°
- **速度制限**: 最大角速度 30°/秒
- **緊急停止**: 異常検出時の安全停止

---

## 7. 実装計画

### 7.1 Phase 1: 基本統合 (1週間)
- [ ] PIDコントローラークラス実装
- [ ] TrackingControllerクラス実装  
- [ ] 既存モジュールの統合
- [ ] 基本的な追跡機能の動作確認

### 7.2 Phase 2: 制御調整 (1週間)  
- [ ] PIDパラメータの調整とチューニング
- [ ] スキャンモードの実装
- [ ] モード遷移ロジックの実装
- [ ] パフォーマンス最適化

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

## 9. リスクと対策

### 9.1 技術的リスク

| リスク | 影響度 | 発生確率 | 対策 |
|--------|--------|----------|------|
| PIDパラメータ調整困難 | 高 | 中 | 段階的調整手順の確立、シミュレーション活用 |
| リアルタイム性能不足 | 高 | 中 | プロファイリング、最適化、解像度調整 |
| サーボ精度不足 | 中 | 低 | キャリブレーション、フィードバック制御 |
| 検出精度不安定 | 中 | 中 | 複数フレーム判定、信頼度フィルタリング |

### 9.2 運用リスク

| リスク | 影響度 | 発生確率 | 対策 |
|--------|--------|----------|------|
| 機械的故障 | 高 | 低 | 定期メンテナンス、予備部品準備 |
| ネットワーク障害 | 中 | 中 | ローカル動作モード、再接続ロジック |
| 電源障害 | 高 | 低 | UPS導入、安全シャットダウン |

---

## 10. 参考文献・調査結果

### 10.1 学術論文
- "Predictive tracking of an object by a pan–tilt camera of a robot" (Nonlinear Dynamics, 2023)
- "Design and implementation of Pan-Tilt control for face tracking" (IEEE, 2017)

### 10.2 実装参考事例
- PyImageSearch: "Pan/tilt face tracking with a Raspberry Pi and OpenCV"
- GitHub: face-track-demo (Raspberry Pi PiCamera, OpenCV Face and Motion Tracking)
- Technology Tutorials: "Using a Pan/Tilt Camera Servo to Track an Object"

### 10.3 技術資料
- Adafruit PCA9685 16-Channel Servo Driver documentation
- YOLOv8 Ultralytics documentation
- OpenCV camera calibration and servo control examples

---

## 11. 変更履歴

| バージョン | 日付 | 変更内容 | 変更者 |
|------------|------|----------|--------|
| 1.0 | 2025-07-22 | 初版作成 | Claude |
| 1.1 | 2025-07-22 | モジュール詳細仕様を追加 | Claude |

---

## 12. 承認

| 役割 | 氏名 | 署名 | 日付 |
|------|------|------|------|
| 作成者 | Claude | - | 2025-07-22 |
| 確認者 | - | - | - |
| 承認者 | - | - | - |

---
*本文書は機密情報を含む可能性があります。関係者以外への開示は禁止されています。*