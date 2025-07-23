# 関数一覧表

## 文書番号: 12-001-FUNC-001
## 作成日: 2025-01-21
## バージョン: 1.0

---

## 1. メインアプリケーション (PetTrackingApp)

| 関数名 | 機能概要 | 所属クラス | 備考 |
|--------|----------|------------|------|
| `__init__` | アプリケーション初期化 | PetTrackingApp | シグナルハンドラーとログ設定 |
| `setup_logging` | ログ設定初期化 | PetTrackingApp | ファイル・コンソール両方に出力 |
| `setup_signal_handlers` | シグナルハンドラー設定 | PetTrackingApp | SIGINT/SIGTERM対応 |
| `print_welcome_message` | ウェルカムメッセージ表示 | PetTrackingApp | ユーザー向け案内表示 |
| `print_hardware_check` | ハードウェア確認表示 | PetTrackingApp | 接続確認とユーザー応答待機 |
| `create_tracker` | 追跡システム作成 | PetTrackingApp | TrackingCoordinatorインスタンス生成 |
| `run` | アプリケーション実行 | PetTrackingApp | メイン処理ループ |
| `stop` | アプリケーション停止 | PetTrackingApp | リソース解放と終了処理 |
| `_status_monitoring_loop` | ステータス監視ループ | PetTrackingApp | 定期的な状態表示 |
| `_print_system_status` | システム状態表示 | PetTrackingApp | 検出状況・角度情報表示 |

---

## 2. 統合制御モジュール (TrackingCoordinator)

| 関数名 | 機能概要 | 所属クラス | 備考 |
|--------|----------|------------|------|
| `__init__` | 統合制御システム初期化 | TrackingCoordinator | 設定値とスレッド準備 |
| `initialize_system` | システム全体初期化 | TrackingCoordinator | 全モジュールの初期化実行 |
| `start_tracking` | 追跡システム開始 | TrackingCoordinator | メインループスレッド開始 |
| `stop_tracking` | 追跡システム停止 | TrackingCoordinator | スレッド終了とリソース解放 |
| `get_system_status` | システム状態取得 | TrackingCoordinator | 現在の動作状況を辞書で返却 |
| `_main_loop` | メインループ | TrackingCoordinator | フレーム処理と追跡制御の中核 |
| `_process_detection` | 検出処理実行 | TrackingCoordinator | YOLOによるペット検出 |
| `_update_tracking_control` | 追跡制御更新 | TrackingCoordinator | Simple P制御とサーボ制御 |
| `_execute_scan_pattern` | スキャンパターン実行 | TrackingCoordinator | 対象未検出時の探索動作 |
| `_create_display_frame` | 表示フレーム作成 | TrackingCoordinator | 検出結果とUI要素の描画 |
| `_draw_system_info` | システム情報描画 | TrackingCoordinator | 動作状況の画面表示 |
| `_update_system_status` | システム状態更新 | TrackingCoordinator | 統計情報と検出回数更新 |
| `_cleanup_resources` | リソース解放 | TrackingCoordinator | 全モジュールのクリーンアップ |

---

## 3. サーボ制御モジュール (ServoController)

| 関数名 | 機能概要 | 所属クラス | 備考 |
|--------|----------|------------|------|
| `__init__` | サーボ制御器初期化 | ServoController | I2C設定とチャンネル設定 |
| `initialize` | ハードウェア初期化 | ServoController | PCA9685とサーボの初期化 |
| `set_pan_angle` | パン角度設定 | ServoController | 左右方向の角度制御 |
| `set_tilt_angle` | チルト角度設定 | ServoController | 上下方向の角度制御 |
| `set_angles` | パン・チルト同時設定 | ServoController | 両軸の角度を同時制御 |
| `get_current_angles` | 現在角度取得 | ServoController | 現在のパン・チルト角度を返却 |
| `move_to_center` | 中央位置移動 | ServoController | サーボを中央位置(0, 0)に移動 |
| `test_movement` | 動作テスト | ServoController | サーボの可動域テスト |
| `is_angle_safe` | 角度安全性確認 | ServoController | 指定角度が安全範囲内かチェック |
| `emergency_stop` | 緊急停止 | ServoController | サーボを即座に停止 |
| `cleanup` | リソース解放 | ServoController | I2C・PCA9685リソース解放 |
| `_angle_to_pulse_width` | 角度→パルス幅変換 | ServoController | サーボ制御用の内部変換 |
| `_validate_angle` | 角度検証 | ServoController | 角度値の妥当性チェック |

---

## 4. Simple P制御モジュール (SimpleProportionalController)

| 関数名 | 機能概要 | 所属クラス | 備考 |
|--------|----------|------------|------|
| `__init__` | Simple P制御器初期化 | SimpleProportionalController | ゲインと画像サイズ設定 |
| `calculate_correction` | 角度補正値計算 | SimpleProportionalController | Simple P制御の中核処理 |
| `calculate_tracking_error` | 追跡誤差計算 | SimpleProportionalController | バウンディングボックスから誤差算出 |
| `set_gains` | ゲイン値設定 | SimpleProportionalController | パン・チルトゲインの動的変更 |
| `set_max_correction` | 最大補正角度設定 | SimpleProportionalController | 安全のための制限値設定 |
| `set_deadband` | 不感帯設定 | SimpleProportionalController | 微小誤差の無視範囲設定 |
| `get_parameters` | パラメータ取得 | SimpleProportionalController | 現在の制御パラメータ取得 |
| `get_state` | 内部状態取得 | SimpleProportionalController | 制御器の動作状態取得 |
| `get_performance_statistics` | 性能統計取得 | SimpleProportionalController | 制御性能の統計情報取得 |
| `reset` | 制御器リセット | SimpleProportionalController | 内部状態と統計情報初期化 |
| `get_status` | ステータス取得 | SimpleProportionalController | 制御器の動作ステータス取得 |
| `cleanup` | リソース解放 | SimpleProportionalController | 制御器の終了処理 |

---

## 5. YOLO検出モジュール (YOLODetector)

| 関数名 | 機能概要 | 所属クラス | 備考 |
|--------|----------|------------|------|
| `__init__` | YOLO検出器初期化 | YOLODetector | モデルパスと閾値設定 |
| `load_model` | YOLOモデル読み込み | YOLODetector | YOLOv8モデルの初期化 |
| `detect_pets` | ペット検出実行 | YOLODetector | フレームから犬・猫を検出 |
| `get_best_detection` | 最高信頼度検出取得 | YOLODetector | 複数検出から最適なものを選択 |
| `calculate_center` | 中心座標計算 | YOLODetector | バウンディングボックスの中心算出 |
| `calculate_tracking_error` | 追跡誤差計算 | YOLODetector | 画像中心からの偏差計算 |
| `draw_detections` | 検出結果描画 | YOLODetector | フレームに検出結果を描画 |
| `get_detection_statistics` | 検出統計取得 | YOLODetector | 検出性能の統計情報取得 |
| `set_confidence_threshold` | 信頼度閾値設定 | YOLODetector | 検出の信頼度閾値変更 |
| `cleanup` | リソース解放 | YOLODetector | YOLOモデルのリソース解放 |
| `_filter_target_classes` | 対象クラスフィルタ | YOLODetector | 犬・猫のみを抽出 |
| `_format_detection` | 検出結果整形 | YOLODetector | 検出結果を標準形式に変換 |

---

## 6. ファクトリ関数・ユーティリティ

| 関数名 | 機能概要 | 所属クラス | 備考 |
|--------|----------|------------|------|
| `create_simple_p_controller` | Simple P制御器ファクトリ | - (グローバル関数) | 制御器の簡単な生成 |
| `parse_arguments` | コマンドライン引数解析 | - (グローバル関数) | main.pyの引数処理 |
| `main` | メイン関数 | - (グローバル関数) | アプリケーションエントリポイント |

---

## 7. データクラス・列挙型

| クラス名 | 機能概要 | 所属パッケージ | 備考 |
|---------|----------|---------------|------|
| `SystemStatus` | システム状態格納 | tracking_coordinator | 追跡システムの動作状況 |
| `SimplePState` | Simple P制御状態格納 | simple_p_controller | 制御器の内部状態 |
| `Detection` | 検出結果格納 | yolo_detector | YOLO検出結果の標準形式 |
| `TrackingMode` | 追跡モード列挙 | tracking_coordinator | STANDBY/SCANNING/TRACKING |
| `SimplePStatus` | 制御器ステータス列挙 | simple_p_controller | 制御器の動作ステータス |
| `ServoAxis` | サーボ軸列挙 | servo_controller | PAN/TILT軸の識別 |

---

## 8. 例外クラス

| クラス名 | 機能概要 | 所属パッケージ | 備考 |
|---------|----------|---------------|------|
| `ServoControllerError` | サーボ制御例外 | servo_controller | ハードウェア・通信エラー |
| `YOLODetectorError` | YOLO検出例外 | yolo_detector | モデル読み込み・推論エラー |
| `SimplePError` | Simple P制御例外 | simple_p_controller | 制御計算・パラメータエラー |

---

## 統計情報

- **総クラス数**: 11クラス
- **総関数数**: 67関数
- **メイン処理関数**: 13関数
- **内部処理関数**: 54関数
- **例外クラス**: 3クラス
- **データクラス**: 3クラス
- **列挙型**: 3クラス

---

## 変更履歴

| バージョン | 日付 | 変更内容 | 変更者 |
|------------|------|----------|--------|
| 1.0 | 2025-01-21 | 初版作成 | Claude |

---

*本文書はシステムの実装完了時点での関数構成を記録したものです。*