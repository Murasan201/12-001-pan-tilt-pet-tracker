#!/usr/bin/env python3
"""
追跡統合制御モジュール - 全モジュールを統合してペット追跡システムを実現

このモジュールは各個別モジュールを統合し、実際に動作するペット追跡システムを構築します。
初心者にも理解しやすいよう、シンプルで分かりやすい実装を心がけています。

主な機能:
- カメラからの映像取得と表示
- YOLOv8による犬猫の自動検出
- Simple P制御による自動追跡
- サーボモータでのカメラ向き調整
- 検出状況のリアルタイム表示

動作モード:
- STANDBY: 待機モード（システム起動時）
- SCANNING: スキャンモード（対象を探している状態）
- TRACKING: 追跡モード（対象を追跡している状態）

教育効果:
各モジュールの動作が分かりやすく表示されるため、
制御システムの動作原理を学習することができます。
"""

import cv2
import time
import logging
import numpy as np
from typing import Optional, Dict, Tuple, List
from dataclasses import dataclass
from enum import Enum
import threading
from datetime import datetime

# プロジェクトモジュールのインポート
from .servo_controller import ServoController
from .yolo_detector import YOLODetector
from .simple_p_controller import SimpleProportionalController


class TrackingMode(Enum):
    """追跡システムの動作モード"""
    STANDBY = "standby"      # 待機モード
    SCANNING = "scanning"    # スキャンモード  
    TRACKING = "tracking"    # 追跡モード


@dataclass
class SystemStatus:
    """システム状態の情報"""
    mode: TrackingMode = TrackingMode.STANDBY
    target_detected: bool = False
    target_class: str = ""
    target_confidence: float = 0.0
    pan_angle: float = 0.0
    tilt_angle: float = 0.0
    correction_applied: Tuple[float, float] = (0.0, 0.0)
    last_detection_time: Optional[datetime] = None
    total_detections: int = 0
    tracking_duration: float = 0.0


class TrackingCoordinator:
    """
    追跡統合制御クラス
    
    各モジュールを統合してペット追跡システムを実現します。
    初心者にも分かりやすいよう、動作状況を詳しく表示します。
    """
    
    def __init__(self, 
                 camera_id: int = 0,
                 image_width: int = 640,
                 image_height: int = 480,
                 detection_interval: float = 0.5,
                 lost_target_timeout: float = 5.0,
                 show_display: bool = True):
        """
        追跡統合制御システム初期化
        
        Args:
            camera_id: カメラID（通常は0）
            image_width: 処理する画像の幅
            image_height: 処理する画像の高さ
            detection_interval: 検出処理の間隔（秒）
            lost_target_timeout: 対象ロスト判定時間（秒）
            show_display: 画面表示の有無
        """
        # 基本設定
        self.camera_id = camera_id
        self.image_width = image_width
        self.image_height = image_height
        self.detection_interval = detection_interval
        self.lost_target_timeout = lost_target_timeout
        self.show_display = show_display
        
        # システム状態
        self.status = SystemStatus()
        self.is_running = False
        self.system_start_time = None
        
        # カメラ設定
        self.camera = None
        
        # モジュール初期化（後で実際に初期化）
        self.servo_controller = None
        self.yolo_detector = None
        self.simple_p_controller = None
        
        # スレッド制御
        self.main_thread = None
        self.lock = threading.Lock()
        
        # ログ設定
        self.logger = logging.getLogger(__name__)
        self.logger.info("追跡統合制御システムを初期化しました")
    
    def initialize_system(self) -> bool:
        """
        システム全体の初期化
        
        Returns:
            bool: 初期化成功時True
        """
        try:
            self.logger.info("=== ペット追跡システム初期化開始 ===")
            
            # 1. カメラの初期化
            self.logger.info("1. カメラを初期化中...")
            self.camera = cv2.VideoCapture(self.camera_id)
            if not self.camera.isOpened():
                self.logger.error("カメラの初期化に失敗しました")
                return False
            
            # カメラ設定
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            
            self.logger.info(f"✓ カメラ初期化完了 ({self.image_width}x{self.image_height})")
            
            # 2. サーボ制御器の初期化
            self.logger.info("2. サーボ制御器を初期化中...")
            self.servo_controller = ServoController()
            if not self.servo_controller.initialize():
                self.logger.error("サーボ制御器の初期化に失敗しました")
                return False
            
            # 中央位置に移動
            self.servo_controller.move_to_center()
            self.status.pan_angle = 0.0
            self.status.tilt_angle = 0.0
            self.logger.info("✓ サーボ制御器初期化完了")
            
            # 3. YOLO検出器の初期化
            self.logger.info("3. YOLO検出器を初期化中...")
            self.yolo_detector = YOLODetector()
            if not self.yolo_detector.load_model():
                self.logger.error("YOLO検出器の初期化に失敗しました")
                return False
            self.logger.info("✓ YOLO検出器初期化完了")
            
            # 4. Simple P制御器の初期化
            self.logger.info("4. Simple P制御器を初期化中...")
            self.simple_p_controller = SimpleProportionalController(
                image_width=self.image_width,
                image_height=self.image_height
            )
            self.logger.info("✓ Simple P制御器初期化完了")
            
            # システム開始時刻記録
            self.system_start_time = datetime.now()
            self.status.mode = TrackingMode.SCANNING
            
            self.logger.info("=== システム初期化完了 ===")
            return True
            
        except Exception as e:
            self.logger.error(f"システム初期化中にエラー: {e}")
            return False
    
    def start_tracking(self) -> None:
        """追跡システム開始"""
        if self.is_running:
            self.logger.warning("システムは既に動作中です")
            return
        
        if not self.initialize_system():
            self.logger.error("システム初期化に失敗したため、追跡を開始できません")
            return
        
        self.is_running = True
        self.logger.info("🐕 ペット追跡システムを開始します 🐱")
        
        # メインループをスレッドで実行
        self.main_thread = threading.Thread(target=self._main_loop, daemon=True)
        self.main_thread.start()
        
        if self.show_display:
            self.logger.info("画面表示: 'q'キーで終了します")
    
    def stop_tracking(self) -> None:
        """追跡システム停止"""
        self.logger.info("追跡システムを停止しています...")
        self.is_running = False
        
        # スレッド終了待機
        if self.main_thread and self.main_thread.is_alive():
            self.main_thread.join(timeout=2.0)
        
        # リソース解放
        self._cleanup_resources()
        self.logger.info("追跡システムが停止しました")
    
    def _main_loop(self) -> None:
        """メインループ（スレッドで実行）"""
        last_detection_time = time.time()
        
        try:
            while self.is_running:
                loop_start_time = time.time()
                
                # フレーム取得
                ret, frame = self.camera.read()
                if not ret:
                    self.logger.warning("カメラからフレームを取得できませんでした")
                    time.sleep(0.1)
                    continue
                
                # 検出処理実行
                detection_results = self._process_detection(frame)
                
                # 追跡制御実行
                self._update_tracking_control(detection_results)
                
                # 表示更新
                if self.show_display:
                    display_frame = self._create_display_frame(frame, detection_results)
                    cv2.imshow("ペット追跡システム", display_frame)
                    
                    # キー入力チェック
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        self.logger.info("ユーザーによる終了要求")
                        break
                
                # システム状態更新
                self._update_system_status(detection_results)
                
                # 処理間隔調整
                elapsed_time = time.time() - loop_start_time
                sleep_time = max(0, self.detection_interval - elapsed_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
        except Exception as e:
            self.logger.error(f"メインループでエラー: {e}")
        finally:
            self.stop_tracking()
    
    def _process_detection(self, frame: np.ndarray) -> List[Dict]:
        """
        検出処理実行
        
        Args:
            frame: カメラフレーム
            
        Returns:
            List[Dict]: 検出結果リスト
        """
        try:
            # YOLO検出実行
            detections = self.yolo_detector.detect_pets(frame)
            
            # 最も信頼度の高い検出結果を取得
            if detections:
                best_detection = self.yolo_detector.get_best_detection(detections)
                return [best_detection] if best_detection else []
            
            return []
            
        except Exception as e:
            self.logger.error(f"検出処理でエラー: {e}")
            return []
    
    def _update_tracking_control(self, detections: List[Dict]) -> None:
        """
        追跡制御更新
        
        Args:
            detections: 検出結果リスト
        """
        try:
            with self.lock:
                if detections:
                    # 対象が検出された場合
                    detection = detections[0]
                    
                    # 追跡モードに切り替え
                    if self.status.mode != TrackingMode.TRACKING:
                        self.status.mode = TrackingMode.TRACKING
                        self.logger.info(f"🎯 追跡開始: {detection['class_name']}")
                    
                    # Simple P制御で補正値計算
                    bbox = detection['bbox']
                    center_x = (bbox[0] + bbox[2]) / 2
                    center_y = (bbox[1] + bbox[3]) / 2
                    
                    correction = self.simple_p_controller.calculate_correction((center_x, center_y))
                    
                    # サーボ角度更新
                    new_pan = self.status.pan_angle + correction[0]
                    new_tilt = self.status.tilt_angle + correction[1]
                    
                    # 角度制限確認
                    if self.servo_controller.is_angle_safe(new_pan, new_tilt):
                        self.servo_controller.set_angles(new_pan, new_tilt)
                        self.status.pan_angle = new_pan
                        self.status.tilt_angle = new_tilt
                        self.status.correction_applied = correction
                    
                    # 検出情報更新
                    self.status.target_detected = True
                    self.status.target_class = detection['class_name']
                    self.status.target_confidence = detection['confidence']
                    self.status.last_detection_time = datetime.now()
                    
                else:
                    # 対象が検出されなかった場合
                    self.status.target_detected = False
                    
                    # 一定時間検出されない場合はスキャンモードに切り替え
                    if (self.status.last_detection_time and 
                        (datetime.now() - self.status.last_detection_time).total_seconds() > self.lost_target_timeout):
                        
                        if self.status.mode == TrackingMode.TRACKING:
                            self.status.mode = TrackingMode.SCANNING
                            self.logger.info("🔍 対象ロスト - スキャンモードに切り替え")
                        
                        # スキャン動作（簡単な左右スイープ）
                        self._execute_scan_pattern()
                
        except Exception as e:
            self.logger.error(f"追跡制御更新でエラー: {e}")
    
    def _execute_scan_pattern(self) -> None:
        """スキャンパターンの実行（簡単な左右スイープ）"""
        try:
            current_time = time.time()
            # 10秒周期でゆっくりと左右にスイープ
            angle_offset = 30 * np.sin(current_time * 0.1)  # ±30度の範囲
            
            new_pan = angle_offset
            if self.servo_controller.is_angle_safe(new_pan, 0):
                self.servo_controller.set_angles(new_pan, 0)
                self.status.pan_angle = new_pan
                self.status.tilt_angle = 0
                
        except Exception as e:
            self.logger.error(f"スキャンパターン実行でエラー: {e}")
    
    def _create_display_frame(self, frame: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """
        表示用フレーム作成
        
        Args:
            frame: 元のフレーム
            detections: 検出結果
            
        Returns:
            np.ndarray: 表示用フレーム
        """
        display_frame = frame.copy()
        
        try:
            # 検出結果の描画
            if detections:
                detection = detections[0]
                bbox = detection['bbox']
                
                # バウンディングボックス描画
                color = (0, 255, 0)  # 緑色
                cv2.rectangle(display_frame, 
                            (int(bbox[0]), int(bbox[1])), 
                            (int(bbox[2]), int(bbox[3])), 
                            color, 2)
                
                # ラベル描画
                label = f"{detection['class_name']}: {detection['confidence']:.2f}"
                cv2.putText(display_frame, label,
                          (int(bbox[0]), int(bbox[1] - 10)),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                
                # 中心点描画
                center_x = int((bbox[0] + bbox[2]) / 2)
                center_y = int((bbox[1] + bbox[3]) / 2)
                cv2.circle(display_frame, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # 画面中央の十字線描画
            center_x, center_y = self.image_width // 2, self.image_height // 2
            cv2.line(display_frame, (center_x - 20, center_y), (center_x + 20, center_y), (255, 255, 255), 1)
            cv2.line(display_frame, (center_x, center_y - 20), (center_x, center_y + 20), (255, 255, 255), 1)
            
            # システム状態表示
            self._draw_system_info(display_frame)
            
        except Exception as e:
            self.logger.error(f"表示フレーム作成でエラー: {e}")
        
        return display_frame
    
    def _draw_system_info(self, frame: np.ndarray) -> None:
        """システム情報をフレームに描画"""
        try:
            y_offset = 30
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            color = (255, 255, 255)
            thickness = 1
            
            # システム情報
            info_lines = [
                f"Mode: {self.status.mode.value.upper()}",
                f"Target: {'YES' if self.status.target_detected else 'NO'}",
                f"Class: {self.status.target_class}",
                f"Confidence: {self.status.target_confidence:.2f}",
                f"Pan: {self.status.pan_angle:.1f}°",
                f"Tilt: {self.status.tilt_angle:.1f}°",
                f"Correction: ({self.status.correction_applied[0]:.2f}, {self.status.correction_applied[1]:.2f})",
                f"Detections: {self.status.total_detections}",
                "Press 'q' to quit"
            ]
            
            for i, line in enumerate(info_lines):
                y_pos = y_offset + (i * 25)
                cv2.putText(frame, line, (10, y_pos), font, font_scale, color, thickness)
                
        except Exception as e:
            self.logger.error(f"システム情報描画でエラー: {e}")
    
    def _update_system_status(self, detections: List[Dict]) -> None:
        """システム状態更新"""
        try:
            with self.lock:
                # 検出回数更新
                if detections:
                    self.status.total_detections += 1
                
                # 追跡時間更新
                if self.system_start_time:
                    self.status.tracking_duration = (datetime.now() - self.system_start_time).total_seconds()
                
        except Exception as e:
            self.logger.error(f"システム状態更新でエラー: {e}")
    
    def get_system_status(self) -> Dict:
        """システム状態取得"""
        with self.lock:
            return {
                'mode': self.status.mode.value,
                'target_detected': self.status.target_detected,
                'target_class': self.status.target_class,
                'target_confidence': self.status.target_confidence,
                'pan_angle': self.status.pan_angle,
                'tilt_angle': self.status.tilt_angle,
                'correction_applied': self.status.correction_applied,
                'total_detections': self.status.total_detections,
                'tracking_duration': self.status.tracking_duration,
                'is_running': self.is_running
            }
    
    def _cleanup_resources(self) -> None:
        """リソース解放"""
        try:
            # カメラ解放
            if self.camera:
                self.camera.release()
                self.logger.info("カメラリソースを解放しました")
            
            # OpenCVウィンドウ閉じる
            if self.show_display:
                cv2.destroyAllWindows()
            
            # サーボを中央位置に戻す
            if self.servo_controller:
                self.servo_controller.move_to_center()
                self.servo_controller.cleanup()
                self.logger.info("サーボを中央位置に戻しました")
            
            # その他のモジュールクリーンアップ
            if self.yolo_detector:
                self.yolo_detector.cleanup()
            
            if self.simple_p_controller:
                self.simple_p_controller.cleanup()
            
            self.logger.info("全リソースの解放が完了しました")
            
        except Exception as e:
            self.logger.error(f"リソース解放中にエラー: {e}")


def main():
    """統合制御モジュール単体テスト用メイン関数"""
    import signal
    import sys
    
    # ログ設定
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 追跡システム作成
    tracker = TrackingCoordinator(
        camera_id=0,
        show_display=True
    )
    
    # シグナルハンドラー設定
    def signal_handler(signum, frame):
        print("\n終了シグナルを受信しました...")
        tracker.stop_tracking()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        print("=== ペット追跡システム テスト ===")
        print("カメラの前に犬や猫を映してください")
        print("終了するには 'q' キーを押すか Ctrl+C を入力してください")
        
        # 追跡開始
        tracker.start_tracking()
        
        # メインスレッドは待機
        while tracker.is_running:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nキーボード割り込みを受信しました")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        tracker.stop_tracking()
        print("テストを終了しました")


if __name__ == "__main__":
    main()