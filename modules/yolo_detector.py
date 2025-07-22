#!/usr/bin/env python3
"""
YOLO検出モジュール - YOLOv8による犬猫検出

このモジュールはYOLOv8モデルを使用してリアルタイムで犬と猫を検出し、
追跡システムで使用するための座標情報を提供します。

主な機能:
- YOLOv8による高精度な犬猫検出
- バウンディングボックスの中心座標計算
- 画像中心からの誤差計算（追跡制御用）
- 信頼度による検出結果フィルタリング
- 検出統計とパフォーマンス監視
- デバッグ用の可視化機能

対象クラス:
- 犬（Dog, class_id: 16）
- 猫（Cat, class_id: 15）

安全注意事項:
- モデルファイルのダウンロードには時間がかかります
- 処理能力に応じて解像度やモデルサイズを調整してください
- カメラが正しく接続されていることを確認してください
"""

import cv2
import time
import logging
import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

try:
    from ultralytics import YOLO
except ImportError as e:
    print(f"YOLOライブラリがインストールされていません: {e}")
    print("以下のコマンドでインストールしてください:")
    print("pip install ultralytics")
    raise


class DetectorError(Exception):
    """YOLO検出器固有の例外"""
    pass


class DetectorStatus(Enum):
    """検出器状態の定義"""
    UNINITIALIZED = "uninitialized"
    LOADING = "loading"
    READY = "ready"
    DETECTING = "detecting"
    ERROR = "error"


@dataclass
class Detection:
    """検出結果を格納するデータクラス"""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    center: Tuple[float, float] = None  # バウンディングボックスの中心座標
    
    def __post_init__(self):
        """中心座標の自動計算"""
        x1, y1, x2, y2 = self.bbox
        self.center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


class YOLODetector:
    """
    YOLO検出器クラス - YOLOv8による犬猫検出
    
    このクラスは既存のcamera_detection_test.pyの機能を拡張し、
    追跡システムで使用するための高精度検出機能を提供します。
    """
    
    def __init__(self, 
                 model_path: str = "yolov8n.pt",
                 confidence_threshold: float = 0.5,
                 target_classes: List[int] = None,
                 image_size: Tuple[int, int] = (640, 480)):
        """
        YOLO検出器初期化
        
        Args:
            model_path: YOLOv8モデルファイルパス
            confidence_threshold: 信頼度閾値
            target_classes: 対象クラスID（None時は犬猫のみ）
            image_size: 処理画像サイズ
        """
        # 基本パラメータ
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.image_size = image_size
        
        # 対象クラスの設定（COCO dataset）
        if target_classes is None:
            self.target_classes = [15, 16]  # cat, dog
        else:
            self.target_classes = target_classes
        
        # COCO class names
        self.class_names = {
            15: "Cat",
            16: "Dog"
        }
        
        # 表示色の設定（BGR形式）
        self.class_colors = {
            15: (0, 255, 0),    # 猫: 緑色
            16: (255, 0, 0),    # 犬: 青色
        }
        
        # モデルインスタンス
        self.model = None
        self.status = DetectorStatus.UNINITIALIZED
        
        # 検出統計
        self.total_detections = 0
        self.detection_history = []  # 最近の検出結果履歴
        self.processing_times = []   # 処理時間履歴
        
        # パフォーマンス監視
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0.0
        
        # ログ設定
        self.logger = logging.getLogger(__name__)
    
    def load_model(self) -> bool:
        """
        YOLOv8モデルの読み込み
        
        Returns:
            bool: 読み込み成功時True、失敗時False
        """
        try:
            self.status = DetectorStatus.LOADING
            self.logger.info(f"YOLOv8モデルを読み込み中: {self.model_path}")
            
            # モデルの読み込み（初回実行時は自動ダウンロード）
            self.model = YOLO(self.model_path)
            
            # モデル情報の表示
            self.logger.info("モデルの読み込みが完了しました")
            self.logger.info(f"  モデル: {self.model_path}")
            self.logger.info(f"  信頼度閾値: {self.confidence_threshold}")
            self.logger.info(f"  対象クラス: {[self.class_names.get(cid, f'ID{cid}') for cid in self.target_classes]}")
            
            self.status = DetectorStatus.READY
            return True
            
        except Exception as e:
            self.status = DetectorStatus.ERROR
            self.logger.error(f"モデル読み込み中にエラー: {e}")
            return False
    
    def detect_pets(self, frame: np.ndarray) -> List[Detection]:
        """
        犬猫検出実行
        
        Args:
            frame: 入力画像フレーム
            
        Returns:
            List[Detection]: 検出結果のリスト
        """
        if self.status != DetectorStatus.READY:
            self.logger.warning(f"検出器が準備状態ではありません: {self.status}")
            return []
        
        detection_start_time = time.time()
        
        try:
            self.status = DetectorStatus.DETECTING
            
            # YOLOv8で推論実行
            results = self.model(frame, conf=self.confidence_threshold, verbose=False)
            
            detections = []
            
            # 検出結果の解析
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # クラスID、信頼度、座標の取得
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])
                        x1, y1, x2, y2 = [int(coord) for coord in box.xyxy[0].tolist()]
                        
                        # 対象クラス（犬または猫）の場合のみ処理
                        if class_id in self.target_classes:
                            detection = Detection(
                                class_id=class_id,
                                class_name=self.class_names.get(class_id, f"Class_{class_id}"),
                                confidence=confidence,
                                bbox=(x1, y1, x2, y2)
                            )
                            detections.append(detection)
            
            # 処理時間の記録
            processing_time = time.time() - detection_start_time
            self.processing_times.append(processing_time)
            
            # 処理時間履歴の管理（最新100フレーム分のみ保持）
            if len(self.processing_times) > 100:
                self.processing_times.pop(0)
            
            # 検出履歴の更新
            self.detection_history.append({
                'timestamp': time.time(),
                'count': len(detections),
                'processing_time': processing_time
            })
            
            # 検出履歴の管理（最新1000フレーム分のみ保持）
            if len(self.detection_history) > 1000:
                self.detection_history.pop(0)
            
            self.total_detections += len(detections)
            self.status = DetectorStatus.READY
            
            return detections
            
        except Exception as e:
            self.status = DetectorStatus.ERROR
            self.logger.error(f"検出処理中にエラー: {e}")
            return []
    
    def get_best_detection(self, detections: List[Detection]) -> Optional[Detection]:
        """
        最も信頼度の高い検出結果を取得
        
        Args:
            detections: 検出結果のリスト
            
        Returns:
            Detection: 最も信頼度の高い検出結果（なければNone）
        """
        if not detections:
            return None
        
        # 信頼度の高い順でソート
        sorted_detections = sorted(detections, key=lambda d: d.confidence, reverse=True)
        return sorted_detections[0]
    
    def calculate_center(self, bbox: Tuple[int, int, int, int]) -> Tuple[float, float]:
        """
        バウンディングボックスの中心座標計算
        
        Args:
            bbox: バウンディングボックス (x1, y1, x2, y2)
            
        Returns:
            Tuple[float, float]: 中心座標 (center_x, center_y)
        """
        x1, y1, x2, y2 = bbox
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        return (center_x, center_y)
    
    def calculate_tracking_error(self, detection: Detection, 
                                image_center: Tuple[float, float]) -> Tuple[float, float]:
        """
        画像中心からの誤差計算（追跡制御用）
        
        Args:
            detection: 検出結果
            image_center: 画像中心座標 (cx, cy)
            
        Returns:
            Tuple[float, float]: 追跡誤差 (pan_error, tilt_error)
        """
        detection_center = detection.center
        
        # 画像中心からのずれを計算（追跡制御誤差）
        pan_error = detection_center[0] - image_center[0]   # X軸方向の誤差
        tilt_error = detection_center[1] - image_center[1]  # Y軸方向の誤差
        
        return (pan_error, tilt_error)
    
    def draw_detections(self, frame: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """
        検出結果の描画（デバッグ用）
        
        Args:
            frame: 入力画像フレーム
            detections: 検出結果のリスト
            
        Returns:
            np.ndarray: 描画済みフレーム
        """
        drawn_frame = frame.copy()
        
        for detection in detections:
            x1, y1, x2, y2 = detection.bbox
            color = self.class_colors.get(detection.class_id, (255, 255, 255))
            
            # バウンディングボックスの描画
            cv2.rectangle(drawn_frame, (x1, y1), (x2, y2), color, 2)
            
            # 中心点の描画
            center_x, center_y = detection.center
            cv2.circle(drawn_frame, (int(center_x), int(center_y)), 5, color, -1)
            
            # ラベルテキストの作成
            label = f"{detection.class_name} {detection.confidence:.2f}"
            
            # ラベル背景の描画
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(drawn_frame, (x1, y1 - text_height - 10), 
                         (x1 + text_width, y1), color, -1)
            
            # ラベルテキストの描画
            cv2.putText(drawn_frame, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return drawn_frame
    
    def draw_tracking_info(self, frame: np.ndarray, 
                          image_center: Tuple[float, float] = None) -> np.ndarray:
        """
        追跡情報の描画（デバッグ用）
        
        Args:
            frame: 入力画像フレーム
            image_center: 画像中心座標（None時は自動計算）
            
        Returns:
            np.ndarray: 情報描画済みフレーム
        """
        height, width = frame.shape[:2]
        
        # 画像中心の計算
        if image_center is None:
            image_center = (width // 2, height // 2)
        
        drawn_frame = frame.copy()
        
        # 画像中心の描画（十字線）
        center_x, center_y = int(image_center[0]), int(image_center[1])
        cv2.line(drawn_frame, (center_x - 20, center_y), (center_x + 20, center_y), 
                (0, 255, 255), 2)
        cv2.line(drawn_frame, (center_x, center_y - 20), (center_x, center_y + 20), 
                (0, 255, 255), 2)
        
        # FPS表示
        fps_text = f"FPS: {self.current_fps:.1f}"
        cv2.putText(drawn_frame, fps_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 検出統計表示
        detection_count = len([d for d in self.detection_history[-10:] if d['count'] > 0])
        stats_text = f"Detections: {detection_count}/10"
        cv2.putText(drawn_frame, stats_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 平均処理時間表示
        if self.processing_times:
            avg_time = np.mean(self.processing_times)
            time_text = f"Process: {avg_time*1000:.1f}ms"
            cv2.putText(drawn_frame, time_text, (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return drawn_frame
    
    def get_detection_statistics(self) -> Dict:
        """
        検出統計情報取得
        
        Returns:
            Dict: 統計情報
        """
        recent_detections = self.detection_history[-100:]  # 最新100フレーム
        
        stats = {
            'total_detections': self.total_detections,
            'recent_detection_rate': len([d for d in recent_detections if d['count'] > 0]) / max(len(recent_detections), 1),
            'average_processing_time': np.mean(self.processing_times) if self.processing_times else 0,
            'current_fps': self.current_fps,
            'model_path': self.model_path,
            'confidence_threshold': self.confidence_threshold,
            'status': self.status.value
        }
        
        return stats
    
    def set_confidence_threshold(self, threshold: float) -> None:
        """
        信頼度閾値の動的変更
        
        Args:
            threshold: 新しい信頼度閾値（0.0-1.0）
        """
        if 0.0 <= threshold <= 1.0:
            self.confidence_threshold = threshold
            self.logger.info(f"信頼度閾値を変更: {threshold}")
        else:
            self.logger.warning(f"信頼度閾値が範囲外です: {threshold}")
    
    def update_fps(self) -> None:
        """FPS計算の更新"""
        self.fps_counter += 1
        
        # 1秒ごとにFPSを更新
        if time.time() - self.fps_start_time >= 1.0:
            self.current_fps = self.fps_counter / (time.time() - self.fps_start_time)
            self.fps_counter = 0
            self.fps_start_time = time.time()
    
    def get_status(self) -> DetectorStatus:
        """ステータス取得"""
        return self.status
    
    def cleanup(self) -> None:
        """リソース解放"""
        try:
            self.logger.info("YOLO検出器をクリーンアップ中...")
            
            # モデルリソースの解放
            if self.model:
                del self.model
                self.model = None
            
            self.status = DetectorStatus.UNINITIALIZED
            self.logger.info("YOLO検出器のクリーンアップが完了しました")
            
        except Exception as e:
            self.logger.error(f"クリーンアップ中にエラー: {e}")


# 旧形式との互換性のためのエイリアス
def create_yolo_detector(model_path: str = "yolov8n.pt") -> YOLODetector:
    """
    YOLO検出器のファクトリ関数
    既存コードとの互換性を保つため
    """
    return YOLODetector(model_path=model_path)


if __name__ == "__main__":
    # モジュール単体テスト用
    import logging
    
    logging.basicConfig(level=logging.INFO)
    
    print("YOLODetectorモジュール単体テスト")
    print("=" * 40)
    
    detector = YOLODetector()
    
    try:
        if detector.load_model():
            print("モデル読み込み成功")
            
            # テスト画像での検出テスト
            test_frame = np.zeros((480, 640, 3), dtype=np.uint8)  # 黒い画像
            
            detections = detector.detect_pets(test_frame)
            print(f"検出結果: {len(detections)}個")
            
            # 統計情報の表示
            stats = detector.get_detection_statistics()
            print(f"統計: {stats}")
            
        else:
            print("モデル読み込み失敗")
    
    except KeyboardInterrupt:
        print("\nテスト中断")
    
    finally:
        detector.cleanup()
        print("テスト終了")