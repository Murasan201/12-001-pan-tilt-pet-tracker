# Pan-Tilt AI Pet Tracker

A beginner-friendly pet tracking camera system using YOLOv8 object detection and Simple P control for automated pan-tilt servo movement.

## 🐕 Features

- **Real-time Pet Detection**: Automatically detects dogs and cats using YOLOv8
- **Simple P Control**: Easy-to-understand proportional control for camera tracking
- **Servo Motor Control**: Automated pan-tilt camera movement via PCA9685 driver
- **Live Video Display**: Real-time video feed with detection visualization
- **Educational Design**: Perfect for learning computer vision and control systems

## 🛠️ Hardware Requirements

### Essential Components
- **Raspberry Pi 5** (recommended) or Raspberry Pi 4
- **Camera Module v3** (or compatible USB camera)
- **PCA9685 16-Channel Servo Driver**
- **SG90 Servo Motors** (x2 for pan and tilt)
- **Pan-Tilt Bracket** for mounting camera and servos

### Hardware Connections
```
Raspberry Pi 5    →    PCA9685 Servo Driver
GPIO 2 (SDA)      →    SDA
GPIO 3 (SCL)      →    SCL  
5V                →    VCC
GND               →    GND

PCA9685           →    Servos
Channel 0         →    Pan Servo (Left/Right)
Channel 1         →    Tilt Servo (Up/Down)
```

## 📦 Software Installation

### 1. System Dependencies
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Python dependencies
sudo apt install python3-pip python3-venv git

# Enable I2C and Camera
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable
# Navigate to: Interface Options → Camera → Enable
sudo reboot
```

### 2. Python Environment Setup
```bash
# Clone repository
git clone https://github.com/yourusername/12-001-pan-tilt-pet-tracker.git
cd 12-001-pan-tilt-pet-tracker

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python packages
pip install -r requirements.txt

# Install hardware-specific libraries (Raspberry Pi only)
pip install adafruit-circuitpython-pca9685 adafruit-circuitpython-motor
```

### 3. YOLOv8 Model Download
```bash
# YOLOv8 nano model will be automatically downloaded on first run
# Or manually download:
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

## 🚀 Usage

### Basic Operation
```bash
# Activate virtual environment
source venv/bin/activate

# Run the pet tracking system
python3 main.py
```

### Advanced Options
```bash
# Run with specific camera
python3 main.py --camera-id 1

# Run with higher resolution
python3 main.py --width 1280 --height 720

# Run without display (headless mode)
python3 main.py --no-display

# Run with debug information
python3 main.py --debug

# Show all available options
python3 main.py --help
```

### Individual Module Testing

Test each component separately before running the full system:

```bash
# Test servo motors
python3 servo_test.py

# Test camera and YOLO detection
python3 camera_detection_test.py

# Test servo initial positioning (for assembly)
python3 servo_initial_position_setup.py

# Run individual module tests
python3 -m pytest tests/ -v
```

## 🎓 Educational Content

This project is designed for learning:

### Simple P Control Concept
```python
# Basic control equation
correction = error × gain

# Implementation example
x_error = detected_x - image_center_x
pan_correction = x_error × pan_gain
```

### System Architecture
```
Camera → YOLOv8 Detection → Simple P Control → Servo Movement
   ↓
Live Display ← System Status ← Tracking Coordinator
```

### Operating Modes
- **STANDBY**: System startup mode
- **SCANNING**: Searching for pets (side-to-side sweep)  
- **TRACKING**: Following detected pet

## 📁 Project Structure

```
12-001-pan-tilt-pet-tracker/
├── main.py                           # Main application
├── modules/                          # Core modules
│   ├── servo_controller.py          # Servo motor control
│   ├── yolo_detector.py             # Pet detection using YOLOv8
│   ├── simple_p_controller.py       # Simple P control implementation
│   └── tracking_coordinator.py      # System integration
├── tests/                           # Unit tests
│   ├── test_servo_controller.py
│   ├── test_yolo_detector.py
│   ├── test_simple_p_controller.py
│   └── test_tracking_coordinator.py
├── servo_test.py                    # Servo testing utility
├── camera_detection_test.py         # Camera/detection testing
├── servo_initial_position_setup.py # Initial servo positioning
└── requirements.txt                 # Python dependencies
```

## 🔧 Configuration

### Control Parameters
Adjust tracking sensitivity in `modules/simple_p_controller.py`:

```python
# Default values (tested and proven)
pan_gain = 0.0156     # Pan control sensitivity
tilt_gain = 0.0208    # Tilt control sensitivity
max_correction = 15.0 # Maximum correction per step (degrees)
deadband = 5.0        # Minimum movement threshold (pixels)
```

### Detection Settings
Modify detection behavior in `modules/yolo_detector.py`:

```python
confidence_threshold = 0.5  # Detection confidence (0.0-1.0)
target_classes = [15, 16]   # COCO classes: 15=cat, 16=dog
```

## 🐛 Troubleshooting

### Common Issues

**Camera not detected**
```bash
# Check camera connection
vcgencmd get_camera

# Test camera manually  
libcamera-hello --preview
```

**Servo not moving**
```bash
# Check I2C connection
sudo i2cdetect -y 1

# Test servo positioning
python3 servo_initial_position_setup.py
```

**Detection not working**
- Ensure good lighting conditions
- Check if pets are clearly visible in frame
- Verify YOLOv8 model download completed

**System running slow**
- Reduce image resolution: `--width 320 --height 240`
- Increase detection interval: `--interval 1.0`

### Debug Mode
```bash
# Enable detailed logging
python3 main.py --debug --log-level DEBUG
```

## 📈 Performance

### Tested Performance (Raspberry Pi 5)
- **Detection Speed**: ~20 FPS at 640x480
- **Control Response**: <200ms delay
- **Tracking Accuracy**: ±2 degrees
- **Power Consumption**: ~15W total system

### Optimization Tips
- Use lower resolution for better performance
- Adjust detection interval based on needs
- Consider hardware acceleration for YOLOv8

## 🤝 Contributing

This is an educational project! Contributions welcome:

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

### Development Setup
```bash
# Install development dependencies
pip install pytest black flake8

# Run tests
python3 -m pytest tests/ -v

# Format code
black *.py modules/ tests/

# Check code style
flake8 *.py modules/ tests/
```

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **YOLOv8**: Ultralytics team for the excellent object detection model
- **Adafruit**: For the excellent CircuitPython libraries
- **OpenCV**: For computer vision capabilities
- **Raspberry Pi Foundation**: For the amazing hardware platform

## 📞 Support

Having issues? Please:

1. Check the troubleshooting section above
2. Review existing [GitHub Issues](../../issues)
3. Create a new issue with:
   - Hardware setup details
   - Error messages and logs
   - Steps to reproduce the problem

---

**Happy Pet Tracking! 🐕🐱📹**