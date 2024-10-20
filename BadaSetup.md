# 설정
## onboard 컴퓨터 방화벽 해제

## fcu, gcu 설정
* fcu
   * usb serial 
* gcu
   * telemetry serial 

## serial 경로 설정
* /dev/myserial 경로로 시리얼 포트 설정 방법
```bash
sudo nano /etc/udev/rules.d/99-my-serial.rules
```

* lsusb 명령어로 확인한 vendor id와 product id를 확인
```
Bus 001 Device 002: ID 0403:6001 Future Technology Devices International, Ltd FT23   

#0403:6001 에서 0403가 vendor id, 6001이 product id
```

* 99-my-serial.rules 파일 내부
```
SUBSYSTEM=="tty", ATTR{idVendor}=="YOUR_VENDOR_ID", ATTR{idProduct}=="YOUR_PRODUCT_ID", SYMLINK+="myserial"
```

* udev 데몬 재시작하여 적용
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```


* /dev/myserial 경로로 시리얼 포트 확인
```bash
ls -l /dev/myserial
```

## 시작프로그램 설정
1. service 파일 생성
```bash
sudo nano /etc/systemd/system/mylaunch.service
```

2. mylaunch.service 파일 내부
```
[Unit]
Description=Starts my ROS2 launch file
After=network.target

[Service]
Type=simple
User=your_username  # 실행할 유저 이름
WorkingDirectory=/home/your_username/ros2_ws/src/mypackage/launch/
ExecStart=/usr/bin/python3 mylaunch.py

[Install]
WantedBy=multi-user.target
```
* service 활성화 및 시작
```bash
sudo systemctl daemon-reload
sudo systemctl enable mylaunch.service
sudo systemctl start mylaunch.service
```

