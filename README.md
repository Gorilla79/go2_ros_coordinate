# go2_ros_coordinate

## ros2 패키지 사용하기
### 1. test 패키지 만들기

1. 작업공간 만들기
2. 패키지 생성
3. 실행 노드 파일 만들기
4. package.xml, setup.py 수정하기
5. colcon build
6. 실행
7. 확인
![Screenshot from 2025-03-24 23-01-12](https://github.com/user-attachments/assets/b5e448b9-56ca-421a-a5b0-cbffff78cc70)

---

### 2. 사용자 정의 패키지 만들기

1. 작업공간 만들기
2. 패키지 생성
3. 실행 노드 파일 만들기(기존 go2.py에 topic발행만 추가)
- topic발행과 데이터 형식 지정해주기

4. package.xml, setup.py 수정하기
5. colcon build
- colcon build --packages-select go2

6. 실행
- ros2 run go2 go2_publisher

7. 확인
- ros2 topic list
- ros2 topic echo /go2_coordinate
![Screenshot from 2025-03-24 23-22-42](https://github.com/user-attachments/assets/cd7aab86-f6d7-441b-b244-ba11c16fedfc)
![Screenshot from 2025-03-24 23-15-11](https://github.com/user-attachments/assets/063de447-ce5f-446f-a0c7-b81b7b8bc915)
