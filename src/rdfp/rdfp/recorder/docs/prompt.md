Ubuntu 22.04 OS 의 ROS2 Humble 환경에서
주어진 토픽에서 발생되는 이미지 ( sensor_msgs/Image )를 받아서 mp4 영상을 생성하는
ROS2 노드 또는 클래스를 작성하고 싶어.
또한 추가의 세션 토픽에서 녹화 시작과 종료와 관련된 제어를 받고 싶어.

다음은 주요 요구사항 이야.
* 개발 언어는 Python이야.
* 노드는 ROS2의 rclpy를 사용해서 작성할 예정이야.
* 노드 클래스 이름은 RdfpImageRecorder로 생각하고 있어. 노드 이름은 "rdfp_image_recorder"로 할 예정이야.
* 이미 개발된 class FFMpegMp4Recorder 클래스를 활용해서 구현하는 것을 고려하고 있어.
* 세션 제어를 위한 토픽은 'session_control_node.py'에서 구현된 SessionControlNode에서 발행하는 토픽을 구독하는 형태로 구현할 예정이야. 토픽의 내용은 이 노드의 구현물을 참고해 줘.
* 녹화의 대상이 되는 이미지 프레임은 "image"라는 토픽에서 수신된다. 이 토픽은 sensor_msgs/Image 타입의 메시지를 발행하는 메시지이다.

* 다음은 RdfpImageRecorder의 동작 방식이다.
  * 'start' 메시지: session topic에서 발행되는 메시지의 state 필드가 "IN_EPISODE"인 메시지를 의미해.
  이 메시지의  header stamp를 'start_ts'라고 부를 예정이야. 이 메시지가 도착하면 녹화 구간이 시작된다고 간주해.
  * 'stop' 메시지: session topic에서 발행되는 메시지의 state 필드가 "IN_SESSION"인 메시지를 의미해. 이 메시지의 header stamp를 'stop_ts'라고 부를 예정이야. 이 메시지가 도착하면 녹화 구간이 종료된다고 간주해.
  * 'frame_ts'는 image 토픽에서 수신되는 이미지 메시지의 header stamp를 의미해.
  * '녹화 구간': frame_ts가 start_ts보다 크거나 같고, stop_ts보다 작은 구간을 의미해. 이 구간에서는 image 토픽에서 수신되는 이미지들을 mp4로 기록하는 구간이야.
  * '비 녹화 구간': 그 외의 구간이야. 이 구간에서는 image 토픽에서 수신되는 이미지 메시지가 mp4로 기록되지 않는 구간이야.
* 녹화 구간이 RdfpImageRecorder에 도착하는 이미지 메시지와 세션 제어 메시지의 도착 순서가 아니라
  메시지 header에 포함된 stamp를 사용하기 때문에 다음의 고려사항이 발생
  * session topic에 'start' 메시지가 도착할 때 'start_ts'보다 큰 frame_ts를
    갖는 이미지 메시지가 이미 도착하는 경우가 발생
  * session topic에 'stop' 메시지가 도착하기 이전에 이미 'stop_ts'보다 크거나 같은
    frame_ts를 갖는 이미지가 이미 도착한 경우 발생
* 이 상태를 해결하기 위해 내부적으로 `pending_image_queue`라는 FIFO (DropOldest 정책 사용)로
  관리되는 queue를 사용한다.
  * 이미지 메시지가 도착하면 바로 녹화하거나 버리지 말고 `pending_image_queue`에 넣고,
    queue의 overflow 발생으로 선택된 victim msg의 header stamp (`frame_ts`)를
    이용하여 다음과 같은 동작을 수행한다.
    * 녹화 구간인 경우: frame_ts >= start_ts 이면 녹화 대상으로
      간주하여 FFMpegMp4Recorder recorder의 write 함수 호출. 그렇지 않은 경우는 drop.
    * 비 녹화 구간인 경우: frame_ts < stop_ts이면 녹화 대상으로 간주하여 FFMpegMp4Recorder recorder의 
      write 함수 호출. 그렇지 않은 경우는 drop. Node가 시작된 직후는 비 녹화 구간이지만 `stop_ts`가 존재하지 않기 때문에 이때는 stop_ts를 0으로 간주한다.
  * `start` 메시지가 도착하는 경우에는 `pending_image_queue`에 포함된 이미지 메시지에 대해
    시간 순서대로 이미지를 iteration 하여 frame_ts < start_ts 인 메시지를 모두 drop 시킨다.
  * `stop` 메시지가 도착하는 경우에는 `pending_image_queue`에 포함된 이미지 메시지에 대해
    시간 순서대로 이미지 msg를 iteration 하여 frame_ts < stop_ts인 메시지를 recorder에 전달하고,
    나머지는 queue에 계속 유지시킨다. `stop` 메시지 도착 이후에 도착하는 이미지는 계속 pending_image_queue에 삽입되지만, 이때는 stop_ts보다 작은 frame_ts를 갖는 이미지 메시지가 삽입되는 경우는 warning 메시지를 출력하고, 이미지 메시지를 drop 시킨다.
  * RdfpImageRecorder가 종료하는 경우는 `pending_image_queue`에 포함된 각 msg에 대해
    녹화/비녹화 구간 여부에 따라 위 작업을 반복함.
* 이 queue의 길이는 `pending_queue_length` 이름의 파라미터를 통해 결정된다.
  * `pending_queue_length`는 프레임 갯수로 표현됨
  * Queue의 길이가 짧으면 비 녹화 단계에서 `start`가 도착할 때, 충분한 수의 이미지가
    queue에 있지 않아 start_ts보다 크거나 같은 frame_ts를 갖은 이미지 queue에 overflow로
    drop되어 녹화에서 누락되거나, 녹화 단계에서 `stop` 메시지가 도착할 때 stop_ts보다 크거나 같은 메시지가 `pending_image_queue`에서 overflow로 이미 queue에서 나와 녹화되는 현상이 발생할 가능성이 있다. 이 경우는 warning 메시지를 출력한다.
* recorder의 write() 함수 호출할 때 예외가 발생되면 warning 메시지를 출력하고, 예외가 발생한 이미지 메시지는 drop 시킨다.

* 기존에 개발된 image_recorder_node.py와는 별도의 노드로 구현할 예정이야.
* 출력 경로/파일명 등 ffmpeg_mp4_recorder 클래스의 생성자에서 사용되는 정보들은 ROS2 파라미터를
  통해 받도록 해줘.
* task_label은 여기서 사용되지 않아.



