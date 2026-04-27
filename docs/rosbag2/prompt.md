rosbag2을 통해 저장된 토픽 메시지들을 정재하여 원하는
메시지들을 필터링하여 DBMS에 저장하는 프로그램을 작성한다.

다음은 프로그램의 요구사항이다.
* Unbuntu 22.04, ROS2 Humble 환경임.
* Python으로 작성한다. rclpy 패키징을 사용
* 대상 rosbag 저장위치는 <rosbag_dir>로 한다. 이 경로는 설정을 통해 변경할 수 있다.
* rosbag에 저장된 모든 메시지의 header에는 stamp가 존재한다고 가정한다.
* rosbag 관리 등에 대한 내용은 @docs/rosbag2/rosbag2 운영 방안.md 을 참고한다.
* rosbag에 저장된 토픽 중에는 rdfp_msgs/msg/SessionCommand 타입의 토픽이 반드시 존재한다.
  이 토픽에 포함된 메시지를 세션 (session) 메시지라 부른다.
* 세션 토픽과 세션 메시지와 관련된 내용은 @docs/session/session_control_guide.md 문서를 참고한다.
* session 메시지들 중에서 state 값이 IN_EPISODE인 메시지를 에피소드 시작 메시지 (start_msg)라
  부르고, 그 메시지의 header.stamp를 start_ts라 부른다. 즉 에피소드 시작 시각을 의미함
* 또한 session 메시지들 중에서 state 값이 IN_SESSION인 메시지를 에피소드 종료
  메시지 (stop_msg)라 부르고, 그 메시지의 header.stamp를 stop_ts라 부른다.
  즉 에피소드 종료 시각을 의미함.
  * 여기서 주의할 점은 모든 state=IN_SESSION인 메시지에서 stop_msg가 아니라,
    이전 session 메시지의 state=IN_EPISODE인 경우에 한정한다. 예를들어 이전 session 메시지의 state=IDLE인 경우가 있는데, 이는 무시된다.
* 범위 지정은 다음의 방법들을 조합하여 지정할 계획임. 혹시 추천하는 추가 범위
  지정 방식이 있으면 알려줘.
  * 시간 구간을 지정하는 방법
  * 대상 토픽을 지정하는 방법

## rosbag 데이터 필터링
* rosbag에 저장된 메시지에서 DBMS에 저장될 메시지를 필터링하는 방법의 대략 다음의 과정을 거친다.
  1. 사용자로부터 범위 지정을 받는다. 범위 지정은 시간 구간을 받는 방법과 대상 토픽명을
    받는다.
    * 만일 session 토픽이 지정되지 않아도, 이는 자동적으로 포함된다.
  2. 대상 토픽들 에서 주어진 시간 범위를 만족하는 메시지를 검색한다.
  3. 검색된 session 토픽 값을 시간 순서대로 읽어서 에피소드 구간을 찾아내고,
     start_ts와 stop_ts 값, 그리고 해당 에피소드 구간의 식별자 값 (episdoe_id)을
     'sessions'라는 테이블의 1개의 레코드로 저장한다. 여기서 episode_id는 테이블에 저장된
     레코드의 내부 식별자를 사용해도 무방하고, 그냥 sequence 번호여도 무방하다.
  4. 검색된 session 토픽 값을 시간 순서대로 읽어서 에피소드 구간을 찾아내고,
     검색된 다른 토픽 메시지들 중에서 이 에피소드 구간에 포함된 메시지만 찾는다.
     메시지 타입 별로 준비된 테이블에 저장한다. 이때 메시지의 소속 episode_id도 함께
     저장된다.
     * 여기서 과정 3, 4는 순차적으로 될 필요는 없고, 구현의 편의상 함께 수행되도 무방함

## 데이터베이스 모델
* 데이터베이스에는 rosbag에 저장된 토픽들의 메시지 타입별로 테이블이 준비되어 있어야 한다.
  * 만일 필터링 과정에서 범위에 포함된 토픽에 해당하는 테이블이 존재하지 않으면 필터링
    과정이 시작될 때 오류를 발생한다.
* 주요 메시지 타입에 대한 테이블 스키마이다.
  * pose_stamped (메시지 타입: rdfp_msgs/msg/SessionCommand)
  ```
  CREATE TABLE sessions (
      id              BIGSERIAL     PRIMARY KEY,
      start_sec       INTEGER       NOT NULL,
      start_nanosec   BIGINT        NOT NULL,
      start_ts        TIMESTAMPTZ   GENERATED ALWAYS AS
                          (TO_TIMESTAMP(stamp_sec) + (stamp_nanosec || ' nanoseconds')::INTERVAL)
                          STORED,
      stop_sec        INTEGER       NOT NULL,
      stop_nanosec    BIGINT        NOT NULL,
      stop_ts         TIMESTAMPTZ   GENERATED ALWAYS AS
                          (TO_TIMESTAMP(stamp_sec) + (stamp_nanosec || ' nanoseconds')::INTERVAL)
                          STORED,

      task_label      TEXT
  );@docs/rosbag2/rosbag2 운영 방안.md  

  CREATE INDEX idx_session_start_ts   ON sessions (start_ts);
  ```

  * twist_stamped (메시지 타입: geometry_msgs/msg/TwistStamped)
  ```
  CREATE TABLE twist_stamped (
      id              BIGSERIAL     PRIMARY KEY,
      stamp_sec       INTEGER       NOT NULL,
      stamp_nanosec   BIGINT        NOT NULL,
      stamp_ts        TIMESTAMPTZ   GENERATED ALWAYS AS
                          (TO_TIMESTAMP(stamp_sec) + (stamp_nanosec || ' nanoseconds')::INTERVAL)
                          STORED,
      twist           DOUBLE PRECISION[6] NOT NULL
                      -- [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
      CHECK (array_length(twist, 1) = 6)
  );

  CREATE INDEX idx_twist_stamp_ts   ON twist_stamped (stamp_ts);
  ```

* pose_stamped (메시지 타입: geometry_msgs/msg/PoseStamped)
```
CREATE TABLE pose_stamped (
    id              BIGSERIAL       PRIMARY KEY,
    stamp_sec       INTEGER         NOT NULL,
    stamp_nanosec   BIGINT          NOT NULL,
    stamp_ts        TIMESTAMPTZ     GENERATED ALWAYS AS (...) STORED,
    position        DOUBLE PRECISION[3] NOT NULL,  -- [x, y, z]
    orientation     DOUBLE PRECISION[4] NOT NULL,  -- [x, y, z, w]
    CHECK (array_length(position, 1) = 3),
    CHECK (array_length(orientation, 1) = 4)
);

CREATE INDEX idx_pose_stamp_ts   ON pose_stamped (stamp_ts);
```

* delta_joint_cmds (메시지 타입: trajectory_msgs/msg/JointTrajectory)
```
CREATE TABLE joint_cmds (
    id               BIGSERIAL           PRIMARY KEY,
    stamp_sec        INTEGER             NOT NULL,
    stamp_nanosec    BIGINT              NOT NULL,
    stamp_ts         TIMESTAMPTZ         GENERATED ALWAYS AS
                         (TO_TIMESTAMP(stamp_sec) + (stamp_nanosec || ' nanoseconds')::INTERVAL)
                         STORED,

    -- 마지막 point의 time_from_start
    tfs_sec          INTEGER             NOT NULL,
    tfs_nanosec      BIGINT              NOT NULL,
    tfs_interval     INTERVAL            GENERATED ALWAYS AS
                         ((tfs_sec || ' seconds')::INTERVAL
                          + (tfs_nanosec || ' nanoseconds')::INTERVAL)
                         STORED,

    positions        DOUBLE PRECISION[]  NOT NULL,
    velocities       DOUBLE PRECISION[]  NOT NULL,
    accelerations    DOUBLE PRECISION[]  NOT NULL,
    effort           DOUBLE PRECISION[]  NOT NULL
);

CREATE INDEX idx_jt_stamp_ts   ON joint_cmds (stamp_ts);

```

* joint_state (메시지 타입: sensor_msgs/msg/JointState)
```
CREATE TABLE joint_state (
    id              BIGSERIAL           PRIMARY KEY,
    stamp_sec       INTEGER             NOT NULL,
    stamp_nanosec   BIGINT              NOT NULL,
    stamp_ts        TIMESTAMPTZ         GENERATED ALWAYS AS
                        (TO_TIMESTAMP(stamp_sec) + (stamp_nanosec || ' nanoseconds')::INTERVAL)
                        STORED,
    position        DOUBLE PRECISION[]  NOT NULL,
    velocity        DOUBLE PRECISION[]  NOT NULL,
    effort          DOUBLE PRECISION[]  NOT NULL
);

CREATE INDEX idx_js_stamp_ts   ON joint_state (stamp_ts);
```
* 카메라 이미지의 경우는 데이터베이스에 저장하지 않고 별도의 mp4파일에 episode 단위로 저장된다.

이 문서를 바탕으로 데이터셋 정제기 개발 계획을 작성해 줘.

