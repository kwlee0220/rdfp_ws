-- 데이터셋 후처리기용 PostgreSQL 스키마.
-- 설계서: docs/rosbag2/데이터셋 후처리기 설계서.md
--
-- stamp_ts/start_ts/stop_ts 는 sec + nanosec 으로부터 파생된 STORED generated
-- column 이다. STORED 는 IMMUTABLE 식만 허용하므로 timestamptz + interval 경로
-- 대신 to_timestamp(double precision) 을 사용한다.

CREATE TABLE IF NOT EXISTS sessions (
    id              BIGSERIAL     PRIMARY KEY,
    start_sec       INTEGER       NOT NULL,
    start_nanosec   BIGINT        NOT NULL,
    start_ts        TIMESTAMPTZ   GENERATED ALWAYS AS
                        (to_timestamp(start_sec::double precision
                                      + start_nanosec::double precision / 1e9))
                        STORED,
    stop_sec        INTEGER       NOT NULL,
    stop_nanosec    BIGINT        NOT NULL,
    stop_ts         TIMESTAMPTZ   GENERATED ALWAYS AS
                        (to_timestamp(stop_sec::double precision
                                      + stop_nanosec::double precision / 1e9))
                        STORED,
    task_label      TEXT,
    -- 작업 성패. NULL 은 **판정이 없다**는 뜻이며 실패가 아니다 — 텔레오퍼레이션
    -- 수집이나 중단 복구처럼 판정 주체가 없었던 경우다. 자동 수집에서 파지 실패는
    -- success=false 인 **유효한 에피소드**이므로 학습셋에서 무조건 제외하지 않는다.
    success         BOOLEAN,
    -- 에피소드 재현·분석에 필요한 부가 정보 (seed, scene 이름, 초기 물체 배치,
    -- 실패 사유 등). 형태가 씬 레시피·백엔드마다 달라지므로 정규화하지 않는다.
    -- 중단된 에피소드는 success IS NULL + metadata->>'abort_reason' 으로 구분한다.
    metadata        JSONB,
    UNIQUE (start_sec, start_nanosec)
);
-- 기존 DB 호환: 위 CREATE TABLE 은 IF NOT EXISTS 이므로 이미 만들어진 테이블에는
-- 컬럼을 추가하지 않는다. success/metadata 도입 이전 DB 를 위해 명시적으로 더한다.
ALTER TABLE sessions ADD COLUMN IF NOT EXISTS success  BOOLEAN;
ALTER TABLE sessions ADD COLUMN IF NOT EXISTS metadata JSONB;
CREATE INDEX IF NOT EXISTS idx_sessions_start_ts ON sessions (start_ts);
-- metadata 에는 인덱스를 두지 않는다. 에피소드는 수천 행 규모라 seq scan 이
-- 밀리초이고, GIN 은 INSERT 비용만 늘린다. 조회가 느려지면 그때 GIN 을 더한다.


CREATE TABLE IF NOT EXISTS topics (
    id              BIGSERIAL     PRIMARY KEY,
    topic_name      TEXT          NOT NULL UNIQUE,
    topic_type      TEXT          NOT NULL
);


-- /delta_twist_stamp → geometry_msgs/msg/TwistStamped
CREATE TABLE IF NOT EXISTS twist_stampeds (
    id              BIGSERIAL           PRIMARY KEY,
    episode_id      BIGINT              NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT              NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    stamp_sec       INTEGER             NOT NULL,
    stamp_nanosec   BIGINT              NOT NULL,
    stamp_ts        TIMESTAMPTZ         GENERATED ALWAYS AS
                        (to_timestamp(stamp_sec::double precision
                                      + stamp_nanosec::double precision / 1e9))
                        STORED,
    twist           DOUBLE PRECISION[6] NOT NULL,
    CHECK (array_length(twist, 1) = 6)
);
CREATE INDEX IF NOT EXISTS idx_twist_stampeds_episode  ON twist_stampeds (episode_id);
CREATE INDEX IF NOT EXISTS idx_twist_stampeds_topic    ON twist_stampeds (topic_id);
CREATE INDEX IF NOT EXISTS idx_twist_stampeds_stamp_ts ON twist_stampeds (stamp_ts);


-- /joint_states → sensor_msgs/msg/JointState
CREATE TABLE IF NOT EXISTS joint_states (
    id              BIGSERIAL           PRIMARY KEY,
    episode_id      BIGINT              NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT              NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    stamp_sec       INTEGER             NOT NULL,
    stamp_nanosec   BIGINT              NOT NULL,
    stamp_ts        TIMESTAMPTZ         GENERATED ALWAYS AS
                        (to_timestamp(stamp_sec::double precision
                                      + stamp_nanosec::double precision / 1e9))
                        STORED,
    position        DOUBLE PRECISION[]  NOT NULL,
    velocity        DOUBLE PRECISION[]  NOT NULL,
    effort          DOUBLE PRECISION[]  NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_joint_states_episode  ON joint_states (episode_id);
CREATE INDEX IF NOT EXISTS idx_joint_states_topic    ON joint_states (topic_id);
CREATE INDEX IF NOT EXISTS idx_joint_states_stamp_ts ON joint_states (stamp_ts);


-- /gripper_control/gripper_cmds → rdfp_msgs/msg/GripperCommand
-- command 는 'open' / 'close' 문자열.
CREATE TABLE IF NOT EXISTS gripper_cmds (
    id              BIGSERIAL           PRIMARY KEY,
    episode_id      BIGINT              NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT              NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    stamp_sec       INTEGER             NOT NULL,
    stamp_nanosec   BIGINT              NOT NULL,
    stamp_ts        TIMESTAMPTZ         GENERATED ALWAYS AS
                        (to_timestamp(stamp_sec::double precision
                                      + stamp_nanosec::double precision / 1e9))
                        STORED,
    -- 학습 데이터의 **action 채널**. 심볼이 아니라 숫자를 남긴다 — 심볼을 쓰면
    -- 그 의미(몇 m 인가)가 노드 상수에 남아 데이터셋이 자기 완결적이지 않게 된다.
    position        DOUBLE PRECISION    NOT NULL,
    max_effort      DOUBLE PRECISION    NOT NULL,
    -- 사람이 읽기 위한 이름('open'/'close'/'grasp'). 제어에 쓰이지 않으며 비어 있을
    -- 수 있다. 학습 입력이 아니라 필터링·가독성용이다.
    label           TEXT                NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS idx_gripper_cmds_episode  ON gripper_cmds (episode_id);
CREATE INDEX IF NOT EXISTS idx_gripper_cmds_topic    ON gripper_cmds (topic_id);
CREATE INDEX IF NOT EXISTS idx_gripper_cmds_stamp_ts ON gripper_cmds (stamp_ts);


-- /gripper_control/gripper_action_states → rdfp_msgs/msg/GripperActionState
CREATE TABLE IF NOT EXISTS gripper_action_states (
    id              BIGSERIAL           PRIMARY KEY,
    episode_id      BIGINT              NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT              NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    stamp_sec       INTEGER             NOT NULL,
    stamp_nanosec   BIGINT              NOT NULL,
    stamp_ts        TIMESTAMPTZ         GENERATED ALWAYS AS
                        (to_timestamp(stamp_sec::double precision
                                      + stamp_nanosec::double precision / 1e9))
                        STORED,
    position        DOUBLE PRECISION    NOT NULL,
    effort          DOUBLE PRECISION    NOT NULL,
    stalled         BOOLEAN             NOT NULL,
    reached_goal    BOOLEAN             NOT NULL,
    -- 액션 goal 상태. action_msgs/msg/GoalStatus 의 STATUS_* 와 동일한 값이다
    -- (4=SUCCEEDED, 5=CANCELED, 6=ABORTED, 2=EXECUTING).
    -- reached_goal=false 라도 status=5 면 실패가 아니라 후속 명령에 의한 선점이다.
    status          SMALLINT            NOT NULL DEFAULT 0
);
-- 기존 DB 호환: 위 CREATE TABLE 은 IF NOT EXISTS 이므로 이미 만들어진 테이블에는
-- 컬럼을 추가하지 않는다. status 도입 이전에 생성된 DB 를 위해 명시적으로 더한다.
ALTER TABLE gripper_action_states ADD COLUMN IF NOT EXISTS status SMALLINT NOT NULL DEFAULT 0;
CREATE INDEX IF NOT EXISTS idx_gripper_action_states_episode  ON gripper_action_states (episode_id);
CREATE INDEX IF NOT EXISTS idx_gripper_action_states_topic    ON gripper_action_states (topic_id);
CREATE INDEX IF NOT EXISTS idx_gripper_action_states_stamp_ts ON gripper_action_states (stamp_ts);


-- /target_joint_states → rdfp_msgs/msg/TargetJointStates
-- 단일 JointTrajectoryPoint 를 배열 컬럼 + tfs_sec/tfs_nanosec 로 평탄화한다.
CREATE TABLE IF NOT EXISTS target_joint_states (
    id              BIGSERIAL           PRIMARY KEY,
    episode_id      BIGINT              NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT              NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    stamp_sec       INTEGER             NOT NULL,
    stamp_nanosec   BIGINT              NOT NULL,
    stamp_ts        TIMESTAMPTZ         GENERATED ALWAYS AS
                        (to_timestamp(stamp_sec::double precision
                                      + stamp_nanosec::double precision / 1e9))
                        STORED,
    positions       DOUBLE PRECISION[]  NOT NULL,
    velocities      DOUBLE PRECISION[]  NOT NULL,
    accelerations   DOUBLE PRECISION[]  NOT NULL,
    effort          DOUBLE PRECISION[]  NOT NULL,
    tfs_sec         INTEGER             NOT NULL,
    tfs_nanosec     BIGINT              NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_target_joint_states_episode  ON target_joint_states (episode_id);
CREATE INDEX IF NOT EXISTS idx_target_joint_states_topic    ON target_joint_states (topic_id);
CREATE INDEX IF NOT EXISTS idx_target_joint_states_stamp_ts ON target_joint_states (stamp_ts);


-- /servo_node/delta_joint_cmds → control_msgs/msg/JointJog
-- ServoNode 의 조인트 단위 jog 입력. joint_names 와 displacements/velocities
-- 배열 길이는 서로 같아야 하며, 둘 중 한쪽은 비어 있을 수 있다.
CREATE TABLE IF NOT EXISTS joint_jogs (
    id              BIGSERIAL           PRIMARY KEY,
    episode_id      BIGINT              NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT              NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    stamp_sec       INTEGER             NOT NULL,
    stamp_nanosec   BIGINT              NOT NULL,
    stamp_ts        TIMESTAMPTZ         GENERATED ALWAYS AS
                        (to_timestamp(stamp_sec::double precision
                                      + stamp_nanosec::double precision / 1e9))
                        STORED,
    joint_names     TEXT[]              NOT NULL,
    displacements   DOUBLE PRECISION[]  NOT NULL,
    velocities      DOUBLE PRECISION[]  NOT NULL,
    duration        DOUBLE PRECISION    NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_joint_jogs_episode  ON joint_jogs (episode_id);
CREATE INDEX IF NOT EXISTS idx_joint_jogs_topic    ON joint_jogs (topic_id);
CREATE INDEX IF NOT EXISTS idx_joint_jogs_stamp_ts ON joint_jogs (stamp_ts);


-- /ee_pose → geometry_msgs/msg/PoseStamped (end-effector pose)
CREATE TABLE IF NOT EXISTS pose_stampeds (
    id              BIGSERIAL             PRIMARY KEY,
    episode_id      BIGINT                NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT                NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    stamp_sec       INTEGER               NOT NULL,
    stamp_nanosec   BIGINT                NOT NULL,
    stamp_ts        TIMESTAMPTZ           GENERATED ALWAYS AS
                        (to_timestamp(stamp_sec::double precision
                                      + stamp_nanosec::double precision / 1e9))
                        STORED,
    position        DOUBLE PRECISION[3]   NOT NULL,
    orientation     DOUBLE PRECISION[4]   NOT NULL,
    CHECK (array_length(position, 1) = 3),
    CHECK (array_length(orientation, 1) = 4)
);
CREATE INDEX IF NOT EXISTS idx_pose_stampeds_episode  ON pose_stampeds (episode_id);
CREATE INDEX IF NOT EXISTS idx_pose_stampeds_topic    ON pose_stampeds (topic_id);
CREATE INDEX IF NOT EXISTS idx_pose_stampeds_stamp_ts ON pose_stampeds (stamp_ts);


-- /camera/image_raw 등 sensor_msgs/msg/Image 토픽의 mp4 sidecar (글로벌 메타).
-- 에피소드×토픽 당 한 행. mp4 파일 자체의 정보 (경로, 코덱, 해상도, fps, frame_id,
-- 총 프레임 수, 생성 시각) 를 보관한다. 프레임별 stamp 는 image_frames 테이블
-- 에 1:1 로 저장된다.
CREATE TABLE IF NOT EXISTS image_streams (
    id              BIGSERIAL     PRIMARY KEY,
    episode_id      BIGINT        NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT        NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    mp4_path        TEXT          NOT NULL,
    codec           TEXT          NOT NULL,
    pixel_format    TEXT          NOT NULL,
    container_fps   INTEGER       NOT NULL,
    frame_id        TEXT          NOT NULL DEFAULT '',
    width           INTEGER       NOT NULL,
    height          INTEGER       NOT NULL,
    frame_count     BIGINT        NOT NULL DEFAULT 0,
    created_at      TIMESTAMPTZ   NOT NULL DEFAULT now(),
    UNIQUE (episode_id, topic_id)
);
CREATE INDEX IF NOT EXISTS idx_image_streams_episode ON image_streams (episode_id);
CREATE INDEX IF NOT EXISTS idx_image_streams_topic   ON image_streams (topic_id);


-- /camera/image_raw 등 sensor_msgs/msg/Image 의 mp4 sidecar (프레임 단위).
-- 영상은 mp4 파일에 저장되고, 본 테이블에는 mp4 의 N번째 프레임에 대한
-- 원본 timestamp 정보가 1:1 로 적재된다 (Mp4ImageRecorder 가 사용).
-- (episode_id, topic_id, frame_index) 가 mp4 파일 내 frame_index 번째 프레임을
-- 가리키며, UNIQUE 제약으로 중복 적재를 방지한다.
CREATE TABLE IF NOT EXISTS image_frames (
    id              BIGSERIAL     PRIMARY KEY,
    episode_id      BIGINT        NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT        NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    frame_index     BIGINT        NOT NULL,
    stamp_sec       INTEGER       NOT NULL,
    stamp_nanosec   BIGINT        NOT NULL,
    stamp_ts        TIMESTAMPTZ   GENERATED ALWAYS AS
                        (to_timestamp(stamp_sec::double precision
                                      + stamp_nanosec::double precision / 1e9))
                        STORED,
    CHECK (frame_index >= 0),
    UNIQUE (episode_id, topic_id, frame_index)
);
CREATE INDEX IF NOT EXISTS idx_image_frames_episode  ON image_frames (episode_id);
CREATE INDEX IF NOT EXISTS idx_image_frames_topic    ON image_frames (topic_id);
CREATE INDEX IF NOT EXISTS idx_image_frames_stamp_ts ON image_frames (stamp_ts);
