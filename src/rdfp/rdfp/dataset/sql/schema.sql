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
    -- 실패 사유 등). 형태가 scene 레시피·백엔드마다 달라지므로 정규화하지 않는다.
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


-- /joint_states · /target_joint_cmds → sensor_msgs/msg/JointState
-- **두 토픽이 같은 테이블에 들어가고 `topic_id` 로만 갈린다** — 하나는 관측(실제 도달값),
-- 하나는 action(컨트롤러로 나간 명령값)이다. export 가 토픽 등록부로 둘을 가른다.
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
    -- 배열의 관절 순서. **순서 규약만으로는 부족하다** — 토픽마다 레이아웃이 다르고
    -- (관측은 손가락 관절을 포함하지만 명령은 팔 7축뿐이다), 차원별 이름을 요구하는
    -- export 형식도 있다. rosbag 은 적재 뒤 지우므로 **이름을 버리면 되살릴 수 없다**
    -- (좌표 공간 문서 권고 4).
    --
    -- ⚠️ **빈 배열이 정상인 경우가 있다.** `target_joint_cmds_publisher` 는 컨트롤러의
    -- `joints` 파라미터 조회가 시간 초과되면 빈 `name` 으로 발행한다. 그때는 배열
    -- 순서만 남으므로 export 가 그 에피소드를 걸러낼 수 있도록 **사실대로 비워 둔다.**
    joint_names     TEXT[]              NOT NULL DEFAULT '{}',
    position        DOUBLE PRECISION[]  NOT NULL,
    velocity        DOUBLE PRECISION[]  NOT NULL,
    effort          DOUBLE PRECISION[]  NOT NULL,
    -- 이름이 있으면 위치 배열과 길이가 같아야 한다. 어긋나면 어느 칸이 어느 관절인지
    -- 알 수 없는데, 값의 범위가 비슷해 **데이터를 열어봐도 안 드러난다.**
    CHECK (array_length(joint_names, 1) IS NULL
           OR array_length(joint_names, 1) = array_length(position, 1))
);
CREATE INDEX IF NOT EXISTS idx_joint_states_episode  ON joint_states (episode_id);
CREATE INDEX IF NOT EXISTS idx_joint_states_topic    ON joint_states (topic_id);
CREATE INDEX IF NOT EXISTS idx_joint_states_stamp_ts ON joint_states (stamp_ts);


-- /gripper_cmds → rdfp_msgs/msg/GripperCommand
-- goal 은 'open' / 'close' / 'grasp' 심볼.
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
    -- 학습 데이터의 **action 채널**. 의도(심볼)만 남기고 숫자는 남기지 않는다 —
    -- position(m) / max_effort(N) 은 그리퍼에 종속이라 다른 기구로 옮기면 틀린 값이
    -- 된다(2026-09-01 결정, GripperCommand.msg 참조).
    --
    -- "실제로 얼마나 닫혔는가" 는 joint_states 에 손가락 관절로 남는다.
    goal            TEXT                NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_gripper_cmds_episode  ON gripper_cmds (episode_id);
CREATE INDEX IF NOT EXISTS idx_gripper_cmds_topic    ON gripper_cmds (topic_id);
CREATE INDEX IF NOT EXISTS idx_gripper_cmds_stamp_ts ON gripper_cmds (stamp_ts);


-- /gripper_states → rdfp_msgs/msg/GripperState
-- 그리퍼의 **연속 상태**. 명령과 무관하게 주기 발행된다.
CREATE TABLE IF NOT EXISTS gripper_states (
    id              BIGSERIAL           PRIMARY KEY,
    episode_id      BIGINT              NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT              NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    stamp_sec       INTEGER             NOT NULL,
    stamp_nanosec   BIGINT              NOT NULL,
    stamp_ts        TIMESTAMPTZ         GENERATED ALWAYS AS
                        (to_timestamp(stamp_sec::double precision
                                      + stamp_nanosec::double precision / 1e9))
                        STORED,
    -- 마지막으로 받은 GripperCommand.goal. 명령 이전에는 ''.
    -- 이것이 있어서 상태가 자기 완결적이다 — 명령 토픽과 시각으로 짝지을 필요가 없다.
    goal            TEXT                NOT NULL,
    -- **개구 폭(m)이며 관절값이 아니다.** 같은 데이터셋의 scene_objects.dimensions
    -- 와 같은 단위라 직접 비교된다. 구할 수 없으면 NaN 이다 — 0 은 '닫혀 있다'는
    -- 거짓말이 된다.
    width           DOUBLE PRECISION    NOT NULL,
    -- 힘을 내는데 안 움직인다. **관측**이지 판정이 아니다.
    -- **false 가 '물지 않았다'를 보장하지 않는다** — 판정 미구현 스택도 false 다.
    stalled         BOOLEAN             NOT NULL,
    -- **시킨 일을 이뤘는가.** 위치 도달이 아니다 (control_msgs 의 reached_goal 과 다르다).
    -- open/close 는 목표 자세 도달 AND NOT stalled, grasp 는 stalled 다.
    -- 자세 도달을 무엇으로 재는지는 구현이 정한다 — width 에 묶여 있지 않다.
    -- 판정은 노드가 이미 했으므로 **이 값 하나만 보면 된다.**
    -- ⚠️ false 는 '실패'와 '진행 중'을 구분하지 못한다 — width 를 함께 본다.
    at_goal         BOOLEAN             NOT NULL
);


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


-- /motion/trajectory → rdfp_msgs/msg/MotionTrajectory
-- 팔 동작 연산 하나를 실행할 때 **컨트롤러로 보낸 궤적**(L2). 학습 데이터의 관절-계획
-- 레이블이 여기서 나온다 (docs/collect/R1_실행_궤적_채널_구현_설계.md).
--
-- **주기 토픽이 아니라 이벤트다** — 연산마다 한 행이고, 궤적 전체가 그 한 행에 들어간다.
-- 그래서 다른 테이블과 달리 행 수가 에피소드 길이가 아니라 **연산 수**를 따른다.
--
-- ⚠️ `stamp_*` 는 발행 시각이 아니라 **궤적의 0초가 시작된 시각(t₀)** 이다. export 는
-- `t₀ + points[i].t` 로 각 경유점의 절대 시각을 만든다. 그 값이 틀리면 레이블 전체가
-- 통째로 밀리므로, 적재는 이 stamp 를 다른 테이블과 같은 방식으로 다루면 안 된다.
--
-- **경유점을 jsonb 로 둔다** — scene_objects 와 같은 이유다. 경유점마다 관절 수만큼의
-- 배열이 셋(위치·속도·가속도) 붙고, 속도·가속도는 계획 방식에 따라 비어 있을 수 있어
-- 고정 컬럼으로 정규화되지 않는다. 원소 형태는
--   {"t": float, "positions": [...], "velocities": [...], "accelerations": [...]}
-- 이고 `t` 는 궤적 시작으로부터의 **경과 초**(time_from_start)다.
CREATE TABLE IF NOT EXISTS motion_trajectories (
    id              BIGSERIAL     PRIMARY KEY,
    episode_id      BIGINT        NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT        NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    stamp_sec       INTEGER       NOT NULL,
    stamp_nanosec   BIGINT        NOT NULL,
    stamp_ts        TIMESTAMPTZ   GENERATED ALWAYS AS
                        (to_timestamp(stamp_sec::double precision
                                      + stamp_nanosec::double precision / 1e9))
                        STORED,
    -- 같은 연산의 motion_events 행과 짝짓는 값. **FK 를 걸지 않는다** — 계획 단계에서
    -- 실패한 연산은 이벤트만 있고 궤적이 없으며(그것이 'failed' 의 정의다), 반대로
    -- 이벤트 발행에 실패하면 궤적만 남는다. 어느 쪽도 적재를 막아서는 안 된다.
    operation_id    TEXT          NOT NULL,
    -- 계획에 적용한 속도 배율과 스트리밍 재생 배율. **아래 points 의 t 에는 이미
    -- 반영돼 있다** — 다시 곱하면 안 되고, 어떤 조건으로 만든 궤적인지 알기 위한 값이다.
    velocity_scaling DOUBLE PRECISION NOT NULL,
    time_scaling     DOUBLE PRECISION NOT NULL,
    -- 경유점 배열의 관절 순서. 배열만 있고 이름이 없으면 어느 관절인지 알 수 없다.
    joint_names     TEXT[]        NOT NULL,
    -- 경유점. **보간 전 · 중력 보상 전** 값이다 (펑션베이 오프셋이 섞이면 백엔드를
    -- 섞을 수 없다).
    points          JSONB         NOT NULL,
    point_count     INTEGER       GENERATED ALWAYS AS (jsonb_array_length(points)) STORED,
    -- 궤적 길이(초) = 마지막 경유점의 t. 연산 이벤트의 중단 시각을 이 안으로 자르는
    -- 기준이며, 레이블 규칙이 "정상 종료 뒤에는 끝점 유지" 라 자주 조회된다.
    -- GENERATED 로 두어 points 와 어긋날 수 없게 한다. 빈 궤적이면 NULL 이다.
    duration_sec    DOUBLE PRECISION GENERATED ALWAYS AS
                        ((points -> -1 ->> 't')::double precision) STORED,
    CHECK (jsonb_typeof(points) = 'array')
);
CREATE INDEX IF NOT EXISTS idx_motion_trajectories_episode ON motion_trajectories (episode_id);
CREATE INDEX IF NOT EXISTS idx_motion_trajectories_topic   ON motion_trajectories (topic_id);
CREATE INDEX IF NOT EXISTS idx_motion_trajectories_stamp_ts ON motion_trajectories (stamp_ts);
-- 두 채널을 잇는 유일한 끈이라 조인이 이 값으로만 일어난다.
CREATE INDEX IF NOT EXISTS idx_motion_trajectories_op ON motion_trajectories (episode_id, operation_id);


-- /motion/event → rdfp_msgs/msg/MotionEvent
-- 팔 동작 연산 하나가 **끝났다**. 위 궤적과 `operation_id` 로 짝지어 쓴다 — 그쪽이
-- "무엇을 보냈나", 이쪽이 "어떤 연산이었고 어떻게 끝났나" 다
-- (docs/collect/R2_연산_이벤트_채널_구현_설계.md).
--
-- **왜 필요한가** — 궤적은 *끝까지 갔을 때의 모습*이라, 연산이 중간에 멈췄는지도 언제
-- 멈췄는지도 모른다. 그 정보가 없으면 중단된 연산 뒤의 레이블이 **팔이 가지 않은 자리**를
-- 가리킨다.
--
-- ⚠️ `stamp_*` 는 **레이블 구간의 끝이 아니다.** 결과를 받은 시각이라 컨트롤러의 goal
-- tolerance 확인과 팔의 물리 지연(isaac 실측 120 ms)이 들어 있다. 구간 끝은 짝 궤적의
-- `stamp_ts + truncated_at_sec` 으로 계산한다. 이 값은 에피소드 창을 거르는 용도다.
CREATE TABLE IF NOT EXISTS motion_events (
    id              BIGSERIAL     PRIMARY KEY,
    episode_id      BIGINT        NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT        NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    stamp_sec       INTEGER       NOT NULL,
    stamp_nanosec   BIGINT        NOT NULL,
    stamp_ts        TIMESTAMPTZ   GENERATED ALWAYS AS
                        (to_timestamp(stamp_sec::double precision
                                      + stamp_nanosec::double precision / 1e9))
                        STORED,
    operation_id    TEXT          NOT NULL,
    -- 이동 방식: 'named_target' | 'joints' | 'cartesian'.
    -- 트윈 REST 이름(move_linear 등)이 **아니다** — 같은 지점을 관절 공간으로 갔는지
    -- 직선으로 갔는지가 재실행 가능성을 가르므로 그쪽을 남긴다.
    operation       TEXT          NOT NULL,
    -- 'succeeded' | 'aborted' | 'failed'. 기준은 **궤적을 컨트롤러로 넘기려 했는가** 다.
    -- goal 거부와 시작 상태 허용 오차 위반은 'failed' 가 아니라 'aborted' 다 —
    -- 그 시점엔 궤적이 이미 나갔다. **'failed' 만 짝 궤적이 없고, 그 불변식이 적재
    -- 검증의 고아 판정이 기대는 유일한 근거다.**
    --
    -- ⚠️ **어휘를 CHECK 로 걸지 않는다.** rosbag 은 적재 뒤 지우므로, 예상 못 한 값
    -- 하나가 에피소드 트랜잭션을 통째로 되돌리면 그 데이터를 되살릴 수 없다. 어휘는
    -- 발행부(motion_event_publisher)가 지키고, 여기서는 사실대로 담는다.
    result          TEXT          NOT NULL,
    -- 왜 그렇게 끝났는지. 사람이 읽는 값이고 레이블 계산에는 쓰지 않는다.
    reason          TEXT          NOT NULL DEFAULT '',
    -- **궤적 기준** 중단 시각(초). 정상 종료면 궤적 길이와 같고 'failed' 면 0 이다.
    -- export 는 `L2(truncated_at_sec)` 로 중단 자세를 얻는다.
    truncated_at_sec DOUBLE PRECISION NOT NULL,
    -- 해석된 목표 — 클라이언트가 **실제로 겨냥한** 값이지 사용자가 준 입력이 아니다.
    -- target_kind 가 'joints' 면 관절 배열을, 'pose' 면 position/orientation 과
    -- target_frame 을 본다. 'failed' 여도 채운다 — 무엇을 하려다 실패했는지가 남아야 한다.
    target_kind     TEXT          NOT NULL DEFAULT '',
    target_joint_names  TEXT[]             NOT NULL DEFAULT '{}',
    target_joint_values DOUBLE PRECISION[] NOT NULL DEFAULT '{}',
    target_position     DOUBLE PRECISION[],
    -- **ROS 규약 xyzw 순서.** 바꾸면 4개 float 에 unit norm 이라 어떤 검사도 통과하면서
    -- '그럴듯하게 틀린 자세'가 된다 (scene_objects 와 같은 함정).
    target_orientation  DOUBLE PRECISION[],
    -- 위 pose 가 **어느 링크의 자세인가** (tip link 이름). 기준 좌표계가 아니다.
    -- 펑션베이는 트윈이 EE 프레임 입력을 tip 기준으로 이미 옮긴 뒤라 사용자가 준 값과
    -- 149 mm 다르다 — 프레임을 안 적으면 그 차이가 데이터에서 드러나지 않는다.
    target_frame    TEXT          NOT NULL DEFAULT '',
    CHECK (target_position IS NULL OR array_length(target_position, 1) = 3),
    CHECK (target_orientation IS NULL OR array_length(target_orientation, 1) = 4),
    CHECK (array_length(target_joint_names, 1) IS NOT DISTINCT FROM
           array_length(target_joint_values, 1))
);
CREATE INDEX IF NOT EXISTS idx_motion_events_episode  ON motion_events (episode_id);
CREATE INDEX IF NOT EXISTS idx_motion_events_topic    ON motion_events (topic_id);
CREATE INDEX IF NOT EXISTS idx_motion_events_stamp_ts ON motion_events (stamp_ts);
CREATE INDEX IF NOT EXISTS idx_motion_events_op ON motion_events (episode_id, operation_id);


-- /scene/objects → rdfp_msgs/msg/SceneObjects
-- scene 안 물체들의 ground-truth 상태. 자동 라벨링(place 성공 판정), 오프라인 큐레이션
-- (파지 오차 계산·실패 에피소드 선별), pose estimator 학습 라벨이 모두 여기서 나온다.
--
-- **정책의 학습 입력(observation)으로 쓸지는 export 단계의 별도 결정**이며 적재 단계에서
-- 정하지 않는다 — 시뮬레이터의 ground truth 는 실기에 존재하지 않으므로 그대로 관측에
-- 넣으면 sim 에서만 도는 정책이 된다. 비교는
-- docs/rosbag2/scene_objects_observation_decision.md 에 있다.
--
-- **한 메시지가 한 행이다.** 물체마다 행을 나누지 않는 이유는 둘이다.
--   1. reader 계약이 **row 1개 → 메시지 1개** 이므로(db/readers/base.py), 물체별 행은
--      복원 시 재조립이 필요해 registry 구조를 벗어난다.
--   2. `dimensions` 길이가 종류마다 다르다 (box 3 / sphere 1 / cylinder 2). 고정 컬럼으로
--      정규화되지 않으며, 이것이 메시지가 배열-of-구조체인 이유이기도 하다.
-- 물체 단위 조회는 `jsonb_array_elements(objects)` 로 푼다.
CREATE TABLE IF NOT EXISTS scene_objects (
    id              BIGSERIAL     PRIMARY KEY,
    episode_id      BIGINT        NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT        NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    stamp_sec       INTEGER       NOT NULL,
    stamp_nanosec   BIGINT        NOT NULL,
    stamp_ts        TIMESTAMPTZ   GENERATED ALWAYS AS
                        (to_timestamp(stamp_sec::double precision
                                      + stamp_nanosec::double precision / 1e9))
                        STORED,
    -- 좌표 기준 프레임. 다른 테이블은 frame_id 를 버리지만 여기서는 남긴다 — 백엔드가
    -- world→base 변환을 빠뜨려도 값 자체는 그럴듯해서, 프레임을 함께 적지 않으면
    -- 데이터를 열어보기 전까지 오염이 드러나지 않기 때문이다. 현재 스택에서는
    -- 'panda_link0' 이어야 한다.
    frame_id        TEXT          NOT NULL DEFAULT '',
    -- SceneObject 배열. 원소 형태는
    --   {"name": str, "type": str, "dimensions": [float, ...],
    --    "position": [x, y, z], "orientation": [x, y, z, w]}
    -- 이며 orientation 은 **ROS 규약 xyzw** 다 (Isaac 의 wxyz 가 아니다).
    -- 물체가 없으면 빈 배열이고, 그것도 'scene 이 비었다'는 유효한 상태다.
    --
    -- **여기 실리는 것은 조작 대상뿐이다.** 탁자 같은 환경 물체는 발행 노드가
    -- 걸러내므로 들어오지 않는다 — 2026-09-01 결정 (docs/scene/scene_objects_guide.md).
    -- `fixture` 키가 있는 행은 그 이전에 적재된 것이며, reader 는 무시한다.
    objects         JSONB         NOT NULL,
    -- 빈 scene 제외 같은 필터를 매번 배열을 풀지 않고 걸 수 있게 한다.
    object_count    INTEGER       GENERATED ALWAYS AS (jsonb_array_length(objects)) STORED,
    CHECK (jsonb_typeof(objects) = 'array')
);
CREATE INDEX IF NOT EXISTS idx_scene_objects_episode  ON scene_objects (episode_id);
CREATE INDEX IF NOT EXISTS idx_scene_objects_topic    ON scene_objects (topic_id);
CREATE INDEX IF NOT EXISTS idx_scene_objects_stamp_ts ON scene_objects (stamp_ts);
-- objects 에는 GIN 을 두지 않는다. sessions.metadata 와 같은 판단이다 — 조회가
-- 느려지면 그때 더한다.


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
