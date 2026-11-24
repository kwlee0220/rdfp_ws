-- 후처리기 스키마 전체 삭제 (테스트 환경 초기화용).
-- 주의: 운영 환경에서 사용 금지.

DROP TABLE IF EXISTS motion_events        CASCADE;
DROP TABLE IF EXISTS motion_trajectories  CASCADE;
DROP TABLE IF EXISTS scene_objects        CASCADE;
DROP TABLE IF EXISTS image_frames         CASCADE;
DROP TABLE IF EXISTS image_streams        CASCADE;
-- 레거시: GripperActionState 폐기(2026-09-02) 이전 DB 에만 있다.
DROP TABLE IF EXISTS gripper_action_states CASCADE;
DROP TABLE IF EXISTS gripper_states       CASCADE;
DROP TABLE IF EXISTS gripper_cmds         CASCADE;
DROP TABLE IF EXISTS target_joint_states  CASCADE;
DROP TABLE IF EXISTS joint_states         CASCADE;
DROP TABLE IF EXISTS joint_jogs           CASCADE;
DROP TABLE IF EXISTS twist_stampeds       CASCADE;
DROP TABLE IF EXISTS pose_stampeds        CASCADE;
-- 적재 메타 (2026-11-23). 동적 매핑 테이블(topic_tables 가 가리키는 것)은 sessions 를
-- CASCADE 로 지울 때 FK 만 사라지고 남는다 — 이름이 사용자 정의라 여기서 열거할 수 없다.
DROP TABLE IF EXISTS ingest_units         CASCADE;
DROP TABLE IF EXISTS episode_sources      CASCADE;
DROP TABLE IF EXISTS ingest_sources       CASCADE;
DROP TABLE IF EXISTS topic_tables         CASCADE;
DROP TABLE IF EXISTS topics               CASCADE;
DROP TABLE IF EXISTS sessions             CASCADE;
