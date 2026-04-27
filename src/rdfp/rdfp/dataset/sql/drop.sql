-- 후처리기 스키마 전체 삭제 (테스트 환경 초기화용).
-- 주의: 운영 환경에서 사용 금지.

DROP TABLE IF EXISTS image_frames         CASCADE;
DROP TABLE IF EXISTS image_streams        CASCADE;
DROP TABLE IF EXISTS gripper_states       CASCADE;
DROP TABLE IF EXISTS gripper_cmds         CASCADE;
DROP TABLE IF EXISTS target_joint_states  CASCADE;
DROP TABLE IF EXISTS joint_states         CASCADE;
DROP TABLE IF EXISTS joint_jogs           CASCADE;
DROP TABLE IF EXISTS twist_stampeds       CASCADE;
DROP TABLE IF EXISTS pose_stampeds        CASCADE;
DROP TABLE IF EXISTS topics               CASCADE;
DROP TABLE IF EXISTS sessions             CASCADE;
