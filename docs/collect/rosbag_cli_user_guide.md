# `rosbag` CLI 사용 가이드 — 에피소드 조회 · 적재 · 삭제

rosbag2 MCAP 아카이브에 **하는 모든 조작**이 `ros2 run rdfp rosbag <서브커맨드>` 하나에 모여 있다. 녹화(쓰기)는 `scripts/record_rosbag.sh` 가, 적재 후의 조회·재생은 DB 기반 CLI(`stats` / `list` / `replay`)가 맡는다.

## 목차

1. [명령 한눈에](#1-명령-한눈에)
2. [공통 선택 인자](#2-공통-선택-인자)
3. [`list-episodes` — 무엇이 담겼나](#3-list-episodes--무엇이-담겼나)
4. [`export` — 범위를 DB 로](#4-export--범위를-db-로)
5. [`delete` — bag 에서 제거](#5-delete--bag-에서-제거)
6. [`list-topics` · `clear`](#6-list-topics--clear)
7. [활용 절차](#7-활용-절차)
8. [주의 사항](#8-주의-사항)
9. [Python API](#9-python-api)
10. [관련 파일](#10-관련-파일)

---

## 1. 명령 한눈에

```bash
ros2 run rdfp rosbag list-topics     # 토픽 요약 (읽기 전용)
ros2 run rdfp rosbag list-episodes   # 에피소드 목록 + DB 적재 여부
ros2 run rdfp rosbag export          # 선택한 에피소드를 DB · MP4 로 적재
ros2 run rdfp rosbag delete          # 선택한 에피소드를 bag 에서 제거 (파괴적)
ros2 run rdfp rosbag clear           # 세션 데이터 전체 삭제 (파괴적)
```

**bag 루트는 `--rosbag-dir` > `--config` 의 `rosbag_dir` > `RDFP_ROSBAG_DIR` 순으로 정한다.** 설정이 환경변수를 이기는 것이 중요하다 — 환경변수가 이기면 **설정이 가리키는 DB 에 다른 bag 을 적재**할 수 있다. 종료 코드는 성공 `0`, 사용자 오류(경로·인자·설정) `2`, 실행 실패(확인 거부·검증 실패) `1` 이다.

> ### 시간 기준이 둘이다 — 헷갈리면 잘못된 범위를 지운다
>
> | 기준 | 값 | 쓰이는 곳 |
> |---|---|---|
> | **log time** | rosbag 이 메시지를 **기록한** 시각. 언제나 녹화기의 벽시계 | 목록 정렬 · `--since` / `--until` |
> | **stamp** | 발행자가 찍은 `header.stamp` | 에피소드 자연키 · DB 조회 · `--start-ns` |
>
> 사용자가 지정하는 시간 범위는 **log time** 이다. `header.stamp` 는 발행자 시계라 시뮬레이터에 따라 경과시간이 섞이는 회차가 있어 사람이 아는 시각과 어긋날 수 있다.

---

## 2. 공통 선택 인자

`list-episodes` · `export` · `delete` 가 같은 문법을 쓴다. 여러 개를 주면 **AND** 다.

| 인자 | 뜻 |
|---|---|
| `--rosbag-dir PATH` | bag 루트. 미지정 시 `--config` 의 `rosbag_dir` → `RDFP_ROSBAG_DIR` |
| `--dates YYYY-MM-DD …` | 날짜 폴더 선행 필터. 파일을 열기 전에 걸러 빠르다 |
| `--task LABEL` | 그 `task_label` 만 |
| `--since TIME` / `--until TIME` | log time 범위. `'YYYY-MM-DD HH:MM[:SS]'` 또는 `'YYYY-MM-DD'` (로컬 시간대) |
| `--episodes SPEC` | 목록 순번. `3` · `3-7` · `3-7,9` |
| `--start-ns NS` | 에피소드 자연키. 여러 번 지정 가능 |
| `--config PATH` | dataset 설정 YAML — **적재 여부 확인과 적재에 쓴다** |

**시간 범위는 걸치기만 해도 포함된다.** 에피소드가 `[since, until]` 에 일부라도 겹치면 선택된다 — 경계에 걸린 것을 반쪽만 다루면 데이터가 조용히 잘리기 때문이다.

> ⚠️ **`--episodes` 의 순번은 안정적이지 않다.** 순번은 그 실행의 목록 안에서만 유효하다. `--dates` 를 바꾸거나 새 세션이 녹화되면 같은 번호가 다른 에피소드를 가리킨다. **사람이 목록을 보고 바로 고를 때만 쓰고, 스크립트·재현에는 `--start-ns` 를 쓴다** (목록 마지막 열에 그 값이 있다).

---

## 3. `list-episodes` — 무엇이 담겼나

```bash
ros2 run rdfp rosbag list-episodes --rosbag-dir /data/rosbag
ros2 run rdfp rosbag list-episodes --config dataset_config.yaml --task peg_in_hole
ros2 run rdfp rosbag list-episodes --config dataset_config.yaml --format json
```

```
   #  recorded (log time)      dur  task         result   export  session                      start_ns
   1  2026-09-11 21:14:03    12.4s  peg_in_hole  success  yes     session_2026-09-11_21-13-40  1789...
   2  2026-09-11 21:15:58    15.1s  peg_in_hole  -        -       session_2026-09-11_21-13-40  1789...
   3  2026-09-11 21:17:22     9.8s  tray_place   failure  yes     session_2026-09-11_21-13-40  1789...

3 episode(s)
```

**기록 시각 오름차순**이다. `--format json` 은 같은 내용에 `log_start_ns` · `log_stop_ns`까지 실어 기계가 읽을 형태로 낸다.

`result` 열은 `SessionCommand.outcome` 이다. **`-` 는 실패가 아니라 판정 없음**이다 — 성패를 기록하지 않은 에피소드가 실패로 집계되면 학습셋에서 조용히 빠진다.

### `export` 열은 세 값이다

| 표시 | 뜻 |
|---|---|
| `yes` | DB 에 그 자연키의 행이 있다 (테이블 이름은 `sessions` 다) |
| `-` | 없다 |
| `?` | **확인하지 않았다** — `--config` 미지정, 또는 DB 접속·설정 읽기 실패 |

`?` 를 '없다' 로 읽으면 이미 적재된 것을 다시 넣게 된다. 그래서 두 값으로 뭉개지 않는다. **적재 상태로 고르는 선택(`--only-new` · `--exported`)은 `?` 가 하나라도 있으면 거부된다.**

적재 여부는 **질의 한 번**으로 확인한다(목록의 최소~최대 `start_sec` 구간). 에피소드마다 묻지 않으므로 목록이 길어도 비용이 늘지 않는다.

---

## 4. `export` — 범위를 DB 로

```bash
# 시간 범위
ros2 run rdfp rosbag export --config dataset_config.yaml \
    --since '2026-09-11 21:00' --until '2026-09-11 22:00'

# 아직 적재되지 않은 것만
ros2 run rdfp rosbag export --config dataset_config.yaml --only-new

# 목록에서 고른 뒤 먼저 확인
ros2 run rdfp rosbag export --config dataset_config.yaml --episodes 2-3 --dry-run
```

`--config` 가 **필수**다 — DB 접속과 mp4 출력 경로가 거기 있다. `--dry-run` 은 선택 결과만 출력하고 아무것도 쓰지 않는다.

**자체 적재 경로가 없다.** 선택 결과를 자연키 목록으로 바꿔 `episode_filter.start_ns_in`에 실어 **`import` 와 같은 파이프라인**에 넘긴다. 덕분에 `on_existing_episode` 정책, 요약 집계, mp4 라우팅, 에피소드 단위 트랜잭션이 한 벌로 유지된다. 즉 `export` 는 "`import` 에 선택을 더한 것" 이다.

### `--only-new` 와 `on_existing_episode: skip` 은 다르다

| | 판정 시점 | 결과 |
|---|---|---|
| `on_existing_episode: skip` | 에피소드 구간을 **읽은 뒤** | 중복은 건너뛰지만 **읽는 비용을 치른다** |
| `--only-new` | 읽기 **전** | 중복은 아예 스트리밍하지 않는다 |

둘은 겹치지 않는다. 대량 재적재에서는 `--only-new` 가 시간을 실제로 줄인다.

### 설정 파일로도 범위를 고정할 수 있다

`dataset_config.yaml` 의 `episode_filter` 에 같은 키가 있다. CLI 인자가 **설정을 이긴다.**

```yaml
episode_filter:
  task_labels_include: [peg_in_hole]
  min_duration_sec: 3.0
  start_ns_in: [1789131071582800000]   # 자연키로 못박는다
```

자연키로 골라도 `min_duration_sec` 같은 품질 조건은 **함께 적용된다** — 그러지 않으면 품질 기준을 우회하는 경로가 생긴다.

---

## 5. `delete` — bag 에서 제거

```bash
# 반드시 먼저 dry-run 으로 대상을 확인한다
ros2 run rdfp rosbag delete --since '2026-09-11 21:00' --until '2026-09-11 22:00' --dry-run

# 확인 후 실행
ros2 run rdfp rosbag delete --since '2026-09-11 21:00' --until '2026-09-11 22:00' --yes

# 이미 DB 에 들어간 것만 정리 (디스크 회수)
ros2 run rdfp rosbag delete --exported --config dataset_config.yaml --yes
```

`--exported` 는 **DB 에 이미 적재된 에피소드만** 고른다 — 적재를 마친 bag 을 비우는 용도다. `--config` 가 함께 있어야 한다(없으면 적재 여부가 `?` 라 거부된다).

확인 없이는 지우지 않는다. 대화형 셸에서는 묻고, **비대화형에서는 `--yes` 가 필수**다 (`clear` 와 같은 규칙).

### 어떻게 지우는가 — split 을 다시 쓴다

split 은 기본 **1시간** 단위라 에피소드 하나보다 훨씬 크다. 통째로 지우면 같은 split 의 다른 에피소드가 함께 사라진다. 그래서 이렇게 한다.

1. MCAP 를 읽어 **지울 구간의 메시지만 빼고** 임시 파일에 새로 쓴다. 메시지 본문은 역직렬화하지 않고 **바이트째 복사**하며, 스키마·채널도 원본 레코드를 그대로 재등록한다 — 인코딩과 메타데이터가 보존된다.
2. 원본을 그 파일로 **원자적으로 교체**한다 (`os.replace`).
3. `metadata.yaml` 의 메시지 수 · 시작 시각 · 구간 · 토픽별 개수를 다시 계산한다.
   `discover_splits` 가 이 파일을 읽으므로, 고치지 않으면 카탈로그가 낡은 값을 보고한다.
4. **에피소드를 다시 감지해 검증한다** — 지우려던 것만 사라지고 나머지가 그대로인지 본다. 어긋나면 `RuntimeError` 로 멈추고 무엇이 어긋났는지 보고한다.

경계의 `/session` 메시지까지 함께 지운다. 시작 이벤트가 남으면 목록에 껍데기 에피소드가 계속 보이고, 종료 이벤트가 남으면 **다음 에피소드의 경계 판정이 틀어진다.**

---

## 6. `list-topics` · `clear`

```bash
ros2 run rdfp rosbag list-topics --rosbag-dir /data/rosbag
ros2 run rdfp rosbag list-topics --format json --sort count     # name|count|rate|type
ros2 run rdfp rosbag clear --rosbag-dir /data/rosbag --yes      # 전체 삭제
```

`list-topics` 는 split 마다 MCAP `Summary` 를 열어 **토픽별 메시지 수 · 타입 · 첫/마지막 시각**을 모아 `ls -l` 스타일로 낸다. 에피소드 경계는 보지 않는다.

`clear` 는 `rosbag_dir` 하위 세션 데이터를 **전부** 지운다. 에피소드 단위로 지우려면 `delete` 를 쓴다. 셸 래퍼도 있다 — `scripts/list_rosbag_topics.sh`, `scripts/clear_rosbag.sh`.

---

## 7. 활용 절차

### 7.1 수집 → 적재 → 정리

```bash
# ① 무엇이 담겼는지 본다 (적재 여부까지)
ros2 run rdfp rosbag list-episodes --config dataset_config.yaml

# ② 아직 안 들어간 것만 적재
ros2 run rdfp rosbag export --config dataset_config.yaml --only-new

# ③ 들어간 것을 확인하고
ros2 run rdfp rosbag list-episodes --config dataset_config.yaml

# ④ 적재된 것만 bag 에서 지워 디스크를 회수
ros2 run rdfp rosbag delete --exported --config dataset_config.yaml --dry-run
ros2 run rdfp rosbag delete --exported --config dataset_config.yaml --yes
```

③ 을 건너뛰지 않는다 — ②가 일부만 성공했을 수 있고, ④는 파괴적이다.

### 7.2 특정 작업만 데이터셋에 넣기

```bash
ros2 run rdfp rosbag list-episodes --config dataset_config.yaml --task peg_in_hole
ros2 run rdfp rosbag export --config dataset_config.yaml --task peg_in_hole --only-new
```

### 7.3 실패한 시도를 버리기

성패는 목록의 `result` 열에 있다. **CLI 에 성패 필터는 없으므로** 순번이나 자연키로 고른다.

```bash
ros2 run rdfp rosbag list-episodes --config dataset_config.yaml     # result 열 확인
ros2 run rdfp rosbag delete --start-ns 1789131071582800000 --dry-run
ros2 run rdfp rosbag delete --start-ns 1789131071582800000 --yes
```

### 7.4 "녹화는 됐는데 DB 에 없다" 진단

```bash
ros2 run rdfp rosbag list-topics      # 토픽·건수는 있는가
ros2 run rdfp rosbag list-episodes    # 에피소드로 잡히는가
```

- `list-topics` 에도 안 보인다 → `metadata.yaml` 누락 (§8.1)
- 토픽은 보이는데 에피소드가 0 → `/session` 이 녹화 목록에 없거나 `header.stamp` 가 비었다
- 에피소드는 보이는데 `export` 가 `-` → 아직 적재 안 됨. `export --only-new`

### 7.5 특정 구간만 다시 적재

```bash
# 기존 행을 지우고 새로 넣는다 — id 가 바뀐다 (§8.4)
ros2 run rdfp rosbag export --config dataset_config.yaml --episodes 5 --dry-run
```

재적재는 설정의 `on_existing_episode` 가 정한다 (`skip` 기본 / `replace` / `error`).

---

## 8. 주의 사항

### 8.1 `metadata.yaml` 이 없는 세션은 조용히 제외된다

`discover_splits` 는 그 세션을 "아직 녹화 중이거나 비정상 종료"로 보고 건너뛴다. 활성
split 이 자동으로 걸러지는 이점이 있지만, rosbag2 가 `SIGKILL` 로 죽었으면 `list-episodes`
가 **빈 목록**을, `export` 가 **빈 요약으로 성공**을 낸다 — 오류가 없다.

```bash
ros2 bag reindex -s mcap /data/rosbag/<YYYY-MM-DD>/<session_dir>
```

`.mcap` 이 있는데도 아무것도 안 보이면 거의 이 경우다.

### 8.2 `header.stamp` 가 비면 에피소드로 잡히지 않는다

에피소드 감지는 `/session` 메시지의 `header.stamp` 로 경계를 만든다. 스탬프가 없거나 0 이면 **그 에피소드가 목록에 아예 나오지 않는다.** `ros2 bag info` 로는 메시지가 보이므로 원인을 찾기 어렵다. 발행자가 `use_sim_time=False` 로 돌 때 자기 시계로 찍어도 안 된다 — 벽시계 값이라 sim time 창 밖이다.

### 8.3 삭제는 되돌릴 수 없다

`delete` 는 split 을 새로 써서 원본을 교체한다. **백업이 없으면 복구 수단이 없다.**

- 반드시 `--dry-run` 으로 대상을 먼저 확인한다.
- `--exported` 는 "DB 에 있다" 만 본다 — **MP4 는 확인하지 않는다** (§8.5).
- 검증(4단계)이 실패하면 그 시점에 멈추지만, **이미 교체된 split 은 되돌아가지 않는다.** 중요한 데이터라면 세션 디렉터리를 복사해 두고 실행한다.

### 8.4 에피소드 식별자는 유도된 자연키다

rosbag2 안에는 **에피소드라는 개체도, 그 식별자도 없다.** `SessionCommand` 에 id 필드가 없고 세션 노드도 만들지 않는다. 식별자로 쓰는 것은 시작 `/session` 메시지의 `header.stamp`(= `start_ns`)이고, DB 의 `sessions.UNIQUE (start_sec, start_nanosec)` 이 같은 값이라 bag ↔ DB 를 잇는다. DB 의 `id` 는 `BIGSERIAL` 대리키라 bag 만 보고는 알 수 없고, `on_existing_episode: replace` 로 재적재하면 **바뀐다.**

⚠️ **`start_ns` 가 언제나 유일하지는 않다.** `header.stamp` 가 경과시간인 회차에서는 시뮬레이터를 재시작할 때 0 부터 다시 시작하므로 **서로 다른 세션의 에피소드가 같은 키**를 갖는다. 그러면 `export` 열의 `yes` 가 다른 세션의 에피소드를 가리킬 수 있다. 목록에 세션 이름이 함께 나오는 이유이며, 의심되면 수집 전에 stamp 기준을 확인한다.

### 8.5 "적재됨" 의 기준은 `sessions` 행 하나다

MP4 나 `image_streams` 행은 보지 않는다. `image_streams` 까지 보면 더 엄격해지지만 **이미지 토픽이 없는 에피소드와 구분할 수 없다.** 영상까지 확인해야 하면 `stats` 로 DB 쪽 집계를 본다.

### 8.6 split 하나는 메모리에 올려 정렬한다

읽기 계층이 split 전체를 적재한 뒤 `header.stamp` 로 정렬한다. split 크기가 곧 메모리 요구량이므로, 분할 주기(`SPLIT_DURATION_SEC`, 기본 3600초)가 이 비용을 정한다.

### 8.7 녹화 토픽 목록은 백엔드마다 다르다

rosbag2 는 목록에 없는 토픽을 조용히 건너뛴다. `config/recording_topics.list` 는 mock 기준(`/camera/image_raw`)이라 Isaac(`/isaac/camera/image_raw`)에 그대로 쓰면 이미지가 한 장도 담기지 않는다. Isaac 은 `config/recording_topics_isaac.list` 를 `RECORDING_TOPICS_FILE` 로 지정하며, **`dataset_config.yaml` 의 `topics_file` 도 같은 파일을 가리켜야 한다.**

### 8.8 없는 기능

- **메시지 내용을 보는 CLI 가 없다.** 개별 값은 `ros2 bag play` / `ros2 bag info` 같은
  표준 도구로 보거나, 적재 후 DB 를 조회한다.
- **성패(`result`) 필터가 없다.** 목록을 보고 `--start-ns` 로 고른다.
- **bag 을 직접 재생하는 경로가 없다.** 재생은 DB 를 거친다.
- **에피소드 단위 이동·복사가 없다.** 지우는 것만 된다.

---

## 9. Python API

CLI 로 안 되는 조회는 라이브러리로 한다. ROS 없이 import 된다 (`mcap_ros2` 만 필요).

```python
from rdfp.rosbag.episode_ops import build_views, load_episodes, select

episodes = load_episodes('/data/rosbag', dates=['2026-09-11'])
views = build_views(episodes, exported_keys=None)          # None = 확인 안 함
picked = select(views, task='peg_in_hole', since_ns=t0, until_ns=t1)
for v in picked:
    print(v.index, v.start_ns, v.bag.session_name, v.bag.episode.success)
```

메시지를 직접 읽으려면 세 함수를 쓴다.

| 함수 | 하는 일 |
|---|---|
| `catalog.discover_splits(dir, dates=, topics=)` | `Split` 목록 (경로 · 세션 · 시각 · 건수 · 토픽) |
| `mcap_reader.iter_split_messages(path, topics=, time_range_ns=)` | split 하나를 `header.stamp` 순으로. `TimedMessage`(`stamp_ns` · `topic` · `type_name` · `msg` · `log_time_ns`) |
| `merged_stream.merge_split_streams(paths, topics=, time_range_ns=)` | 여러 split 을 `heapq.merge` 로 stamp 순 단일 스트림 |

> `mcap_ros2` 의 `start_time`/`end_time` 은 **`log_time` 기준**이라 `iter_split_messages`
> 는 그 인자를 쓰지 않고 `header.stamp` 로 다시 필터한다. 두 시각은 같지 않다.

적재 파이프라인도 같은 API 를 쓴다 — split 수집(`discover_splits`), 에피소드 경계 검출(`episode.from_bag.episodes_from_splits`, **CLI 와 같은 함수**), 토픽 분류, 에피소드 구간 스트리밍 적재.

---

## 10. 관련 파일

| 경로 | 내용 |
|---|---|
| `src/rdfp/rdfp/rosbag/cli.py` | 서브커맨드 · 출력 포맷 |
| `src/rdfp/rdfp/rosbag/episode_ops.py` | 선택 · 적재 여부 · 삭제(MCAP 재작성 · metadata 재계산 · 검증) |
| `src/rdfp/rdfp/rosbag/catalog.py` | split 탐색 · `Split` |
| `src/rdfp/rdfp/rosbag/mcap_reader.py` | split 하나 읽기 · `TimedMessage` |
| `src/rdfp/rdfp/rosbag/merged_stream.py` | k-way 시각 머지 |
| `src/rdfp/rdfp/dataset/episode_key.py` | 자연키 (`split_ns` / `join_ns` / `fetch_exported_keys`) |
| `src/rdfp/rdfp/dataset/ingest/episode/from_bag.py` | split → 에피소드 (`BagEpisode`) |
| `src/rdfp/rdfp/dataset/ingest/episode/detector.py` | `/session` 전이 → `Episode` |
| `src/rdfp/rdfp/rosbag/tests/` | `test_catalog.py` · `test_rosbag_cli.py` · `test_episode_ops.py` + fixtures |

- [dataset_ingest_design.md](dataset_ingest_design.md) — MCAP 장기 저장 · 분할 설계
- [dataset_ingest_design.md](dataset_ingest_design.md) — 에피소드 분할 규칙과 적재 설계
- [dataset_postproc_user_guide.md](dataset_postproc_user_guide.md) — `import` / `stats` / `list` / `replay`
