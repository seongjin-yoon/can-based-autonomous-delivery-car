-- ============================================================================
-- RPi1 Vehicle Onboard SQLite Database Initialization
-- 파일: init_rpi1.sql
-- 용도: 차량 탑재 온보드 컴퓨터의 로컬 데이터베이스 생성
-- 경로: /home/pi/catnip_vehicle/database/vehicle.db
-- ============================================================================
-- 설계 기준: RPi3 중앙 서버와의 일관성 유지
-- - delivery_table, password_table, event_log 구조 통일
-- - MQTT Topic과 QoS 설정 규격화
-- ============================================================================

-- ============================================================================
-- 1. 배달 테이블 (RPi3와 동기화)
-- ============================================================================
CREATE TABLE IF NOT EXISTS delivery_table (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    delivery_id TEXT NOT NULL UNIQUE,
    vehicle_id TEXT NOT NULL,
    destination TEXT NOT NULL,
    receiver TEXT NOT NULL,
    order_time DATETIME DEFAULT CURRENT_TIMESTAMP,
    start_time DATETIME,
    arrive_time DATETIME,
    complete_time DATETIME,
    status TEXT DEFAULT 'ordered'    -- ordered, ready, in_transit, arrived, completed
);

-- ============================================================================
-- 2. 비밀번호/PIN 테이블 (RPi3와 동기화)
-- ============================================================================
CREATE TABLE IF NOT EXISTS password_table (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    vehicle_id TEXT NOT NULL,
    delivery_id TEXT NOT NULL UNIQUE,
    pin_code TEXT NOT NULL UNIQUE,
    pin_type TEXT NOT NULL,                 
    created_time DATETIME DEFAULT CURRENT_TIMESTAMP,
    expire_time DATETIME,
    used INTEGER DEFAULT 0,                 -- 0: 미사용, 1: 사용됨
    attempt_count INTEGER DEFAULT 0         -- 시도 횟수
);

-- ============================================================================
-- 3. 이벤트 로그 테이블 (RPi3와 동기화)
-- ============================================================================
CREATE TABLE IF NOT EXISTS event_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    vehicle_id TEXT,
    event_type TEXT NOT NULL,              -- delivery_assigned, delivery_started, etc.
    delivery_id TEXT,
    detail TEXT,
    severity TEXT DEFAULT 'info'           -- info, warning, critical
);

-- ============================================================================
-- RPi1 추가 테이블 (온보드 전용 로깅)
-- ============================================================================

-- 4. PIN 인증 시도 로그 (보안 감시용)
CREATE TABLE IF NOT EXISTS pin_attempt_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    delivery_id TEXT NOT NULL,
    vehicle_id TEXT NOT NULL,
    pin_type TEXT NOT NULL,                
    result TEXT NOT NULL,                  -- success, failed, timeout
    attempt_number INTEGER,                -- 시도 번호
    input_length INTEGER,                  -- 입력된 PIN의 길이 (전체 내용 저장 X)
    error_type TEXT                        -- too_short, too_long, invalid_char, mismatch
);

-- 5. 경고/알림 로그 (비상 상황)
CREATE TABLE IF NOT EXISTS alert_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    delivery_id TEXT,
    vehicle_id TEXT NOT NULL,
    alert_type TEXT NOT NULL,              -- e_stop, pin_max_failed, security_breach
    severity TEXT DEFAULT 'warning',       -- warning, critical
    description TEXT,
    resolved INTEGER DEFAULT 0,            -- 0: 미해결, 1: 해결됨
    resolved_time DATETIME,
    resolved_by TEXT
);

-- 6. 배달 이력 (완료된 배달 기록)
CREATE TABLE IF NOT EXISTS delivery_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    delivery_id TEXT NOT NULL UNIQUE,
    vehicle_id TEXT NOT NULL,
    destination TEXT,
    receiver TEXT,
    order_time DATETIME,
    start_time DATETIME,
    arrive_time DATETIME,
    complete_time DATETIME,
    total_duration_minutes INTEGER,        -- 배달에 소요된 시간
    distance_km REAL,                      -- 배달 거리 (옵션)
    status TEXT DEFAULT 'completed',
    offboard_pin_attempts INTEGER,
    onboard_pin_attempts INTEGER,
    notes TEXT,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);

-- 7. 차량 상태 히스토리 (성능 분석)
CREATE TABLE IF NOT EXISTS vehicle_status_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    delivery_id TEXT,
    vehicle_id TEXT NOT NULL,
    status TEXT,                           -- idle, ready, in_transit, arrived
    temperature REAL,                      -- CPU 온도 (옵션)
    battery_level INTEGER,                 -- 배터리 레벨 (옵션)
    memory_usage_mb INTEGER,               -- 메모리 사용량 (옵션)
    uptime_seconds INTEGER                 -- 가동 시간
);

-- 8. MQTT 통신 로그 (디버깅용)
CREATE TABLE IF NOT EXISTS mqtt_communication_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    direction TEXT NOT NULL,               -- send, receive
    topic TEXT NOT NULL,
    qos INTEGER,
    payload_size INTEGER,
    success INTEGER DEFAULT 1,             -- 0: 실패, 1: 성공
    error_message TEXT,
    delivery_id TEXT
);

-- 9. 성능 지표 (통계용)
CREATE TABLE IF NOT EXISTS performance_metrics (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    delivery_id TEXT UNIQUE,
    vehicle_id TEXT NOT NULL,
    delivery_count_total INTEGER,          -- 누적 배달 수
    average_completion_time_minutes REAL,  -- 평균 배달 시간
    pin_success_rate REAL,                 -- PIN 인증 성공률
    system_uptime_hours REAL,              -- 시스템 가동 시간
    alert_count_total INTEGER,             -- 누적 경고 수
    last_delivery_time DATETIME,
    notes TEXT
);

-- 10. 시스템 설정 (메타데이터)
CREATE TABLE IF NOT EXISTS system_settings (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    setting_key TEXT NOT NULL UNIQUE,
    setting_value TEXT NOT NULL,
    data_type TEXT,                        -- string, integer, boolean, float
    description TEXT,
    last_updated DATETIME DEFAULT CURRENT_TIMESTAMP
);

-- ============================================================================
-- 인덱스 생성 (쿼리 성능 최적화)
-- ============================================================================

-- delivery_table 인덱스
CREATE INDEX IF NOT EXISTS idx_delivery_vehicle ON delivery_table(vehicle_id);
CREATE INDEX IF NOT EXISTS idx_delivery_id ON delivery_table(delivery_id);
CREATE INDEX IF NOT EXISTS idx_delivery_status ON delivery_table(status);

-- password_table 인덱스
CREATE INDEX IF NOT EXISTS idx_password_delivery ON password_table(delivery_id);
CREATE INDEX IF NOT EXISTS idx_password_vehicle ON password_table(vehicle_id);
CREATE INDEX IF NOT EXISTS idx_password_expire ON password_table(expire_time);

-- event_log 인덱스
CREATE INDEX IF NOT EXISTS idx_event_timestamp ON event_log(timestamp);
CREATE INDEX IF NOT EXISTS idx_event_delivery_id ON event_log(delivery_id);
CREATE INDEX IF NOT EXISTS idx_event_type ON event_log(event_type);
CREATE INDEX IF NOT EXISTS idx_event_severity ON event_log(severity);

-- pin_attempt_log 인덱스
CREATE INDEX IF NOT EXISTS idx_pin_attempt_delivery ON pin_attempt_log(delivery_id);
CREATE INDEX IF NOT EXISTS idx_pin_attempt_timestamp ON pin_attempt_log(timestamp);
CREATE INDEX IF NOT EXISTS idx_pin_attempt_result ON pin_attempt_log(result);

-- alert_log 인덱스
CREATE INDEX IF NOT EXISTS idx_alert_timestamp ON alert_log(timestamp);
CREATE INDEX IF NOT EXISTS idx_alert_delivery_id ON alert_log(delivery_id);
CREATE INDEX IF NOT EXISTS idx_alert_type ON alert_log(alert_type);

-- delivery_history 인덱스
CREATE INDEX IF NOT EXISTS idx_history_delivery_id ON delivery_history(delivery_id);
CREATE INDEX IF NOT EXISTS idx_history_created ON delivery_history(created_at);

-- vehicle_status_history 인덱스
CREATE INDEX IF NOT EXISTS idx_status_history_timestamp ON vehicle_status_history(timestamp);
CREATE INDEX IF NOT EXISTS idx_status_history_delivery ON vehicle_status_history(delivery_id);

-- mqtt_communication_log 인덱스
CREATE INDEX IF NOT EXISTS idx_mqtt_timestamp ON mqtt_communication_log(timestamp);
CREATE INDEX IF NOT EXISTS idx_mqtt_topic ON mqtt_communication_log(topic);

-- ============================================================================
-- 초기 시스템 설정 데이터
-- ============================================================================

INSERT OR IGNORE INTO system_settings (setting_key, setting_value, data_type, description)
VALUES 
    ('vehicle_id', 'vehicle_001', 'string', '차량 고유 ID'),
    ('broker_host', '10.42.0.1', 'string', 'MQTT 브로커 주소'),
    ('broker_port', '1883', 'integer', 'MQTT 브로커 포트'),
    ('mqtt_username', 'hoji', 'string', 'MQTT 사용자명'),
    ('heartbeat_interval_ms', '500', 'integer', '하트비트 전송 주기 (ms)'),
    ('heartbeat_timeout_ms', '5000', 'integer', '하트비트 타임아웃 (ms)'),
    ('pin_max_attempts', '5', 'integer', 'PIN 최대 시도 횟수'),
    ('pin_attempt_lockout_minutes', '5', 'integer', 'PIN 잠금 시간 (분)'),
    ('database_version', '1.0', 'string', '데이터베이스 버전'),
    ('created_at', datetime('now'), 'string', '데이터베이스 생성 시간'),
    ('auto_cleanup_days', '90', 'integer', '자동 정리 기간 (일)');

-- ============================================================================
-- 데이터베이스 최적화 (PRAGMA)
-- ============================================================================

PRAGMA journal_mode = WAL;                  -- Write-Ahead Logging 모드
PRAGMA synchronous = NORMAL;                -- 정상 동기화 모드
PRAGMA cache_size = 10000;                  -- 캐시 크기
PRAGMA foreign_keys = ON;                   -- 외래키 제약 활성화
PRAGMA temp_store = MEMORY;                 -- 임시 테이블을 메모리에 저장

-- ============================================================================
-- 뷰 생성 (분석 및 조회용)
-- ============================================================================

-- 현재 배달 상태 뷰
CREATE VIEW IF NOT EXISTS v_current_delivery_status AS
SELECT 
    d.delivery_id,
    d.vehicle_id,
    d.status,
    d.destination,
    d.receiver,
    d.start_time,
    d.arrive_time,
    CASE 
        WHEN d.status = 'completed' THEN d.complete_time
        ELSE NULL 
    END AS complete_time,
    CAST((julianday('now') - julianday(d.start_time)) * 24 * 60 AS INTEGER) AS duration_minutes
FROM delivery_table d;

-- PIN 인증 통계 뷰
CREATE VIEW IF NOT EXISTS v_pin_statistics AS
SELECT 
    delivery_id,
    pin_type,
    COUNT(*) as total_attempts,
    SUM(CASE WHEN result = 'success' THEN 1 ELSE 0 END) as successful_attempts,
    SUM(CASE WHEN result = 'failed' THEN 1 ELSE 0 END) as failed_attempts,
    ROUND(SUM(CASE WHEN result = 'success' THEN 1 ELSE 0 END) * 100.0 / COUNT(*), 2) as success_rate
FROM pin_attempt_log
GROUP BY delivery_id, pin_type;

-- 배달 성능 통계 뷰
CREATE VIEW IF NOT EXISTS v_delivery_performance AS
SELECT 
    COUNT(*) as total_deliveries,
    ROUND(AVG(CAST(total_duration_minutes AS REAL)), 2) as avg_duration_minutes,
    MIN(total_duration_minutes) as min_duration_minutes,
    MAX(total_duration_minutes) as max_duration_minutes,
    COUNT(CASE WHEN status = 'completed' THEN 1 END) as completed_count
FROM delivery_history
WHERE created_at >= datetime('now', '-30 days');

-- 경고 통계 뷰
CREATE VIEW IF NOT EXISTS v_alert_statistics AS
SELECT 
    alert_type,
    COUNT(*) as total_count,
    SUM(CASE WHEN resolved = 1 THEN 1 ELSE 0 END) as resolved_count,
    SUM(CASE WHEN resolved = 0 THEN 1 ELSE 0 END) as unresolved_count,
    MAX(timestamp) as latest_alert
FROM alert_log
GROUP BY alert_type;

-- 최근 배달 이력 뷰
CREATE VIEW IF NOT EXISTS v_recent_deliveries AS
SELECT 
    delivery_id,
    receiver,
    destination,
    complete_time,
    total_duration_minutes,
    offboard_pin_attempts,
    onboard_pin_attempts
FROM delivery_history
ORDER BY complete_time DESC
LIMIT 20;

-- ============================================================================
-- 트리거 생성 (자동 유지보수)
-- ============================================================================

-- delivery_table 업데이트 시간 자동 갱신
CREATE TRIGGER IF NOT EXISTS tr_delivery_update_time
AFTER UPDATE ON delivery_table
FOR EACH ROW
BEGIN
    UPDATE delivery_table 
    SET order_time = CASE WHEN NEW.order_time IS NULL THEN CURRENT_TIMESTAMP ELSE NEW.order_time END
    WHERE id = NEW.id;
END;

-- ============================================================================
-- 마이그레이션 메모
-- ============================================================================

-- Version 1.0
-- - RPi3와의 일관성 유지
--   * delivery_table: 배달 상태 관리
--   * password_table: PIN 저장 (일회용)
--   * event_log: 상세 이벤트 로깅
--   * pin_attempt_log: PIN 인증 보안 로깅
--   * alert_log: 긴급 상황 기록
--   * delivery_history: 완료된 배달 이력
--   * vehicle_status_history: 차량 상태 변화 기록
--   * mqtt_communication_log: MQTT 통신 로그
--   * performance_metrics: 성능 지표
--   * system_settings: 시스템 설정

-- MQTT Topic 명세 (QoS 기준)
-- Topic                           QoS    Direction    설명
-- delivery/pin/{id}/3to1         2      ← RPi3       PIN 정보 전달 (정확히 1회)
-- delivery/command/{id}          1      ← RPi3       출동 명령 (최소 1회)
-- delivery/start/{id}/1to3       1      → RPi3       배달 시작 (최소 1회)
-- delivery/vehicle/{id}/status   0      → RPi3       차량 상태 (유실 무방)
-- delivery/vehicle/{id}/alert    1      → RPi3       경보 (최소 1회)
-- delivery/arrived/{id}/1to3     1      → RPi3       도착 (최소 1회)
-- delivery/unlock/{id}           1      → RPi3       잠금 해제 (최소 1회)
-- delivery/log/{id}              1      → RPi3       인증 로그 (최소 1회)
-- delivery/complete/{id}/1to3    1      → RPi3       배달 완료 (최소 1회)

-- ============================================================================
-- END OF INITIALIZATION SCRIPT
-- ============================================================================
