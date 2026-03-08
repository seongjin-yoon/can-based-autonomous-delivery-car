-- 차량 정보 테이블
CREATE TABLE IF NOT EXISTS vehicle_table (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    vehicle_id TEXT NOT NULL
);

-- 배송 주문 정보 테이블
CREATE TABLE IF NOT EXISTS order_table (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    order_id TEXT NOT NULL,
    destination TEXT NOT NULL
);

-- vehicle_id 빠른 검색을 위한 인덱스
CREATE INDEX IF NOT EXISTS idx_vehicle_id ON vehicle_table(vehicle_id);

-- 차량 테이블의 타임스탬프 쿼리를 위한 인덱스
CREATE INDEX IF NOT EXISTS idx_vehicle_ts ON vehicle_table(timestamp);

-- order_id 빠른 검색을 위한 인덱스
CREATE INDEX IF NOT EXISTS idx_order_id ON order_table(order_id);

-- 주문 테이블의 타임스탬프 쿼리를 위한 인덱스
CREATE INDEX IF NOT EXISTS idx_order_ts ON order_table(timestamp);

CREATE INDEX IF NOT EXISTS idx_order_dest ON order_table(destination);

-- 차량 Heartbeat 정보 테이블 (10초마다 업데이트)
CREATE TABLE IF NOT EXISTS heartbeat_table (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    vehicle_id TEXT NOT NULL,
    status TEXT DEFAULT 'active'
);

-- heartbeat 테이블의 vehicle_id 빠른 검색을 위한 인덱스
CREATE INDEX IF NOT EXISTS idx_heartbeat_vehicle_id ON heartbeat_table(vehicle_id);

-- heartbeat 테이블의 타임스탬프 쿼리를 위한 인덱스
CREATE INDEX IF NOT EXISTS idx_heartbeat_ts ON heartbeat_table(timestamp);