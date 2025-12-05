BEGIN;

CREATE TABLE alembic_version (
    version_num VARCHAR(32) NOT NULL, 
    CONSTRAINT alembic_version_pkc PRIMARY KEY (version_num)
);

-- Running upgrade  -> 9db158d0969e

CREATE TABLE users (
    id VARCHAR(36) NOT NULL, 
    name VARCHAR, 
    email VARCHAR NOT NULL, 
    email_verified_at TIMESTAMP WITHOUT TIME ZONE, 
    hashed_password VARCHAR NOT NULL, 
    is_admin BOOLEAN, 
    created_at TIMESTAMP WITHOUT TIME ZONE, 
    updated_at TIMESTAMP WITHOUT TIME ZONE, 
    deleted_at TIMESTAMP WITHOUT TIME ZONE, 
    verification_code VARCHAR, 
    verification_code_expires_at TIMESTAMP WITHOUT TIME ZONE, 
    password_reset_code VARCHAR, 
    password_reset_code_expires_at TIMESTAMP WITHOUT TIME ZONE, 
    PRIMARY KEY (id)
);

CREATE UNIQUE INDEX ix_users_email ON users (email);

CREATE TABLE devices (
    id VARCHAR(36) NOT NULL, 
    name VARCHAR, 
    serial_number VARCHAR, 
    api_key VARCHAR, 
    model VARCHAR, 
    firmware_version VARCHAR, 
    is_active BOOLEAN, 
    registered_at TIMESTAMP WITHOUT TIME ZONE, 
    user_id VARCHAR(36), 
    created_at TIMESTAMP WITHOUT TIME ZONE, 
    updated_at TIMESTAMP WITHOUT TIME ZONE, 
    deleted_at TIMESTAMP WITHOUT TIME ZONE, 
    PRIMARY KEY (id), 
    FOREIGN KEY(user_id) REFERENCES users (id)
);

CREATE UNIQUE INDEX ix_devices_api_key ON devices (api_key);

CREATE UNIQUE INDEX ix_devices_serial_number ON devices (serial_number);

CREATE TABLE issues (
    id VARCHAR(36) NOT NULL, 
    issue_type VARCHAR, 
    description VARCHAR, 
    severity VARCHAR(8), 
    detected_at TIMESTAMP WITHOUT TIME ZONE, 
    resolved BOOLEAN, 
    user_id VARCHAR(36), 
    created_at TIMESTAMP WITHOUT TIME ZONE, 
    updated_at TIMESTAMP WITHOUT TIME ZONE, 
    deleted_at TIMESTAMP WITHOUT TIME ZONE, 
    PRIMARY KEY (id), 
    FOREIGN KEY(user_id) REFERENCES users (id)
);

CREATE TABLE metrics (
    id VARCHAR(36) NOT NULL, 
    metric_type VARCHAR(19), 
    value FLOAT, 
    unit VARCHAR, 
    sensor_model VARCHAR, 
    timestamp TIMESTAMP WITHOUT TIME ZONE, 
    user_id VARCHAR(36), 
    created_at TIMESTAMP WITHOUT TIME ZONE, 
    updated_at TIMESTAMP WITHOUT TIME ZONE, 
    deleted_at TIMESTAMP WITHOUT TIME ZONE, 
    PRIMARY KEY (id), 
    FOREIGN KEY(user_id) REFERENCES users (id)
);

CREATE INDEX ix_metrics_timestamp ON metrics (timestamp);

INSERT INTO alembic_version (version_num) VALUES ('9db158d0969e') RETURNING alembic_version.version_num;

COMMIT;

