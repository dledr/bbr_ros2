use std::path::PathBuf;

extern crate rusqlite;
use rusqlite::NO_PARAMS;
// use rusqlite::types::ToSql;
use rusqlite::{params, Connection, Result};

pub fn convert(input: PathBuf) -> Result<()> {
    let mut conn = Connection::open(input)?;
    let tx = conn.transaction()?;
    tx.execute("
        ALTER TABLE topics ADD COLUMN
            bbr_nonce BLOB NOT NULL DEFAULT
            x'0000000000000000000000000000000000000000000000000000000000000000';",
        NO_PARAMS)?;
    tx.execute("
        ALTER TABLE topics ADD COLUMN
            bbr_digest BLOB NOT NULL DEFAULT
            x'0000000000000000000000000000000000000000000000000000000000000000';",
        NO_PARAMS)?;
    tx.execute("
        ALTER TABLE messages ADD COLUMN
            bbr_digest BLOB NOT NULL DEFAULT
            x'0000000000000000000000000000000000000000000000000000000000000000';",
        NO_PARAMS)?;
    tx.commit()
}