use std::path::PathBuf;

extern crate rusqlite;
use rusqlite::{Connection, Result};
use rusqlite::NO_PARAMS;

pub fn convert(input: PathBuf) -> Result<()> {
    let mut conn = Connection::open(input)?;
    let tx = conn.transaction()?;

    tx.execute("
        ALTER TABLE topics ADD COLUMN
            bbr_nonce BLOB NOT NULL DEFAULT 0;",
        NO_PARAMS)?;
    tx.execute("
        ALTER TABLE topics ADD COLUMN
            bbr_digest BLOB NOT NULL DEFAULT 0;",
        NO_PARAMS)?;
    tx.execute("
        ALTER TABLE messages ADD COLUMN
            bbr_digest BLOB NOT NULL DEFAULT 0;",
        NO_PARAMS)?;
    tx.commit()
}