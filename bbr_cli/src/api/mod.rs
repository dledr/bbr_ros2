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
    tx.commit()?;

    #[derive(Debug)]
    struct Message {
        id: i64,
        topic_id: i64,
        timestamp: i64,
        data: Option<Vec<u8>>,
        bbr_digest: Option<Vec<u8>>,
    }

    let mut stmt = conn.prepare("SELECT id, topic_id, timestamp, data, bbr_digest FROM messages")?;
    let messages_iter = stmt.query_map(params![], |row| {
        Ok(Message {
            id: row.get(0)?,
            topic_id: row.get(1)?,
            timestamp: row.get(2)?,
            data: row.get(3)?,
            bbr_digest: row.get(4)?,
        })
    })?;

    for message in messages_iter {
        println!("Found message {:?}", message.unwrap());
    }
    println!("Done");
    Ok(())
}