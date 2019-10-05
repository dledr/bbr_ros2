// extern crate hmac;
extern crate rusqlite;
// extern crate sha2;

// use hmac::{Hmac, Mac};
// use sha2::Sha256;

use std::path::PathBuf;

use rusqlite::NO_PARAMS;
// use rusqlite::types::ToSql;
use rusqlite::{params, Connection, Result};

mod hmacsha256;
use hmacsha256::HMACSHA256;

// type HmacSha256 = Hmac<Sha256>;

pub fn convert(input: PathBuf) -> Result<()> {
    let mut conn = Connection::open(&input)?;
    // let tx = conn.transaction()?;
    // tx.execute("
    //     ALTER TABLE topics ADD COLUMN
    //         bbr_nonce BLOB NOT NULL DEFAULT
    //         x'0000000000000000000000000000000000000000000000000000000000000000';",
    //     NO_PARAMS)?;
    // tx.execute("
    //     ALTER TABLE topics ADD COLUMN
    //         bbr_digest BLOB NOT NULL DEFAULT
    //         x'0000000000000000000000000000000000000000000000000000000000000000';",
    //     NO_PARAMS)?;
    // tx.execute("
    //     ALTER TABLE messages ADD COLUMN
    //         bbr_digest BLOB NOT NULL DEFAULT
    //         x'0000000000000000000000000000000000000000000000000000000000000000';",
    //     NO_PARAMS)?;
    // tx.commit()?;

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

    let mut _conn = Connection::open(&input)?;
    let tx = _conn.transaction()?;

    let mut hmac_key = HMACSHA256::generate_key();
    for message in messages_iter {
        let mut msg = message.unwrap();
        let tag = HMACSHA256::create_tag(
            &msg.data.unwrap().as_slice(),
            &hmac_key);
        // mac.input(msg.data.unwrap().as_slice());
        // let result = mac.result();
        msg.bbr_digest = Some(tag.to_vec());
        println!("Found message {:?}", msg.bbr_digest);
        // mac.reset();
        // hmac_key = HMACSHA256Key::from_slice(&tag)
        tx.execute("
            UPDATE messages SET 
                bbr_digest = (?1)
            WHERE id = (?2)",
        params![msg.bbr_digest, msg.id],
        )?;
        hmac_key = HMACSHA256::clone_key_from_slice(&tag);
    }
    tx.commit()?;

    println!("Done");
    Ok(())
}