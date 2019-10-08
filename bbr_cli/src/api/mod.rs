use diesel::prelude::*;
// use diesel::sqlite::Sqlite;

// use crate::models::topic::Topic;
// use crate::schema::topics;

extern crate chrono;
use chrono::prelude::Utc;

use std::error::Error;
use std::path::PathBuf;

mod hmacsha256;
use hmacsha256::HMACSHA256;

use crate::proto::bbr::hash;
use protobuf::{parse_from_bytes,Message};


// pub fn alter_tables(conn: &mut Connection) -> Result<()> {
//     let tx = conn.transaction()?;
//     tx.execute("
//         ALTER TABLE topics ADD COLUMN
//             bbr_nonce BLOB NOT NULL DEFAULT
//             x'0000000000000000000000000000000000000000000000000000000000000000';",
//         NO_PARAMS)?;
//     tx.execute("
//         ALTER TABLE topics ADD COLUMN
//             bbr_digest BLOB NOT NULL DEFAULT
//             x'0000000000000000000000000000000000000000000000000000000000000000';",
//         NO_PARAMS)?;
//     tx.commit()
// }

// fn create_meta(conn: &mut Connection, meta: &Meta) -> Result<()> {
//     let tx = conn.transaction()?;
//     tx.execute("
//         CREATE TABLE metas (
//             id INTEGER PRIMARY KEY,
//             name TEXT NOT NULL,
//             timestamp INTEGER NOT NULL,
//             bbr_nonce BLOB NOT NULL,
//             bbr_digest BLOB NOT NULL)",
//         NO_PARAMS)?;
//     tx.execute("
//         INSERT INTO metas (
//             name,
//             timestamp,
//             bbr_nonce,
//             bbr_digest)
//         VALUES (?1, ?2, ?3, ?4)",
//         params![
//             meta.name,
//             meta.timestamp,
//             meta.bbr_nonce,
//             meta.bbr_digest])?;
//     tx.commit()
// }

pub fn get_unix_timestamp_us() -> i64 {
    let now = Utc::now();
    now.timestamp_nanos() as i64
}

pub fn convert(input: PathBuf) -> Result<(), Box<dyn Error>> {
    let database_url = &input.to_str().unwrap();
    let connection = SqliteConnection::establish(&database_url)
        .expect(&format!("Error connecting to {}", &database_url));
    
    // use crate::schema::topics::dsl::*;

    // let results = topics::table.load::<Topic>(&connection)
    //     .expect("Error loading topics");
    
    // let results = topics.filter(topic.name.eq("/chatter"))
    //     .limit(5)
    //     .load::<Topic>(&connection)
    //     .expect("Error loading posts");

    // println!("Displaying {} posts", results.len());
    // for post in results {
    //     println!("{}", post.title);
    //     println!("----------\n");
    //     println!("{}", post.body);
    // }

    // let mut conn = Connection::open(&input)?;

    // let hmac_key = HMACSHA256::generate_key();
    // let mut meta = Meta {
    //     id: 0,
    //     name: String::from(
    //         input.file_stem().unwrap()
    //         .to_str().unwrap()),
    //     timestamp: get_unix_timestamp_us(),
    //     bbr_nonce: Some(hmac_key
    //             .get_bytes()
    //             .clone()
    //             .to_vec()),
    //     bbr_digest: None,
    // };

    // let mut bag_info = hash::BagInfo::new();
    // bag_info.set_name(meta.name.clone());
    // bag_info.set_stamp(meta.timestamp.clone());
    // meta.bbr_digest = Some(
    //     HMACSHA256::create_tag(
    //         &bag_info.write_to_bytes().unwrap(),
    //         &hmac_key).to_vec());

    // // create_meta(&mut conn)?;
    // create_meta(&mut conn, &meta)?;
    // alter_tables(&mut conn)?;

    // let mut topics_stmt = conn.prepare("
    //     SELECT
    //         id,
    //         name,
    //         type,
    //         serialization_format
    //     FROM topics")?;

    // let topics_iter = topics_stmt.query_map(params![], |row| {
    //     Ok(Topic {
    //         id: row.get(0)?,
    //         name: row.get(1)?,
    //         serialization_type: row.get(2)?,
    //         serialization_format: row.get(3)?,
    //         bbr_nonce: None,
    //         bbr_digest: None,
    //     })
    // })?;

    // let mut _conn = Connection::open(&input)?;
    // let tx = _conn.transaction()?;

    // let mut hmac_key = HMACSHA256::clone_key_from_slice(
    //     meta.bbr_digest.unwrap().as_slice());
    // for topic_result in topics_iter {
    //     let mut topic = topic_result.unwrap();
    //     let tag = HMACSHA256::create_tag(
    //         &topic.name.as_bytes(),
    //         &hmac_key);
    //     topic.bbr_nonce = Some(hmac_key.get_bytes().to_vec());
    //     topic.bbr_digest = Some(tag.to_vec());
    //     println!("Found topic {:?}", &topic.name);
    //     tx.execute("
    //         UPDATE topics SET 
    //             bbr_nonce = (?1),
    //             bbr_digest = (?2)
    //         WHERE id = (?3)",
    //         params![topic.bbr_nonce, topic.bbr_digest, topic.id],
    //     )?;
    //     hmac_key = HMACSHA256::clone_key_from_slice(&tag);
    // }
    // tx.commit()?;

    println!("Done");
    Ok(())
}