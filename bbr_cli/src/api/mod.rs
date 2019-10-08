use diesel::prelude::*;
use diesel::result::Error;

use crate::models::meta::*;
use crate::models::topic::*;
use crate::proto::bbr::hash;

use chrono::prelude::Utc;

// use std::error::Error;
use std::path::PathBuf;

mod hmacsha256;
use hmacsha256::HMACSHA256;

use protobuf::Message;

pub fn alter_tables(conn: &SqliteConnection) -> Result<(), Error> {
    conn.execute(
        "ALTER TABLE topics ADD COLUMN
            bbr_nonce BLOB NOT NULL DEFAULT
            x'0000000000000000000000000000000000000000000000000000000000000000';",
    )?;
    conn.execute(
        "ALTER TABLE topics ADD COLUMN
            bbr_digest BLOB NOT NULL DEFAULT
            x'0000000000000000000000000000000000000000000000000000000000000000';",
    )?;
    Ok(())
}

pub fn create_tables(conn: &SqliteConnection) -> Result<(), Error> {
    conn.execute(
        "CREATE TABLE metas (
            id INTEGER PRIMARY KEY,
            name TEXT NOT NULL,
            timestamp INTEGER NOT NULL,
            bbr_nonce BLOB NOT NULL,
            bbr_digest BLOB NOT NULL)",
    )?;
    Ok(())
}

pub fn insert_meta(conn: &SqliteConnection, input: &PathBuf) -> Result<(Vec<u8>), Error> {
    use crate::schema::metas;
    let name = String::from(input.file_stem().unwrap().to_str().unwrap());
    let timestamp = get_unix_timestamp_us();

    let mut bag_info = hash::BagInfo::new();
    bag_info.set_name(name.clone());
    bag_info.set_stamp(timestamp.clone());
    let bbr_nonce = HMACSHA256::generate_key();
    let bbr_digest =
        HMACSHA256::create_tag(&bag_info.write_to_bytes().unwrap(), &bbr_nonce).to_vec();

    let new_meta = NewMeta {
        name: name,
        timestamp: timestamp,
        bbr_nonce: bbr_nonce.get_bytes().clone().to_vec(),
        bbr_digest: bbr_digest.clone(),
    };

    diesel::insert_into(metas::table)
        .values(vec![&new_meta])
        .execute(conn)?;
    Ok(bbr_digest)
}

pub fn get_unix_timestamp_us() -> i64 {
    let now = Utc::now();
    now.timestamp_nanos() as i64
}

pub fn establish_connection(input: &PathBuf) -> SqliteConnection {
    let database_url = &input.to_str().unwrap();
    SqliteConnection::establish(&database_url)
        .expect(&format!("Error connecting to {}", &database_url))
}

pub fn convert(input: PathBuf) -> Result<(), Box<dyn std::error::Error>> {
    use crate::schema::topics;
    let conn = establish_connection(&input);

    let bbr_digest = conn.transaction::<_, Error, _>(|| {
        alter_tables(&conn)?;
        create_tables(&conn)?;
        Ok(insert_meta(&conn, &input)?)
    })?;

    let results = topics::table
        .load::<Topic>(&conn)
        .expect("Error loading topics");

    let mut hmac_key = HMACSHA256::clone_key_from_slice(bbr_digest.as_slice());
    for result in results {
        let mut topic_format = hash::TopicFormat::new();
        topic_format.set_serialization_type(result.serialization_type.clone());
        topic_format.set_serialization_format(result.serialization_format.clone());
        let tag = HMACSHA256::create_tag(&topic_format.write_to_bytes().unwrap(), &hmac_key);

        let topic_form = TopicForm {
            id: result.id,
            bbr_nonce: hmac_key.get_bytes().to_vec(),
            bbr_digest: tag.to_vec(),
        };
        // topic_form.save_changes(&conn)?;
        diesel::update(&topic_form)
            .set(&topic_form)
            .execute(&conn)?;
        println!("Found topic {:?}", &result.name);
        hmac_key = HMACSHA256::clone_key_from_slice(&tag);
    }

    println!("Done");
    Ok(())
}
