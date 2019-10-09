use diesel::prelude::*;
use diesel::result::Error;

use crate::models::message;
use crate::models::meta;
use crate::models::topic;
use crate::proto::bbr::hash;

use chrono::prelude::Utc;

// use std::error::Error;
use std::path::PathBuf;

mod hmacsha256;

use protobuf::Message;

use ring::hmac;

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
    conn.execute(
        "ALTER TABLE messages ADD COLUMN
            bbr_digest BLOB;",
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

pub fn insert_meta(conn: &SqliteConnection, input: &PathBuf) -> Result<(hmacsha256::Key), Error> {
    use crate::schema::metas;
    let name = String::from(input.file_stem().unwrap().to_str().unwrap());
    let timestamp = get_unix_timestamp_us();

    let mut bag_info = hash::BagInfo::new();
    bag_info.set_name(name.clone());
    bag_info.set_timestamp(timestamp.clone());

    let bbr_nonce = hmacsha256::Key::generate();
    let bbr_digest = hmacsha256::Key::new(
        hmac::sign(&bbr_nonce.s_key, &bag_info.write_to_bytes().unwrap()).as_ref(),
    );

    let new_meta = meta::NewMeta {
        name: name,
        timestamp: timestamp,
        bbr_nonce: bbr_nonce.to_vec(),
        bbr_digest: bbr_digest.to_vec(),
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
    use crate::schema::messages;
    use crate::schema::topics;
    let conn = establish_connection(&input);

    let meta_digest = conn.transaction::<_, Error, _>(|| {
        alter_tables(&conn)?;
        create_tables(&conn)?;
        Ok(insert_meta(&conn, &input)?)
    })?;

    let topic_results = topics::table
        .load::<topic::Topic>(&conn)
        .expect("Error loading topics");

    let mut topic_nonce = meta_digest.clone();
    for topic_result in topic_results {
        println!("Found topic {:?}", &topic_result.name);

        let mut topic_info = hash::TopicInfo::new();
        topic_info.set_serialization_type(topic_result.serialization_type.clone());
        topic_info.set_serialization_format(topic_result.serialization_format.clone());

        let topic_digest = hmacsha256::Key::new(
            hmac::sign(&topic_nonce.s_key, &topic_info.write_to_bytes().unwrap()).as_ref(),
        );

        let topic_form = topic::TopicForm {
            id: topic_result.id,
            bbr_nonce: topic_nonce.to_vec(),
            bbr_digest: topic_digest.to_vec(),
        };
        // topic_form.save_changes(&conn)?;
        diesel::update(&topic_form)
            .set(&topic_form)
            .execute(&conn)?;

        use crate::schema::messages::dsl::topic_id;
        let message_results = messages::table
            .filter(topic_id.eq(topic_result.id))
            .load::<message::Message>(&conn)
            .expect("Error loading messages");

        let mut message_nonce = topic_digest.clone();
        for message_result in message_results {
            println!("Found message {:?}", &message_result.id);

            let mut message_info = hash::MessageInfo::new();
            message_info.set_timestamp(message_result.timestamp.clone());

            let mut s_ctx = hmac::Context::with_key(&message_nonce.s_key);
            s_ctx.update(&message_info.write_to_bytes().unwrap());
            s_ctx.update(&message_result.data.as_slice());
            let message_digest = hmacsha256::Key::new(s_ctx.sign().as_ref());

            let topic_form = topic::TopicForm {
                id: message_result.id,
                bbr_nonce: message_nonce.to_vec(),
                bbr_digest: message_digest.to_vec(),
            };
            // topic_form.save_changes(&conn)?;
            diesel::update(&topic_form)
                .set(&topic_form)
                .execute(&conn)?;
            message_nonce = message_digest.clone();
        }

        let mut topic_meta = hash::TopicMeta::new();
        topic_meta.set_name(topic_result.name.clone());

        topic_nonce = hmacsha256::Key::new(
            hmac::sign(&topic_digest.s_key, &topic_meta.write_to_bytes().unwrap()).as_ref(),
        );
    }

    println!("Done");
    Ok(())
}
