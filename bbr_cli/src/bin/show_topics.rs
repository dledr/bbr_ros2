extern crate diesel;
extern crate bbr_cli;

use self::models::topic::*;
use diesel::prelude::*;
use bbr_cli::*;

use dotenv::dotenv;
use std::env;
use std::error::Error;

pub fn establish_connection() -> SqliteConnection {
    dotenv().ok();

    let database_url = env::var("DATABASE_URL").expect("DATABASE_URL must be set");
    SqliteConnection::establish(&database_url)
        .unwrap_or_else(|_| panic!("Error connecting to {}", database_url))
}

fn main() -> Result<(), Box<dyn Error>> {
    use crate::schema::topics;

    let conn = establish_connection();
    let results = topics::table
        // .filter(published.eq(true))
        // .limit(5)
        .load::<Topic>(&conn)
        .expect("Error loading topics");

    println!("Displaying {} topics", results.len());
    for result in results {
        println!("{}", result.name);
        println!("----------\n");
        println!("{}", result.serialization_format);

        let topic_form = TopicForm{
            id: result.id,
            bbr_nonce: result.bbr_nonce,
            bbr_digest: result.bbr_digest,
        };
        // topic_form.save_changes(&conn);
        diesel::update(&topic_form)
            .set(&topic_form)
            .execute(&conn)?;
    }
    Ok(())
}