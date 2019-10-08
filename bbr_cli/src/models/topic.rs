use crate::schema::topics;

#[derive(Debug, diesel::Queryable)]
pub struct Topic {
    pub id: i64,
    pub name: String,
    pub serialization_type: String,
    pub serialization_format: String,
    pub bbr_nonce: Vec<u8>,
    pub bbr_digest: Vec<u8>,
}

#[derive(diesel::AsChangeset, diesel::Identifiable)]
#[table_name = "topics"]
pub struct TopicForm {
    pub id: i64,
    pub bbr_nonce: Vec<u8>,
    pub bbr_digest: Vec<u8>,
}
