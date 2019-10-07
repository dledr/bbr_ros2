#[derive(Debug, Queryable)]
pub struct Message {
    pub id: i64,
    pub topic_id: i64,
    pub timestamp: i64,
    pub data: Vec<u8>,
    pub bbr_digest: Option<Vec<u8>>,
}
