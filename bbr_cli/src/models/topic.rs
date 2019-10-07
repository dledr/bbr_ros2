#[derive(Debug, Queryable)]
pub struct Topic {
    pub id: i64,
    pub name: String,
    pub serialization_type: String,
    pub serialization_format: String,
    pub bbr_nonce: Vec<u8>,
    pub bbr_digest: Vec<u8>,
}
