#[derive(Debug, Queryable)]
pub struct Meta {
    pub id: i64,
    pub name: String,
    pub timestamp: i64,
    pub bbr_nonce: Vec<u8>,
    pub bbr_digest: Vec<u8>,
}
