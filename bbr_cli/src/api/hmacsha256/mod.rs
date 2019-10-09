use ring::{digest, hmac, rand};

#[derive(Clone)]
pub struct Key {
    pub key_value: [u8; digest::SHA256_OUTPUT_LEN],
    pub s_key: hmac::Key,
}

impl Key {
    pub fn generate() -> Self {
        let rng = rand::SystemRandom::new();
        let key_value: [u8; digest::SHA256_OUTPUT_LEN] = rand::generate(&rng)
            .expect("msg: &str")
            .expose();
        Self::_new(key_value)
    }
    
    fn _new(key_value: [u8; digest::SHA256_OUTPUT_LEN]) -> Self {
        let s_key = hmac::Key::new(hmac::HMAC_SHA256, key_value.as_ref());
        Self {
            key_value,
            s_key,
        }
    }
    
    pub fn new(key_bytes: &[u8]) -> Self {
        let mut key_value: [u8; digest::SHA256_OUTPUT_LEN] = Default::default();
        key_value.copy_from_slice(&key_bytes[..digest::SHA256_OUTPUT_LEN]);
        let s_key = hmac::Key::new(hmac::HMAC_SHA256, key_value.as_ref());
        Self {
            key_value,
            s_key,
        }
    }

    #[inline(always)]
    pub fn to_vec(&self) -> Vec<u8> {
        self.key_value.to_vec()
    }
}
