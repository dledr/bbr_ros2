table! {
    messages (id) {
        id -> BigInt,
        topic_id -> BigInt,
        timestamp -> BigInt,
        data -> Binary,
        bbr_digest -> Nullable<Binary>,
    }
}

table! {
    metas (id) {
        id -> BigInt,
        name -> Text,
        timestamp -> BigInt,
        bbr_nonce -> Binary,
        bbr_digest -> Binary,
    }
}

table! {
    topics (id) {
        id -> BigInt,
        name -> Text,
        #[sql_name = "type"]
        serialization_type -> Text,
        serialization_format -> Text,
        bbr_nonce -> Binary,
        bbr_digest -> Binary,
    }
}

allow_tables_to_appear_in_same_query!(
    messages,
    metas,
    topics,
);
