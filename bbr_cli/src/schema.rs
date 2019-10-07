table! {
    messages (id) {
        id -> Nullable<BigInt>,
        topic_id -> BigInt,
        timestamp -> BigInt,
        data -> Binary,
    }
}

table! {
    metas (id) {
        id -> Nullable<BigInt>,
        name -> Text,
        timestamp -> BigInt,
        bbr_nonce -> Binary,
        bbr_digest -> Binary,
    }
}

table! {
    topics (id) {
        id -> Nullable<BigInt>,
        name -> Text,
        #[sql_name = "type"]
        type_ -> Text,
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
