table! {
    messages (id) {
        id -> Nullable<Integer>,
        topic_id -> Integer,
        timestamp -> Integer,
        data -> Binary,
    }
}

table! {
    metas (id) {
        id -> Nullable<Integer>,
        name -> Text,
        timestamp -> Integer,
        bbr_nonce -> Binary,
        bbr_digest -> Binary,
    }
}

table! {
    topics (id) {
        id -> Nullable<Integer>,
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
