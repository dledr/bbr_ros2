# Add prior stage to cache/copy from
FROM rust AS cache
WORKDIR /tmp

# Copy from build context
COPY ./ ./ws

# Filter or glob files to cache upon
RUN mkdir ./cache && cd ./ws && \
    find ./ -name "Cargo.*" -o \
      -name "lib.rs" -o -name "main.rs" | \
      xargs cp --parents -t ../cache && \
    cd ../cache && \
    find ./ -name "lib.rs" | \
      xargs -I% sh -c "echo > %" && \
    find ./ -name "main.rs" | \
      xargs -I% sh -c "echo 'fn main() { }' > %"

# Continue with build stage
FROM rust as build
WORKDIR /ws

RUN apt-get update && apt-get install -y \
      libprotobuf-dev \
      libprotoc-dev \
      protobuf-compiler

# copy workspace dependency metadata
COPY --from=cache /tmp/cache ./

# fetch and build dependencies
RUN cargo fetch && cargo build

# # copy rest of the workspace code
COPY --from=cache /tmp/ws ./

# # build code with cached dependencies
RUN cargo build
