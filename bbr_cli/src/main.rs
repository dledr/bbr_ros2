mod cli;
use cli::param;

fn main() {
    param::completions();
    let opt = param::Opt::from_args();
    println!("{:?}", opt);
}