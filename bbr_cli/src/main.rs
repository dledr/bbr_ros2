use std::path::PathBuf;
use structopt::{clap::{Shell}, StructOpt};

#[derive(Debug, StructOpt)]
#[structopt(name = "bbr",
    about = "A command line interface for bbr")]
struct Opt {
    #[structopt(subcommand)]
    cmd: Option<Command>,

    // /// enable debug mode
    // #[structopt(short, long)]
    // debug: bool,

    /// Enable verbose output
    #[structopt(short, long)]
    verbose: bool,
}

#[derive(Debug, StructOpt)]
enum Command {
    /// Bagfile subcommand
    Bag(Bag),

    #[structopt(
        name = "completions",
        about = "Completions for bbr",
        usage = "Examples:
        $ source <(bbr completions bash)
        $ bbr completions fish | source")]
    Completions(Completions),
}

#[derive(Debug, StructOpt)]
enum Bag {
    /// Checks bagfile validity and authenticity
    Check(Check),

    /// Converts bagfile to bbr format
    Convert(Convert),

    /// Finalizes bagfile by ending all records
    Finalize(Finalize),
}

#[derive(Debug, StructOpt)]
struct Check {

    /// Input file
    #[structopt(short, long)]
    input: PathBuf,

    // Degree of parallelism
    #[structopt(short, long, default_value = "1")]
    parallelism: u32,
}

#[derive(Debug, StructOpt)]
struct Convert {

    /// Input file
    #[structopt(short, long)]
    input: PathBuf,

    /// Output file
    #[structopt(short, long)]
    output: PathBuf,
}

#[derive(Debug, StructOpt)]
struct Finalize {

    /// Input file
    #[structopt(short, long)]
    input: PathBuf,
}

#[derive(Debug, StructOpt)]
struct Completions {

    /// The shell to generate the script for
    shell: Shell,
}

fn main() {

    // Check completions subcommand
    let matches = Opt::clap().get_matches();
    if let ("completions", Some(sub_matches)) = matches.subcommand() {
    let shell = sub_matches.value_of("shell").unwrap();
    Opt::clap().gen_completions_to(
        "bbr",
        shell.parse::<Shell>().unwrap(),
        &mut std::io::stdout());
    std::process::exit(0);
    }

    let opt = Opt::from_args();
    println!("{:?}", opt);
}