mod bag;
mod completions;

use rusqlite::{Result};
use structopt::{StructOpt};

#[derive(Debug, StructOpt)]
#[structopt(name = "bbr", about = "A command line interface for bbr")]
pub struct Opt {

    #[structopt(subcommand)]
    cmd: Option<Command>,

    // /// enable debug mode
    // #[structopt(short, long)]
    // debug: bool,

    /// Enable verbose output
    #[structopt(short, long)]
    verbose: bool,
}

impl Opt {
    pub fn from_args() -> Opt { <Opt as StructOpt>::from_args() }

    pub fn handle(opt: Opt) -> Result<()> {

        match opt.cmd.unwrap() {
            Command::Bag(bag) => {
                bag::Bag::handle(bag)?;
            }
            Command::Completions(completions) => {
                completions::Completions::handle(completions);
            }
            // _ => {
            //     println!("This subcommand is not yet implemented.");
            // }
        }

        Ok(())
    }
}

#[derive(Debug, StructOpt)]
enum Command {
    Bag(bag::Bag),

    #[structopt(
        about = "Completions for bbr",
        usage = "Examples:
        $ source <(bbr completions bash)
        $ bbr completions fish | source")]
    Completions(completions::Completions),
}
