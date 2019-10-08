mod bag;
mod completions;

use std::error::Error;
use structopt::StructOpt;

#[derive(Debug, StructOpt)]
#[structopt(name = "bbr", about = "A command line interface for bbr")]
pub struct Opt {
    #[structopt(subcommand)]
    cmd: Option<Command>,

    /// Enable verbose output
    #[structopt(short, long)]
    verbose: bool,
}

impl Opt {
    pub fn from_args() -> Opt {
        <Opt as StructOpt>::from_args()
    }

    pub fn handle(opt: Opt) -> Result<(), Box<dyn Error>> {
        match opt.cmd.unwrap() {
            Command::Bag(bag) => {
                bag::Bag::handle(bag)?;
            }
            Command::Completions(completions) => {
                completions::Completions::handle(completions);
            } // _ => {
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
        $ bbr completions fish | source"
    )]
    Completions(completions::Completions),
}
