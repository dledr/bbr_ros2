mod cli;

use cli::param;
// use exitfailure::ExitFailure;
// use failure::ResultExt;

use structopt::{StructOpt};

extern crate rusqlite;

use rusqlite::{Connection, Result};
use rusqlite::NO_PARAMS;

fn main() -> Result<()> {
    let opt = param::Opt::from_args();
    println!("{:?}", opt);

    let cmd = opt.cmd.unwrap();

    match cmd {
        param::Command::Bag(bag) => {
            param::bag(bag);
        }
        param::Command::Completions(completions) => {
            param::completions(completions);
        }
        _ => {
            println!("This subcommand is not yet implemented.");
        }
    }

    Ok(())
}